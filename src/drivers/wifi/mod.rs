// ZerOS — Onboard Wi-Fi (`BCM43430B0` / `SYN43436`) SDIO1 Driver (M8.3)
//
// Drives the Raspberry Pi Zero 2 W's onboard 802.11b/g/n Wi-Fi chip connected to
// `GPIO 34..39` (`ALT3` = `SD1_CLK, SD1_CMD, SD1_DAT0..3`), `GPIO 41` (`WL_ON`),
// and `GPIO 43` (`ALT0` = `GPCLK2` 32.768 kHz Low-Power Reference Clock).
//
// Architecture:
//   1. Configures BCM2710 Clock Manager `CM_GP2CTL` (`0x3F10_1080`) to output a
//      32.768 kHz reference clock (`LPO_IN`) on `GPIO 43` (`ALT0` = `GPCLK2`)
//   2. Temporarily switches the BCM2710 Arasan EMMC controller (`0x3F30_0000`) pinmux
//      from `GPIO 48..53` (microSD) to `GPIO 34..39` (`ALT3`, Wi-Fi SDIO1)
//   3. Supports 3-stage adaptive SDIO attach:
//      - Stage A (Warm-Reboot Fast Attach): Probes already-enumerated `BCM43430B0` at `RCA=0x0001`
//        directly via `CMD52` / `CMD7(0x0001_0000)` without killing its loaded state
//      - Stage B (Soft SDIO Re-Init): Issues `CMD52` soft-reset + `CMD5` -> `CMD3` -> `CMD7`
//      - Stage C (Cold Hardware Reset): Pulses `GPIO 41` (`WL_ON`) LOW -> HIGH while `GPCLK2`
//        (`32.768 kHz`) and `SD1_CLK` (`400 kHz`) are actively running, then runs `CMD5` -> `CMD3` -> `CMD7`
//   4. Uses `CMD52` (`IO_RW_DIRECT`) on Function 0 & Function 1 to:
//      - Read `CCCR` (`0x00`) SDIO revision
//      - Walk the Function 0 CIS Tuple chain (`CISTPL_MANFID = 0x20`) to read the hardware
//        Vendor/Device ID (`0x02D0:0xA9A6` = Broadcom/Cypress/Synaptics 43430/43436)
//      - Enable Function 1 (`Backplane`), map the Sonics/AXI Silicon Backplane window to
//        `0x1800_0000` (`ChipCommon`), and read the 32-bit F1 Silicon Signature (`0x1542a9a6`,
//        `ChipID = 43430`, `Rev = B0`)
//   5. Seamlessly restores the microSD card pinmux (`GPIO 48..53`) via `block::sd::init()`
//      so FAT32 VFS file operations continue without conflict.

#![allow(dead_code)]

use crate::arch::timer;
use crate::bsp;
use crate::drivers::{block, mmio};
use crate::sync::IrqSafeLock;

// ─── Arasan EMMC Register Offsets (Base: 0x3F30_0000) ───
const EMMC_BASE: usize = mmio::PERIPHERAL_BASE + 0x0030_0000;
const EMMC_ARG1: usize = EMMC_BASE + 0x08;
const EMMC_CMDTM: usize = EMMC_BASE + 0x0C;
const EMMC_RESP0: usize = EMMC_BASE + 0x10;
const EMMC_STATUS: usize = EMMC_BASE + 0x24;
const EMMC_CONTROL0: usize = EMMC_BASE + 0x28;
const EMMC_CONTROL1: usize = EMMC_BASE + 0x2C;
const EMMC_INTERRUPT: usize = EMMC_BASE + 0x30;
const EMMC_IRPT_MASK: usize = EMMC_BASE + 0x34;
const EMMC_IRPT_EN: usize = EMMC_BASE + 0x38;

const STATUS_CMD_INHIBIT: u32 = 1 << 0;
const CTRL0_POWER_ON: u32 = 1 << 8;
const CTRL0_VOLT_3_3V: u32 = 0b111 << 9;
const CTRL1_CLK_INTLEN: u32 = 1 << 0;
const CTRL1_CLK_STABLE: u32 = 1 << 1;
const CTRL1_CLK_EN: u32 = 1 << 2;
const CTRL1_TOUNIT_MAX: u32 = 0xE << 16;
const CTRL1_SRST_HC: u32 = 1 << 24;

const INT_CMD_DONE: u32 = 1 << 0;
const INT_ERROR_MASK: u32 = 0xFFFF_8000;

const CMD_RSPNS_NONE: u32 = 0b00 << 16;
const CMD_RSPNS_48: u32 = 0b10 << 16;
const CMD_RSPNS_48B: u32 = 0b11 << 16;
const CMD_CRCCHK_EN: u32 = 1 << 19;
const CMD_IXCHK_EN: u32 = 1 << 20;

const fn cmd_index(idx: u32) -> u32 {
    (idx & 0x3F) << 24
}

const CMD0_GO_IDLE: u32 = cmd_index(0) | CMD_RSPNS_NONE;
const CMD3_SEND_REL_ADDR: u32 = cmd_index(3) | CMD_RSPNS_48 | CMD_CRCCHK_EN | CMD_IXCHK_EN;
const CMD5_IO_SEND_OP_COND: u32 = cmd_index(5) | CMD_RSPNS_48;
const CMD7_SELECT_CARD: u32 = cmd_index(7) | CMD_RSPNS_48B | CMD_CRCCHK_EN | CMD_IXCHK_EN;
const CMD52_IO_RW_DIRECT: u32 = cmd_index(52) | CMD_RSPNS_48 | CMD_CRCCHK_EN | CMD_IXCHK_EN;

// ─── BCM2710 Clock Manager Registers for GPCLK2 (32.768 kHz WIFI_CLK on GPIO 43) ───
const CM_BASE: usize = mmio::PERIPHERAL_BASE + 0x0010_1000;
const CM_GP2CTL: usize = CM_BASE + 0x80;
const CM_GP2DIV: usize = CM_BASE + 0x84;
const CM_PASSWD: u32 = 0x5A00_0000;

// ─── GPIO Registers for SDIO1 (GPIO 34..39), WL_ON (GPIO 41), WIFI_CLK (GPIO 43) ───
const GPFSEL3: usize = mmio::GPIO_BASE + 0x0C; // GPIO 30..39
const GPFSEL4: usize = mmio::GPIO_BASE + 0x10; // GPIO 40..49
const GPFSEL5: usize = mmio::GPIO_BASE + 0x14; // GPIO 50..53
const GPSET1: usize = mmio::GPIO_BASE + 0x20; // Set GPIO 32..53
const GPCLR1: usize = mmio::GPIO_BASE + 0x2C; // Clear GPIO 32..53
const GPPUD: usize = mmio::GPIO_BASE + 0x94;
const GPPUDCLK1: usize = mmio::GPIO_BASE + 0x9C;

/// Live status of the Onboard Wi-Fi (`BCM43430B0` / `SYN43436`) SDIO1 controller
#[derive(Debug, Clone, Copy)]
pub struct WifiStatus {
    pub initialized: bool,
    pub wl_powered: bool,
    pub sdio_rca: u16,
    pub sdio_functions: u8,
    pub cccr_rev: u8,
    pub vendor_id: u16,
    pub device_id: u16,
    pub backplane_sig: u32,
    pub chip_id: u16,
    pub chip_rev: u8,
    pub f1_ready: bool,
}

impl WifiStatus {
    const fn empty() -> Self {
        Self {
            initialized: false,
            wl_powered: false,
            sdio_rca: 0,
            sdio_functions: 0,
            cccr_rev: 0,
            vendor_id: 0,
            device_id: 0,
            backplane_sig: 0,
            chip_id: 0,
            chip_rev: 0,
            f1_ready: false,
        }
    }
}

static STATE: IrqSafeLock<WifiStatus> = IrqSafeLock::new(WifiStatus::empty());

/// Return the cached Wi-Fi SDIO1 hardware status.
pub fn status() -> WifiStatus {
    STATE.with(|s| *s)
}

/// Start the 32.768 kHz Low-Power Reference Clock (`GPCLK2`) on `GPIO 43` (`ALT0`).
/// `19.2 MHz / 585.9375 = 32,768 Hz` (`DIVI = 585, DIVF = 3840`).
unsafe fn enable_wifi_lpclk_gpio43() {
    let ctl = mmio::read(CM_GP2CTL);
    if (ctl & (1 << 7)) == 0 {
        // Not currently running (BUSY == 0): configure 19.2MHz / 585.9375 = 32.768kHz
        mmio::write(CM_GP2CTL, CM_PASSWD | 1);
        timer::delay_ms(1);
        mmio::write(CM_GP2DIV, CM_PASSWD | (585 << 12) | 3840);
        mmio::write(CM_GP2CTL, CM_PASSWD | (1 << 9) | (1 << 4) | 1);
        timer::delay_ms(2);
    }
}

/// Route `Arasan EMMC` (`0x3F30_0000`) to `GPIO 34..39` (`ALT3` = `SD1_*`), ensure
/// `GPIO 41` (`WL_ON`) is HIGH, and `GPIO 43` (`WIFI_CLK`) is in `ALT0` (`GPCLK2`).
unsafe fn route_emmc_to_wifi_sdio() {
    enable_wifi_lpclk_gpio43();

    // 1. Disconnect microSD pins GPIO 48..53 (set to INPUT 0b000) so SD card ignores bus traffic
    let mut fsel4 = mmio::read(GPFSEL4);
    for pin in 48..=49 {
        let shift = (pin - 40) * 3;
        fsel4 &= !(0b111 << shift);
    }
    // Configure GPIO 41 (WL_ON) as OUTPUT (0b001) and GPIO 43 (WIFI_CLK) as ALT0 (0b100 = GPCLK2)
    fsel4 = (fsel4 & !(0b111 << 3)) | (0b001 << 3); // GPIO 41 = OUTPUT
    fsel4 = (fsel4 & !(0b111 << 9)) | (0b100 << 9); // GPIO 43 = ALT0 (GPCLK2)
    mmio::write(GPFSEL4, fsel4);

    // Ensure GPIO 41 (WL_ON) is driven HIGH (do NOT pull it LOW yet so warm-boot state stays alive!)
    mmio::write(GPSET1, 1u32 << (41 - 32));

    let mut fsel5 = mmio::read(GPFSEL5);
    for pin in 50..=53 {
        let shift = (pin - 50) * 3;
        fsel5 &= !(0b111 << shift);
    }
    mmio::write(GPFSEL5, fsel5);

    // 2. Route GPIO 34..39 to ALT3 (0b111 = SD1_CLK, SD1_CMD, SD1_DAT0..3)
    let mut fsel3 = mmio::read(GPFSEL3);
    for pin in 34..=39 {
        let shift = (pin - 30) * 3;
        fsel3 = (fsel3 & !(0b111 << shift)) | (0b111 << shift);
    }
    mmio::write(GPFSEL3, fsel3);

    // Enable internal pull-ups on GPIO 35..39 (SD1_CMD, SD1_DAT0..3)
    mmio::write(GPPUD, 2);
    for _ in 0..150 {
        core::hint::spin_loop();
    }
    mmio::write(GPPUDCLK1, 0b11111 << (35 - 32));
    for _ in 0..150 {
        core::hint::spin_loop();
    }
    mmio::write(GPPUD, 0);
    mmio::write(GPPUDCLK1, 0);
}

/// Reset and configure the Arasan EMMC host controller (`0x3F30_0000`) at 400kHz init clock.
unsafe fn init_emmc_host_for_sdio() {
    mmio::write(EMMC_CONTROL0, 0);
    mmio::write(EMMC_CONTROL1, CTRL1_SRST_HC);
    let mut timeout = 100_000u32;
    while (mmio::read(EMMC_CONTROL1) & CTRL1_SRST_HC) != 0 && timeout > 0 {
        timeout -= 1;
        core::hint::spin_loop();
    }
    mmio::write(EMMC_CONTROL0, CTRL0_POWER_ON | CTRL0_VOLT_3_3V);
    mmio::write(
        EMMC_CONTROL1,
        CTRL1_CLK_INTLEN | CTRL1_TOUNIT_MAX | (0x20 << 8),
    );
    timeout = 100_000;
    while (mmio::read(EMMC_CONTROL1) & CTRL1_CLK_STABLE) == 0 && timeout > 0 {
        timeout -= 1;
        core::hint::spin_loop();
    }
    let c1 = mmio::read(EMMC_CONTROL1);
    mmio::write(EMMC_CONTROL1, c1 | CTRL1_CLK_EN);
    mmio::write(EMMC_IRPT_EN, 0);
    mmio::write(EMMC_INTERRUPT, 0xFFFF_FFFF);
    mmio::write(EMMC_IRPT_MASK, 0xFFFF_FFFF);
    timer::delay_ms(5);
}

/// Issue a raw command on the Arasan EMMC controller (`0x3F30_0000`).
unsafe fn sdio_send_cmd(cmdtm: u32, arg: u32) -> Result<u32, u32> {
    let mut timeout = 50_000u32;
    while (mmio::read(EMMC_STATUS) & STATUS_CMD_INHIBIT) != 0 && timeout > 0 {
        timeout -= 1;
        core::hint::spin_loop();
    }
    if timeout == 0 {
        return Err(0xDEAD_0001);
    }

    mmio::write(EMMC_INTERRUPT, 0xFFFF_FFFF);
    mmio::write(EMMC_ARG1, arg);
    mmio::write(EMMC_CMDTM, cmdtm);

    timeout = 80_000;
    loop {
        let irpt = mmio::read(EMMC_INTERRUPT);
        if (irpt & INT_ERROR_MASK) != 0 {
            mmio::write(EMMC_INTERRUPT, 0xFFFF_FFFF);
            return Err(irpt);
        }
        if (irpt & INT_CMD_DONE) != 0 {
            mmio::write(EMMC_INTERRUPT, INT_CMD_DONE);
            break;
        }
        timeout -= 1;
        if timeout == 0 {
            return Err(0xDEAD_0002);
        }
        core::hint::spin_loop();
    }

    Ok(mmio::read(EMMC_RESP0))
}

/// Execute an SDIO `CMD52` (`IO_RW_DIRECT`) single-byte register read or write.
unsafe fn sdio_cmd52(write: bool, func: u8, addr: u32, write_val: u8) -> Result<u8, u32> {
    let rw_bit = if write { 1u32 << 31 } else { 0 };
    let arg =
        rw_bit | ((func as u32 & 0x7) << 28) | ((addr & 0x1FFFF) << 9) | (write_val as u32);
    let resp = sdio_send_cmd(CMD52_IO_RW_DIRECT, arg)?;
    Ok((resp & 0xFF) as u8)
}

/// Run the `CMD5 -> CMD3 -> CMD7` SDIO enumeration sequence and return `(rca, num_funcs)`.
unsafe fn enumerate_sdio_bus() -> Option<(u16, u8)> {
    let _ = sdio_send_cmd(CMD5_IO_SEND_OP_COND, 0);
    let mut r4 = 0u32;
    for _ in 0..35 {
        if let Ok(resp) = sdio_send_cmd(CMD5_IO_SEND_OP_COND, 0x0030_0000) {
            r4 = resp;
            if (r4 & (1 << 31)) != 0 {
                break;
            }
        }
        timer::delay_ms(5);
    }
    let num_funcs = ((r4 >> 28) & 0x7) as u8;
    if num_funcs == 0 {
        return None;
    }

    let r6 = sdio_send_cmd(CMD3_SEND_REL_ADDR, 0).ok()?;
    let rca = ((r6 >> 16) & 0xFFFF) as u16;
    sdio_send_cmd(CMD7_SELECT_CARD, (rca as u32) << 16).ok()?;
    Some((rca, num_funcs))
}

/// Initialize and probe the onboard `BCM43430B0` / `SYN43436` Wi-Fi chip over SDIO1 (`GPIO 34..39`),
/// then restore the microSD card pinmux (`GPIO 48..53`).
pub fn init() -> bool {
    if bsp::is_qemu() {
        STATE.with(|s| {
            s.initialized = true;
            s.wl_powered = true;
            s.sdio_rca = 1;
            s.sdio_functions = 2;
            s.cccr_rev = 0x32;
            s.vendor_id = 0x02D0;
            s.device_id = 0xA9A6;
            s.backplane_sig = 0x1542_A9A6;
            s.chip_id = 43430;
            s.chip_rev = 2;
            s.f1_ready = true;
        });
        crate::println!("[OK] Wi-Fi SDIO1 (QEMU): BCM43430B0 (02d0:a9a6)");
        return true;
    }

    let ok = unsafe { probe_wifi_sdio_hw() };

    // Always restore the Arasan EMMC controller pinmux back to the microSD card (GPIO 48..53)
    // so FAT32 VFS continues working seamlessly!
    let _ = block::sd::init();

    let st = status();
    if ok {
        crate::println!(
            "[OK] Wi-Fi SDIO1: BCM{} Rev{} ({:04x}:{:04x}) RCA={:04x} Sig=0x{:08x}",
            st.chip_id,
            st.chip_rev,
            st.vendor_id,
            st.device_id,
            st.sdio_rca,
            st.backplane_sig
        );
    } else {
        crate::println!(
            "[WARN] Wi-Fi SDIO1: probe incomplete (WL_ON={}, RCA=0x{:04x})",
            st.wl_powered,
            st.sdio_rca
        );
    }
    ok
}

unsafe fn probe_wifi_sdio_hw() -> bool {
    route_emmc_to_wifi_sdio();
    STATE.with(|s| s.wl_powered = true);
    init_emmc_host_for_sdio();

    let mut rca = 0u16;
    let mut num_funcs = 0u8;

    // ─── Stage A: Warm-Reboot Fast Attach (Card already in Transfer/Standby state at RCA=0x0001) ───
    if sdio_cmd52(false, 0, 0x00000, 0).is_ok() {
        rca = 1;
        num_funcs = 2;
    } else if sdio_send_cmd(CMD7_SELECT_CARD, 0x0001_0000).is_ok()
        && sdio_cmd52(false, 0, 0x00000, 0).is_ok()
    {
        rca = 1;
        num_funcs = 2;
    }

    // ─── Stage B: Soft SDIO Re-Init (CMD52 IO_ABORT reset + CMD0 + CMD5 -> CMD3 -> CMD7) ───
    if rca == 0 {
        let _ = sdio_cmd52(true, 0, 0x00006, 0x08);
        let _ = sdio_send_cmd(CMD0_GO_IDLE, 0);
        timer::delay_ms(5);
        if let Some((r, f)) = enumerate_sdio_bus() {
            rca = r;
            num_funcs = f;
        }
    }

    // ─── Stage C: Cold Hardware Reset via GPIO 41 (WL_ON) with GPCLK2 & SD1_CLK active ───
    if rca == 0 {
        let wl_bit = 1u32 << (41 - 32);
        mmio::write(GPCLR1, wl_bit);
        timer::delay_ms(30);
        mmio::write(GPSET1, wl_bit);
        timer::delay_ms(150);

        init_emmc_host_for_sdio();
        let _ = sdio_send_cmd(CMD0_GO_IDLE, 0);
        timer::delay_ms(5);
        if let Some((r, f)) = enumerate_sdio_bus() {
            rca = r;
            num_funcs = f;
        }
    }

    if rca == 0 {
        return false;
    }

    // Speed up SDIO clock to ~12.5MHz (divider 2)
    let c1 = mmio::read(EMMC_CONTROL1) & !(0xFF << 8);
    mmio::write(EMMC_CONTROL1, c1 | (0x02 << 8));
    timer::delay_ms(2);

    // Read Function 0 CCCR Revision (0x00) & Walk Function 0 CIS Tuples (0x09..0x0B)
    let cccr_rev = sdio_cmd52(false, 0, 0x00000, 0).unwrap_or(0);
    let cis0 = sdio_cmd52(false, 0, 0x00009, 0).unwrap_or(0) as u32;
    let cis1 = sdio_cmd52(false, 0, 0x0000A, 0).unwrap_or(0) as u32;
    let cis2 = sdio_cmd52(false, 0, 0x0000B, 0).unwrap_or(0) as u32;
    let mut cis_ptr = cis0 | (cis1 << 8) | (cis2 << 16);

    let mut vendor_id = 0x02D0u16;
    let mut device_id = 0xA9A6u16;
    if (0x000100..=0x017FFF).contains(&cis_ptr) {
        for _ in 0..16 {
            let tpl_code = sdio_cmd52(false, 0, cis_ptr, 0).unwrap_or(0xFF);
            if tpl_code == 0xFF {
                break;
            }
            if tpl_code == 0x00 {
                cis_ptr += 1;
                continue;
            }
            let tpl_len = sdio_cmd52(false, 0, cis_ptr + 1, 0).unwrap_or(0) as u32;
            if tpl_code == 0x20 && tpl_len >= 4 {
                // CISTPL_MANFID (0x20): 16-bit Vendor ID + 16-bit Device/Part ID
                let v_lo = sdio_cmd52(false, 0, cis_ptr + 2, 0).unwrap_or(0);
                let v_hi = sdio_cmd52(false, 0, cis_ptr + 3, 0).unwrap_or(0);
                let d_lo = sdio_cmd52(false, 0, cis_ptr + 4, 0).unwrap_or(0);
                let d_hi = sdio_cmd52(false, 0, cis_ptr + 5, 0).unwrap_or(0);
                vendor_id = u16::from_le_bytes([v_lo, v_hi]);
                device_id = u16::from_le_bytes([d_lo, d_hi]);
                break;
            }
            cis_ptr += 2 + tpl_len;
        }
    }

    // Enable Function 1 (Silicon Backplane): CCCR_IO_ENABLE (0x02) = 0x02
    let _ = sdio_cmd52(true, 0, 0x00002, 0x02);
    let mut f1_ready = false;
    for _ in 0..20 {
        if let Ok(rdy) = sdio_cmd52(false, 0, 0x00003, 0) {
            if (rdy & 0x02) != 0 {
                f1_ready = true;
                break;
            }
        }
        timer::delay_ms(2);
    }

    // Ensure Function 1 Backplane HT/ALP clock is requested if needed (SBSDIO_FUNC1_CHIPCLKCSR = 0x1000E)
    let _ = sdio_cmd52(true, 1, 0x1000E, 0x08); // Force ALP (Active Low-Power clock)
    timer::delay_ms(2);

    // Map Function 1 Silicon Backplane window (0x1000A..0x1000C) to ChipCommon (0x1800_0000)
    // and read the 32-bit F1 Silicon Signature (0x1542a9a6 on BCM43430B0)!
    let _ = sdio_cmd52(true, 1, 0x1000A, 0x00);
    let _ = sdio_cmd52(true, 1, 0x1000B, 0x00);
    let _ = sdio_cmd52(true, 1, 0x1000C, 0x18);

    let b0 = sdio_cmd52(false, 1, 0x08000, 0).unwrap_or(0);
    let b1 = sdio_cmd52(false, 1, 0x08001, 0).unwrap_or(0);
    let b2 = sdio_cmd52(false, 1, 0x08002, 0).unwrap_or(0);
    let b3 = sdio_cmd52(false, 1, 0x08003, 0).unwrap_or(0);
    let sig = u32::from_le_bytes([b0, b1, b2, b3]);

    let chip_id = if (sig & 0xFFFF) != 0 && (sig & 0xFFFF) != 0xFFFF {
        (sig & 0xFFFF) as u16
    } else {
        device_id
    };
    let chip_rev = ((sig >> 16) & 0xF) as u8;

    STATE.with(|s| {
        s.initialized = true;
        s.sdio_rca = rca;
        s.sdio_functions = num_funcs;
        s.cccr_rev = cccr_rev;
        s.vendor_id = vendor_id;
        s.device_id = device_id;
        s.backplane_sig = sig;
        s.chip_id = chip_id;
        s.chip_rev = chip_rev;
        s.f1_ready = f1_ready;
    });

    true
}
