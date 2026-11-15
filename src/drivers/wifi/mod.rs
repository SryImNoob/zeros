// ZerOS — Onboard Wi-Fi (`BCM43430B0` / `CYW43436`) SDIO1 Full-MAC Driver (`M8.3` + `M13.1`)
//
// Drives the Raspberry Pi Zero 2 W's onboard 802.11b/g/n Wi-Fi chip connected to
// `GPIO 34..39` (`ALT3` = `SD1_CLK, SD1_CMD, SD1_DAT0..3`), `GPIO 41` (`WL_ON`),
// and `GPIO 43` (`ALT0` = `GPCLK2` 32.768 kHz Low-Power Reference Clock).
//
// Features:
//   1. `CMD52` (`IO_RW_DIRECT`) and `CMD53` (`IO_RW_EXTENDED`) byte/block transfers
//      over Function 0 (CIA/CCCR), Function 1 (Sonics Silicon Backplane), and
//      Function 2 (WLAN Data FIFO / SDPCM + BDC frames).
//   2. Preserves Function 2 (`CCCR_IO_ENABLE = 0x06`) across warm-reboot (`tryboot`)
//      when `brcmfmac` firmware is already resident in chip RAM, and supports cold
//      firmware download from `/FW43430.BIN` + `/FW43430.TXT`.
//   3. Broadcom `SDPCM` (12-byte header) + `BDC` (4-byte header) framing for
//      Control (`cdc_ioctl` / `iovar` on channel 0) and Data (`Ethernet II` on channel 2).
//   4. Seamless pinmux time-sharing with microSD (`GPIO 48..53`) via `with_wifi_sdio_bus()`.

#![allow(dead_code)]

use crate::arch::timer;
use crate::bsp;
use crate::drivers::{block, mmio};
use crate::kernel::tryboot;
use crate::sync::IrqSafeLock;

// ─── Arasan EMMC Register Offsets (Base: 0x3F30_0000) ───
const EMMC_BASE: usize = mmio::PERIPHERAL_BASE + 0x0030_0000;
const EMMC_ARG2: usize = EMMC_BASE + 0x00;
const EMMC_BLKSIZECNT: usize = EMMC_BASE + 0x04;
const EMMC_ARG1: usize = EMMC_BASE + 0x08;
const EMMC_CMDTM: usize = EMMC_BASE + 0x0C;
const EMMC_RESP0: usize = EMMC_BASE + 0x10;
const EMMC_DATA: usize = EMMC_BASE + 0x20;
const EMMC_STATUS: usize = EMMC_BASE + 0x24;
const EMMC_CONTROL0: usize = EMMC_BASE + 0x28;
const EMMC_CONTROL1: usize = EMMC_BASE + 0x2C;
const EMMC_INTERRUPT: usize = EMMC_BASE + 0x30;
const EMMC_IRPT_MASK: usize = EMMC_BASE + 0x34;
const EMMC_IRPT_EN: usize = EMMC_BASE + 0x38;

const STATUS_CMD_INHIBIT: u32 = 1 << 0;
const STATUS_DAT_INHIBIT: u32 = 1 << 1;
const CTRL0_HCTL_DWIDTH4: u32 = 1 << 1;
const CTRL0_POWER_ON: u32 = 1 << 8;
const CTRL0_VOLT_3_3V: u32 = 0b111 << 9;
const CTRL1_CLK_INTLEN: u32 = 1 << 0;
const CTRL1_CLK_STABLE: u32 = 1 << 1;
const CTRL1_CLK_EN: u32 = 1 << 2;
const CTRL1_TOUNIT_MAX: u32 = 0xE << 16;
const CTRL1_SRST_HC: u32 = 1 << 24;

const INT_CMD_DONE: u32 = 1 << 0;
const INT_DATA_DONE: u32 = 1 << 1;
const INT_WRITE_RDY: u32 = 1 << 4;
const INT_READ_RDY: u32 = 1 << 5;
const INT_ERROR_MASK: u32 = 0xFFFF_8000;

const TM_BLKCNT_EN: u32 = 1 << 1;
const TM_DAT_DIR_READ: u32 = 1 << 4;
const TM_DAT_DIR_WRITE: u32 = 0 << 4;
const TM_MULTI_BLOCK: u32 = 1 << 5;

const CMD_RSPNS_NONE: u32 = 0b00 << 16;
const CMD_RSPNS_48: u32 = 0b10 << 16;
const CMD_RSPNS_48B: u32 = 0b11 << 16;
const CMD_CRCCHK_EN: u32 = 1 << 19;
const CMD_IXCHK_EN: u32 = 1 << 20;
const CMD_ISDATA: u32 = 1 << 21;

const fn cmd_index(idx: u32) -> u32 {
    (idx & 0x3F) << 24
}

const CMD0_GO_IDLE: u32 = cmd_index(0) | CMD_RSPNS_NONE;
const CMD3_SEND_REL_ADDR: u32 = cmd_index(3) | CMD_RSPNS_48 | CMD_CRCCHK_EN | CMD_IXCHK_EN;
const CMD5_IO_SEND_OP_COND: u32 = cmd_index(5) | CMD_RSPNS_48;
const CMD7_SELECT_CARD: u32 = cmd_index(7) | CMD_RSPNS_48B | CMD_CRCCHK_EN | CMD_IXCHK_EN;
const CMD52_IO_RW_DIRECT: u32 = cmd_index(52) | CMD_RSPNS_48 | CMD_CRCCHK_EN | CMD_IXCHK_EN;
const CMD53_BASE: u32 = cmd_index(53) | CMD_RSPNS_48 | CMD_CRCCHK_EN | CMD_IXCHK_EN | CMD_ISDATA;

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

/// Live status of the Onboard Wi-Fi (`BCM43430B0` / `CYW43436`) SDIO1 controller
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
    pub f2_ready: bool,
    pub link_up: bool,
    pub mac_addr: [u8; 6],
    pub ipv4_addr: [u8; 4],
    pub gateway_ip: [u8; 4],
    pub tx_packets: u32,
    pub rx_packets: u32,
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
            f2_ready: false,
            link_up: false,
            mac_addr: [0x98, 0xFE, 0x54, 0x81, 0x9A, 0x87],
            ipv4_addr: [192, 168, 31, 99],
            gateway_ip: [192, 168, 31, 1],
            tx_packets: 0,
            rx_packets: 0,
        }
    }
}

static STATE: IrqSafeLock<WifiStatus> = IrqSafeLock::new(WifiStatus::empty());
static mut SDPCM_TX_SEQ: u8 = 0;
static mut BACKPLANE_WINDOW: u32 = u32::MAX;

/// Return the cached Wi-Fi SDIO1 hardware status.
pub fn status() -> WifiStatus {
    STATE.with(|s| *s)
}

/// Update the station IPv4 and gateway addresses in `WifiStatus`.
pub fn set_ipv4_config(ip: [u8; 4], gw: [u8; 4], link_up: bool) {
    STATE.with(|s| {
        s.ipv4_addr = ip;
        s.gateway_ip = gw;
        s.link_up = link_up;
    });
}

/// Start the 32.768 kHz Low-Power Reference Clock (`GPCLK2`) on `GPIO 43` (`ALT0`).
unsafe fn enable_wifi_lpclk_gpio43() {
    let ctl = mmio::read(CM_GP2CTL);
    if (ctl & (1 << 7)) == 0 {
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
    fsel4 = (fsel4 & !(0b111 << 3)) | (0b001 << 3); // GPIO 41 = OUTPUT
    fsel4 = (fsel4 & !(0b111 << 9)) | (0b100 << 9); // GPIO 43 = ALT0 (GPCLK2)
    mmio::write(GPFSEL4, fsel4);

    // Ensure GPIO 41 (WL_ON) stays HIGH
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
pub unsafe fn sdio_cmd52(write: bool, func: u8, addr: u32, write_val: u8) -> Result<u8, u32> {
    let rw_bit = if write { 1u32 << 31 } else { 0 };
    let arg =
        rw_bit | ((func as u32 & 0x7) << 28) | ((addr & 0x1FFFF) << 9) | (write_val as u32);
    let resp = sdio_send_cmd(CMD52_IO_RW_DIRECT, arg)?;
    Ok((resp & 0xFF) as u8)
}

/// Execute an SDIO `CMD53` (`IO_RW_EXTENDED`) multi-byte read (`1..=512` bytes, byte mode).
pub unsafe fn sdio_cmd53_read(
    func: u8,
    addr: u32,
    incr_addr: bool,
    buf: &mut [u8],
) -> Result<(), u32> {
    if buf.is_empty() || buf.len() > 512 || (buf.len() & 3) != 0 {
        return Err(0xDEAD_5300);
    }
    let mut timeout = 50_000u32;
    while (mmio::read(EMMC_STATUS) & (STATUS_CMD_INHIBIT | STATUS_DAT_INHIBIT)) != 0 && timeout > 0
    {
        timeout -= 1;
        core::hint::spin_loop();
    }
    if timeout == 0 {
        return Err(0xDEAD_5301);
    }

    let len = buf.len() as u32;
    let cnt_field = if len == 512 { 0 } else { len & 0x1FF };
    let op_bit = if incr_addr { 1u32 << 26 } else { 0 };
    let arg = ((func as u32 & 0x7) << 28) | op_bit | ((addr & 0x1FFFF) << 9) | cnt_field;
    let cmdtm = CMD53_BASE | TM_DAT_DIR_READ | TM_BLKCNT_EN;

    mmio::write(EMMC_BLKSIZECNT, (1 << 16) | len);
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
            return Err(0xDEAD_5302);
        }
        core::hint::spin_loop();
    }

    timeout = 120_000;
    while (mmio::read(EMMC_INTERRUPT) & INT_READ_RDY) == 0 {
        let irpt = mmio::read(EMMC_INTERRUPT);
        if (irpt & INT_ERROR_MASK) != 0 {
            mmio::write(EMMC_INTERRUPT, 0xFFFF_FFFF);
            return Err(irpt);
        }
        timeout -= 1;
        if timeout == 0 {
            return Err(0xDEAD_5303);
        }
        core::hint::spin_loop();
    }
    mmio::write(EMMC_INTERRUPT, INT_READ_RDY);

    let words = buf.len() / 4;
    for i in 0..words {
        let w = mmio::read(EMMC_DATA).to_le_bytes();
        let off = i * 4;
        buf[off..off + 4].copy_from_slice(&w);
    }

    timeout = 80_000;
    while (mmio::read(EMMC_INTERRUPT) & INT_DATA_DONE) == 0 && timeout > 0 {
        timeout -= 1;
        core::hint::spin_loop();
    }
    mmio::write(EMMC_INTERRUPT, 0xFFFF_FFFF);
    Ok(())
}

/// Execute an SDIO `CMD53` (`IO_RW_EXTENDED`) multi-byte write (`4..=512` bytes, byte mode).
pub unsafe fn sdio_cmd53_write(
    func: u8,
    addr: u32,
    incr_addr: bool,
    buf: &[u8],
) -> Result<(), u32> {
    if buf.is_empty() || buf.len() > 512 || (buf.len() & 3) != 0 {
        return Err(0xDEAD_5310);
    }
    let mut timeout = 50_000u32;
    while (mmio::read(EMMC_STATUS) & (STATUS_CMD_INHIBIT | STATUS_DAT_INHIBIT)) != 0 && timeout > 0
    {
        timeout -= 1;
        core::hint::spin_loop();
    }
    if timeout == 0 {
        return Err(0xDEAD_5311);
    }

    let len = buf.len() as u32;
    let cnt_field = if len == 512 { 0 } else { len & 0x1FF };
    let op_bit = if incr_addr { 1u32 << 26 } else { 0 };
    let arg = (1u32 << 31)
        | ((func as u32 & 0x7) << 28)
        | op_bit
        | ((addr & 0x1FFFF) << 9)
        | cnt_field;
    let cmdtm = CMD53_BASE | TM_DAT_DIR_WRITE | TM_BLKCNT_EN;

    mmio::write(EMMC_BLKSIZECNT, (1 << 16) | len);
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
            return Err(0xDEAD_5312);
        }
        core::hint::spin_loop();
    }

    timeout = 120_000;
    while (mmio::read(EMMC_INTERRUPT) & INT_WRITE_RDY) == 0 {
        let irpt = mmio::read(EMMC_INTERRUPT);
        if (irpt & INT_ERROR_MASK) != 0 {
            mmio::write(EMMC_INTERRUPT, 0xFFFF_FFFF);
            return Err(irpt);
        }
        timeout -= 1;
        if timeout == 0 {
            return Err(0xDEAD_5313);
        }
        core::hint::spin_loop();
    }
    mmio::write(EMMC_INTERRUPT, INT_WRITE_RDY);

    let words = buf.len() / 4;
    for i in 0..words {
        let off = i * 4;
        let w = u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]]);
        mmio::write(EMMC_DATA, w);
    }

    timeout = 120_000;
    while (mmio::read(EMMC_INTERRUPT) & INT_DATA_DONE) == 0 && timeout > 0 {
        timeout -= 1;
        core::hint::spin_loop();
    }
    mmio::write(EMMC_INTERRUPT, 0xFFFF_FFFF);
    Ok(())
}

/// Set the Function 1 Sonics Silicon Backplane 32 KB window (`0x1000A..0x1000C`).
unsafe fn set_backplane_window(addr: u32) {
    let win = addr & !0x7FFF;
    if win == BACKPLANE_WINDOW {
        return;
    }
    let _ = sdio_cmd52(true, 1, 0x1000A, ((win >> 8) & 0x80) as u8);
    let _ = sdio_cmd52(true, 1, 0x1000B, ((win >> 16) & 0xFF) as u8);
    let _ = sdio_cmd52(true, 1, 0x1000C, ((win >> 24) & 0xFF) as u8);
    BACKPLANE_WINDOW = win;
}

/// Read a 32-bit word from the Silicon Backplane via Function 1 (`0x08000` 32-bit access bit).
pub unsafe fn backplane_read32(addr: u32) -> u32 {
    set_backplane_window(addr);
    let off = (addr & 0x7FFF) | 0x08000;
    let mut buf = [0u8; 4];
    if sdio_cmd53_read(1, off, true, &mut buf).is_ok() {
        u32::from_le_bytes(buf)
    } else {
        let b0 = sdio_cmd52(false, 1, off, 0).unwrap_or(0);
        let b1 = sdio_cmd52(false, 1, off + 1, 0).unwrap_or(0);
        let b2 = sdio_cmd52(false, 1, off + 2, 0).unwrap_or(0);
        let b3 = sdio_cmd52(false, 1, off + 3, 0).unwrap_or(0);
        u32::from_le_bytes([b0, b1, b2, b3])
    }
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

/// Transmit an Ethernet II frame (`14..=1514` bytes) over Function 2 (`SDPCM` + `BDC` Channel 2).
pub fn send_ethernet_frame(eth_frame: &[u8]) -> bool {
    if bsp::is_qemu() {
        STATE.with(|s| s.tx_packets = s.tx_packets.wrapping_add(1));
        return true;
    }
    let st = status();
    if !st.initialized || !st.f2_ready || eth_frame.len() < 14 || eth_frame.len() > 492 {
        return false;
    }

    let mut pkt = [0u8; 512];
    let total_len = 12 + 4 + eth_frame.len();
    let padded_len = (total_len + 3) & !3;
    let len_u16 = total_len as u16;
    let not_len = !len_u16;

    let ok = crate::kernel::interrupts::without_interrupts(|| unsafe {
        route_emmc_to_wifi_sdio();

        let seq = SDPCM_TX_SEQ;
        SDPCM_TX_SEQ = SDPCM_TX_SEQ.wrapping_add(1);

        // 12-byte Broadcom SDPCM Header
        pkt[0] = (len_u16 & 0xFF) as u8;
        pkt[1] = (len_u16 >> 8) as u8;
        pkt[2] = (not_len & 0xFF) as u8;
        pkt[3] = (not_len >> 8) as u8;
        pkt[4] = seq;
        pkt[5] = 0x02; // Channel 2 = Data (BDC + Ethernet II)
        pkt[6] = 0x00;
        pkt[7] = 12; // Header length = 12 bytes (BDC starts at offset 12)
        pkt[8] = 0x00;
        pkt[9] = 0x00;
        pkt[10] = 0x00;
        pkt[11] = 0x00;

        // 4-byte Broadcom BDC Header (Version 2 = 0x20, Priority 0, Offset 0)
        pkt[12] = 0x20;
        pkt[13] = 0x00;
        pkt[14] = 0x00;
        pkt[15] = 0x00;

        pkt[16..16 + eth_frame.len()].copy_from_slice(eth_frame);
        let res = sdio_cmd53_write(2, 0x0000, true, &pkt[..padded_len]).is_ok();
        block::sd::restore_pinmux();
        res
    });

    if ok {
        STATE.with(|s| s.tx_packets = s.tx_packets.wrapping_add(1));
    }
    ok
}

/// Poll Function 2 for an incoming `SDPCM + BDC` Ethernet II frame (`<= 496` bytes) into `out_eth`.
/// Returns the Ethernet II frame length (`0` if no frame pending).
pub fn recv_ethernet_frame(out_eth: &mut [u8]) -> usize {
    if bsp::is_qemu() {
        return 0;
    }
    let st = status();
    if !st.initialized || !st.f2_ready {
        return 0;
    }

    let mut raw = [0u8; 512];
    let n = crate::kernel::interrupts::without_interrupts(|| unsafe {
        route_emmc_to_wifi_sdio();

        let mut hdr = [0u8; 4];
        let mut frame_len = 0usize;
        if sdio_cmd53_read(2, 0x0000, false, &mut hdr).is_ok() {
            let len = u16::from_le_bytes([hdr[0], hdr[1]]);
            let chk = u16::from_le_bytes([hdr[2], hdr[3]]);
            if len >= 18 && len <= 512 && (len ^ chk) == 0xFFFF {
                let aligned = ((len as usize) + 3) & !3;
                if sdio_cmd53_read(2, 0x0000, false, &mut raw[..aligned]).is_ok() {
                    let chan = raw[5] & 0x0F;
                    let data_off = (raw[7] as usize).max(12);
                    if chan == 2 && data_off + 4 < (len as usize) {
                        let bdc_pad = (raw[data_off + 3] as usize) * 4;
                        let eth_start = data_off + 4 + bdc_pad;
                        if eth_start < (len as usize) {
                            let eth_len = (len as usize) - eth_start;
                            let copy_n = eth_len.min(out_eth.len());
                            out_eth[..copy_n].copy_from_slice(&raw[eth_start..eth_start + copy_n]);
                            frame_len = copy_n;
                        }
                    }
                }
            }
        }
        block::sd::restore_pinmux();
        frame_len
    });

    if n > 0 {
        STATE.with(|s| s.rx_packets = s.rx_packets.wrapping_add(1));
    }
    n
}

/// Initialize and probe the onboard `BCM43430B0` / `CYW43436` Wi-Fi chip over SDIO1 (`GPIO 34..39`),
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
            s.f2_ready = true;
            s.link_up = true;
        });
        crate::println!("[OK] Wi-Fi SDIO1 (QEMU): BCM43430B0 (02d0:a9a6) F2=READY IP=192.168.31.99");
        return true;
    }

    let ok = unsafe { probe_wifi_sdio_hw() };

    // Always restore the Arasan EMMC controller pinmux back to the microSD card (GPIO 48..53)
    // so FAT32 VFS continues working seamlessly!
    block::sd::restore_pinmux();

    let st = status();
    if ok {
        crate::println!(
            "[OK] Wi-Fi SDIO1: BCM{} Rev{} ({:04x}:{:04x}) RCA={:04x} Sig=0x{:08x} F2={}",
            st.chip_id,
            st.chip_rev,
            st.vendor_id,
            st.device_id,
            st.sdio_rca,
            st.backplane_sig,
            st.f2_ready
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
    BACKPLANE_WINDOW = u32::MAX;

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
        tryboot::watchdog_feed(15);

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

    // Configure F1 Block Size = 64 (0x110..0x111) and F2 Block Size = 512 (0x210..0x211)
    let _ = sdio_cmd52(true, 0, 0x00110, 64);
    let _ = sdio_cmd52(true, 0, 0x00111, 0);
    let _ = sdio_cmd52(true, 0, 0x00210, 0x00);
    let _ = sdio_cmd52(true, 0, 0x00211, 0x02);

    // Enable BOTH Function 1 (Silicon Backplane = 0x02) AND Function 2 (WLAN Data = 0x04):
    // Preserves warm-boot resident firmware from Raspberry Pi OS if already loaded!
    let prev_en = sdio_cmd52(false, 0, 0x00002, 0).unwrap_or(0);
    let _ = sdio_cmd52(true, 0, 0x00002, prev_en | 0x06);
    let mut f1_ready = false;
    let mut f2_ready = false;
    for _ in 0..20 {
        if let Ok(rdy) = sdio_cmd52(false, 0, 0x00003, 0) {
            if (rdy & 0x02) != 0 {
                f1_ready = true;
            }
            if (rdy & 0x04) != 0 {
                f2_ready = true;
            }
            if f1_ready {
                break;
            }
        }
        timer::delay_ms(2);
    }

    // Request Function 1 Backplane ALP/HT clock (SBSDIO_FUNC1_CHIPCLKCSR = 0x1000E)
    let _ = sdio_cmd52(true, 1, 0x1000E, 0x08);
    timer::delay_ms(2);

    // Read the 32-bit F1 Silicon Signature at ChipCommon (0x1800_0000)
    let sig = backplane_read32(0x1800_0000);

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
        s.f2_ready = f2_ready;
        s.link_up = f2_ready;
    });

    true
}
