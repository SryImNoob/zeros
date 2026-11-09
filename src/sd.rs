// ZerOS — BCM2710 Arasan EMMC (SD Card) Driver (M6.2)
//
// Drives the Arasan SDHCI/EMMC controller at 0x3F30_0000.
// Supports SD / SDHC cards in both QEMU (-M raspi3b -drive file=...,if=sd)
// and real Raspberry Pi Zero 2 W hardware.
//
// Provides 512-byte sector read and write operations.

#![allow(dead_code)]

use crate::mmio;

// ─── EMMC Register Offsets (Base: 0x3F30_0000) ───
const EMMC_BASE:       usize = mmio::PERIPHERAL_BASE + 0x0030_0000;
const EMMC_ARG2:       usize = EMMC_BASE + 0x00;
const EMMC_BLKSIZECNT: usize = EMMC_BASE + 0x04;
const EMMC_ARG1:       usize = EMMC_BASE + 0x08;
const EMMC_CMDTM:      usize = EMMC_BASE + 0x0C;
const EMMC_RESP0:      usize = EMMC_BASE + 0x10;
const EMMC_RESP1:      usize = EMMC_BASE + 0x14;
const EMMC_RESP2:      usize = EMMC_BASE + 0x18;
const EMMC_RESP3:      usize = EMMC_BASE + 0x1C;
const EMMC_DATA:       usize = EMMC_BASE + 0x20;
const EMMC_STATUS:     usize = EMMC_BASE + 0x24;
const EMMC_CONTROL0:   usize = EMMC_BASE + 0x28;
const EMMC_CONTROL1:   usize = EMMC_BASE + 0x2C;
const EMMC_INTERRUPT:  usize = EMMC_BASE + 0x30;
const EMMC_IRPT_MASK:  usize = EMMC_BASE + 0x34;
const EMMC_IRPT_EN:    usize = EMMC_BASE + 0x38;

// ─── STATUS register bits ───
const STATUS_CMD_INHIBIT: u32 = 1 << 0;
const STATUS_DAT_INHIBIT: u32 = 1 << 1;

// ─── CONTROL0 register bits (SDHCI Host Control / Power Control) ───
const CTRL0_POWER_ON:  u32 = 1 << 8;
const CTRL0_VOLT_3_3V: u32 = 0b111 << 9;

// ─── CONTROL1 register bits (Clock / Timeout / Reset) ───
const CTRL1_CLK_INTLEN: u32 = 1 << 0;
const CTRL1_CLK_STABLE: u32 = 1 << 1;
const CTRL1_CLK_EN:     u32 = 1 << 2;
const CTRL1_TOUNIT_MAX: u32 = 0xE << 16;
const CTRL1_SRST_HC:    u32 = 1 << 24;

// ─── INTERRUPT register bits ───
const INT_CMD_DONE:  u32 = 1 << 0;
const INT_DATA_DONE: u32 = 1 << 1;
const INT_WRITE_RDY: u32 = 1 << 4;
const INT_READ_RDY:  u32 = 1 << 5;
const INT_ERROR_MASK: u32 = 0xFFFF_8000;

// ─── CMDTM flags ───
const TM_BLKCNT_EN:    u32 = 1 << 1;
const TM_DAT_DIR_READ: u32 = 1 << 4;
const TM_DAT_DIR_WRITE: u32 = 0 << 4;
const CMD_RSPNS_NONE:  u32 = 0b00 << 16;
const CMD_RSPNS_136:   u32 = 0b01 << 16;
const CMD_RSPNS_48:    u32 = 0b10 << 16;
const CMD_RSPNS_48B:   u32 = 0b11 << 16;
const CMD_CRCCHK_EN:   u32 = 1 << 19;
const CMD_IXCHK_EN:    u32 = 1 << 20;
const CMD_ISDATA:      u32 = 1 << 21;

const fn cmd_index(idx: u32) -> u32 {
    (idx & 0x3F) << 24
}

// ─── Predefined SD Commands ───
const CMD0_GO_IDLE:      u32 = cmd_index(0)  | CMD_RSPNS_NONE;
const CMD2_ALL_SEND_CID: u32 = cmd_index(2)  | CMD_RSPNS_136 | CMD_CRCCHK_EN;
const CMD3_SEND_REL_ADDR: u32 = cmd_index(3) | CMD_RSPNS_48  | CMD_CRCCHK_EN | CMD_IXCHK_EN;
const CMD7_SELECT_CARD:  u32 = cmd_index(7)  | CMD_RSPNS_48B | CMD_CRCCHK_EN | CMD_IXCHK_EN;
const CMD8_SEND_IF_COND: u32 = cmd_index(8)  | CMD_RSPNS_48  | CMD_CRCCHK_EN | CMD_IXCHK_EN;
const CMD16_SET_BLK_LEN: u32 = cmd_index(16) | CMD_RSPNS_48  | CMD_CRCCHK_EN | CMD_IXCHK_EN;
const CMD17_READ_SINGLE: u32 = cmd_index(17) | CMD_RSPNS_48  | CMD_CRCCHK_EN | CMD_IXCHK_EN
                             | CMD_ISDATA | TM_DAT_DIR_READ | TM_BLKCNT_EN;
const CMD24_WRITE_SINGLE: u32 = cmd_index(24) | CMD_RSPNS_48 | CMD_CRCCHK_EN | CMD_IXCHK_EN
                              | CMD_ISDATA | TM_DAT_DIR_WRITE | TM_BLKCNT_EN;
const CMD55_APP_CMD:     u32 = cmd_index(55) | CMD_RSPNS_48  | CMD_CRCCHK_EN | CMD_IXCHK_EN;
const ACMD41_SEND_OP_COND: u32 = cmd_index(41) | CMD_RSPNS_48;

/// Sector size in bytes
pub const SECTOR_SIZE: usize = 512;

/// Global SD card state
struct SdState {
    initialized: bool,
    rca: u32,
    is_sdhc: bool,
}

static mut SD_STATE: SdState = SdState {
    initialized: false,
    rca: 0,
    is_sdhc: false,
};

/// Route the Arasan EMMC controller (0x3F30_0000) to the physical microSD slot (GPIO 48..53)
/// and disconnect it from the Wi-Fi SDIO chip (GPIO 34..39) on real RPi Zero 2 W hardware.
unsafe fn route_emmc_to_sdcard() {
    const GPFSEL3: usize = mmio::GPIO_BASE + 0x0C; // GPIO 30..39
    const GPFSEL4: usize = mmio::GPIO_BASE + 0x10; // GPIO 40..49
    const GPFSEL5: usize = mmio::GPIO_BASE + 0x14; // GPIO 50..53

    // Set GPIO 34..39 to INPUT (0b000) so EMMC disconnects from Wi-Fi SDIO
    let mut fsel3 = mmio::read(GPFSEL3);
    for pin in 34..=39 {
        let shift = (pin - 30) * 3;
        fsel3 &= !(0b111 << shift);
    }
    mmio::write(GPFSEL3, fsel3);

    // Set GPIO 48..49 to ALT3 (0b111) for EMMC SD_CLK / SD_CMD
    let mut fsel4 = mmio::read(GPFSEL4);
    for pin in 48..=49 {
        let shift = (pin - 40) * 3;
        fsel4 = (fsel4 & !(0b111 << shift)) | (0b111 << shift);
    }
    mmio::write(GPFSEL4, fsel4);

    // Set GPIO 50..53 to ALT3 (0b111) for EMMC SD_DAT0..3
    let mut fsel5 = mmio::read(GPFSEL5);
    for pin in 50..=53 {
        let shift = (pin - 50) * 3;
        fsel5 = (fsel5 & !(0b111 << shift)) | (0b111 << shift);
    }
    mmio::write(GPFSEL5, fsel5);
}

/// Initialize the EMMC controller and SD card.
/// Returns `true` if a card was detected and initialized.
pub fn init() -> bool {
    unsafe {
        if !crate::timer::is_qemu() {
            route_emmc_to_sdcard();
        }

        // 1. Reset the Host Controller
        mmio::write(EMMC_CONTROL0, 0);
        mmio::write(EMMC_CONTROL1, CTRL1_SRST_HC);

        let mut timeout = 100_000;
        while mmio::read(EMMC_CONTROL1) & CTRL1_SRST_HC != 0 {
            timeout -= 1;
            if timeout == 0 {
                crate::println!("[WARN] SD: EMMC reset timeout");
                return false;
            }
            core::hint::spin_loop();
        }

        // 2. Power on the SD bus (3.3V)
        mmio::write(EMMC_CONTROL0, CTRL0_POWER_ON | CTRL0_VOLT_3_3V);

        // 3. Enable internal clock + max data timeout + clock divider
        // Divider bitfield [15:8] = 0x20 (~400kHz init clock)
        let clk_div: u32 = 0x20 << 8;
        mmio::write(EMMC_CONTROL1, CTRL1_CLK_INTLEN | CTRL1_TOUNIT_MAX | clk_div);

        timeout = 100_000;
        while mmio::read(EMMC_CONTROL1) & CTRL1_CLK_STABLE == 0 {
            timeout -= 1;
            if timeout == 0 {
                crate::println!("[WARN] SD: clock failed to stabilize");
                return false;
            }
            core::hint::spin_loop();
        }

        // Enable SD clock output
        let c1 = mmio::read(EMMC_CONTROL1);
        mmio::write(EMMC_CONTROL1, c1 | CTRL1_CLK_EN);

        // 4. Enable interrupt status flags (polling mode, so IRPT_EN = 0)
        mmio::write(EMMC_IRPT_EN, 0);
        mmio::write(EMMC_INTERRUPT, 0xFFFF_FFFF);
        mmio::write(EMMC_IRPT_MASK, 0xFFFF_FFFF);

        // 5. CMD0: Reset card to Idle state
        if !send_cmd(CMD0_GO_IDLE, 0) {
            crate::println!("[WARN] SD: CMD0 failed (no SD card attached?)");
            return false;
        }

        // 6. CMD8: Check voltage range (SD v2.0+)
        let _ = send_cmd(CMD8_SEND_IF_COND, 0x0000_01AA);

        // 7. ACMD41 loop: Wait for card initialization
        let mut ocr = 0u32;
        let mut ready = false;
        for _ in 0..1000 {
            if !send_cmd(CMD55_APP_CMD, 0) {
                break;
            }
            // Bit 30 (HCS) = 1 (support SDHC), voltage window = 3.2-3.4V
            if send_cmd(ACMD41_SEND_OP_COND, 0x40FF_8000) {
                ocr = mmio::read(EMMC_RESP0);
                if ocr & 0x8000_0000 != 0 {
                    ready = true;
                    break;
                }
            }
            for _ in 0..1000 {
                core::hint::spin_loop();
            }
        }

        if !ready {
            crate::println!("[WARN] SD: card did not initialize (no SD image attached)");
            return false;
        }

        let is_sdhc = (ocr & (1 << 30)) != 0;

        // 8. CMD2: Read CID
        if !send_cmd(CMD2_ALL_SEND_CID, 0) {
            crate::println!("[WARN] SD: CMD2 failed");
            return false;
        }
        let _cid0 = mmio::read(EMMC_RESP0);
        let _cid1 = mmio::read(EMMC_RESP1);
        let _cid2 = mmio::read(EMMC_RESP2);
        let _cid3 = mmio::read(EMMC_RESP3);

        // 9. CMD3: Publish Relative Card Address (RCA)
        if !send_cmd(CMD3_SEND_REL_ADDR, 0) {
            crate::println!("[WARN] SD: CMD3 failed");
            return false;
        }
        let rca = mmio::read(EMMC_RESP0) & 0xFFFF_0000;

        // 10. CMD7: Select card (enter Transfer state)
        if !send_cmd(CMD7_SELECT_CARD, rca) {
            crate::println!("[WARN] SD: CMD7 failed");
            return false;
        }

        // 11. CMD16: Set block length to 512 bytes
        if !send_cmd(CMD16_SET_BLK_LEN, SECTOR_SIZE as u32) {
            crate::println!("[WARN] SD: CMD16 failed");
            return false;
        }

        // Increase clock speed for data transfers (divider = 2 -> ~25MHz)
        let fast_div: u32 = 0x02 << 8;
        mmio::write(
            EMMC_CONTROL1,
            CTRL1_CLK_INTLEN | CTRL1_CLK_EN | CTRL1_TOUNIT_MAX | fast_div,
        );

        SD_STATE = SdState {
            initialized: true,
            rca,
            is_sdhc,
        };

        crate::println!(
            "[OK] SD Card: initialized (type={}, RCA=0x{:04x})",
            if is_sdhc { "SDHC" } else { "SDSC" },
            rca >> 16
        );
        true
    }
}

/// Check if the SD card is initialized and available.
pub fn is_initialized() -> bool {
    unsafe { SD_STATE.initialized }
}

/// Read a single 512-byte sector at Logical Block Address `lba`.
pub fn read_sector(lba: u32, buf: &mut [u8; SECTOR_SIZE]) -> bool {
    crate::interrupts::without_interrupts(|| unsafe {
        if !SD_STATE.initialized {
            return false;
        }

        let arg = if SD_STATE.is_sdhc {
            lba
        } else {
            lba * (SECTOR_SIZE as u32)
        };

        // Wait until data line is free
        let mut timeout = 500_000;
        while mmio::read(EMMC_STATUS) & (STATUS_CMD_INHIBIT | STATUS_DAT_INHIBIT) != 0 {
            timeout -= 1;
            if timeout == 0 {
                return false;
            }
            core::hint::spin_loop();
        }

        // 1 block of 512 bytes
        mmio::write(EMMC_BLKSIZECNT, (1 << 16) | (SECTOR_SIZE as u32));
        mmio::write(EMMC_INTERRUPT, 0xFFFF_FFFF);
        mmio::write(EMMC_ARG1, arg);
        mmio::write(EMMC_CMDTM, CMD17_READ_SINGLE);

        // Wait for command complete
        if !wait_interrupt(INT_CMD_DONE) {
            return false;
        }

        // Wait for buffer read ready
        if !wait_interrupt(INT_READ_RDY) {
            return false;
        }

        // Read 128 × 32-bit words (512 bytes) from EMMC_DATA
        for chunk in buf.chunks_exact_mut(4) {
            let word = mmio::read(EMMC_DATA);
            let bytes = word.to_le_bytes();
            chunk[0] = bytes[0];
            chunk[1] = bytes[1];
            chunk[2] = bytes[2];
            chunk[3] = bytes[3];
        }

        // Wait for data transfer complete
        let _ = wait_interrupt(INT_DATA_DONE);
        mmio::write(EMMC_INTERRUPT, 0xFFFF_FFFF);
        true
    })
}

/// Write a single 512-byte sector at Logical Block Address `lba`.
pub fn write_sector(lba: u32, buf: &[u8; SECTOR_SIZE]) -> bool {
    crate::interrupts::without_interrupts(|| unsafe {
        if !SD_STATE.initialized {
            return false;
        }

        let arg = if SD_STATE.is_sdhc {
            lba
        } else {
            lba * (SECTOR_SIZE as u32)
        };

        let mut timeout = 500_000;
        while mmio::read(EMMC_STATUS) & (STATUS_CMD_INHIBIT | STATUS_DAT_INHIBIT) != 0 {
            timeout -= 1;
            if timeout == 0 {
                return false;
            }
            core::hint::spin_loop();
        }

        // 1 block of 512 bytes
        mmio::write(EMMC_BLKSIZECNT, (1 << 16) | (SECTOR_SIZE as u32));
        mmio::write(EMMC_INTERRUPT, 0xFFFF_FFFF);
        mmio::write(EMMC_ARG1, arg);
        mmio::write(EMMC_CMDTM, CMD24_WRITE_SINGLE);

        if !wait_interrupt(INT_CMD_DONE) {
            return false;
        }

        if !wait_interrupt(INT_WRITE_RDY) {
            return false;
        }

        // Write 128 × 32-bit words (512 bytes) to EMMC_DATA
        for chunk in buf.chunks_exact(4) {
            let word = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            mmio::write(EMMC_DATA, word);
        }

        let _ = wait_interrupt(INT_DATA_DONE);
        mmio::write(EMMC_INTERRUPT, 0xFFFF_FFFF);
        true
    })
}

// ─── Internal helpers ───

unsafe fn send_cmd(cmd: u32, arg: u32) -> bool {
    let mut timeout = 500_000;
    while mmio::read(EMMC_STATUS) & STATUS_CMD_INHIBIT != 0 {
        timeout -= 1;
        if timeout == 0 {
            return false;
        }
        core::hint::spin_loop();
    }

    mmio::write(EMMC_INTERRUPT, 0xFFFF_FFFF);
    mmio::write(EMMC_ARG1, arg);
    mmio::write(EMMC_CMDTM, cmd);

    wait_interrupt(INT_CMD_DONE)
}

unsafe fn wait_interrupt(mask: u32) -> bool {
    let mut timeout = 500_000;
    loop {
        let irpt = mmio::read(EMMC_INTERRUPT);
        if irpt & INT_ERROR_MASK != 0 {
            mmio::write(EMMC_INTERRUPT, irpt);
            return false;
        }
        if irpt & mask != 0 {
            mmio::write(EMMC_INTERRUPT, mask);
            return true;
        }
        timeout -= 1;
        if timeout == 0 {
            return false;
        }
        core::hint::spin_loop();
    }
}
