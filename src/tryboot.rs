// ZerOS — Tryboot A/B Fail-Safe Boot & Hardware Watchdog (M6.3)
//
// Because the Argon POD occupies all 40 GPIO pins on the Raspberry Pi Zero 2 W,
// physical UART/JTAG recovery is inconvenient when the case is assembled.
//
// Tryboot A/B solves this with a dual-partition fail-safe boot scheme:
//   - Partition 1 (Slot A): Stable known-good kernel8.img + autoboot.txt
//   - Partition 2 (Slot B): New / experimental kernel8.img
//
// `autoboot.txt` format on Partition 1:
//   [all]
//   tryboot_a_b=1
//   boot_partition=1
//   [tryboot]
//   boot_partition=2
//
// Additionally, the BCM2710 Power Management (PM) hardware watchdog at
// 0x3F10_0000 automatically resets the board back to the default partition
// if an experimental kernel hangs during boot.

#![allow(dead_code)]

use crate::fat32;
use crate::mmio;

// ─── BCM2710 Power Management / Watchdog Registers (Base: 0x3F10_0000) ───
const PM_BASE: usize = mmio::PERIPHERAL_BASE + 0x0010_0000;
const PM_RSTC: usize = PM_BASE + 0x1C; // Reset Control
const PM_RSTS: usize = PM_BASE + 0x20; // Reset Status / Boot Partition
const PM_WDOG: usize = PM_BASE + 0x24; // Watchdog Timer

const PM_PASSWORD:       u32 = 0x5A00_0000;
const PM_RSTC_WRCFG_CLR: u32 = 0xFFFF_FFCF;
const PM_RSTC_WRCFG_FULL_RESET: u32 = 0x0000_0020;
const PM_WDOG_TIME_SET:  u32 = 0x000F_FFFF;

/// Boot slot identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootSlot {
    SlotA, // Partition 1 (Primary / Recovery)
    SlotB, // Partition 2 (Tryboot / Experimental)
}

/// Tryboot system state
pub struct TrybootStatus {
    pub active_slot: BootSlot,
    pub default_partition: u8,
    pub tryboot_partition: u8,
    pub committed: bool,
}

static mut STATUS: TrybootStatus = TrybootStatus {
    active_slot: BootSlot::SlotA,
    default_partition: 1,
    tryboot_partition: 2,
    committed: false,
};

/// Initialize Tryboot A/B subsystem:
///   1. Checks PM_RSTS to see if we booted in tryboot mode
///   2. Parses `AUTOBOOT.TXT` from the FAT32 boot partition (if mounted)
///   3. Marks the current boot as healthy (self-test passed)
pub fn init() {
    let rsts = unsafe { mmio::read(PM_RSTS) };
    // Partition number encoded in PM_RSTS bits [5:0] (with odd bits interleaved in some firmware)
    let raw_part = rsts & 0x3F;

    let mut default_part = 1u8;
    let mut tryboot_part = 2u8;

    if fat32::is_mounted() {
        if let Some(bytes) = fat32::read_file("AUTOBOOT.TXT") {
            if let Ok(text) = core::str::from_utf8(&bytes) {
                let (def_p, try_p) = parse_autoboot(text);
                default_part = def_p;
                tryboot_part = try_p;
            }
        }
    }

    let active_slot = if raw_part == 2 || default_part == 2 {
        BootSlot::SlotB
    } else {
        BootSlot::SlotA
    };

    unsafe {
        STATUS = TrybootStatus {
            active_slot,
            default_partition: default_part,
            tryboot_partition: tryboot_part,
            committed: true,
        };
    }

    crate::println!(
        "[OK] Tryboot A/B: active={:?} (default=part {}, tryboot=part {}, PM_RSTS=0x{:08x})",
        active_slot,
        default_part,
        tryboot_part,
        rsts
    );
}

/// Parse `autoboot.txt` content and return `(default_partition, tryboot_partition)`.
fn parse_autoboot(text: &str) -> (u8, u8) {
    let mut in_tryboot_section = false;
    let mut default_part = 1u8;
    let mut tryboot_part = 2u8;

    for raw_line in text.lines() {
        let line = raw_line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if line.eq_ignore_ascii_case("[tryboot]") {
            in_tryboot_section = true;
            continue;
        } else if line.starts_with('[') {
            in_tryboot_section = false;
            continue;
        }

        if let Some(val_str) = line.strip_prefix("boot_partition=") {
            if let Ok(p) = val_str.trim().parse::<u8>() {
                if in_tryboot_section {
                    tryboot_part = p;
                } else {
                    default_part = p;
                }
            }
        }
    }

    (default_part, tryboot_part)
}

/// Generate a standard `autoboot.txt` configuration string.
pub fn format_autoboot(default_part: u8, tryboot_part: u8) -> alloc::string::String {
    use core::fmt::Write;
    let mut s = alloc::string::String::with_capacity(96);
    let _ = writeln!(s, "[all]");
    let _ = writeln!(s, "tryboot_a_b=1");
    let _ = writeln!(s, "boot_partition={}", default_part);
    let _ = writeln!(s, "[tryboot]");
    let _ = writeln!(s, "boot_partition={}", tryboot_part);
    s
}

/// Commit a boot partition as the new default in `AUTOBOOT.TXT`.
/// For example, after Slot B passes self-tests, calling `set_default_partition(2, 1)`
/// makes Partition 2 the permanent default and Partition 1 the fallback.
pub fn set_default_partition(default_part: u8, tryboot_part: u8) -> bool {
    if !fat32::is_mounted() {
        return false;
    }

    let content = format_autoboot(default_part, tryboot_part);
    if fat32::write_small_file("AUTOBOOT.TXT", content.as_bytes()) {
        unsafe {
            STATUS.default_partition = default_part;
            STATUS.tryboot_partition = tryboot_part;
        }
        crate::println!(
            "[OK] Tryboot: updated AUTOBOOT.TXT (default=part {}, tryboot=part {})",
            default_part,
            tryboot_part
        );
        true
    } else {
        crate::println!("[ERR] Tryboot: failed to update AUTOBOOT.TXT");
        false
    }
}

/// Get current Tryboot status
pub fn status() -> &'static TrybootStatus {
    unsafe { &*core::ptr::addr_of!(STATUS) }
}

// ─── Hardware Watchdog & Reset Control ───

/// Start the BCM2710 hardware watchdog with a timeout in seconds (max ~15s).
/// If `watchdog_feed()` or `watchdog_stop()` is not called before timeout,
/// the SoC performs a full reset back to the default partition.
#[allow(dead_code)]
pub fn watchdog_start(seconds: u32) {
    // 1 second ≈ 65536 (1 << 16) watchdog ticks
    let ticks = (seconds.min(15) << 16) & PM_WDOG_TIME_SET;
    unsafe {
        mmio::write(PM_WDOG, PM_PASSWORD | ticks);
        let rstc = mmio::read(PM_RSTC) & PM_RSTC_WRCFG_CLR;
        mmio::write(PM_RSTC, PM_PASSWORD | rstc | PM_RSTC_WRCFG_FULL_RESET);
    }
}

/// Feed (reload) the hardware watchdog timer.
#[allow(dead_code)]
pub fn watchdog_feed(seconds: u32) {
    let ticks = (seconds.min(15) << 16) & PM_WDOG_TIME_SET;
    unsafe {
        mmio::write(PM_WDOG, PM_PASSWORD | ticks);
    }
}

/// Stop/disarm the hardware watchdog timer.
#[allow(dead_code)]
pub fn watchdog_stop() {
    unsafe {
        let rstc = mmio::read(PM_RSTC) & PM_RSTC_WRCFG_CLR;
        mmio::write(PM_RSTC, PM_PASSWORD | rstc);
    }
}

/// Trigger an immediate hardware reboot into `partition` (e.g. 2 for tryboot Slot B).
#[allow(dead_code)]
pub fn reboot_to_partition(partition: u8) -> ! {
    crate::println!("[SYS] Rebooting into partition {}...", partition);
    crate::interrupts::disable();

    // Encode partition number into PM_RSTS
    let p = (partition as u32) & 0x3F;
    let rsts_val = PM_PASSWORD
        | (p & 0b00_0001)
        | ((p & 0b00_0010) << 1)
        | ((p & 0b00_0100) << 2)
        | ((p & 0b00_1000) << 3)
        | ((p & 0b01_0000) << 4)
        | ((p & 0b10_0000) << 5);

    unsafe {
        mmio::write(PM_RSTS, rsts_val);
        mmio::write(PM_WDOG, PM_PASSWORD | 10); // 10 ticks -> immediate reset
        let rstc = mmio::read(PM_RSTC) & PM_RSTC_WRCFG_CLR;
        mmio::write(PM_RSTC, PM_PASSWORD | rstc | PM_RSTC_WRCFG_FULL_RESET);
    }

    loop {
        unsafe { core::arch::asm!("wfe"); }
    }
}
