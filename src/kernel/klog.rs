// ZerOS — Persistent Kernel Blackbox Logger & Soft-Lockup Detector (`ZEROS.LOG`)
//
// Features:
//   1. `64 KB` lock-free/IRQ-safe in-memory log buffer (`KLOG_BUF`, 128 sectors × 512B)
//      that automatically captures every `print!` / `println!` with a `[SSSS.mmm] `
//      hardware counter timestamp.
//   2. Pre-indexed physical LBA table (`LOG_SECTOR_LBAS`) for `/ZEROS.LOG` on the
//      FAT32 boot partition (`/boot/firmware/ZEROS.LOG`), enabling direct `sd::write_sector`
//      flushes in ~1.5 ms without traversing FAT32 directory structures.
//   3. Incremental `klog::flush()` at key milestones + periodic 1Hz background sync.
//   4. `klog::flush_emergency(reason)` on Synchronous Exceptions (Data/Inst Abort),
//      Rust `panic!`, Soft-Lockup / IRQ Storm detection, and `B4` warm reboot.

use core::fmt::{self, Write};
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use crate::arch::timer;
use crate::drivers::block::sd::{self, SECTOR_SIZE};
use crate::fs::fat32;
use crate::kernel::{interrupts, tryboot};

pub const KLOG_SECTORS: usize = 128; // 128 * 512B = 64 KB
pub const KLOG_CAPACITY: usize = KLOG_SECTORS * SECTOR_SIZE;

static mut KLOG_BUF: [u8; KLOG_CAPACITY] = [b' '; KLOG_CAPACITY];
static mut WRITE_POS: usize = 0;
static mut FLUSHED_SECTOR: usize = 0;
static mut AT_LINE_START: bool = true;

static mut LOG_LBAS: [u32; KLOG_SECTORS] = [0u32; KLOG_SECTORS];
static mut LOG_LBA_COUNT: usize = 0;

static KLOG_READY: AtomicBool = AtomicBool::new(false);
static FLUSH_IN_PROGRESS: AtomicBool = AtomicBool::new(false);
static LOCKUP_TRIGGERED: AtomicBool = AtomicBool::new(false);

// ─── Soft-Lockup & IRQ Budget Telemetry ───
static LAST_HEARTBEAT_TICK: AtomicU64 = AtomicU64::new(0);
static MAX_IRQ_US: AtomicU64 = AtomicU64::new(0);
static LAST_IRQ_US: AtomicU64 = AtomicU64::new(0);
static mut LONG_IRQ_STREAK: u32 = 0;

struct BufWriter;

impl fmt::Write for BufWriter {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        unsafe {
            append_raw_unlocked(s.as_bytes());
        }
        Ok(())
    }
}

#[inline]
unsafe fn push_byte_unlocked(b: u8) {
    let pos = WRITE_POS;
    if pos < KLOG_CAPACITY - 128 {
        KLOG_BUF[pos] = b;
        WRITE_POS = pos + 1;
    } else {
        // Wrap around preserving the first 4 KB (boot log header) and overwriting 4KB..64KB
        let wrap_start = 8 * SECTOR_SIZE;
        KLOG_BUF[wrap_start] = b;
        WRITE_POS = wrap_start + 1;
        if FLUSHED_SECTOR > 8 {
            FLUSHED_SECTOR = 8;
        }
    }
}

unsafe fn write_timestamp_unlocked() {
    let freq = timer::frequency();
    if freq == 0 {
        for &b in b"[0000.000] " {
            push_byte_unlocked(b);
        }
        return;
    }
    let ms = (timer::counter() * 1000) / freq;
    let sec = (ms / 1000) % 10000;
    let rem_ms = ms % 1000;

    push_byte_unlocked(b'[');
    push_byte_unlocked(b'0' + ((sec / 1000) % 10) as u8);
    push_byte_unlocked(b'0' + ((sec / 100) % 10) as u8);
    push_byte_unlocked(b'0' + ((sec / 10) % 10) as u8);
    push_byte_unlocked(b'0' + (sec % 10) as u8);
    push_byte_unlocked(b'.');
    push_byte_unlocked(b'0' + ((rem_ms / 100) % 10) as u8);
    push_byte_unlocked(b'0' + ((rem_ms / 10) % 10) as u8);
    push_byte_unlocked(b'0' + (rem_ms % 10) as u8);
    push_byte_unlocked(b']');
    push_byte_unlocked(b' ');
}

unsafe fn append_raw_unlocked(bytes: &[u8]) {
    for &b in bytes {
        if AT_LINE_START && b != b'\n' && b != b'\r' {
            AT_LINE_START = false;
            write_timestamp_unlocked();
        }
        push_byte_unlocked(b);
        if b == b'\n' {
            AT_LINE_START = true;
        }
    }
}

/// Append formatted arguments to the in-memory blackbox ring buffer (`KLOG_BUF`).
/// Called automatically from `uart::_print()` with interrupts already disabled.
pub fn write_fmt_unlocked(args: fmt::Arguments) {
    let _ = BufWriter.write_fmt(args);
}

/// Bind `/ZEROS.LOG` on the FAT32 boot partition to direct physical LBA sectors
/// and immediately flush all early boot logs to SD card.
pub fn init() {
    LAST_HEARTBEAT_TICK.store(timer::ticks(), Ordering::Relaxed);

    if !fat32::is_mounted() {
        return;
    }

    let Some(entry) = fat32::resolve_path("/ZEROS.LOG") else {
        crate::println!("[WARN] KLOG: /ZEROS.LOG not found on FAT32 root; run `make tryboot` to preallocate");
        return;
    };

    let chain = fat32::build_cluster_chain(&entry);
    if chain.is_empty() {
        return;
    }

    let spc = fat32::sectors_per_cluster();
    if spc == 0 {
        return;
    }

    let mut count = 0usize;
    unsafe {
        for &cluster in chain.iter() {
            let base_lba = fat32::cluster_to_lba(cluster);
            if base_lba == 0 {
                break;
            }
            for s in 0..spc {
                if count < KLOG_SECTORS {
                    LOG_LBAS[count] = base_lba + (s as u32);
                    count += 1;
                }
            }
            if count >= KLOG_SECTORS {
                break;
            }
        }
        LOG_LBA_COUNT = count;
    }

    if count > 0 {
        KLOG_READY.store(true, Ordering::Relaxed);
        crate::println!(
            "[OK] KLOG: Bound /ZEROS.LOG ({} sectors = {} KB, first LBA={})",
            count,
            (count * SECTOR_SIZE) / 1024,
            unsafe { LOG_LBAS[0] }
        );
        // Clear remaining sectors of ZEROS.LOG on boot so old logs from previous boots don't mix
        unsafe {
            let blank = [b' '; SECTOR_SIZE];
            for idx in 0..count {
                let mut sec = blank;
                sec[SECTOR_SIZE - 1] = b'\n';
                let _ = sd::write_sector(LOG_LBAS[idx], &sec);
            }
            FLUSHED_SECTOR = 0;
        }
        flush();
    }
}

/// Feed the main-thread (`ui_task`) soft-lockup heartbeat counter.
#[inline]
pub fn heartbeat() {
    LAST_HEARTBEAT_TICK.store(timer::ticks(), Ordering::Relaxed);
}

/// Record the duration of a Timer IRQ in microseconds and check for Soft-Lockup or IRQ Storm.
/// Called at the end of `irq_dispatch` with the interrupted `elr` and `lr` (`x30`).
pub fn record_irq_and_check_lockup(irq_us: u64, elr: u64, lr: u64) {
    LAST_IRQ_US.store(irq_us, Ordering::Relaxed);
    let prev_max = MAX_IRQ_US.load(Ordering::Relaxed);
    if irq_us > prev_max {
        MAX_IRQ_US.store(irq_us, Ordering::Relaxed);
    }

    if LOCKUP_TRIGGERED.load(Ordering::Relaxed) {
        return;
    }

    let streak = unsafe {
        if irq_us >= 9_500 {
            LONG_IRQ_STREAK = LONG_IRQ_STREAK.saturating_add(1);
        } else {
            LONG_IRQ_STREAK = 0;
        }
        LONG_IRQ_STREAK
    };

    let now_tick = timer::ticks();
    let last_hb = LAST_HEARTBEAT_TICK.load(Ordering::Relaxed);
    let stalled_ticks = now_tick.saturating_sub(last_hb);

    // Trigger emergency lockup recovery ONLY when the main thread (`ui_task`) has genuinely
    // stalled for > 5.0 seconds (`500` ticks at 100Hz) after system boot (`now_tick > 200`).
    if now_tick > 200 && stalled_ticks > 500 {
        LOCKUP_TRIGGERED.store(true, Ordering::Relaxed);
        crate::drivers::bluetooth::emergency_stop_a2dp();
        crate::println!(
            "\n!!! [SOFT-LOCKUP / IRQ STORM DETECTED] !!!\n  tick={} last_hb={} stalled={}0ms irq_streak={}\n  last_irq={}us max_irq={}us\n  Interrupted PC (ELR_EL1)=0x{:016x} LR (x30)=0x{:016x}",
            now_tick,
            last_hb,
            stalled_ticks,
            streak,
            irq_us,
            MAX_IRQ_US.load(Ordering::Relaxed),
            elr,
            lr
        );
        flush_emergency("SOFT_LOCKUP_AUTO_REBOOT");
        tryboot::reboot_to_partition(0);
    }
}

/// Return `(last_irq_us, max_irq_us)` for UI / telemetry inspection.
pub fn irq_stats_us() -> (u64, u64) {
    (
        LAST_IRQ_US.load(Ordering::Relaxed),
        MAX_IRQ_US.load(Ordering::Relaxed),
    )
}

/// Incrementally flush any newly written 512-byte sectors in `KLOG_BUF` to `/ZEROS.LOG`.
/// Safe to call from `ui_task()` or between Bluetooth steps; skips if already flushing.
pub fn flush() {
    if !KLOG_READY.load(Ordering::Relaxed) || crate::bsp::is_qemu() {
        return;
    }

    let acquired = interrupts::without_interrupts(|| {
        if FLUSH_IN_PROGRESS.load(Ordering::Relaxed) {
            false
        } else {
            FLUSH_IN_PROGRESS.store(true, Ordering::Relaxed);
            true
        }
    });
    if !acquired {
        return;
    }

    unsafe {
        let count = LOG_LBA_COUNT;
        let write_pos = WRITE_POS;
        let end_sec = ((write_pos + SECTOR_SIZE - 1) / SECTOR_SIZE).min(count);
        let start_sec = FLUSHED_SECTOR.min(end_sec);

        let mut sec_buf = [b' '; SECTOR_SIZE];
        for sec_idx in start_sec..end_sec {
            let byte_off = sec_idx * SECTOR_SIZE;
            let valid_end = write_pos.min(byte_off + SECTOR_SIZE);
            let valid_len = valid_end.saturating_sub(byte_off);
            sec_buf.fill(b' ');
            if valid_len > 0 {
                sec_buf[..valid_len].copy_from_slice(&KLOG_BUF[byte_off..byte_off + valid_len]);
            }
            if valid_len < SECTOR_SIZE {
                sec_buf[SECTOR_SIZE - 1] = b'\n';
            }
            let _ = sd::write_sector(LOG_LBAS[sec_idx], &sec_buf);
        }
        // Keep FLUSHED_SECTOR at the last partially-filled sector so new lines in the same sector overwrite cleanly
        FLUSHED_SECTOR = write_pos / SECTOR_SIZE;
    }

    FLUSH_IN_PROGRESS.store(false, Ordering::Relaxed);
}

/// Force-flush the entire active log buffer to `/ZEROS.LOG` with interrupts masked.
/// Called before `reboot_to_partition(0)`, on `panic!`, on Synchronous Exceptions, or Soft-Lockup.
pub fn flush_emergency(reason: &str) {
    interrupts::disable();
    let _ = BufWriter.write_fmt(format_args!(
        "\n=== [KLOG EMERGENCY FLUSH: {} | ticks={} | max_irq={}us] ===\n",
        reason,
        timer::ticks(),
        MAX_IRQ_US.load(Ordering::Relaxed)
    ));

    if !KLOG_READY.load(Ordering::Relaxed) {
        return;
    }

    unsafe {
        let count = LOG_LBA_COUNT;
        let write_pos = WRITE_POS;
        let end_sec = ((write_pos + SECTOR_SIZE - 1) / SECTOR_SIZE).min(count);
        let mut sec_buf = [b' '; SECTOR_SIZE];
        for sec_idx in 0..end_sec {
            let byte_off = sec_idx * SECTOR_SIZE;
            let valid_end = write_pos.min(byte_off + SECTOR_SIZE);
            let valid_len = valid_end.saturating_sub(byte_off);
            sec_buf.fill(b' ');
            if valid_len > 0 {
                sec_buf[..valid_len].copy_from_slice(&KLOG_BUF[byte_off..byte_off + valid_len]);
            }
            if valid_len < SECTOR_SIZE {
                sec_buf[SECTOR_SIZE - 1] = b'\n';
            }
            let _ = sd::write_sector(LOG_LBAS[sec_idx], &sec_buf);
        }
        FLUSHED_SECTOR = write_pos / SECTOR_SIZE;
    }
}
