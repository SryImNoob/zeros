// ZerOS — ARM Generic Timer Driver
//
// Uses the EL1 Physical Timer (CNTP) for periodic tick interrupts.
// Timer frequency is read from CNTFRQ_EL0 at runtime.
//
// On RPi Zero 2 W: typically 54 MHz
// On QEMU raspi3b: may vary (often 62.5 MHz)

use core::sync::atomic::{AtomicU64, Ordering};

use crate::interrupts;

/// Global tick counter (incremented every timer IRQ)
static TICKS: AtomicU64 = AtomicU64::new(0);

/// Timer interval in counter ticks (set during init)
static mut INTERVAL: u64 = 0;

/// Read the timer counter frequency (Hz)
fn frequency() -> u64 {
    let freq: u64;
    unsafe { core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq); }
    freq
}

/// Read the current counter value
#[allow(dead_code)]
pub fn counter() -> u64 {
    let cnt: u64;
    unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) cnt); }
    cnt
}

/// Get the current tick count
pub fn ticks() -> u64 {
    TICKS.load(Ordering::Relaxed)
}

/// Initialize the timer with a periodic interrupt
///
/// `hz` — interrupt frequency (e.g., 1 for 1 tick/sec, 10 for 10 ticks/sec)
pub fn init(hz: u64) {
    let freq = frequency();
    let interval = freq / hz;

    crate::println!("[OK] Timer: freq={}Hz, interval={} ticks ({}Hz tick)",
        freq, interval, hz);

    unsafe {
        INTERVAL = interval;

        // Set timer value (countdown to first interrupt)
        core::arch::asm!("msr cntp_tval_el0, {}", in(reg) interval);

        // Enable timer: ENABLE=1, IMASK=0 (don't mask interrupt)
        core::arch::asm!("msr cntp_ctl_el0, {}", in(reg) 1u64);
    }

    // Enable timer IRQ routing through local interrupt controller
    interrupts::enable_timer_irq();
}

/// Handle timer IRQ — called from irq_dispatch
pub fn handle_irq() {
    let tick = TICKS.fetch_add(1, Ordering::Relaxed) + 1;

    // Reload timer for next interrupt
    unsafe {
        core::arch::asm!("msr cntp_tval_el0, {}", in(reg) INTERVAL);
    }

    // Poll buttons for debouncing (runs every tick for responsiveness)
    crate::input::poll_buttons();

    // Poll IR decoder and push any decoded frame to the input queue
    unsafe {
        let decoder = &mut *core::ptr::addr_of_mut!(crate::ir::IR_DECODER);
        if let Some(frame) = decoder.poll() {
            if !frame.is_repeat {
                let code = ((frame.address as u32) << 8) | (frame.command as u32);
                crate::input::push_event(crate::input::InputEvent::IrCode(code));
            }
        }
    }

    // Notify scheduler of timer tick (wakes sleeping tasks & accounts CPU ticks)
    crate::task::on_tick(tick);
}

/// Busy-wait delay in milliseconds (uses counter, no IRQ needed)
#[allow(dead_code)]
pub fn delay_ms(ms: u64) {
    let freq = frequency();
    let target = counter() + (freq * ms / 1000);
    while counter() < target {
        core::hint::spin_loop();
    }
}
