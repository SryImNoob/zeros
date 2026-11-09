// ZerOS — Interrupt Controller
//
// BCM2710 / RPi Zero 2 W uses two interrupt controllers:
//   1. Legacy GPU interrupt controller at PERIPHERAL_BASE + 0xB200
//   2. ARM Local Peripherals (QA7) at 0x4000_0000 — per-core timers, mailboxes
//
// The ARM Generic Timer IRQ is routed through the Local Peripheral controller,
// NOT through the legacy GPU controller.

use crate::mmio;
use crate::timer;

/// ARM Local Peripherals base address (QA7)
const LOCAL_BASE: usize = 0x4000_0000;

/// Core 0 Timer Interrupt Control register
const CORE0_TIMER_IRQCNTL: usize = LOCAL_BASE + 0x40;

/// Core 0 IRQ Source register
const CORE0_IRQ_SOURCE: usize = LOCAL_BASE + 0x60;

/// IRQ source bit for nCNTPNSIRQ (EL1 physical timer, non-secure)
const IRQ_SRC_CNTPNSIRQ: u32 = 1 << 1;

/// Enable timer IRQ for core 0
pub fn enable_timer_irq() {
    unsafe {
        // Bit 1: route nCNTPNSIRQ to core 0's IRQ
        mmio::write(CORE0_TIMER_IRQCNTL, IRQ_SRC_CNTPNSIRQ);
    }
}

/// Disable timer IRQ for core 0
#[allow(dead_code)]
pub fn disable_timer_irq() {
    unsafe {
        mmio::write(CORE0_TIMER_IRQCNTL, 0);
    }
}

/// Top-level IRQ dispatcher — called from the exception vector table
#[no_mangle]
pub extern "C" fn irq_dispatch(_ctx: &crate::exception::ExceptionContext) {
    let source = unsafe { mmio::read(CORE0_IRQ_SOURCE) };

    if source & IRQ_SRC_CNTPNSIRQ != 0 {
        timer::handle_irq();
    }

    // Future: check other interrupt sources here
}

/// Enable IRQs globally (unmask DAIF.I)
pub fn enable() {
    unsafe {
        core::arch::asm!("msr daifclr, #2"); // Clear IRQ mask bit
    }
}

/// Disable IRQs globally (mask DAIF.I)
#[allow(dead_code)]
pub fn disable() {
    unsafe {
        core::arch::asm!("msr daifset, #2"); // Set IRQ mask bit
    }
}
