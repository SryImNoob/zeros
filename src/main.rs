// ZerOS — A bare-metal OS for Raspberry Pi Zero 2 W
//
// Hardware: RPi Zero 2 W (BCM2710A1, Cortex-A53) + Argon POD (ILI9341 2.8" LCD)
//
// M1: Minimal boot + UART ✓
// M2: Exception handling, interrupts, timer, GPIO

#![no_std]
#![no_main]

mod boot;
mod exception;
mod gpio;
mod interrupts;
mod mmio;
mod timer;
mod uart;

use core::panic::PanicInfo;

/// Kernel entry point — called from boot stub after EL2→EL1 transition
#[no_mangle]
pub extern "C" fn kernel_main() -> ! {
    // ── Stage 1: UART (serial debug output) ──
    uart::init();

    println!();
    println!("=================================");
    println!("  ZerOS v0.1.0");
    println!("  Raspberry Pi Zero 2 W");
    println!("  AArch64 bare-metal kernel");
    println!("=================================");
    println!();
    println!("[OK] UART initialized");
    println!("[OK] Running at EL1 on core 0");
    println!("[OK] Exception vector table installed");

    // ── Stage 2: GPIO (activity LED) ──
    gpio::init();

    // ── Stage 3: Timer (1 Hz periodic tick) ──
    timer::init(1); // 1 tick per second

    // ── Stage 4: Enable interrupts ──
    interrupts::enable();
    println!("[OK] Interrupts enabled — timer ticking!");
    println!();

    // TODO: M3 — Initialize memory management (MMU, page allocator)
    // TODO: M4 — Initialize SPI + ILI9341 display driver
    // TODO: M5 — Initialize button & IR input
    // TODO: M6 — Initialize task scheduler

    // ── Main loop: idle, let IRQs do the work ──
    loop {
        unsafe {
            core::arch::asm!("wfe"); // Wait For Event (low-power idle)
        }
    }
}

/// Panic handler — prints the panic message via UART and halts
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // Disable interrupts to prevent re-entry
    interrupts::disable();

    println!();
    println!("!!! KERNEL PANIC !!!");
    if let Some(location) = info.location() {
        println!("  at {}:{}:{}", location.file(), location.line(), location.column());
    }
    if let Some(msg) = info.message().as_str() {
        println!("  {}", msg);
    }

    loop {
        unsafe { core::arch::asm!("wfe"); }
    }
}
