// ZerOS — A bare-metal OS for Raspberry Pi Zero 2 W
//
// Hardware: RPi Zero 2 W (BCM2710A1, Cortex-A53) + Argon POD (ILI9341 2.8" LCD)
//
// This is the kernel entry point, called from boot.rs after basic setup.

#![no_std]
#![no_main]

mod boot;
mod mmio;
mod uart;

use core::panic::PanicInfo;

/// Kernel entry point — called from the boot assembly stub
#[no_mangle]
pub extern "C" fn kernel_main() -> ! {
    // Initialize UART for debug output
    uart::init();

    // Say hello!
    uart::puts("\n");
    uart::puts("=================================\n");
    uart::puts("  ZerOS v0.1.0\n");
    uart::puts("  Raspberry Pi Zero 2 W\n");
    uart::puts("  AArch64 bare-metal kernel\n");
    uart::puts("=================================\n");
    uart::puts("\n");
    uart::puts("[OK] UART initialized\n");
    uart::puts("[..] Kernel running on core 0\n");

    // TODO: M2 — Initialize interrupts, timers
    // TODO: M3 — Initialize memory management (MMU, page allocator)
    // TODO: M4 — Initialize SPI + ILI9341 display driver
    // TODO: M5 — Initialize button & IR input
    // TODO: M6 — Initialize task scheduler

    uart::puts("\n[ZerOS] Nothing more to do. Halting.\n");

    // Halt: loop forever
    loop {
        // WFE = Wait For Event (low-power idle)
        unsafe {
            core::arch::asm!("wfe");
        }
    }
}

/// Panic handler — prints the panic message via UART and halts
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    uart::puts("\n!!! KERNEL PANIC !!!\n");
    if let Some(location) = info.location() {
        uart::puts("  at ");
        uart::puts(location.file());
        uart::puts("\n");
    }
    loop {
        unsafe {
            core::arch::asm!("wfe");
        }
    }
}
