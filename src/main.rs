// ZerOS — A bare-metal OS for Raspberry Pi Zero 2 W
//
// Hardware: RPi Zero 2 W (BCM2710A1, Cortex-A53) + Argon POD (ILI9341 2.8" LCD)
//
// M1: Minimal boot + UART ✓
// M2: Exception handling, interrupts, timer, GPIO ✓
// M3: Physical page allocator, MMU, kernel heap ✓
// M4: Framebuffer display + text console

#![no_std]
#![no_main]

extern crate alloc;

mod boot;
mod console;
mod exception;
mod font;
mod framebuffer;
mod gpio;
mod heap;
mod ili9341;
mod interrupts;
mod mailbox;
mod mmio;
mod mmu;
mod page_alloc;
mod spi;
mod timer;
mod uart;

use core::panic::PanicInfo;

/// Kernel entry point
#[no_mangle]
pub extern "C" fn kernel_main() -> ! {
    // ── Stage 1: UART ──
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

    // ── Stage 2: GPIO ──
    gpio::init();

    // ── Stage 3: Memory management ──
    let kernel_end = page_alloc::kernel_end_addr();
    let heap_end = heap::init(kernel_end);
    let pages_start = page_alloc::align_up(heap_end, page_alloc::PAGE_SIZE);
    page_alloc::init(pages_start);
    mmu::init();

    // ── Stage 4: Display + Console ──
    // Strategy: try GPU framebuffer first (works in QEMU and HDMI),
    // then fall back to SPI ILI9341 (Argon POD real hardware).
    if framebuffer::init(320, 240) {
        // GPU framebuffer available (QEMU or HDMI)
        console::init();
        println!("[OK] Console: 40x30 text, dual output (UART + screen)");
    } else {
        // No GPU framebuffer — try SPI display (real Argon POD hardware)
        println!("[..] GPU framebuffer not available, trying SPI display...");
        spi::init();
        if ili9341::init() {
            println!("[OK] ILI9341 SPI display initialized");
        } else {
            println!("[WARN] No display available — UART only mode");
        }
    }

    // ── Stage 5: Timer ──
    timer::init(1);

    // ── Stage 6: Enable interrupts ──
    interrupts::enable();
    println!("[OK] Interrupts enabled — timer ticking!");
    println!();
    println!("ZerOS is ready. Free pages: {} ({}MB)",
        page_alloc::free_count(),
        page_alloc::free_count() * page_alloc::PAGE_SIZE / 1024 / 1024);
    println!();

    // ── Main loop ──
    loop {
        unsafe { core::arch::asm!("wfe"); }
    }
}

/// Panic handler
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    interrupts::disable();
    println!();
    println!("!!! KERNEL PANIC !!!");
    if let Some(location) = info.location() {
        println!("  at {}:{}:{}", location.file(), location.line(), location.column());
    }
    if let Some(msg) = info.message().as_str() {
        println!("  {}", msg);
    }
    loop { unsafe { core::arch::asm!("wfe"); } }
}
