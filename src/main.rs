// ZerOS — A bare-metal OS for Raspberry Pi Zero 2 W
//
// Hardware: RPi Zero 2 W (BCM2710A1, Cortex-A53) + Argon POD (ILI9341 2.8" LCD)
//
// M1: Minimal boot + UART ✓
// M2: Exception handling, interrupts, timer, GPIO ✓
// M3: Physical page allocator, MMU, kernel heap

#![no_std]
#![no_main]

extern crate alloc;

mod boot;
mod exception;
mod gpio;
mod heap;
mod interrupts;
mod mmio;
mod mmu;
mod page_alloc;
mod timer;
mod uart;

use alloc::string::String;
use alloc::vec::Vec;
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

    // ── Stage 3: Memory management ──
    // 3a: Kernel heap (fixed region after kernel image)
    let kernel_end = page_alloc::kernel_end_addr();
    let heap_end = heap::init(kernel_end);

    // 3b: Physical page allocator (free pages after heap)
    let pages_start = page_alloc::align_up(heap_end, page_alloc::PAGE_SIZE);
    page_alloc::init(pages_start);

    // 3c: MMU (identity mapping, enables caches)
    mmu::init();

    // ── Stage 4: Demo — alloc crate works! ──
    demo_alloc();

    // ── Stage 5: Timer (1 Hz periodic tick) ──
    timer::init(1);

    // ── Stage 6: Enable interrupts ──
    interrupts::enable();
    println!("[OK] Interrupts enabled — timer ticking!");
    println!();

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

/// Demonstrate that dynamic allocation works
fn demo_alloc() {
    println!();
    println!("[demo] Testing alloc crate...");

    // Vec
    let mut v: Vec<u32> = Vec::new();
    for i in 0..10 {
        v.push(i * i);
    }
    println!("[demo] Vec<u32>: {:?}", v.as_slice());

    // String
    let s = String::from("Hello from ZerOS heap!");
    println!("[demo] String: {}", s);

    // Box
    let boxed = alloc::boxed::Box::new(42u64);
    println!("[demo] Box<u64>: {}", *boxed);

    // Heap stats
    let (used, total) = heap::stats();
    println!("[demo] Heap: {}/{} bytes used", used, total);

    // Free pages
    println!("[demo] Free pages: {} ({}MB)", 
        page_alloc::free_count(),
        page_alloc::free_count() * page_alloc::PAGE_SIZE / 1024 / 1024);

    println!("[demo] alloc crate works!");
    println!();

    // v, s, boxed are dropped here — memory freed back to heap
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

    loop {
        unsafe { core::arch::asm!("wfe"); }
    }
}
