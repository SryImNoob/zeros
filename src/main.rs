// ZerOS — A bare-metal OS for Raspberry Pi Zero 2 W
//
// Hardware: RPi Zero 2 W (BCM2710A1, Cortex-A53) + Argon POD (ILI9341 2.8" LCD)
//
// M1: Minimal boot + UART ✓
// M2: Exception handling, interrupts, timer, GPIO ✓
// M3: Physical page allocator, MMU, kernel heap ✓
// M4: Framebuffer display + text console + SPI ILI9341 ✓
// M5: Button & IR input handling ✓
// M6.1: Multitasking & task scheduler ✓

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
mod input;
mod interrupts;
mod ir;
mod mailbox;
mod mmio;
mod mmu;
mod page_alloc;
mod spi;
mod task;
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

    // ── Stage 5: Input (buttons + IR) ──
    input::init();

    // ── Stage 6: Task Scheduler ──
    task::init();
    task::spawn("heartbeat", heartbeat_task);
    task::spawn("input_mon", input_monitor_task);
    task::spawn("self_test", self_test_task);

    // ── Stage 7: Timer (100Hz for responsive input & task wakeup) ──
    timer::init(100);

    // ── Stage 8: Enable interrupts ──
    interrupts::enable();
    println!("[OK] Interrupts enabled — timer ticking at 100Hz!");
    println!();
    println!("ZerOS is ready. Free pages: {} ({}MB), Active tasks: {}",
        page_alloc::free_count(),
        page_alloc::free_count() * page_alloc::PAGE_SIZE / 1024 / 1024,
        task::active_count());
    println!();

    // ── Idle task loop (Task 0) ──
    loop {
        task::yield_now();
        unsafe { core::arch::asm!("wfe"); }
    }
}

/// Task #1: Periodic heartbeat (1Hz) — toggles LED and prints system status
fn heartbeat_task() {
    let mut seconds = 0u64;
    loop {
        task::sleep_ms(1000);
        seconds += 1;
        gpio::toggle_activity_led();
        println!("[TICK] {}s uptime (active tasks: {})", seconds, task::active_count());
        if seconds == 2 {
            task::print_tasks();
        }
    }
}

/// Task #2: Input event monitor — processes button presses and IR remote codes
fn input_monitor_task() {
    loop {
        while let Some(event) = input::poll_event() {
            match event {
                input::InputEvent::ButtonPress(btn) => {
                    println!("[BTN] Button {} pressed", btn + 1);
                }
                input::InputEvent::ButtonRelease(btn) => {
                    println!("[BTN] Button {} released", btn + 1);
                }
                input::InputEvent::IrCode(code) => {
                    println!("[IR] Code: 0x{:04X}", code);
                }
            }
        }
        task::sleep_ms(20);
    }
}

/// Task #3: Background self-test task — runs two steps and exits cleanly
fn self_test_task() {
    println!("[TASK] self_test: starting background check...");
    task::sleep_ms(500);

    // Test dynamic allocation from a background task stack
    let mut vec = alloc::vec::Vec::new();
    for i in 1..=10u32 {
        vec.push(i * i);
    }
    let sum: u32 = vec.iter().sum();
    println!("[TASK] self_test: heap check OK (sum of squares 1..10 = {})", sum);

    task::sleep_ms(500);
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
