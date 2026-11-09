// ZerOS — A bare-metal OS for Raspberry Pi Zero 2 W
//
// Hardware: RPi Zero 2 W (BCM2710A1, Cortex-A53) + Argon POD (ILI9341 2.8" LCD)
//
// M1: Minimal boot + UART ✓
// M2: Exception handling, interrupts, timer, GPIO ✓
// M3: Physical page allocator, MMU, kernel heap ✓
// M4: Framebuffer display + text console + SPI ILI9341 ✓
// M5: Button & IR input handling ✓
// M6: Task scheduler, SD/FAT32 filesystem, Tryboot A/B ✓

#![no_std]
#![no_main]

extern crate alloc;

mod boot;
mod console;
mod exception;
mod fat32;
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
mod sd;
mod spi;
mod task;
mod timer;
mod tryboot;
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

    // ── Stage 6: SD Card + FAT32 Filesystem + Tryboot A/B ──
    if sd::init() {
        if fat32::init() {
            let entries = fat32::list_root();
            println!("[OK] FAT32 root directory ({} files):", entries.len());
            for entry in &entries {
                println!("     - {:<14} {:>7} bytes (cluster {})",
                    entry.name, entry.size, entry.first_cluster);
            }
            if let Some(ver_bytes) = fat32::read_file("VERSION.TXT") {
                if let Ok(ver_str) = core::str::from_utf8(&ver_bytes) {
                    println!("[OK] Read VERSION.TXT: {}", ver_str.trim());
                }
            }
        }
    }
    tryboot::init();

    // ── Stage 7: Task Scheduler ──
    task::init();
    task::spawn("heartbeat", heartbeat_task);
    task::spawn("input_mon", input_monitor_task);
    task::spawn("self_test", self_test_task);

    // ── Stage 8: Timer (100Hz for responsive input & task wakeup) ──
    timer::init(100);

    // ── Stage 9: Enable interrupts ──
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

/// Task #1: Periodic heartbeat (1Hz) — toggles LED, updates top status bar, prints status
fn heartbeat_task() {
    let mut seconds = 0u64;
    loop {
        task::sleep_ms(1000);
        seconds += 1;
        gpio::toggle_activity_led();

        let slot_str = match tryboot::status().active_slot {
            tryboot::BootSlot::SlotA => "A",
            tryboot::BootSlot::SlotB => "B",
        };
        let active = task::active_count();
        console::update_status_bar(seconds, active, slot_str);

        println!("[TICK] {}s uptime (slot: {}, tasks: {})", seconds, slot_str, active);
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

/// Task #3: Background self-test task — verifies heap & Tryboot A/B persistence
fn self_test_task() {
    println!("[TASK] self_test: starting background check...");
    task::sleep_ms(500);

    // 1. Test dynamic allocation from a background task stack
    let mut vec = alloc::vec::Vec::new();
    for i in 1..=10u32 {
        vec.push(i * i);
    }
    let sum: u32 = vec.iter().sum();
    println!("[TASK] self_test: heap check OK (sum of squares 1..10 = {})", sum);

    // 2. Test FAT32 + Tryboot A/B configuration update round-trip
    if fat32::is_mounted() {
        let partitions = fat32::partitions();
        let part_count = partitions.iter().filter(|p| p.is_some()).count();
        println!("[TASK] self_test: verified {} MBR partitions on SD card", part_count);

        // Test committing Slot A as default (default=1, tryboot=2)
        if tryboot::set_default_partition(1, 2) {
            println!("[TASK] self_test: Tryboot A/B commit test OK");
        }
    }

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
