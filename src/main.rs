// ZerOS — A Bare-Metal Operating System for Raspberry Pi Zero 2 W + Argon POD
//
// Architecture (M7.0 Modular Kernel Layout):
//   - `arch/`    : AArch64 CPU boot, exception vectors, MMU page tables, generic timer
//   - `bsp/`     : Board Support Package (BCM2710A1 memory map & Argon POD GPIO pinout)
//   - `mm/`      : Physical page frame allocator & free-list kernel heap (`GlobalAlloc`)
//   - `drivers/` : Hardware drivers (UART, GPIO, SPI0, Mailbox, ILI9341/FB, EMMC SD, Buttons/IR/Touch)
//   - `kernel/`  : Core OS services (IRQ routing, multitasking scheduler, console, Tryboot A/B)
//   - `fs/`      : MBR partition parser & FAT32 filesystem
//   - `sync/`    : Interrupt-safe synchronization primitives (`IrqSafeLock`)
//   - `ui/`      : Interactive 4-Tab Card Desktop / Launcher (M7.1)

#![no_std]
#![no_main]

extern crate alloc;

#[macro_use]
pub mod drivers;

pub mod arch;
pub mod bsp;
pub mod fs;
pub mod kernel;
pub mod mm;
pub mod sync;
pub mod ui;

use core::panic::PanicInfo;

use arch::{mmu, timer};
use drivers::block::sd;
use drivers::display::{framebuffer, ili9341};
use drivers::{gpio, input, spi, uart};
use fs::fat32;
use kernel::{console, interrupts, task, tryboot};
use mm::{heap, page_alloc};

/// Kernel entry point
#[no_mangle]
pub extern "C" fn kernel_main() -> ! {
    // ── Stage 0: Arm hardware watchdog (15s auto-recovery on real hardware) ──
    if !bsp::is_qemu() {
        tryboot::watchdog_start(15);
    }

    // ── Stage 1: UART ──
    uart::init();
    println!();
    println!("=================================");
    println!("  ZerOS v0.2.0 (M7.1 GUI)");
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
    let display_ok = if bsp::is_qemu() {
        framebuffer::init(320, 240)
    } else {
        spi::init();
        let ok = ili9341::init();
        tryboot::watchdog_feed(15);
        ok
    };
    if display_ok {
        console::init();
        println!("== ZerOS v0.2.0 (AArch64 Bare-Metal) ==");
        println!("[OK] Console: 40x30 text + status bar");
    }

    // ── Stage 5: Input (4 buttons + NEC IR + Resistive Touch) ──
    input::init();

    // ── Stage 6: SD Card + FAT32 Filesystem + Tryboot A/B ──
    if sd::init() {
        if !bsp::is_qemu() {
            tryboot::watchdog_feed(15);
        }
        if fat32::init() {
            let entries = fat32::list_root();
            println!("[OK] FAT32 root directory ({} files):", entries.len());
            for entry in entries.iter().take(5) {
                println!(
                    "     - {:<14} {:>7} bytes (cluster {})",
                    entry.name, entry.size, entry.first_cluster
                );
            }
            if let Some(ver_bytes) = fat32::read_file("VERSION.TXT") {
                if let Ok(ver_str) = core::str::from_utf8(&ver_bytes) {
                    println!("[OK] Read VERSION.TXT: {}", ver_str.trim());
                }
            }
        }
    }
    tryboot::init();

    // ── Stage 7: Task Scheduler & Interactive GUI (M7.1) ──
    task::init();
    task::spawn("heartbeat", heartbeat_task);
    task::spawn("ui_desktop", ui::ui_task);
    task::spawn("self_test", self_test_task);

    // ── Stage 8: Timer (100Hz for responsive input & task wakeup) ──
    timer::init(100);

    // ── Stage 9: Enable interrupts ──
    interrupts::enable();
    println!("[OK] Interrupts enabled — timer ticking at 100Hz!");
    println!(
        "ZerOS ready: {} free pages ({}MB), {} tasks",
        page_alloc::free_count(),
        page_alloc::free_count() * page_alloc::PAGE_SIZE / 1024 / 1024,
        task::active_count()
    );

    // ── Idle task loop (Task 0) ──
    loop {
        task::yield_now();
        unsafe {
            core::arch::asm!("wfe");
        }
    }
}

/// Task #1: Periodic heartbeat (1Hz) — feeds watchdog, toggles LED, updates top status bar
fn heartbeat_task() {
    let mut seconds = 0u64;
    loop {
        task::sleep_ms(1000);
        seconds += 1;
        if !bsp::is_qemu() {
            tryboot::watchdog_feed(15);
        }
        gpio::toggle_activity_led();

        let slot_str = match tryboot::status().active_slot {
            tryboot::BootSlot::SlotA => "A",
            tryboot::BootSlot::SlotB => "B",
        };
        let active = task::active_count();
        console::update_status_bar(seconds, active, slot_str);

        println!(
            "[TICK] {}s uptime (slot: {}, tasks: {})",
            seconds, slot_str, active
        );

        // Auto-return safety net after 120s on real hardware (or press Button 4 anytime)
        if !bsp::is_qemu() && seconds >= 120 {
            println!("[SYS] 120s tryboot session complete -> rebooting to RPi OS...");
            tryboot::reboot_to_partition(0);
        }
    }
}

/// Task #3: Background self-test task — verifies heap & Tryboot A/B persistence
fn self_test_task() {
    println!("[TASK] self_test: starting background check...");
    task::sleep_ms(500);

    let mut vec = alloc::vec::Vec::new();
    for i in 1..=10u32 {
        vec.push(i * i);
    }
    let sum: u32 = vec.iter().sum();
    println!(
        "[TASK] self_test: heap check OK (sum of squares 1..10 = {})",
        sum
    );

    if fat32::is_mounted() {
        let partitions = fat32::partitions();
        let part_count = partitions.iter().filter(|p| p.is_some()).count();
        println!(
            "[TASK] self_test: verified {} MBR partitions on SD card",
            part_count
        );

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
    console::set_gui_mode(false);
    println!();
    println!("!!! KERNEL PANIC !!!");
    if let Some(location) = info.location() {
        println!(
            "  at {}:{}:{}",
            location.file(),
            location.line(),
            location.column()
        );
    }
    if let Some(msg) = info.message().as_str() {
        println!("  {}", msg);
    }
    loop {
        unsafe {
            core::arch::asm!("wfe");
        }
    }
}
