# ZerOS 🚀

A bare-metal operating system for **Raspberry Pi Zero 2 W** with **Argon POD** (ILI9341 2.8" LCD), written in **Rust** (`#![no_std]`, `aarch64-unknown-none`).

## Hardware

- **SoC:** BCM2710A1 (4× ARM Cortex-A53 @ 1GHz, AArch64)
- **RAM:** 512 MB (1GB emulated in QEMU)
- **Display:** 2.8" TFT LCD (320×240, ILI9341, SPI0)
- **Case:** Argon POD (4 hardware buttons + NEC IR receiver)

## Build & Run

```bash
# Prerequisites
rustup target add aarch64-unknown-none
rustup component add rust-src llvm-tools-preview
brew install qemu

# Build kernel8.img
make build

# Build kernel8.img + dual-partition FAT32 SD card image (sdcard.img)
make sdimage

# Run in QEMU (headless UART + FAT32 SD card)
make qemu

# Run in QEMU with graphical framebuffer window (320×240 + status bar)
make qemu-display

# Deploy to physical SD card
make sdcard
```

## Project Structure

```
zeros/
├── Cargo.toml              # Rust package manifest
├── Makefile                # Build, SD image, QEMU & deploy targets
├── linker.ld               # Linker script (load at 0x80000)
├── scripts/
│   └── mk_sdcard.py        # Pure-Python dual-partition MBR + FAT32 image builder
├── .cargo/
│   └── config.toml         # Cargo target & linker config
└── src/
    ├── main.rs             # Kernel entry point, tasks & panic handler
    ├── boot.rs             # AArch64 boot assembly (EL3→EL2→EL1, FPU enable)
    ├── mmio.rs             # Memory-mapped I/O helpers
    ├── uart.rs             # PL011 UART driver + dual-output print!/println!
    ├── exception.rs        # Exception vector table & context save/restore
    ├── interrupts.rs       # ARM local interrupt controller (QA7)
    ├── timer.rs            # Generic timer (100Hz tick)
    ├── gpio.rs             # Activity LED (GPIO 29)
    ├── page_alloc.rs       # Physical page frame allocator (O(1) free-list)
    ├── mmu.rs              # MMU setup (identity mapping, 4KB granule, 2MB blocks)
    ├── heap.rs             # Kernel heap allocator (256KB, GlobalAlloc)
    ├── mailbox.rs          # VideoCore property mailbox interface (channel 8)
    ├── framebuffer.rs      # GPU framebuffer (320×240 RGB565, QEMU/HDMI)
    ├── font.rs             # 8×8 bitmap font (ASCII)
    ├── console.rs          # Text console (40×30, top status bar, scrolling)
    ├── spi.rs              # BCM2710 SPI0 master driver
    ├── ili9341.rs          # ILI9341 TFT display driver (Argon POD pinout)
    ├── input.rs            # 4-button input (debounced + ring-buffer event queue)
    ├── ir.rs               # NEC infrared protocol state-machine decoder
    ├── task.rs             # Round-robin task scheduler (__switch_context, 16KB stacks)
    ├── sd.rs               # BCM2710 Arasan EMMC / SDHC sector read/write driver
    ├── fat32.rs            # MBR partition parser + FAT32 filesystem (read/write)
    └── tryboot.rs          # Tryboot A/B fail-safe boot & PM hardware watchdog
```

## Roadmap

- [x] **M1:** Minimal boot + UART output
- [x] **M2:** Exception vectors, IRQ controller, 100Hz generic timer, GPIO LED
- [x] **M3:** Memory management (page allocator, MMU + caches, kernel heap `alloc`)
- [x] **M4:** Display (GPU framebuffer + ILI9341 SPI driver + 40×30 text console)
- [x] **M5:** Button & NEC IR remote input handling
- [x] **M6:** Multitasking scheduler, SD/FAT32 filesystem, Tryboot A/B + watchdog

## Boot Sequence

```
EL3 → EL2 → EL1 (enable FPU, install VBAR_EL1)
  → Stage 1: UART0 init (115200 8N1)
  → Stage 2: GPIO init (Activity LED GPIO 29)
  → Stage 3: Memory (256KB Heap → Page Allocator → MMU + L1/L2 caches)
  → Stage 4: Display (GPU Framebuffer in QEMU / SPI0 ILI9341 on Argon POD)
  → Stage 5: Input (Buttons GPIO 16,20,21,26 + IR Receiver GPIO 23)
  → Stage 6: Storage & Boot (Arasan EMMC SD → MBR → FAT32 → Tryboot A/B)
  → Stage 7: Scheduler (Task #0 idle, #1 heartbeat, #2 input_mon, #3 self_test)
  → Stage 8: Generic Timer (100Hz tick)
  → Stage 9: Enable IRQs → Multitasking loop
```

## License

MIT
