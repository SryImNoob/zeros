# ZerOS 🚀

A bare-metal operating system for **Raspberry Pi Zero 2 W** with **Argon POD** (ILI9341 2.8" LCD), written in **Rust**.

## Hardware

- **SoC:** BCM2710A1 (4× ARM Cortex-A53 @ 1GHz, AArch64)
- **RAM:** 512 MB (1GB emulated in QEMU)
- **Display:** 2.8" TFT LCD (320×240, ILI9341, SPI)
- **Case:** Argon POD (4 buttons + IR receiver)

## Build

```bash
# Prerequisites
rustup target add aarch64-unknown-none
rustup component add rust-src llvm-tools-preview
brew install qemu

# Build kernel8.img
make build

# Run in QEMU (UART only)
make qemu

# Run in QEMU with display window
make qemu-display

# Deploy to SD card
make sdcard
```

## Project Structure

```
zeros/
├── Cargo.toml              # Rust package manifest
├── Makefile                 # Build & run shortcuts
├── linker.ld               # Linker script (load at 0x80000)
├── .cargo/
│   └── config.toml         # Cargo target & linker config
└── src/
    ├── main.rs             # Kernel entry point + panic handler
    ├── boot.rs             # AArch64 boot assembly (EL3→EL1)
    ├── mmio.rs             # Memory-mapped I/O helpers
    ├── uart.rs             # PL011 UART driver + print macros
    ├── exception.rs        # Exception vector table
    ├── interrupts.rs       # ARM local interrupt controller
    ├── timer.rs            # Generic timer (100Hz tick)
    ├── gpio.rs             # Activity LED (GPIO 29)
    ├── page_alloc.rs       # Physical page frame allocator
    ├── mmu.rs              # MMU setup (identity mapping, 4KB/2MB)
    ├── heap.rs             # Kernel heap allocator (256KB)
    ├── mailbox.rs          # VideoCore mailbox interface
    ├── framebuffer.rs      # GPU framebuffer (QEMU/HDMI)
    ├── font.rs             # 8×8 bitmap font (ASCII)
    ├── console.rs          # Text console (40×30, scrolling)
    ├── spi.rs              # BCM2710 SPI0 master driver
    ├── ili9341.rs          # ILI9341 TFT display driver
    ├── input.rs            # Button input (debounced, events)
    └── ir.rs               # NEC IR protocol decoder
```

## Roadmap

- [x] M1: Minimal boot + UART output
- [x] M2: Interrupts, timers, GPIO
- [x] M3: Memory management (page allocator, MMU, heap)
- [x] M4: Display (GPU framebuffer + ILI9341 SPI + text console)
- [x] M5: Button & IR input handling
- [ ] M6: Task scheduler, filesystem, tryboot A/B

## Boot Sequence

```
EL3 → EL2 → EL1
  → UART init
  → GPIO init (activity LED)
  → Memory: heap → page allocator → MMU
  → Display: GPU framebuffer (QEMU) or SPI ILI9341 (real hw)
  → Console: 40×30 text, dual output (UART + screen)
  → Input: 4 buttons + IR receiver
  → Timer: 100Hz tick (button polling, heartbeat)
  → Enable interrupts → Main loop (WFE)
```

## License

MIT
