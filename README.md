# ZerOS 🚀

A bare-metal operating system for **Raspberry Pi Zero 2 W** with **Argon POD** (ILI9341 2.8" LCD), written in **Rust**.

## Hardware

- **SoC:** BCM2710A1 (4× ARM Cortex-A53 @ 1GHz, AArch64)
- **RAM:** 512 MB
- **Display:** 2.8" TFT LCD (320×240, ILI9341, SPI)
- **Case:** Argon POD (4 buttons + IR receiver)

## Build

```bash
# Prerequisites
rustup target add aarch64-unknown-none
rustup component add rust-src llvm-tools-preview
cargo install cargo-binutils
brew install qemu

# Build kernel8.img
make build

# Run in QEMU
make qemu

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
    ├── boot.rs             # AArch64 boot assembly stub
    ├── mmio.rs             # Memory-mapped I/O helpers
    └── uart.rs             # PL011 UART driver (serial debug)
```

## Roadmap

- [x] M1: Minimal boot + UART output
- [ ] M2: Interrupts, timers, MMIO framework
- [ ] M3: Memory management (MMU, page allocator)
- [ ] M4: SPI + ILI9341 display driver
- [ ] M5: Button & IR input handling
- [ ] M6: Task scheduler, filesystem, tryboot A/B

## License

MIT
