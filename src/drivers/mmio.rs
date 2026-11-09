// ZerOS — MMIO (Memory-Mapped I/O) helpers
// BCM2710 peripheral base: 0x3F000000

#![allow(dead_code)]

use core::ptr;

/// BCM2710 peripheral base address (same as BCM2837 / Pi 3)
pub const PERIPHERAL_BASE: usize = 0x3F00_0000;

/// GPIO base address
pub const GPIO_BASE: usize = PERIPHERAL_BASE + 0x0020_0000;

/// UART0 (PL011) base address
pub const UART0_BASE: usize = PERIPHERAL_BASE + 0x0020_1000;

/// SPI0 base address (for ILI9341 display)
pub const SPI0_BASE: usize = PERIPHERAL_BASE + 0x0020_4000;

/// Mailbox base address (for framebuffer etc.)
pub const MAILBOX_BASE: usize = PERIPHERAL_BASE + 0x0000_B880;

/// Volatile MMIO write
#[inline(always)]
pub unsafe fn write(reg: usize, val: u32) {
    ptr::write_volatile(reg as *mut u32, val);
}

/// Volatile MMIO read
#[inline(always)]
pub unsafe fn read(reg: usize) -> u32 {
    ptr::read_volatile(reg as *const u32)
}
