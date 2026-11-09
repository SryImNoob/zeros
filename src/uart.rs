// ZerOS — UART driver (PL011)
// Used for serial debug output via QEMU

use crate::mmio;

const UART_DR: usize = mmio::UART0_BASE;           // Data register
const UART_FR: usize = mmio::UART0_BASE + 0x18;    // Flag register
const UART_IBRD: usize = mmio::UART0_BASE + 0x24;  // Integer baud rate divisor
const UART_FBRD: usize = mmio::UART0_BASE + 0x28;  // Fractional baud rate divisor
const UART_LCRH: usize = mmio::UART0_BASE + 0x2C;  // Line control register
const UART_CR: usize = mmio::UART0_BASE + 0x30;    // Control register
const UART_ICR: usize = mmio::UART0_BASE + 0x44;   // Interrupt clear register

/// Flag register bits
const FR_TXFF: u32 = 1 << 5;  // Transmit FIFO full

/// Initialize UART0 (PL011)
pub fn init() {
    unsafe {
        // Disable UART
        mmio::write(UART_CR, 0);

        // Clear all interrupts
        mmio::write(UART_ICR, 0x7FF);

        // Set baud rate: 115200 @ 48MHz clock
        // Divider = 48000000 / (16 * 115200) = 26.0417
        // Integer part = 26, Fractional part = 0.0417 * 64 + 0.5 = 3
        mmio::write(UART_IBRD, 26);
        mmio::write(UART_FBRD, 3);

        // 8-bit, no parity, FIFO enabled
        mmio::write(UART_LCRH, (1 << 4) | (1 << 5) | (1 << 6));

        // Enable UART, TX and RX
        mmio::write(UART_CR, (1 << 0) | (1 << 8) | (1 << 9));
    }
}

/// Send a single character
pub fn putc(c: u8) {
    unsafe {
        // Wait until TX FIFO is not full
        while mmio::read(UART_FR) & FR_TXFF != 0 {
            core::hint::spin_loop();
        }
        mmio::write(UART_DR, c as u32);
    }
}

/// Send a string
pub fn puts(s: &str) {
    for byte in s.bytes() {
        if byte == b'\n' {
            putc(b'\r');
        }
        putc(byte);
    }
}
