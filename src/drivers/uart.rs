// ZerOS — UART driver (PL011) + print macros
//
// Provides serial debug output via the PL011 UART.
// In QEMU: output appears on `-serial stdio`
// On real hardware: accessible via mini-HDMI or USB serial (if configured)

use crate::drivers::mmio;
use core::fmt;

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
        // Disable UART during configuration
        mmio::write(UART_CR, 0);

        // Clear all interrupts
        mmio::write(UART_ICR, 0x7FF);

        // Set baud rate: 115200 @ 48MHz clock
        mmio::write(UART_IBRD, 26);
        mmio::write(UART_FBRD, 3);

        // 8-bit word length, FIFO enabled, no parity
        mmio::write(UART_LCRH, (1 << 4) | (1 << 5) | (1 << 6));

        // Enable UART, TX and RX
        mmio::write(UART_CR, (1 << 0) | (1 << 8) | (1 << 9));
    }
}

/// Send a single character (with timeout so stalled/unrouted UART never blocks boot)
pub fn putc(c: u8) {
    unsafe {
        let mut timeout = 2000u32;
        while mmio::read(UART_FR) & FR_TXFF != 0 {
            timeout -= 1;
            if timeout == 0 {
                return;
            }
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

// ─── fmt::Write support for print!/println! macros ───

/// Writer type for UART output
pub struct Writer;

impl fmt::Write for Writer {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        puts(s);
        Ok(())
    }
}

/// Internal print helper — used by print! and println! macros.
/// Writes to UART and (if initialized) the screen console.
#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    crate::kernel::interrupts::without_interrupts(|| {
        use fmt::Write;
        // Always write to UART
        Writer.write_fmt(args).unwrap();
        // Also write to screen console / log history
        crate::kernel::console::write_fmt(args);
    });
}

/// Print formatted text to UART (no newline)
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {
        $crate::drivers::uart::_print(format_args!($($arg)*))
    };
}

/// Print formatted text to UART (with newline)
#[macro_export]
macro_rules! println {
    () => { $crate::print!("\n") };
    ($fmt:expr) => {
        $crate::drivers::uart::_print(format_args!(concat!($fmt, "\n")))
    };
    ($fmt:expr, $($arg:tt)*) => {{
        $crate::drivers::uart::_print(format_args!($fmt, $($arg)*));
        $crate::drivers::uart::_print(format_args!("\n"));
    }};
}
