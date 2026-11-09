// ZerOS — Text Console on Framebuffer
//
// A VGA-style text console that renders characters using the 8×8 font.
// Supports cursor tracking, newlines, and scrolling.
//
// Console dimensions for 320×240 @ 8×8 font:
//   40 columns × 30 rows
//
// Integration: uart::_print() writes to both UART and console,
// so all println!() output appears on serial AND screen.

use crate::framebuffer::{self, Color, colors};
use core::fmt;

/// Console columns (320 / 8)
const COLS: u32 = 40;
/// Console rows (240 / 8)
const ROWS: u32 = 30;

/// Global console instance
pub static mut CONSOLE: Option<Console> = None;

pub struct Console {
    cursor_col: u32,
    cursor_row: u32,
    fg: Color,
    bg: Color,
}

/// Initialize the console (call after framebuffer::init)
pub fn init() {
    let console = Console {
        cursor_col: 0,
        cursor_row: 0,
        fg: colors::GREEN,
        bg: colors::BLACK,
    };

    // Clear screen
    unsafe {
        if let Some(ref fb) = *core::ptr::addr_of!(framebuffer::FB) {
            fb.clear(colors::BLACK);
        }
    }

    unsafe { CONSOLE = Some(console); }
}

impl Console {
    /// Write a single byte to the console
    pub fn write_byte(&mut self, byte: u8) {
        let fb = match unsafe { &*core::ptr::addr_of!(framebuffer::FB) } {
            Some(fb) => fb,
            None => return,
        };

        match byte {
            b'\n' | b'\r' => {
                if byte == b'\n' {
                    self.cursor_col = 0;
                    self.cursor_row += 1;
                }
                if byte == b'\r' {
                    self.cursor_col = 0;
                }
            }
            byte => {
                let x = self.cursor_col * 8;
                let y = self.cursor_row * 8;
                fb.draw_char(x, y, byte, self.fg, self.bg);

                self.cursor_col += 1;
                if self.cursor_col >= COLS {
                    self.cursor_col = 0;
                    self.cursor_row += 1;
                }
            }
        }

        // Scroll if needed
        if self.cursor_row >= ROWS {
            self.scroll();
        }
    }

    /// Scroll the screen up by one line
    fn scroll(&mut self) {
        let fb = match unsafe { &*core::ptr::addr_of!(framebuffer::FB) } {
            Some(fb) => fb,
            None => return,
        };

        unsafe {
            let pitch_pixels = fb.pitch / 2; // pitch is in bytes, we use u16
            let row_pixels = 8 * pitch_pixels; // pixels per text row

            // Copy rows 1..ROWS to 0..ROWS-1
            let src = fb.buffer.offset(row_pixels as isize);
            let dst = fb.buffer;
            let count = ((ROWS - 1) * row_pixels) as usize;
            core::ptr::copy(src, dst, count);

            // Clear the last row
            let last_row_start = fb.buffer.offset(((ROWS - 1) * row_pixels) as isize);
            core::ptr::write_bytes(last_row_start, 0, row_pixels as usize);
        }

        self.cursor_row = ROWS - 1;
    }
}

impl fmt::Write for Console {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for byte in s.bytes() {
            self.write_byte(byte);
        }
        Ok(())
    }
}
