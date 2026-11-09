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
use crate::ili9341;
use core::fmt;

/// Console columns (320 / 8)
const COLS: u32 = 40;
/// Console rows (240 / 8)
const ROWS: u32 = 30;
/// Top row for scrolling console (row 0 is reserved for status bar)
const TOP_ROW: u32 = 1;

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
        cursor_row: TOP_ROW,
        fg: colors::GREEN,
        bg: colors::BLACK,
    };

    // Clear screen and draw initial status bar
    unsafe {
        if let Some(ref fb) = *core::ptr::addr_of!(framebuffer::FB) {
            fb.clear(colors::BLACK);
        }
        if let Some(ref ili) = *core::ptr::addr_of!(ili9341::DISPLAY) {
            ili.clear(colors::BLACK);
        }
    }

    unsafe { CONSOLE = Some(console); }
    update_status_bar(0, 1, "A");
}

/// Render the top status bar (row 0, y=0..8)
pub fn update_status_bar(uptime_sec: u64, tasks: usize, slot: &str) {
    use core::fmt::Write;
    let mut bar = alloc::string::String::with_capacity(40);
    let _ = write!(
        bar,
        " ZerOS v0.1 | Slot:{} | Tsk:{} | Up:{}s",
        slot, tasks, uptime_sec
    );

    crate::interrupts::without_interrupts(|| unsafe {
        if let Some(ref fb) = *core::ptr::addr_of!(framebuffer::FB) {
            fb.fill_rect(0, 0, fb.width, 8, colors::BLUE);
            fb.draw_str(0, 0, &bar, colors::WHITE, colors::BLUE);
        }
        if let Some(ref ili) = *core::ptr::addr_of!(ili9341::DISPLAY) {
            ili.fill_rect(0, 0, ili9341::WIDTH, 8, colors::BLUE);
            ili.draw_str(0, 0, &bar, colors::WHITE, colors::BLUE);
        }
    });
}

impl Console {
    /// Write a single byte to the console
    pub fn write_byte(&mut self, byte: u8) {
        let fb_opt = unsafe { (&*core::ptr::addr_of!(framebuffer::FB)).as_ref() };
        let ili_opt = unsafe { (&*core::ptr::addr_of!(ili9341::DISPLAY)).as_ref() };
        if fb_opt.is_none() && ili_opt.is_none() {
            return;
        }

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
                if let Some(fb) = fb_opt {
                    fb.draw_char(x, y, byte, self.fg, self.bg);
                }
                if let Some(ili) = ili_opt {
                    ili.draw_char(x, y, byte, self.fg, self.bg);
                }

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
        unsafe {
            if let Some(ref fb) = *core::ptr::addr_of!(framebuffer::FB) {
                let pitch_pixels = fb.pitch / 2; // pitch is in bytes, we use u16
                let row_pixels = 8 * pitch_pixels; // pixels per text row

                // Copy rows (TOP_ROW+1)..ROWS to TOP_ROW..(ROWS-1)
                let src = fb.buffer.offset(((TOP_ROW + 1) * row_pixels) as isize);
                let dst = fb.buffer.offset((TOP_ROW * row_pixels) as isize);
                let count = ((ROWS - TOP_ROW - 1) * row_pixels) as usize;
                core::ptr::copy(src, dst, count);

                // Clear the last row
                let last_row_start = fb.buffer.offset(((ROWS - 1) * row_pixels) as isize);
                core::ptr::write_bytes(last_row_start, 0, row_pixels as usize);

                // If SPI ILI9341 display is also active, flush scrolled rows from RAM FB
                if let Some(ref ili) = *core::ptr::addr_of!(ili9341::DISPLAY) {
                    ili.flush_from_fb(fb, TOP_ROW * 8);
                }
            } else if let Some(ref ili) = *core::ptr::addr_of!(ili9341::DISPLAY) {
                // SPI-only fallback: clear console region and wrap to TOP_ROW
                ili.fill_rect(0, TOP_ROW * 8, ili9341::WIDTH, (ROWS - TOP_ROW) * 8, self.bg);
                self.cursor_row = TOP_ROW;
                return;
            }
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
