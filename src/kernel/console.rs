// ZerOS — Text Console & Kernel Log Buffer
//
// A 40×30 character console (using the 8×8 bitmap font on 320×240 display).
// Row 0 (`y = 0..8`) is reserved for the live system status bar.
//
// Also maintains a ring buffer of recent console lines so the interactive
// GUI Launcher (M7.1) can display kernel logs without screen tearing when
// background tasks call `println!()`.

#![allow(dead_code)]

use crate::drivers::display::{self, colors, framebuffer, ili9341, Color};
use crate::sync::IrqSafeLock;
use core::fmt;

/// Console columns (320 / 8)
const COLS: usize = 40;
/// Console rows (240 / 8)
const ROWS: u32 = 30;
/// Top row for scrolling console (row 0 is reserved for status bar)
const TOP_ROW: u32 = 1;

/// Number of log lines retained in history for the GUI Log Viewer
pub const LOG_HISTORY_LINES: usize = 18;

pub struct ConsoleState {
    initialized: bool,
    gui_mode: bool,
    cursor_col: u32,
    cursor_row: u32,
    fg: Color,
    bg: Color,
    history: [[u8; COLS]; LOG_HISTORY_LINES],
    history_len: [u8; LOG_HISTORY_LINES],
    history_head: usize,
    current_line: [u8; COLS],
    current_len: usize,
}

impl ConsoleState {
    const fn new() -> Self {
        Self {
            initialized: false,
            gui_mode: false,
            cursor_col: 0,
            cursor_row: TOP_ROW,
            fg: colors::GREEN,
            bg: colors::BLACK,
            history: [[b' '; COLS]; LOG_HISTORY_LINES],
            history_len: [0; LOG_HISTORY_LINES],
            history_head: 0,
            current_line: [b' '; COLS],
            current_len: 0,
        }
    }

    fn push_current_line(&mut self) {
        let idx = self.history_head;
        self.history[idx] = self.current_line;
        self.history_len[idx] = self.current_len as u8;
        self.history_head = (self.history_head + 1) % LOG_HISTORY_LINES;
        self.current_line = [b' '; COLS];
        self.current_len = 0;
    }

    pub fn write_byte(&mut self, byte: u8) {
        if !self.initialized {
            return;
        }

        // Always record into log history ring buffer
        match byte {
            b'\n' => {
                self.push_current_line();
            }
            b'\r' => {
                self.current_len = 0;
            }
            b => {
                if self.current_len < COLS {
                    self.current_line[self.current_len] = b;
                    self.current_len += 1;
                }
            }
        }

        // If the interactive GUI owns the main screen (`y = 8..240`),
        // don't overwrite GUI pixels with raw console characters.
        if self.gui_mode {
            return;
        }

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
                if self.cursor_col >= COLS as u32 {
                    self.cursor_col = 0;
                    self.cursor_row += 1;
                }
            }
        }

        if self.cursor_row >= ROWS {
            self.scroll();
        }
    }

    fn scroll(&mut self) {
        unsafe {
            if let Some(ref fb) = *core::ptr::addr_of!(framebuffer::FB) {
                let pitch_pixels = fb.pitch / 2;
                let row_pixels = 8 * pitch_pixels;

                let src = fb.buffer.offset(((TOP_ROW + 1) * row_pixels) as isize);
                let dst = fb.buffer.offset((TOP_ROW * row_pixels) as isize);
                let count = ((ROWS - TOP_ROW - 1) * row_pixels) as usize;
                core::ptr::copy(src, dst, count);

                let last_row_start = fb.buffer.offset(((ROWS - 1) * row_pixels) as isize);
                core::ptr::write_bytes(last_row_start, 0, row_pixels as usize);

                if let Some(ref ili) = *core::ptr::addr_of!(ili9341::DISPLAY) {
                    ili.flush_from_fb(fb, TOP_ROW * 8);
                }
            } else if let Some(ref ili) = *core::ptr::addr_of!(ili9341::DISPLAY) {
                ili.fill_rect(0, TOP_ROW * 8, ili9341::WIDTH, (ROWS - TOP_ROW) * 8, self.bg);
                self.cursor_row = TOP_ROW;
                return;
            }
        }

        self.cursor_row = ROWS - 1;
    }
}

impl fmt::Write for ConsoleState {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for byte in s.bytes() {
            self.write_byte(byte);
        }
        Ok(())
    }
}

static CONSOLE: IrqSafeLock<ConsoleState> = IrqSafeLock::new(ConsoleState::new());

/// Initialize the console and draw the initial status bar.
pub fn init() {
    display::clear(colors::BLACK);
    CONSOLE.with(|c| {
        c.initialized = true;
        c.gui_mode = false;
        c.cursor_col = 0;
        c.cursor_row = TOP_ROW;
    });
    update_status_bar(0, 1, "A");
}

/// Write formatted arguments to the console (called from `uart::_print`).
pub fn write_fmt(args: fmt::Arguments) {
    CONSOLE.with(|c| {
        if c.initialized {
            let _ = fmt::Write::write_fmt(c, args);
        }
    });
}

/// Enable or disable GUI mode (`true` suppresses raw console character drawing on rows 1..29).
pub fn set_gui_mode(enabled: bool) {
    CONSOLE.with(|c| {
        c.gui_mode = enabled;
    });
}

/// Copy the most recent `count` log lines (up to `LOG_HISTORY_LINES`) to a callback.
pub fn with_recent_logs<F: FnMut(usize, &str)>(count: usize, mut f: F) {
    CONSOLE.with(|c| {
        let n = count.min(LOG_HISTORY_LINES);
        for i in 0..n {
            let idx = (c.history_head + LOG_HISTORY_LINES - n + i) % LOG_HISTORY_LINES;
            let len = (c.history_len[idx] as usize).min(COLS);
            let s = core::str::from_utf8(&c.history[idx][..len]).unwrap_or("");
            f(i, s);
        }
    });
}

/// Render the top status bar (row 0, `y = 0..8`) when in text console mode.
pub fn update_status_bar(uptime_sec: u64, tasks: usize, slot: &str) {
    let is_gui = CONSOLE.with(|c| c.gui_mode);
    if is_gui {
        return;
    }
    use core::fmt::Write;
    let mut bar = alloc::string::String::with_capacity(40);
    let _ = write!(
        bar,
        " ZerOS v0.4 | Slot:{} | Tsk:{} | Up:{}s",
        slot, tasks, uptime_sec
    );

    display::fill_rect(0, 0, 320, 8, colors::BLUE);
    display::draw_str(0, 0, &bar, colors::WHITE, colors::BLUE);
}
