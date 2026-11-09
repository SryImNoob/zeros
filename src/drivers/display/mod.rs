// ZerOS — Unified Display & 2D Graphics Subsystem
//
// Abstracts over both the VideoCore GPU Framebuffer (used in QEMU) and the
// SPI0 ILI9341 320x240 TFT LCD (used on real Argon POD hardware), providing
// thread/IRQ-safe 2D drawing primitives for both the text console and the
// interactive GUI launcher (M7.1).

#![allow(dead_code)]

pub mod font;
pub mod framebuffer;
pub mod ili9341;

pub use framebuffer::{colors, Color};

use crate::kernel::interrupts;

/// Fill a solid rectangle `(x, y, w, h)` with RGB565 `color` on whichever display is active.
pub fn fill_rect(x: u32, y: u32, w: u32, h: u32, color: Color) {
    interrupts::without_interrupts(|| unsafe {
        if let Some(ref fb) = *core::ptr::addr_of!(framebuffer::FB) {
            fb.fill_rect(x, y, w, h, color);
        }
        if let Some(ref lcd) = *core::ptr::addr_of!(ili9341::DISPLAY) {
            lcd.fill_rect(x, y, w, h, color);
        }
    });
}

/// Clear the entire 320x240 display with `color`.
pub fn clear(color: Color) {
    fill_rect(0, 0, 320, 240, color);
}

/// Draw a 1-pixel or `thickness`-pixel rectangular border around `(x, y, w, h)`.
pub fn draw_rect_border(x: u32, y: u32, w: u32, h: u32, thickness: u32, color: Color) {
    if w == 0 || h == 0 || thickness == 0 {
        return;
    }
    // Top & bottom edges
    fill_rect(x, y, w, thickness, color);
    if h > thickness {
        fill_rect(x, y + h - thickness, w, thickness, color);
    }
    // Left & right edges
    fill_rect(x, y, thickness, h, color);
    if w > thickness {
        fill_rect(x + w - thickness, y, thickness, h, color);
    }
}

/// Draw a single 8x8 character at `(x, y)` with `fg` and `bg` colors.
pub fn draw_char(x: u32, y: u32, c: u8, fg: Color, bg: Color) {
    interrupts::without_interrupts(|| unsafe {
        if let Some(ref fb) = *core::ptr::addr_of!(framebuffer::FB) {
            fb.draw_char(x, y, c, fg, bg);
        }
        if let Some(ref lcd) = *core::ptr::addr_of!(ili9341::DISPLAY) {
            lcd.draw_char(x, y, c, fg, bg);
        }
    });
}

/// Draw an ASCII string at `(x, y)` using the 8x8 bitmap font.
pub fn draw_str(x: u32, y: u32, s: &str, fg: Color, bg: Color) {
    interrupts::without_interrupts(|| unsafe {
        if let Some(ref fb) = *core::ptr::addr_of!(framebuffer::FB) {
            fb.draw_str(x, y, s, fg, bg);
        }
        if let Some(ref lcd) = *core::ptr::addr_of!(ili9341::DISPLAY) {
            lcd.draw_str(x, y, s, fg, bg);
        }
    });
}

/// Draw a horizontal progress bar at `(x, y, w, h)` with `percent` (0..=100).
pub fn draw_progress_bar(
    x: u32,
    y: u32,
    w: u32,
    h: u32,
    percent: u32,
    fill_color: Color,
    bg_color: Color,
    border_color: Color,
) {
    let pct = percent.min(100);
    draw_rect_border(x, y, w, h, 1, border_color);
    if w <= 4 || h <= 4 {
        return;
    }
    let inner_w = w - 4;
    let inner_h = h - 4;
    let filled_w = (inner_w * pct) / 100;
    if filled_w > 0 {
        fill_rect(x + 2, y + 2, filled_w, inner_h, fill_color);
    }
    if filled_w < inner_w {
        fill_rect(x + 2 + filled_w, y + 2, inner_w - filled_w, inner_h, bg_color);
    }
}
