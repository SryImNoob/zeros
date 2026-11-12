// ZerOS — 2D Off-Screen Canvas, Anti-Aliased CJK Font Engine & Compositor (M9)
//
// Provides:
//   - `Canvas`: 320x240 RGB565 off-screen framebuffer (150 KB) with dirty-rectangle tracking
//   - Anti-aliased 2-bit (4-level alpha) Chinese character rendering (`15x15` and `12x12`)
//     backed by a 450-glyph built-in kernel dictionary (`cjk_font_data.rs`) AND full
//     21,030-glyph Unicode CJK (`U+4E00..U+9FA5`) on-demand `O(1)` sector streaming
//     from `/FONT16.BIN` on the FAT32 SD card
//   - Rounded cards with translucent drop shadows, gradients, Bresenham lines,
//     circular ring gauges, and mixed UTF-8 CJK + ASCII text layout.

#![allow(dead_code)]

pub mod cjk_font_data;

use alloc::vec;
use alloc::vec::Vec;
use crate::drivers::block::sd::SECTOR_SIZE;
use crate::drivers::display::{self, colors, font, Color};
use crate::fs::fat32;
use cjk_font_data::{
    BUILTIN_CJK_CODEPOINTS, BUILTIN_CJK_GLYPHS_12, BUILTIN_CJK_GLYPHS_15,
};

pub const SCREEN_W: i32 = 320;
pub const SCREEN_H: i32 = 240;

// ─── External FONT16.BIN Sector Cache & O(1) Cluster Chain ───

struct FontFileState {
    clusters: Vec<u32>,
    /// 96 cached 512-byte sectors (= 768 CJK glyphs cached in RAM)
    cache: Vec<(usize, [u8; SECTOR_SIZE])>,
    next_evict: usize,
}

static mut FONT_STATE: Option<FontFileState> = None;

/// Initialize the CJK font subsystem by indexing `/FONT16.BIN` on the FAT32 partition if present.
pub fn init() {
    if !fat32::is_mounted() {
        return;
    }
    if let Some(entry) = fat32::resolve_path("/FONT16.BIN") {
        let clusters = fat32::build_cluster_chain(&entry);
        crate::println!(
            "[OK] GFX: Indexed /FONT16.BIN ({} KB, {} clusters) for full Unicode CJK",
            entry.size / 1024,
            clusters.len()
        );
        unsafe {
            FONT_STATE = Some(FontFileState {
                clusters,
                cache: Vec::with_capacity(96),
                next_evict: 0,
            });
        }
    } else {
        crate::println!(
            "[INFO] GFX: /FONT16.BIN not found on SD; using built-in {} CJK glyphs",
            cjk_font_data::BUILTIN_CJK_COUNT
        );
    }
}

/// Map a Unicode BMP codepoint to its glyph index (0..21030) in `FONT16.BIN`.
fn font16_glyph_index(cp: u16) -> Option<usize> {
    if (0x4E00..=0x9FA5).contains(&cp) {
        return Some(128 + (cp - 0x4E00) as usize);
    }
    if (0x3000..=0x303F).contains(&cp) {
        return Some((cp - 0x3000) as usize);
    }
    if (0xFF00..=0xFF2F).contains(&cp) {
        return Some(64 + (cp - 0xFF00) as usize);
    }
    let special: [u16; 16] = [
        0xFF1A, 0xFF1B, 0xFF1F, 0xFF01, 0xFF08, 0xFF09, 0xFF0C, 0x3001,
        0x00B7, 0x2014, 0x2026, 0x2018, 0x2019, 0x201C, 0x201D, 0x266A,
    ];
    for (i, &sc) in special.iter().enumerate() {
        if sc == cp {
            return Some(112 + i);
        }
    }
    None
}

/// Look up a `15x15` 2-bit anti-aliased glyph (64 bytes) for Unicode codepoint `cp`.
pub fn get_cjk_glyph_15(cp: u16) -> Option<[u8; 64]> {
    // 1. Fast binary search in built-in static kernel table (0 I/O)
    if let Ok(idx) = BUILTIN_CJK_CODEPOINTS.binary_search(&cp) {
        return Some(BUILTIN_CJK_GLYPHS_15[idx]);
    }

    // 2. Check /FONT16.BIN on SD card (8 glyphs per 512-byte sector)
    let glyph_idx = font16_glyph_index(cp)?;
    let sector_idx = glyph_idx / 8;
    let byte_off = (glyph_idx % 8) * 64;

    unsafe {
        let state = (*core::ptr::addr_of_mut!(FONT_STATE)).as_mut()?;
        if state.clusters.is_empty() {
            return None;
        }

        // Check RAM sector cache first
        for (cached_sec, data) in state.cache.iter() {
            if *cached_sec == sector_idx {
                let mut out = [0u8; 64];
                out.copy_from_slice(&data[byte_off..byte_off + 64]);
                return Some(out);
            }
        }

        // Cache miss: read 1 sector in O(1) using precomputed cluster chain
        let mut sec_buf = [0u8; SECTOR_SIZE];
        if !fat32::read_sector_indexed(&state.clusters, sector_idx, &mut sec_buf) {
            return None;
        }

        let mut out = [0u8; 64];
        out.copy_from_slice(&sec_buf[byte_off..byte_off + 64]);

        if state.cache.len() < 96 {
            state.cache.push((sector_idx, sec_buf));
        } else {
            let slot = state.next_evict % state.cache.len();
            state.cache[slot] = (sector_idx, sec_buf);
            state.next_evict = state.next_evict.wrapping_add(1);
        }

        Some(out)
    }
}

/// Look up a `12x12` 2-bit anti-aliased glyph (64 bytes) for Unicode codepoint `cp`.
pub fn get_cjk_glyph_12(cp: u16) -> Option<[u8; 64]> {
    if let Ok(idx) = BUILTIN_CJK_CODEPOINTS.binary_search(&cp) {
        return Some(BUILTIN_CJK_GLYPHS_12[idx]);
    }
    get_cjk_glyph_15(cp)
}

// ─── Fast Integer RGB565 Alpha Blending Helpers ───

/// Blend foreground `fg` over background `bg` with 2-bit level `q` (`1` = 33%, `2` = 67%, `3` = 100%).
#[inline(always)]
pub fn blend_rgb565_q2(bg: Color, fg: Color, q: u32) -> Color {
    if q >= 3 {
        return fg;
    }
    if q == 0 {
        return bg;
    }
    let br = ((bg >> 11) & 0x1F) as u32;
    let bg_g = ((bg >> 5) & 0x3F) as u32;
    let bb = (bg & 0x1F) as u32;

    let fr = ((fg >> 11) & 0x1F) as u32;
    let fg_g = ((fg >> 5) & 0x3F) as u32;
    let fb = (fg & 0x1F) as u32;

    let inv = 3 - q;
    let r = (br * inv + fr * q + 1) / 3;
    let g = (bg_g * inv + fg_g * q + 1) / 3;
    let b = (bb * inv + fb * q + 1) / 3;

    ((r as u16) << 11) | ((g as u16) << 5) | (b as u16)
}

/// Blend `fg` over `bg` with 8-bit alpha (`0..=255`).
#[inline(always)]
pub fn blend_rgb565_alpha(bg: Color, fg: Color, alpha: u8) -> Color {
    if alpha == 255 {
        return fg;
    }
    if alpha == 0 {
        return bg;
    }
    let a = alpha as u32;
    let inv = 255 - a;

    let br = ((bg >> 11) & 0x1F) as u32;
    let bg_g = ((bg >> 5) & 0x3F) as u32;
    let bb = (bg & 0x1F) as u32;

    let fr = ((fg >> 11) & 0x1F) as u32;
    let fg_g = ((fg >> 5) & 0x3F) as u32;
    let fb = (fg & 0x1F) as u32;

    let r = (br * inv + fr * a) / 255;
    let g = (bg_g * inv + fg_g * a) / 255;
    let b = (bb * inv + fb * a) / 255;

    ((r as u16) << 11) | ((g as u16) << 5) | (b as u16)
}

// ─── Off-Screen 320x240 RGB565 Canvas with Dirty-Rect Compositor ───

pub struct Canvas {
    pub pixels: Vec<u16>,
    dirty_x0: i32,
    dirty_y0: i32,
    dirty_x1: i32,
    dirty_y1: i32,
}

impl Canvas {
    pub fn new() -> Self {
        Self {
            pixels: vec![0u16; (SCREEN_W * SCREEN_H) as usize],
            dirty_x0: 0,
            dirty_y0: 0,
            dirty_x1: SCREEN_W,
            dirty_y1: SCREEN_H,
        }
    }

    #[inline(always)]
    pub fn mark_dirty(&mut self, x: i32, y: i32, w: i32, h: i32) {
        let x0 = x.clamp(0, SCREEN_W);
        let y0 = y.clamp(0, SCREEN_H);
        let x1 = (x + w).clamp(0, SCREEN_W);
        let y1 = (y + h).clamp(0, SCREEN_H);
        if x0 < x1 && y0 < y1 {
            if x0 < self.dirty_x0 {
                self.dirty_x0 = x0;
            }
            if y0 < self.dirty_y0 {
                self.dirty_y0 = y0;
            }
            if x1 > self.dirty_x1 {
                self.dirty_x1 = x1;
            }
            if y1 > self.dirty_y1 {
                self.dirty_y1 = y1;
            }
        }
    }

    pub fn mark_all_dirty(&mut self) {
        self.dirty_x0 = 0;
        self.dirty_y0 = 0;
        self.dirty_x1 = SCREEN_W;
        self.dirty_y1 = SCREEN_H;
    }

    /// Flush only the accumulated dirty rectangle to the physical display in a single SPI burst.
    pub fn flush(&mut self) {
        if self.dirty_x0 < self.dirty_x1 && self.dirty_y0 < self.dirty_y1 {
            let x = self.dirty_x0 as u32;
            let y = self.dirty_y0 as u32;
            let w = (self.dirty_x1 - self.dirty_x0) as u32;
            let h = (self.dirty_y1 - self.dirty_y0) as u32;
            display::blit_rect(x, y, w, h, &self.pixels, SCREEN_W as u32);
            self.dirty_x0 = SCREEN_W;
            self.dirty_y0 = SCREEN_H;
            self.dirty_x1 = 0;
            self.dirty_y1 = 0;
        }
    }

    /// Immediately flush a specific rectangle `(x, y, w, h)` to the display.
    pub fn flush_rect(&mut self, x: i32, y: i32, w: i32, h: i32) {
        let x0 = x.clamp(0, SCREEN_W);
        let y0 = y.clamp(0, SCREEN_H);
        let x1 = (x + w).clamp(0, SCREEN_W);
        let y1 = (y + h).clamp(0, SCREEN_H);
        if x0 < x1 && y0 < y1 {
            display::blit_rect(
                x0 as u32,
                y0 as u32,
                (x1 - x0) as u32,
                (y1 - y0) as u32,
                &self.pixels,
                SCREEN_W as u32,
            );
        }
    }

    #[inline(always)]
    pub fn set_px(&mut self, x: i32, y: i32, col: Color) {
        if x >= 0 && x < SCREEN_W && y >= 0 && y < SCREEN_H {
            self.pixels[(y * SCREEN_W + x) as usize] = col;
        }
    }

    #[inline(always)]
    pub fn set_px_alpha(&mut self, x: i32, y: i32, col: Color, alpha: u8) {
        if x >= 0 && x < SCREEN_W && y >= 0 && y < SCREEN_H {
            let idx = (y * SCREEN_W + x) as usize;
            self.pixels[idx] = blend_rgb565_alpha(self.pixels[idx], col, alpha);
        }
    }

    #[inline(always)]
    pub fn set_px_q2(&mut self, x: i32, y: i32, col: Color, q: u32) {
        if x >= 0 && x < SCREEN_W && y >= 0 && y < SCREEN_H {
            let idx = (y * SCREEN_W + x) as usize;
            self.pixels[idx] = blend_rgb565_q2(self.pixels[idx], col, q);
        }
    }

    pub fn fill_rect(&mut self, x: i32, y: i32, w: i32, h: i32, col: Color) {
        let x0 = x.clamp(0, SCREEN_W);
        let y0 = y.clamp(0, SCREEN_H);
        let x1 = (x + w).clamp(0, SCREEN_W);
        let y1 = (y + h).clamp(0, SCREEN_H);
        for py in y0..y1 {
            let row_base = (py * SCREEN_W) as usize;
            for px in x0..x1 {
                self.pixels[row_base + px as usize] = col;
            }
        }
        self.mark_dirty(x0, y0, x1 - x0, y1 - y0);
    }

    pub fn fill_gradient_v(
        &mut self,
        x: i32,
        y: i32,
        w: i32,
        h: i32,
        top_rgb: (u8, u8, u8),
        bot_rgb: (u8, u8, u8),
    ) {
        if h <= 0 || w <= 0 {
            return;
        }
        let denom = (h - 1).max(1);
        for dy in 0..h {
            let py = y + dy;
            if py < 0 || py >= SCREEN_H {
                continue;
            }
            let r = ((top_rgb.0 as i32) * (denom - dy) + (bot_rgb.0 as i32) * dy) / denom;
            let g = ((top_rgb.1 as i32) * (denom - dy) + (bot_rgb.1 as i32) * dy) / denom;
            let b = ((top_rgb.2 as i32) * (denom - dy) + (bot_rgb.2 as i32) * dy) / denom;
            let col = colors::rgb(r as u8, g as u8, b as u8);
            let x0 = x.clamp(0, SCREEN_W);
            let x1 = (x + w).clamp(0, SCREEN_W);
            let row_base = (py * SCREEN_W) as usize;
            for px in x0..x1 {
                self.pixels[row_base + px as usize] = col;
            }
        }
        self.mark_dirty(x, y, w, h);
    }

    #[inline(always)]
    fn in_rounded(dx: i32, dy: i32, w: i32, h: i32, r: i32) -> bool {
        if r <= 0 {
            return true;
        }
        let r2 = r * r;
        if dx < r && dy < r {
            let ex = dx - r;
            let ey = dy - r;
            return ex * ex + ey * ey <= r2;
        }
        if dx >= w - r && dy < r {
            let ex = dx - (w - 1 - r);
            let ey = dy - r;
            return ex * ex + ey * ey <= r2;
        }
        if dx < r && dy >= h - r {
            let ex = dx - r;
            let ey = dy - (h - 1 - r);
            return ex * ex + ey * ey <= r2;
        }
        if dx >= w - r && dy >= h - r {
            let ex = dx - (w - 1 - r);
            let ey = dy - (h - 1 - r);
            return ex * ex + ey * ey <= r2;
        }
        true
    }

    pub fn fill_rrect(&mut self, x: i32, y: i32, w: i32, h: i32, r: i32, col: Color) {
        for dy in 0..h {
            for dx in 0..w {
                if Self::in_rounded(dx, dy, w, h, r) {
                    self.set_px(x + dx, y + dy, col);
                }
            }
        }
        self.mark_dirty(x, y, w, h);
    }

    pub fn fill_rrect_alpha(
        &mut self,
        x: i32,
        y: i32,
        w: i32,
        h: i32,
        r: i32,
        col: Color,
        alpha: u8,
    ) {
        for dy in 0..h {
            for dx in 0..w {
                if Self::in_rounded(dx, dy, w, h, r) {
                    self.set_px_alpha(x + dx, y + dy, col, alpha);
                }
            }
        }
        self.mark_dirty(x, y, w, h);
    }

    pub fn stroke_rrect(&mut self, x: i32, y: i32, w: i32, h: i32, r: i32, col: Color) {
        for dy in 0..h {
            for dx in 0..w {
                if Self::in_rounded(dx, dy, w, h, r) {
                    let is_edge = dx == 0
                        || dx == w - 1
                        || dy == 0
                        || dy == h - 1
                        || !Self::in_rounded(dx - 1, dy, w, h, r)
                        || !Self::in_rounded(dx + 1, dy, w, h, r)
                        || !Self::in_rounded(dx, dy - 1, w, h, r)
                        || !Self::in_rounded(dx, dy + 1, w, h, r);
                    if is_edge {
                        self.set_px(x + dx, y + dy, col);
                    }
                }
            }
        }
        self.mark_dirty(x, y, w, h);
    }

    /// Draw a modern rounded card with drop shadow, solid fill, and 1px rounded border.
    pub fn draw_card(
        &mut self,
        x: i32,
        y: i32,
        w: i32,
        h: i32,
        r: i32,
        bg: Color,
        border: Color,
    ) {
        let shadow_col = colors::rgb(5, 8, 14);
        self.fill_rrect_alpha(x + 2, y + 2, w, h, r, shadow_col, 155);
        self.fill_rrect(x, y, w, h, r, bg);
        self.stroke_rrect(x, y, w, h, r, border);
    }

    pub fn draw_line(&mut self, mut x0: i32, mut y0: i32, x1: i32, y1: i32, col: Color) {
        let min_x = x0.min(x1);
        let min_y = y0.min(y1);
        let max_x = x0.max(x1);
        let max_y = y0.max(y1);

        let dx = (x1 - x0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let dy = -(y1 - y0).abs();
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;

        loop {
            self.set_px(x0, y0, col);
            if x0 == x1 && y0 == y1 {
                break;
            }
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x0 += sx;
            }
            if e2 <= dx {
                err += dx;
                y0 += sy;
            }
        }
        self.mark_dirty(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
    }

    pub fn fill_circle(&mut self, cx: i32, cy: i32, r: i32, col: Color) {
        let r2 = r * r;
        for dy in -r..=r {
            for dx in -r..=r {
                if dx * dx + dy * dy <= r2 {
                    self.set_px(cx + dx, cy + dy, col);
                }
            }
        }
        self.mark_dirty(cx - r, cy - r, r * 2 + 1, r * 2 + 1);
    }

    /// Draw a circular ring gauge (`pct` = 0..=100) using integer octant/angle approximation.
    pub fn draw_ring(
        &mut self,
        cx: i32,
        cy: i32,
        r_outer: i32,
        r_inner: i32,
        pct: u32,
        fg: Color,
        track: Color,
    ) {
        let ro2 = r_outer * r_outer;
        let ri2 = r_inner * r_inner;
        let pct_clamped = pct.min(100) as i32;

        for dy in -r_outer..=r_outer {
            for dx in -r_outer..=r_outer {
                let d2 = dx * dx + dy * dy;
                if d2 >= ri2 && d2 <= ro2 {
                    let deg = approx_clockwise_deg(dx, dy);
                    let active = deg * 100 <= pct_clamped * 360;
                    self.set_px(cx + dx, cy + dy, if active { fg } else { track });
                }
            }
        }
        self.mark_dirty(
            cx - r_outer,
            cy - r_outer,
            r_outer * 2 + 1,
            r_outer * 2 + 1,
        );
    }

    /// Draw a single 8x8 ASCII character transparently at `(x, y)`.
    pub fn draw_ascii_char(&mut self, x: i32, y: i32, ch: u8, fg: Color) {
        let glyph = font::glyph(ch);
        for row in 0..8i32 {
            let bits = glyph[row as usize];
            for col in 0..8i32 {
                if (bits >> col) & 1 != 0 {
                    self.set_px(x + col, y + row, fg);
                }
            }
        }
        self.mark_dirty(x, y, 8, 8);
    }

    /// Draw an 8x8 ASCII string transparently at `(x, y)`.
    pub fn draw_str_ascii(&mut self, x: i32, y: i32, s: &str, fg: Color) {
        let mut cx = x;
        for b in s.bytes() {
            if cx >= SCREEN_W {
                break;
            }
            self.draw_ascii_char(cx, y, b, fg);
            cx += 8;
        }
    }

    /// Measure the pixel width of a mixed UTF-8 Chinese + ASCII string.
    pub fn measure_mixed_text(text: &str, cjk_size: u8) -> i32 {
        let step_cjk = (cjk_size as i32) + 1;
        let mut w = 0i32;
        for ch in text.chars() {
            if (ch as u32) < 128 {
                w += 8;
            } else {
                w += step_cjk;
            }
        }
        w
    }

    /// Draw a mixed UTF-8 Chinese (`15x15` or `12x12` 2-bit AA) + ASCII (`8x8`) string at `(x, y)`.
    /// Returns the total horizontal advance in pixels.
    pub fn draw_mixed_text(&mut self, x: i32, y: i32, text: &str, fg: Color, cjk_size: u8) -> i32 {
        let size = (cjk_size as i32).clamp(12, 16);
        let ascii_y_off = (size - 8) / 2 + 1;
        let mut cx = x;

        for ch in text.chars() {
            if cx >= SCREEN_W - 4 {
                break;
            }
            let cp = ch as u32;
            if cp < 128 {
                self.draw_ascii_char(cx, y + ascii_y_off, cp as u8, fg);
                cx += 8;
            } else {
                let cp16 = (cp & 0xFFFF) as u16;
                let glyph_opt = if size <= 12 {
                    get_cjk_glyph_12(cp16)
                } else {
                    get_cjk_glyph_15(cp16)
                };

                if let Some(glyph) = glyph_opt {
                    for gy in 0..size {
                        let base = (gy as usize) * 4;
                        let row_u32 = u32::from_le_bytes([
                            glyph[base],
                            glyph[base + 1],
                            glyph[base + 2],
                            glyph[base + 3],
                        ]);
                        if row_u32 == 0 {
                            continue;
                        }
                        for gx in 0..size {
                            let q = (row_u32 >> (gx * 2)) & 0b11;
                            if q != 0 {
                                self.set_px_q2(cx + gx, y + gy, fg, q);
                            }
                        }
                    }
                    self.mark_dirty(cx, y, size, size);
                } else {
                    // Fallback small box if exotic codepoint and no FONT16.BIN
                    self.stroke_rrect(cx + 1, y + 2, size - 2, size - 3, 2, fg);
                }
                cx += size + 1;
            }
        }
        cx - x
    }
}

/// Fast integer clockwise angle in degrees `0..360` starting from 12 o'clock (`(0, -r)`).
fn approx_clockwise_deg(dx: i32, dy: i32) -> i32 {
    if dx == 0 && dy == 0 {
        return 0;
    }
    let adx = dx.abs();
    let ady = dy.abs();
    // Angle within 0..45 deg octant: 45 * min / max
    let oct_deg = if adx > ady {
        (45 * ady) / adx.max(1)
    } else {
        (45 * adx) / ady.max(1)
    };

    if dx >= 0 && dy < 0 {
        // Quadrant 1 (12 o'clock to 3 o'clock): 0..90
        if ady >= adx {
            oct_deg
        } else {
            90 - oct_deg
        }
    } else if dx > 0 && dy >= 0 {
        // Quadrant 2 (3 o'clock to 6 o'clock): 90..180
        if adx >= ady {
            90 + oct_deg
        } else {
            180 - oct_deg
        }
    } else if dx <= 0 && dy > 0 {
        // Quadrant 3 (6 o'clock to 9 o'clock): 180..270
        if ady >= adx {
            180 + oct_deg
        } else {
            270 - oct_deg
        }
    } else {
        // Quadrant 4 (9 o'clock to 12 o'clock): 270..360
        if adx >= ady {
            270 + oct_deg
        } else {
            360 - oct_deg
        }
    }
}
