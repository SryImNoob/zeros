// ZerOS — GPU Framebuffer via VideoCore Mailbox
//
// Allocates a framebuffer through the GPU mailbox interface.
// Works in QEMU (-display sdl/gtk) and on real hardware (HDMI output).
//
// Resolution: matches Argon POD display (320×240, RGB565)
// On QEMU: shows a window with the framebuffer
// On real hardware: outputs via HDMI (ILI9341 uses SPI separately)

use crate::drivers::mailbox;

/// RGB565 color type
pub type Color = u16;

/// Common colors (RGB565 format)
#[allow(dead_code)]
pub mod colors {
    use super::Color;
    pub const BLACK: Color = 0x0000;
    pub const WHITE: Color = 0xFFFF;
    pub const RED: Color = 0xF800;
    pub const GREEN: Color = 0x07E0;
    pub const BLUE: Color = 0x001F;
    pub const YELLOW: Color = 0xFFE0;
    pub const CYAN: Color = 0x07FF;
    pub const MAGENTA: Color = 0xF81F;
    pub const DARK_GRAY: Color = 0x4208;
    pub const LIGHT_GRAY: Color = 0xC618;

    /// Create RGB565 from 8-bit RGB components
    pub const fn rgb(r: u8, g: u8, b: u8) -> Color {
        ((r as u16 & 0xF8) << 8) | ((g as u16 & 0xFC) << 3) | ((b as u16) >> 3)
    }
}

/// Framebuffer state
pub struct Framebuffer {
    pub buffer: *mut u16,
    pub width: u32,
    pub height: u32,
    pub pitch: u32, // bytes per row
}

/// Global framebuffer instance
pub static mut FB: Option<Framebuffer> = None;

/// Initialize the GPU framebuffer via mailbox.
/// Returns true on success.
pub fn init(width: u32, height: u32) -> bool {
    // Property tag buffer — must be 16-byte aligned
    #[repr(C, align(16))]
    struct MboxBuf {
        data: [u32; 36],
    }

    let mut buf = MboxBuf { data: [0; 36] };
    let d = &mut buf.data;

    d[0] = 36 * 4;       // Buffer size in bytes
    d[1] = 0;            // Request code

    // Tag: Set physical (display) size
    d[2] = mailbox::TAG_SET_PHYS_SIZE;
    d[3] = 8;            // Value buffer size
    d[4] = 0;            // Request (0 = request)
    d[5] = width;
    d[6] = height;

    // Tag: Set virtual (buffer) size
    d[7] = mailbox::TAG_SET_VIRT_SIZE;
    d[8] = 8;
    d[9] = 0;
    d[10] = width;
    d[11] = height;

    // Tag: Set color depth
    d[12] = mailbox::TAG_SET_DEPTH;
    d[13] = 4;
    d[14] = 0;
    d[15] = 16;          // 16 bits per pixel (RGB565)

    // Tag: Set pixel order (RGB)
    d[16] = mailbox::TAG_SET_PIXEL_ORDER;
    d[17] = 4;
    d[18] = 0;
    d[19] = 1;           // 1 = RGB (not BGR)

    // Tag: Allocate buffer
    d[20] = mailbox::TAG_ALLOC_BUFFER;
    d[21] = 8;
    d[22] = 4;           // Request: alignment
    d[23] = 16;          // Align to 16 bytes
    d[24] = 0;           // Response: size (filled by GPU)

    // Tag: Get pitch (bytes per row)
    d[25] = mailbox::TAG_GET_PITCH;
    d[26] = 4;
    d[27] = 0;
    d[28] = 0;           // Response: pitch

    // End tag
    d[29] = mailbox::TAG_END;

    if !mailbox::call(d) {
        crate::println!("[ERR] Framebuffer: mailbox call failed");
        return false;
    }

    // Parse response
    let fb_bus_addr = d[23];
    let _fb_size = d[24];
    let pitch = d[28];

    if fb_bus_addr == 0 {
        crate::println!("[ERR] Framebuffer: GPU returned null address");
        return false;
    }

    // Convert GPU bus address to ARM physical address
    let fb_phys = fb_bus_addr & 0x3FFF_FFFF;

    let fb = Framebuffer {
        buffer: fb_phys as *mut u16,
        width,
        height,
        pitch,
    };

    crate::println!("[OK] Framebuffer: {}x{} RGB565, addr=0x{:08x}, pitch={}",
        width, height, fb_phys, pitch);

    unsafe { FB = Some(fb); }
    true
}

impl Framebuffer {
    /// Set a single pixel
    #[inline(always)]
    pub fn put_pixel(&self, x: u32, y: u32, color: Color) {
        if x >= self.width || y >= self.height {
            return;
        }
        unsafe {
            let offset = (y * self.pitch / 2 + x) as isize;
            *self.buffer.offset(offset) = color;
        }
    }

    /// Fill a rectangle
    pub fn fill_rect(&self, x: u32, y: u32, w: u32, h: u32, color: Color) {
        for row in y..(y + h).min(self.height) {
            for col in x..(x + w).min(self.width) {
                self.put_pixel(col, row, color);
            }
        }
    }

    /// Clear the entire screen
    pub fn clear(&self, color: Color) {
        self.fill_rect(0, 0, self.width, self.height, color);
    }

    /// Draw a character at pixel position (x, y) using the 8×8 font
    pub fn draw_char(&self, x: u32, y: u32, c: u8, fg: Color, bg: Color) {
        let glyph = super::font::glyph(c);
        for row in 0..8u32 {
            let bits = glyph[row as usize];
            for col in 0..8u32 {
                let color = if bits & (1 << col) != 0 { fg } else { bg };
                self.put_pixel(x + col, y + row, color);
            }
        }
    }

    /// Draw a string at pixel position (x, y)
    pub fn draw_str(&self, x: u32, y: u32, s: &str, fg: Color, bg: Color) {
        let mut cx = x;
        for byte in s.bytes() {
            if cx + 8 > self.width {
                break;
            }
            self.draw_char(cx, y, byte, fg, bg);
            cx += 8;
        }
    }
}
