// ZerOS — ILI9341 TFT Display Driver
//
// Drives the Argon POD's 2.8" ILI9341 TFT LCD (320×240, RGB565) via SPI.
// This driver handles the hardware-specific initialization and pixel output
// for the real display. On QEMU, use the GPU framebuffer (framebuffer.rs) instead.
//
// Argon POD pin mapping (verified from /boot/firmware/overlays/tft9341.dtbo & argonpodd.py):
//   SPI0 CE0 (GPIO 8)  — Chip Select (reg = <0>)
//   GPIO 22 (0x16)     — DC (Data/Command select: dc-gpios = <&gpio 22 0>)
//   GPIO 27 (0x1b)     — RST (Hardware Reset: reset-gpios = <&gpio 27 1>)
//   GPIO 18            — Backlight (PIN_BRIGHTNESS = 18)

#![allow(dead_code)]

use crate::bsp::argon_pod::{LCD_HEIGHT, LCD_WIDTH, PIN_LCD_BL, PIN_LCD_DC, PIN_LCD_RST};
use crate::drivers::{mmio, spi};

// ─── GPIO pins for the Argon POD ILI9341 ───
const PIN_DC:      u32 = PIN_LCD_DC;  // Data/Command select: LOW=command, HIGH=data (0x16 in tft9341.dtbo)
const PIN_RST:     u32 = PIN_LCD_RST; // Hardware reset (active low, 0x1b in tft9341.dtbo)
const PIN_RST_ALT: u32 = 25;          // Alternate reset pin on generic ILI9341 hats
const PIN_BL:      u32 = PIN_LCD_BL;  // Backlight control (GPIO 18 in argonpodd.py)

/// Display SPI chip select (CE0)
const SPI_CS_DISPLAY: u8 = 0;

/// Display dimensions
pub const WIDTH:  u32 = LCD_WIDTH;
pub const HEIGHT: u32 = LCD_HEIGHT;

// ─── ILI9341 Commands ───
const CMD_NOP:          u8 = 0x00;
const CMD_SWRESET:      u8 = 0x01; // Software Reset
const CMD_SLEEP_OUT:    u8 = 0x11; // Sleep Out
const CMD_DISPLAY_OFF:  u8 = 0x28; // Display OFF
const CMD_DISPLAY_ON:   u8 = 0x29; // Display ON
const CMD_COLUMN_ADDR:  u8 = 0x2A; // Column Address Set
const CMD_PAGE_ADDR:    u8 = 0x2B; // Page Address Set
const CMD_MEM_WRITE:    u8 = 0x2C; // Memory Write
const CMD_MEM_ACCESS:   u8 = 0x36; // Memory Access Control (MADCTL)
const CMD_PIXEL_FORMAT: u8 = 0x3A; // Pixel Format Set
const CMD_FRAME_CTRL:   u8 = 0xB1; // Frame Rate Control
const CMD_DISPLAY_FUNC: u8 = 0xB6; // Display Function Control
const CMD_POWER_CTRL1:  u8 = 0xC0; // Power Control 1
const CMD_POWER_CTRL2:  u8 = 0xC1; // Power Control 2
const CMD_VCOM_CTRL1:   u8 = 0xC5; // VCOM Control 1
const CMD_VCOM_CTRL2:   u8 = 0xC7; // VCOM Control 2

// ─── MADCTL flags ───
const MADCTL_MY:  u8 = 0x80; // Row Address Order
const MADCTL_MX:  u8 = 0x40; // Column Address Order
const MADCTL_MV:  u8 = 0x20; // Row/Column Exchange
const MADCTL_ML:  u8 = 0x10; // Vertical Refresh Order
const MADCTL_BGR: u8 = 0x08; // BGR color order
const MADCTL_MH:  u8 = 0x04; // Horizontal Refresh Order

// ─── GPIO register offsets ───
const GPIO_BASE: usize = mmio::PERIPHERAL_BASE + 0x0020_0000;
const GPIO_SET0: usize = GPIO_BASE + 0x1C; // Output Set
const GPIO_CLR0: usize = GPIO_BASE + 0x28; // Output Clear

/// Global ILI9341 display state
pub static mut DISPLAY: Option<Ili9341> = None;

pub struct Ili9341 {
    width: u32,
    height: u32,
}

/// Initialize the ILI9341 display.
/// Must be called after spi::init().
pub fn init() -> bool {
    // ── Configure GPIO pins as outputs ──
    gpio_output(PIN_DC);
    gpio_output(PIN_RST);
    gpio_output(PIN_RST_ALT);
    gpio_output(PIN_BL);

    // ── Hardware reset sequence ──
    gpio_set(PIN_RST, true);
    gpio_set(PIN_RST_ALT, true);
    delay_ms(5);
    gpio_set(PIN_RST, false);
    gpio_set(PIN_RST_ALT, false);
    delay_ms(20);
    gpio_set(PIN_RST, true);
    gpio_set(PIN_RST_ALT, true);
    delay_ms(150);

    // ── Turn on backlight ──
    gpio_set(PIN_BL, true);

    // ── ILI9341 initialization commands ──
    write_cmd(CMD_SWRESET);
    delay_ms(150);

    write_cmd(CMD_DISPLAY_OFF);

    // Power Control 1: GVDD = 4.60V
    write_cmd(CMD_POWER_CTRL1);
    write_data(&[0x23]);

    // Power Control 2: step-up factor
    write_cmd(CMD_POWER_CTRL2);
    write_data(&[0x10]);

    // VCOM Control 1
    write_cmd(CMD_VCOM_CTRL1);
    write_data(&[0x3E, 0x28]);

    // VCOM Control 2
    write_cmd(CMD_VCOM_CTRL2);
    write_data(&[0x86]);

    // Memory Access Control: landscape 270° rotation for Argon POD (MV | BGR)
    // Note: MY flips the horizontal (320) axis when MV=1; clearing MY fixes horizontal mirror!
    write_cmd(CMD_MEM_ACCESS);
    write_data(&[MADCTL_MV | MADCTL_BGR]);

    // Pixel format: 16-bit RGB565
    write_cmd(CMD_PIXEL_FORMAT);
    write_data(&[0x55]); // 16-bit/pixel

    // Frame rate: 70Hz
    write_cmd(CMD_FRAME_CTRL);
    write_data(&[0x00, 0x18]);

    // Display function control
    write_cmd(CMD_DISPLAY_FUNC);
    write_data(&[0x08, 0x82, 0x27]);

    // Exit sleep
    write_cmd(CMD_SLEEP_OUT);
    delay_ms(120);

    let display = Ili9341 {
        width: WIDTH,
        height: HEIGHT,
    };

    // Clear GRAM to black BEFORE turning display on so there is no white snow flash
    display.clear(0x0000);

    // Display ON
    write_cmd(CMD_DISPLAY_ON);
    delay_ms(20);

    crate::println!("[OK] ILI9341: {}x{} RGB565 display initialized (SPI)", WIDTH, HEIGHT);

    unsafe { DISPLAY = Some(display); }
    true
}

impl Ili9341 {
    /// Set the drawing window (column and page address)
    pub fn set_window(&self, x0: u16, y0: u16, x1: u16, y1: u16) {
        write_cmd(CMD_COLUMN_ADDR);
        write_data(&[
            (x0 >> 8) as u8, (x0 & 0xFF) as u8,
            (x1 >> 8) as u8, (x1 & 0xFF) as u8,
        ]);

        write_cmd(CMD_PAGE_ADDR);
        write_data(&[
            (y0 >> 8) as u8, (y0 & 0xFF) as u8,
            (y1 >> 8) as u8, (y1 & 0xFF) as u8,
        ]);
    }

    /// Set a single pixel
    pub fn put_pixel(&self, x: u32, y: u32, color: u16) {
        if x >= self.width || y >= self.height {
            return;
        }
        self.set_window(x as u16, y as u16, x as u16, y as u16);
        write_cmd(CMD_MEM_WRITE);
        write_data(&[(color >> 8) as u8, (color & 0xFF) as u8]);
    }

    /// Fill a rectangle with a solid color in a single uninterrupted SPI burst
    pub fn fill_rect(&self, x: u32, y: u32, w: u32, h: u32, color: u16) {
        if x >= self.width || y >= self.height || w == 0 || h == 0 {
            return;
        }
        let x1 = (x + w - 1).min(self.width - 1);
        let y1 = (y + h - 1).min(self.height - 1);
        let total_pixels = ((x1 - x + 1) * (y1 - y + 1)) as usize;

        self.set_window(x as u16, y as u16, x1 as u16, y1 as u16);
        write_cmd(CMD_MEM_WRITE);
        gpio_set(PIN_DC, true); // Data mode
        spi::write_repeat_u16(SPI_CS_DISPLAY, color, total_pixels);
    }

    /// Clear the entire display
    pub fn clear(&self, color: u16) {
        self.fill_rect(0, 0, self.width, self.height, color);
    }

    /// Draw a character using the 8×8 font
    pub fn draw_char(&self, x: u32, y: u32, c: u8, fg: u16, bg: u16) {
        if x + 8 > self.width || y + 8 > self.height {
            return;
        }

        let glyph = super::font::glyph(c);

        // Set window for the entire character to minimize SPI overhead
        self.set_window(x as u16, y as u16, (x + 7) as u16, (y + 7) as u16);
        write_cmd(CMD_MEM_WRITE);

        // Build pixel data for the full 8×8 character
        let mut buf = [0u8; 8 * 8 * 2]; // 64 pixels × 2 bytes
        for row in 0..8u32 {
            let bits = glyph[row as usize];
            for col in 0..8u32 {
                let color = if bits & (1 << col) != 0 { fg } else { bg };
                let idx = ((row * 8 + col) * 2) as usize;
                buf[idx] = (color >> 8) as u8;
                buf[idx + 1] = (color & 0xFF) as u8;
            }
        }

        write_data(&buf);
    }

    /// Draw a string at pixel position (x, y)
    pub fn draw_str(&self, x: u32, y: u32, s: &str, fg: u16, bg: u16) {
        let mut cx = x;
        for byte in s.bytes() {
            if cx + 8 > self.width {
                break;
            }
            self.draw_char(cx, y, byte, fg, bg);
            cx += 8;
        }
    }

    /// Copy the contents of the RAM framebuffer (rows `start_y..self.height`) to the ILI9341
    pub fn flush_from_fb(&self, fb: &super::framebuffer::Framebuffer, start_y: u32) {
        let y0 = start_y.min(self.height - 1);
        self.set_window(0, y0 as u16, (self.width - 1) as u16, (self.height - 1) as u16);
        write_cmd(CMD_MEM_WRITE);

        let pitch_pixels = fb.pitch / 2;
        let mut row_buf = [0u8; (WIDTH as usize) * 2];

        unsafe {
            for row in y0..self.height {
                for col in 0..self.width {
                    let offset = (row * pitch_pixels + col) as isize;
                    let pixel = *fb.buffer.offset(offset);
                    let idx = (col * 2) as usize;
                    row_buf[idx] = (pixel >> 8) as u8;
                    row_buf[idx + 1] = (pixel & 0xFF) as u8;
                }
                write_data(&row_buf);
            }
        }
    }
}

// ─── SPI command/data helpers ───

/// Send a command byte (DC=LOW)
fn write_cmd(cmd: u8) {
    gpio_set(PIN_DC, false); // Command mode
    spi::transfer_byte(SPI_CS_DISPLAY, cmd);
}

/// Send data bytes (DC=HIGH)
fn write_data(data: &[u8]) {
    gpio_set(PIN_DC, true); // Data mode
    spi::write_bytes(SPI_CS_DISPLAY, data);
}

// ─── GPIO helpers ───

/// Configure a GPIO pin as output
fn gpio_output(pin: u32) {
    let reg = GPIO_BASE + ((pin / 10) * 4) as usize;
    let shift = ((pin % 10) * 3) as u32;
    unsafe {
        let val = mmio::read(reg);
        let val = (val & !(0b111 << shift)) | (0b001 << shift); // Output
        mmio::write(reg, val);
    }
}

/// Set a GPIO pin high or low
fn gpio_set(pin: u32, high: bool) {
    let reg = if high { GPIO_SET0 } else { GPIO_CLR0 };
    unsafe {
        mmio::write(reg, 1 << pin);
    }
}

/// Simple busy-wait delay (approximate, uses ARM generic timer)
fn delay_ms(ms: u32) {
    unsafe {
        let freq: u64;
        core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq);
        let ticks_per_ms = freq / 1000;
        let target_ticks = (ms as u64) * ticks_per_ms;

        let start: u64;
        core::arch::asm!("mrs {}, cntpct_el0", out(reg) start);

        loop {
            let now: u64;
            core::arch::asm!("mrs {}, cntpct_el0", out(reg) now);
            if now - start >= target_ticks {
                break;
            }
            core::hint::spin_loop();
        }
    }
}
