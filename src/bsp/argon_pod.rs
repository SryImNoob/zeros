// ZerOS — Argon POD Display Module Pinout & Hardware Constants
//
// Verified against the physical Argon POD device tree (`/boot/firmware/overlays/tft9341.dtbo`)
// and `argonpodd.py` daemon on Raspberry Pi Zero 2 W:
//
//   - SPI0 Master:
//       CE0  = BCM GPIO 8   (ILI9341 LCD Chip Select)
//       CE1  = BCM GPIO 7   (ADS7846 / XPT2046 Touch Chip Select)
//       MISO = BCM GPIO 9
//       MOSI = BCM GPIO 10
//       SCLK = BCM GPIO 11
//   - ILI9341 2.8" 320x240 RGB565 TFT LCD:
//       DC   = BCM GPIO 22  (Data/Command select)
//       RST  = BCM GPIO 27  (Active-low hardware reset)
//       BL   = BCM GPIO 18  (PWM / Digital Backlight enable, active-high)
//   - ADS7846 / XPT2046 Resistive Touch Controller:
//       PENIRQ = BCM GPIO 17 (Active-low touch pen interrupt)
//   - 4 Hardware Tactile Buttons (Active-low, internal/external pull-ups):
//       BTN1 = BCM GPIO 16
//       BTN2 = BCM GPIO 20
//       BTN3 = BCM GPIO 21
//       BTN4 = BCM GPIO 26
//   - Infrared Receiver (NEC 38kHz):
//       IR   = BCM GPIO 23

#![allow(dead_code)]

/// Display resolution (landscape mode)
pub const LCD_WIDTH: u32 = 320;
pub const LCD_HEIGHT: u32 = 240;

/// ILI9341 LCD GPIO pins
pub const PIN_LCD_DC: u32 = 22;
pub const PIN_LCD_RST: u32 = 27;
pub const PIN_LCD_BL: u32 = 18;

/// ADS7846 / XPT2046 Touch controller pins (on SPI0 CE1)
pub const PIN_TOUCH_CS: u32 = 7;
pub const PIN_TOUCH_IRQ: u32 = 17;

/// 4 tactile buttons around the Argon POD bezel (active-low)
pub const PIN_BTN_1: u32 = 16;
pub const PIN_BTN_2: u32 = 20;
pub const PIN_BTN_3: u32 = 21;
pub const PIN_BTN_4: u32 = 26;
pub const BUTTON_PINS: [u32; 4] = [PIN_BTN_1, PIN_BTN_2, PIN_BTN_3, PIN_BTN_4];

/// NEC IR receiver pin
pub const PIN_IR_RECV: u32 = 23;

/// RPi Zero 2 W onboard Activity LED (active-low on BCM GPIO 47)
pub const PIN_ACT_LED: u32 = 47;
