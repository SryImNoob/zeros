// ZerOS — ADS7846 / XPT2046 Resistive Touch Controller Driver (Argon POD)
//
// Verified against `/boot/firmware/overlays/tft9341.dtbo`:
//   - SPI0 CE1 (`cs = 1`, BCM GPIO 7)
//   - `pendown-gpio` = BCM GPIO 17 (active-low when panel is touched)
//   - `ti,swap-xy` enabled for 320x240 landscape orientation
//   - Calibration bounds: raw ADC 200..3800 (12-bit 0..4095)

#![allow(dead_code)]

use crate::bsp::argon_pod::{LCD_HEIGHT, LCD_WIDTH, PIN_TOUCH_IRQ};
use crate::drivers::{mmio, spi};

const GPIO_BASE: usize = mmio::PERIPHERAL_BASE + 0x0020_0000;
const GPIO_GPLEV0: usize = GPIO_BASE + 0x34;
const GPIO_GPPUD: usize = GPIO_BASE + 0x94;
const GPIO_GPPUDCLK0: usize = GPIO_BASE + 0x98;

/// ADS7846 / XPT2046 control bytes (Start=1, 12-bit mode=0, Differential=0, PD=00 so PENIRQ stays enabled)
const CMD_READ_X: u8 = 0xD0; // Channel X (A2..A0 = 101)
const CMD_READ_Y: u8 = 0x90; // Channel Y (A2..A0 = 001)

/// Initialize GPIO 17 (PENIRQ) as input with internal pull-up.
pub fn init() {
    let pin = PIN_TOUCH_IRQ;
    let reg = GPIO_BASE + ((pin / 10) * 4) as usize;
    let shift = ((pin % 10) * 3) as u32;
    unsafe {
        let val = mmio::read(reg) & !(0b111 << shift); // Input (000)
        mmio::write(reg, val);

        // Enable internal pull-up so unconnected/idle pin stays HIGH
        mmio::write(GPIO_GPPUD, 2);
        for _ in 0..150 {
            core::hint::spin_loop();
        }
        mmio::write(GPIO_GPPUDCLK0, 1 << pin);
        for _ in 0..150 {
            core::hint::spin_loop();
        }
        mmio::write(GPIO_GPPUD, 0);
        mmio::write(GPIO_GPPUDCLK0, 0);
    }
}

/// Returns `true` if the resistive touch screen is currently being pressed (`PENIRQ` = LOW).
#[inline]
pub fn is_touched() -> bool {
    if crate::bsp::is_qemu() {
        return false;
    }
    let level = unsafe { mmio::read(GPIO_GPLEV0) };
    (level & (1 << PIN_TOUCH_IRQ)) == 0
}

/// Read raw 12-bit ADC channel from ADS7846 on SPI0 CE1 (`cs = 1`).
fn read_adc(cmd: u8) -> u16 {
    let tx = [cmd, 0x00, 0x00];
    let mut rx = [0u8; 3];
    spi::transfer_bytes_slow(1, &tx, &mut rx);
    (((rx[1] as u16) << 8) | (rx[2] as u16)) >> 3
}

/// Poll the touch controller. Returns `Some((x, y))` in 320x240 screen coordinates
/// when a valid touch is detected.
pub fn poll_touch() -> Option<(u16, u16)> {
    if !is_touched() {
        return None;
    }

    let raw_a = read_adc(CMD_READ_X);
    let raw_b = read_adc(CMD_READ_Y);

    // Filter out noise / released panel readings at extremes
    if raw_a < 150 || raw_a > 3950 || raw_b < 150 || raw_b > 3950 {
        return None;
    }

    // `ti,swap-xy` mapping from dtbo (200..3800 -> 320x240)
    let x_clamped = (raw_b as i32 - 200).clamp(0, 3600) as u32;
    let y_clamped = (3800 - raw_a as i32).clamp(0, 3600) as u32;

    let x = ((x_clamped * LCD_WIDTH) / 3600).min(LCD_WIDTH - 1) as u16;
    let y = ((y_clamped * LCD_HEIGHT) / 3600).min(LCD_HEIGHT - 1) as u16;

    Some((x, y))
}
