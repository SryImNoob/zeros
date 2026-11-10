// ZerOS — BCM2710 Hardware PWM0 & Clock Manager Driver (M8.1)
//
// Drives the BCM2710 Hardware Pulse-Width Modulator (`PWM0` at `0x3F20_C000`)
// and Clock Manager (`CM_PWM` at `0x3F10_10A0`).
//
// On the Argon POD display module, BCM GPIO 18 controls the ILI9341 LCD
// backlight and is wired to Hardware `PWM0_0` when configured in `ALT5` (`0b010`)
// function mode.
//
// Clock setup:
//   - Oscillator source: 19.2 MHz (`CM_SRC_OSC = 1`)
//   - Clock divider: 19 (`19.2 MHz / 19 ≈ 1.01 MHz`)
//   - PWM Range (`PWM_RNG1`): 100 (`1.01 MHz / 100 ≈ 10.1 kHz` carrier, flicker-free)
//   - PWM Data (`PWM_DAT1`): 0..=100 (duty cycle percentage in Mark/Space mode)

#![allow(dead_code)]

use crate::bsp::argon_pod::PIN_LCD_BL;
use crate::drivers::mmio;
use crate::sync::IrqSafeLock;

// ─── Clock Manager (CM_PWM) Registers ───
const CM_BASE: usize = mmio::PERIPHERAL_BASE + 0x0010_1000;
const CM_PWMCTL: usize = CM_BASE + 0xA0;
const CM_PWMDIV: usize = CM_BASE + 0xA4;

const CM_PASSWD: u32 = 0x5A00_0000;
const CM_CTL_ENAB: u32 = 1 << 4;
const CM_CTL_KILL: u32 = 1 << 5;
const CM_CTL_BUSY: u32 = 1 << 7;
const CM_SRC_OSC: u32 = 1; // 19.2 MHz crystal oscillator

// ─── PWM0 Controller Registers ───
const PWM_BASE: usize = mmio::PERIPHERAL_BASE + 0x0020_C000;
const PWM_CTL: usize = PWM_BASE + 0x00;
const PWM_STA: usize = PWM_BASE + 0x04;
const PWM_RNG1: usize = PWM_BASE + 0x10;
const PWM_DAT1: usize = PWM_BASE + 0x14;

const PWM_CTL_PWEN1: u32 = 1 << 0; // Channel 1 Enable
const PWM_CTL_CLRF1: u32 = 1 << 6; // Clear FIFO
const PWM_CTL_MSEN1: u32 = 1 << 7; // Channel 1 Mark/Space Mode

// ─── GPIO Registers ───
const GPFSEL1: usize = mmio::GPIO_BASE + 0x04;

/// Current backlight brightness percentage (10..=100)
static BRIGHTNESS_PCT: IrqSafeLock<u32> = IrqSafeLock::new(100);

/// Short delay for hardware register synchronization (~10us)
#[inline]
fn short_delay() {
    for _ in 0..1000 {
        core::hint::spin_loop();
    }
}

/// Initialize BCM2710 Hardware PWM0 on GPIO 18 (`ALT5`) at 10.1 kHz, 100% brightness.
pub fn init() {
    if crate::bsp::is_qemu() {
        crate::println!("[OK] PWM0: virtual backlight initialized (100% duty)");
        return;
    }

    unsafe {
        // 1. Route GPIO 18 (PIN_LCD_BL) to ALT5 (0b010 = PWM0_0)
        let shift = (PIN_LCD_BL % 10) * 3; // GPIO 18 -> bits [26:24] in GPFSEL1
        let mut fsel1 = mmio::read(GPFSEL1);
        fsel1 = (fsel1 & !(0b111 << shift)) | (0b010 << shift);
        mmio::write(GPFSEL1, fsel1);

        // 2. Disable PWM0 before changing clock
        mmio::write(PWM_CTL, 0);
        short_delay();

        // 3. Stop PWM clock in Clock Manager and wait for BUSY to clear
        mmio::write(CM_PWMCTL, CM_PASSWD | CM_SRC_OSC);
        short_delay();
        let mut timeout = 10_000u32;
        while (mmio::read(CM_PWMCTL) & CM_CTL_BUSY) != 0 && timeout > 0 {
            timeout -= 1;
            core::hint::spin_loop();
        }
        if (mmio::read(CM_PWMCTL) & CM_CTL_BUSY) != 0 {
            mmio::write(CM_PWMCTL, CM_PASSWD | CM_CTL_KILL);
            short_delay();
        }

        // 4. Set integer clock divider = 19 (19.2 MHz / 19 ≈ 1.01 MHz)
        mmio::write(CM_PWMDIV, CM_PASSWD | (19 << 12));
        short_delay();

        // 5. Enable PWM clock with Oscillator source
        mmio::write(CM_PWMCTL, CM_PASSWD | CM_CTL_ENAB | CM_SRC_OSC);
        short_delay();

        // 6. Configure PWM0 Channel 1: Range = 100 (10.1 kHz), Duty = 100%
        mmio::write(PWM_RNG1, 100);
        mmio::write(PWM_DAT1, 100);
        mmio::write(PWM_CTL, PWM_CTL_MSEN1 | PWM_CTL_PWEN1);
    }

    set_brightness(100);
    crate::println!(
        "[OK] PWM0: GPIO {} (ALT5) @ 10.1kHz Mark/Space (100% duty)",
        PIN_LCD_BL
    );
}

/// Set the Argon POD LCD backlight brightness (`percent` clamped to `10..=100` so screen stays visible).
pub fn set_brightness(percent: u32) {
    let pct = percent.clamp(10, 100);
    BRIGHTNESS_PCT.with(|b| *b = pct);

    if !crate::bsp::is_qemu() {
        unsafe {
            mmio::write(PWM_DAT1, pct);
        }
    }
}

/// Get the current LCD backlight brightness percentage (`10..=100`).
pub fn brightness() -> u32 {
    BRIGHTNESS_PCT.with(|b| *b)
}

/// Cycle LCD backlight brightness through 5 presets (`100% -> 75% -> 50% -> 25% -> 10% -> 100%`).
pub fn cycle_brightness() -> u32 {
    let cur = brightness();
    let next = match cur {
        80..=100 => 75,
        60..=79 => 50,
        35..=59 => 25,
        15..=34 => 10,
        _ => 100,
    };
    set_brightness(next);
    crate::println!("[PWM0] Backlight brightness set to {}%", next);
    next
}
