// ZerOS — GPIO Driver
//
// BCM2710 GPIO for basic pin control.
// Primary use: toggle the activity LED (GPIO 29, active-low, on-board).
//
// The activity LED is on the Pi Zero 2 W board itself,
// NOT on the GPIO header, so it works even with Argon POD attached!

use crate::mmio;

/// GPIO register offsets from GPIO_BASE (0x3F200000)
const GPFSEL2: usize = mmio::GPIO_BASE + 0x08;  // Function select for GPIO 20-29
const GPSET0: usize = mmio::GPIO_BASE + 0x1C;   // Pin output set 0 (GPIO 0-31)
const GPCLR0: usize = mmio::GPIO_BASE + 0x28;   // Pin output clear 0 (GPIO 0-31)

/// Activity LED pin
const ACT_LED_PIN: u32 = 29;

/// Track LED state (no atomic needed — only toggled from IRQ context)
static mut LED_ON: bool = false;

/// Initialize GPIO (set activity LED pin as output)
pub fn init() {
    unsafe {
        // GPFSEL2 controls GPIO 20-29
        // GPIO 29 is bits [29:27] — set to 001 (output)
        let mut fsel = mmio::read(GPFSEL2);
        fsel &= !(0b111 << 27);    // Clear bits [29:27]
        fsel |= 0b001 << 27;       // Set to output
        mmio::write(GPFSEL2, fsel);

        // Start with LED off (GPIO 29 is active-low, so SET = off)
        mmio::write(GPSET0, 1 << ACT_LED_PIN);
    }

    crate::println!("[OK] GPIO: activity LED (GPIO {}) configured", ACT_LED_PIN);
}

/// Turn activity LED on (active-low: clear pin)
#[allow(dead_code)]
pub fn led_on() {
    unsafe {
        mmio::write(GPCLR0, 1 << ACT_LED_PIN);
        LED_ON = true;
    }
}

/// Turn activity LED off (active-low: set pin)
#[allow(dead_code)]
pub fn led_off() {
    unsafe {
        mmio::write(GPSET0, 1 << ACT_LED_PIN);
        LED_ON = false;
    }
}

/// Toggle the activity LED
pub fn toggle_activity_led() {
    unsafe {
        if LED_ON {
            led_off();
        } else {
            led_on();
        }
    }
}
