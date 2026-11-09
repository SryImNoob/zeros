// ZerOS — Input System (Buttons + IR Receiver)
//
// Argon POD has 4 hardware buttons and an IR receiver:
//   Button 1: GPIO 16  (active low, internal pull-up)
//   Button 2: GPIO 20
//   Button 3: GPIO 21
//   Button 4: GPIO 26
//   IR Recv:  GPIO 23
//
// Design:
//   - Buttons are polled with software debouncing (no GPIO IRQ needed for now)
//   - Event queue stores button press/release events
//   - IR decoding is NEC protocol (38kHz carrier, common for TV remotes)

#![allow(dead_code)]

use crate::bsp::argon_pod::{BUTTON_PINS, PIN_BTN_1, PIN_BTN_2, PIN_BTN_3, PIN_BTN_4, PIN_IR_RECV};
use crate::drivers::mmio;
use crate::sync::IrqSafeLock;
use core::sync::atomic::{AtomicU32, Ordering};

const BUTTONS: [u32; 4] = BUTTON_PINS;

// ─── GPIO registers ───
const GPIO_BASE: usize = mmio::PERIPHERAL_BASE + 0x0020_0000;
const GPIO_GPLEV0: usize = GPIO_BASE + 0x34; // Pin Level (read)
const GPIO_GPPUD:  usize = GPIO_BASE + 0x94; // Pull-Up/Down Enable
const GPIO_GPPUDCLK0: usize = GPIO_BASE + 0x98; // Pull-Up/Down Clock

// ─── Debounce state ───
/// Stores last raw button state (bit per button: bit 0 = BTN_1, etc.)
static BUTTON_STATE: AtomicU32 = AtomicU32::new(0);
/// Stores debounced button state
static BUTTON_DEBOUNCED: AtomicU32 = AtomicU32::new(0);
/// Debounce counter per button (packed: 4 bits each × 4 buttons)
static DEBOUNCE_COUNT: AtomicU32 = AtomicU32::new(0);

/// Debounce threshold (number of consistent samples before accepting state change)
const DEBOUNCE_THRESHOLD: u32 = 3;

// ─── Event Queue ───
/// Input event types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputEvent {
    ButtonPress(u8),    // Button index 0-3
    ButtonRelease(u8),  // Button index 0-3
    IrCode(u32),        // NEC IR code (addr + command)
    Touch(u16, u16),    // Resistive touch screen coordinates (x, y) in 320x240
}

/// Simple ring buffer for events protected by `IrqSafeLock` (no `static mut`)
const EVENT_QUEUE_SIZE: usize = 16;

struct EventQueue {
    buf: [Option<InputEvent>; EVENT_QUEUE_SIZE],
    head: usize,
    tail: usize,
}

static EVENT_QUEUE: IrqSafeLock<EventQueue> = IrqSafeLock::new(EventQueue {
    buf: [None; EVENT_QUEUE_SIZE],
    head: 0,
    tail: 0,
});

/// Push an event into the queue
pub fn push_event(event: InputEvent) {
    EVENT_QUEUE.with(|q| {
        let next_head = (q.head + 1) % EVENT_QUEUE_SIZE;
        if next_head != q.tail {
            q.buf[q.head] = Some(event);
            q.head = next_head;
        }
    });
}

/// Pop an event from the queue
pub fn poll_event() -> Option<InputEvent> {
    EVENT_QUEUE.with(|q| {
        if q.tail == q.head {
            return None;
        }
        let event = q.buf[q.tail].take();
        q.tail = (q.tail + 1) % EVENT_QUEUE_SIZE;
        event
    })
}

/// Check if the event queue has pending events
pub fn has_events() -> bool {
    EVENT_QUEUE.with(|q| q.tail != q.head)
}

/// Initialize the input system.
/// Configures button GPIO pins as inputs with pull-ups, IR input, and Touch PENIRQ.
pub fn init() {
    // Configure each button pin as input with pull-up
    for &pin in &BUTTONS {
        gpio_input_pullup(pin);
    }

    // Configure IR pin as input (no pull needed, has external pull-up on module)
    gpio_input(PIN_IR_RECV);

    // Configure Touch PENIRQ pin (GPIO 17)
    super::touch::init();

    crate::println!("[OK] Input: 4 buttons (GPIO {},{},{},{}), IR (GPIO {}), Touch (GPIO 17)",
        PIN_BTN_1, PIN_BTN_2, PIN_BTN_3, PIN_BTN_4, PIN_IR_RECV);
}

/// Poll buttons for state changes (call from timer tick or main loop).
/// Implements software debouncing.
/// On QEMU, GPIO pins read as LOW (no pull-up emulation), causing false
/// presses. The caller can check init state to skip polling if needed.
pub fn poll_buttons() {
    // Skip if all 4 buttons read as pressed simultaneously on first poll —
    // this means we're likely in QEMU where GPIO reads as 0
    static SKIP: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
    static FIRST_POLL: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(true);

    if FIRST_POLL.load(Ordering::Relaxed) {
        FIRST_POLL.store(false, Ordering::Relaxed);
        let raw = read_buttons_raw();
        if raw == 0x0F {
            // All 4 buttons "pressed" = QEMU, skip all polling
            SKIP.store(true, Ordering::Relaxed);
            return;
        }
    }

    if SKIP.load(Ordering::Relaxed) {
        return;
    }

    let raw = read_buttons_raw();
    let prev_debounced = BUTTON_DEBOUNCED.load(Ordering::Relaxed);
    let prev_raw = BUTTON_STATE.load(Ordering::Relaxed);
    let mut counts = DEBOUNCE_COUNT.load(Ordering::Relaxed);
    let mut new_debounced = prev_debounced;

    for i in 0..4u32 {
        let raw_bit = (raw >> i) & 1;
        let prev_bit = (prev_raw >> i) & 1;
        let debounced_bit = (prev_debounced >> i) & 1;

        // Extract this button's 4-bit counter
        let shift = i * 4;
        let count = (counts >> shift) & 0xF;

        if raw_bit == prev_bit {
            // Stable reading — increment counter
            let new_count = (count + 1).min(DEBOUNCE_THRESHOLD + 1);
            counts = (counts & !(0xF << shift)) | (new_count << shift);

            if new_count >= DEBOUNCE_THRESHOLD && raw_bit != debounced_bit {
                // State has changed and is stable
                if raw_bit == 1 {
                    new_debounced |= 1 << i;
                    push_event(InputEvent::ButtonPress(i as u8));
                } else {
                    new_debounced &= !(1 << i);
                    push_event(InputEvent::ButtonRelease(i as u8));
                }
            }
        } else {
            // Reading changed — reset counter
            counts = counts & !(0xF << shift);
        }
    }

    BUTTON_STATE.store(raw, Ordering::Relaxed);
    BUTTON_DEBOUNCED.store(new_debounced, Ordering::Relaxed);
    DEBOUNCE_COUNT.store(counts, Ordering::Relaxed);
}

/// Read raw button states (active low → inverted to: 1=pressed, 0=released).
/// Returns a bitmask: bit 0 = BTN_1, bit 1 = BTN_2, etc.
fn read_buttons_raw() -> u32 {
    let level = unsafe { mmio::read(GPIO_GPLEV0) };
    let mut result = 0u32;
    for (i, &pin) in BUTTONS.iter().enumerate() {
        // Active low: pin LOW means pressed → invert
        if level & (1 << pin) == 0 {
            result |= 1 << i;
        }
    }
    result
}

/// Get current debounced button state as a bitmask.
/// Bit 0 = Button 1, etc. 1 = pressed.
pub fn button_state() -> u32 {
    BUTTON_DEBOUNCED.load(Ordering::Relaxed)
}

/// Check if a specific button is currently pressed (debounced).
pub fn is_pressed(button: u8) -> bool {
    assert!(button < 4, "Button index must be 0-3");
    button_state() & (1 << button) != 0
}

// ─── GPIO helpers ───

/// Configure a GPIO pin as input (function = 000)
fn gpio_input(pin: u32) {
    let reg = GPIO_BASE + ((pin / 10) * 4) as usize;
    let shift = ((pin % 10) * 3) as u32;
    unsafe {
        let val = mmio::read(reg);
        let val = val & !(0b111 << shift); // Function 000 = input
        mmio::write(reg, val);
    }
}

/// Configure a GPIO pin as input with internal pull-up resistor.
/// BCM2835/2710 pull-up/down sequence:
///   1. Write to GPPUD (0=off, 1=pull-down, 2=pull-up)
///   2. Wait 150 cycles
///   3. Write to GPPUDCLK0/1 to clock the control signal
///   4. Wait 150 cycles
///   5. Clear GPPUD and GPPUDCLK
fn gpio_input_pullup(pin: u32) {
    // First set as input
    gpio_input(pin);

    unsafe {
        // Step 1: Select pull-up
        mmio::write(GPIO_GPPUD, 2); // 2 = pull-up

        // Step 2: Wait 150 cycles
        for _ in 0..150 {
            core::hint::spin_loop();
        }

        // Step 3: Clock the control signal for this pin
        mmio::write(GPIO_GPPUDCLK0, 1 << pin);

        // Step 4: Wait 150 cycles
        for _ in 0..150 {
            core::hint::spin_loop();
        }

        // Step 5: Clear
        mmio::write(GPIO_GPPUD, 0);
        mmio::write(GPIO_GPPUDCLK0, 0);
    }
}
