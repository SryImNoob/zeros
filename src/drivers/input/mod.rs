// ZerOS — Input Device Drivers (4 Tactile Buttons, NEC IR Receiver, Resistive Touch)

pub mod buttons;
pub mod ir;
pub mod touch;

pub use buttons::{
    button_state, has_events, init, is_pressed, poll_buttons, poll_event, push_event, InputEvent,
};
