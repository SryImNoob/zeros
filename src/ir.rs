// ZerOS — NEC IR Protocol Decoder
//
// Decodes NEC infrared remote control signals from the Argon POD IR receiver
// on GPIO 23. The NEC protocol is the most common consumer IR protocol.
//
// NEC Protocol timing (38kHz carrier):
//   Leader:  9ms mark + 4.5ms space
//   Bit '0': 562.5µs mark + 562.5µs space
//   Bit '1': 562.5µs mark + 1687.5µs space
//   Repeat:  9ms mark + 2.25ms space + 562.5µs mark
//
// Frame format: 8-bit address + 8-bit ~address + 8-bit command + 8-bit ~command
// Total: 32 bits, LSB first
//
// This module provides a polling-based decoder. Call ir_poll() frequently
// (e.g. from the timer tick at a high rate) to sample the IR pin.

use crate::mmio;

// ─── GPIO ───
const GPIO_BASE: usize = mmio::PERIPHERAL_BASE + 0x0020_0000;
const GPIO_GPLEV0: usize = GPIO_BASE + 0x34;
const IR_PIN: u32 = 23;

// ─── NEC timing (in microseconds) ───
const LEADER_MARK_US:  u64 = 9000;
const LEADER_SPACE_US: u64 = 4500;
const REPEAT_SPACE_US: u64 = 2250;
const BIT_MARK_US:     u64 = 562;
const BIT_0_SPACE_US:  u64 = 562;
const BIT_1_SPACE_US:  u64 = 1687;
const TOLERANCE_US:    u64 = 300;  // ±300µs tolerance

/// Decoded IR result
#[derive(Debug, Clone, Copy)]
pub struct IrFrame {
    pub address: u8,
    pub command: u8,
    pub is_repeat: bool,
}

/// IR decoder state machine
#[derive(Debug, Clone, Copy, PartialEq)]
enum IrState {
    Idle,
    LeaderMark,
    LeaderSpace,
    DataMark,
    DataSpace,
}

/// Global IR decoder state
pub static mut IR_DECODER: IrDecoder = IrDecoder::new();

pub struct IrDecoder {
    state: IrState,
    last_edge_us: u64,
    bits: u32,
    bit_count: u8,
    last_level: bool,
}

impl IrDecoder {
    pub const fn new() -> Self {
        IrDecoder {
            state: IrState::Idle,
            last_edge_us: 0,
            bits: 0,
            bit_count: 0,
            last_level: true, // IR receiver output is HIGH when idle
        }
    }

    /// Poll the IR pin and attempt to decode.
    /// Returns Some(IrFrame) when a complete frame is decoded.
    /// Should be called frequently (ideally every 50-100µs).
    pub fn poll(&mut self) -> Option<IrFrame> {
        let level = read_ir_pin();
        if level == self.last_level {
            return None; // No edge detected
        }

        let now_us = timestamp_us();
        let duration_us = now_us.wrapping_sub(self.last_edge_us);

        self.last_level = level;
        self.last_edge_us = now_us;

        // IR receiver is active-low: LOW = receiving carrier (mark), HIGH = no carrier (space)
        let is_mark_end = level; // Rising edge = end of mark (active-low receiver)

        match self.state {
            IrState::Idle => {
                if !is_mark_end {
                    // Falling edge: start of leader mark
                    self.state = IrState::LeaderMark;
                }
                None
            }

            IrState::LeaderMark => {
                if is_mark_end && in_range(duration_us, LEADER_MARK_US, TOLERANCE_US * 2) {
                    // Leader mark complete (~9ms), wait for space
                    self.state = IrState::LeaderSpace;
                } else {
                    self.reset();
                }
                None
            }

            IrState::LeaderSpace => {
                if !is_mark_end {
                    // Falling edge: end of leader space
                    if in_range(duration_us, LEADER_SPACE_US, TOLERANCE_US * 2) {
                        // Normal frame: 4.5ms space
                        self.state = IrState::DataMark;
                        self.bits = 0;
                        self.bit_count = 0;
                    } else if in_range(duration_us, REPEAT_SPACE_US, TOLERANCE_US) {
                        // Repeat code: 2.25ms space
                        self.reset();
                        return Some(IrFrame {
                            address: 0,
                            command: 0,
                            is_repeat: true,
                        });
                    } else {
                        self.reset();
                    }
                } else {
                    self.reset();
                }
                None
            }

            IrState::DataMark => {
                if is_mark_end && in_range(duration_us, BIT_MARK_US, TOLERANCE_US) {
                    // Data mark complete (~562µs), wait for space
                    self.state = IrState::DataSpace;
                } else {
                    self.reset();
                }
                None
            }

            IrState::DataSpace => {
                if !is_mark_end {
                    // Falling edge: end of space, determine bit value
                    let bit = if in_range(duration_us, BIT_0_SPACE_US, TOLERANCE_US) {
                        0u32
                    } else if in_range(duration_us, BIT_1_SPACE_US, TOLERANCE_US) {
                        1u32
                    } else {
                        self.reset();
                        return None;
                    };

                    self.bits |= bit << self.bit_count;
                    self.bit_count += 1;

                    if self.bit_count >= 32 {
                        // Frame complete — validate and return
                        let result = self.decode_frame();
                        self.reset();
                        return result;
                    }

                    self.state = IrState::DataMark;
                } else {
                    self.reset();
                }
                None
            }
        }
    }

    /// Decode a complete 32-bit frame
    fn decode_frame(&self) -> Option<IrFrame> {
        let addr     = (self.bits & 0xFF) as u8;
        let addr_inv = ((self.bits >> 8) & 0xFF) as u8;
        let cmd      = ((self.bits >> 16) & 0xFF) as u8;
        let cmd_inv  = ((self.bits >> 24) & 0xFF) as u8;

        // Validate: address + ~address = 0xFF, command + ~command = 0xFF
        if (addr ^ addr_inv) == 0xFF && (cmd ^ cmd_inv) == 0xFF {
            Some(IrFrame {
                address: addr,
                command: cmd,
                is_repeat: false,
            })
        } else {
            // Extended NEC: address is 16-bit, only validate command
            if (cmd ^ cmd_inv) == 0xFF {
                Some(IrFrame {
                    address: addr, // lower 8 bits of extended address
                    command: cmd,
                    is_repeat: false,
                })
            } else {
                None // Invalid frame
            }
        }
    }

    fn reset(&mut self) {
        self.state = IrState::Idle;
        self.bits = 0;
        self.bit_count = 0;
    }
}

// ─── Helpers ───

/// Read the IR receiver pin (active-low: LOW = receiving)
fn read_ir_pin() -> bool {
    let level = unsafe { mmio::read(GPIO_GPLEV0) };
    (level & (1 << IR_PIN)) != 0
}

/// Check if a duration is within range ± tolerance
fn in_range(value: u64, target: u64, tolerance: u64) -> bool {
    value >= target.saturating_sub(tolerance) && value <= target + tolerance
}

/// Get current timestamp in microseconds from the ARM generic timer
fn timestamp_us() -> u64 {
    unsafe {
        let cnt: u64;
        let freq: u64;
        core::arch::asm!("mrs {}, cntpct_el0", out(reg) cnt);
        core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq);
        cnt * 1_000_000 / freq
    }
}
