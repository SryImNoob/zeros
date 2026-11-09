// ZerOS — VideoCore Mailbox Interface
//
// The Raspberry Pi uses a mailbox system for ARM ↔ GPU communication.
// Channel 8 is the "property tags" channel for framebuffer, clock, etc.
//
// Protocol:
//   1. Prepare a 16-byte aligned buffer with property tags
//   2. Write buffer address | channel to MBOX_WRITE
//   3. Read response from MBOX_READ (poll until channel matches)
//   4. Parse response tags in the buffer

use crate::drivers::mmio;

// ─── Mailbox registers ───
const MBOX_BASE: usize = mmio::PERIPHERAL_BASE + 0x0000_B880;
const MBOX_READ: usize = MBOX_BASE + 0x00;
const MBOX_STATUS: usize = MBOX_BASE + 0x18;
const MBOX_WRITE: usize = MBOX_BASE + 0x20;

const MBOX_FULL: u32 = 1 << 31;
const MBOX_EMPTY: u32 = 1 << 30;
const CHANNEL_PROP: u32 = 8;

// ─── Property tag IDs ───
pub const TAG_SET_PHYS_SIZE: u32 = 0x0004_8003;
pub const TAG_SET_VIRT_SIZE: u32 = 0x0004_8004;
pub const TAG_SET_DEPTH: u32 = 0x0004_8005;
pub const TAG_SET_PIXEL_ORDER: u32 = 0x0004_8006;
pub const TAG_ALLOC_BUFFER: u32 = 0x0004_0001;
pub const TAG_GET_PITCH: u32 = 0x0004_0008;
pub const TAG_GET_CLOCK_RATE: u32 = 0x0003_0002;
pub const TAG_GET_TEMPERATURE: u32 = 0x0003_0006;
pub const TAG_END: u32 = 0;

// Response codes
const RESPONSE_OK: u32 = 0x8000_0000;

/// Clean and invalidate data cache lines covering `buffer` (64-byte cache lines on Cortex-A53)
unsafe fn flush_cache_range(start_addr: usize, byte_len: usize) {
    let start = start_addr & !63;
    let end = (start_addr + byte_len + 63) & !63;
    let mut addr = start;
    while addr < end {
        core::arch::asm!("dc civac, {}", in(reg) addr);
        addr += 64;
    }
    core::arch::asm!("dsb sy");
}

/// Send a property tag request via the mailbox.
/// `buffer` must be a 16-byte aligned mutable slice of u32.
/// Returns true on success.
pub fn call(buffer: &mut [u32]) -> bool {
    let addr = buffer.as_ptr() as u32;
    assert!(addr & 0xF == 0, "Mailbox buffer must be 16-byte aligned");
    let byte_len = buffer.len() * 4;

    unsafe {
        // Flush CPU L1/L2 cache lines so VideoCore GPU reads fresh data from RAM
        flush_cache_range(addr as usize, byte_len);

        // Wait for mailbox not full
        while mmio::read(MBOX_STATUS) & MBOX_FULL != 0 {
            core::hint::spin_loop();
        }

        // Write address (upper 28 bits) | channel (lower 4 bits)
        mmio::write(MBOX_WRITE, addr | CHANNEL_PROP);

        // Wait for response
        loop {
            while mmio::read(MBOX_STATUS) & MBOX_EMPTY != 0 {
                core::hint::spin_loop();
            }

            let resp = mmio::read(MBOX_READ);
            if resp & 0xF == CHANNEL_PROP {
                // Invalidate cache lines so CPU reads GPU's response from RAM
                flush_cache_range(addr as usize, byte_len);
                return buffer[1] == RESPONSE_OK;
            }
        }
    }
}

#[repr(C, align(16))]
struct AlignedPropBuf([u32; 8]);

/// Query the VideoCore GPU for the SoC die temperature in millicelsius (e.g. 42500 = 42.5°C).
pub fn get_soc_temp_mc() -> Option<u32> {
    let mut msg = AlignedPropBuf([
        8 * 4,               // Total buffer size in bytes
        0,                   // Request code
        TAG_GET_TEMPERATURE, // Tag: Get Temperature
        8,                   // Value buffer size (8 bytes)
        0,                   // Request length
        0,                   // Temp ID 0 (SoC)
        0,                   // Returned temperature (millicelsius)
        TAG_END,
    ]);
    if call(&mut msg.0) && msg.0[6] > 0 {
        Some(msg.0[6])
    } else {
        None
    }
}

/// Query the VideoCore GPU for the current ARM CPU clock rate in Hz (Clock ID = 3).
pub fn get_arm_clock_hz() -> Option<u32> {
    let mut msg = AlignedPropBuf([
        8 * 4,              // Total buffer size in bytes
        0,                  // Request code
        TAG_GET_CLOCK_RATE, // Tag: Get Clock Rate
        8,                  // Value buffer size (8 bytes)
        0,                  // Request length
        3,                  // Clock ID 3 = ARM CPU
        0,                  // Returned frequency (Hz)
        TAG_END,
    ]);
    if call(&mut msg.0) && msg.0[6] > 0 {
        Some(msg.0[6])
    } else {
        None
    }
}
