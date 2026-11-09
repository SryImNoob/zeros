// ZerOS — BCM2710 SPI0 Master Driver
//
// Hardware SPI0 on the Raspberry Pi for communicating with the
// ILI9341 display and ADS7846 touch controller on the Argon POD.
//
// SPI0 pins (active in ALT0 mode):
//   GPIO 10 — MOSI
//   GPIO  9 — MISO
//   GPIO 11 — SCLK
//   GPIO  8 — CE0 (display chip select)
//   GPIO  7 — CE1 (touch chip select)

#![allow(dead_code)]

use crate::drivers::mmio;

// ─── SPI0 Register Offsets ───
const SPI0_BASE: usize = mmio::PERIPHERAL_BASE + 0x0020_4000;

const SPI_CS:   usize = SPI0_BASE + 0x00; // Control and Status
const SPI_FIFO: usize = SPI0_BASE + 0x04; // TX/RX FIFO
const SPI_CLK:  usize = SPI0_BASE + 0x08; // Clock Divider
const SPI_DLEN: usize = SPI0_BASE + 0x0C; // Data Length

// ─── CS register bits ───
const CS_LEN_LONG: u32 = 1 << 25;
const CS_DMA_LEN:  u32 = 1 << 24;
const CS_CSPOL2:   u32 = 1 << 23;
const CS_CSPOL1:   u32 = 1 << 22;
const CS_CSPOL0:   u32 = 1 << 21;
const CS_RXF:      u32 = 1 << 20; // RX FIFO Full
const CS_RXR:      u32 = 1 << 19; // RX FIFO needs reading
const CS_TXD:      u32 = 1 << 18; // TX FIFO can accept data
const CS_RXD:      u32 = 1 << 17; // RX FIFO contains data
const CS_DONE:     u32 = 1 << 16; // Transfer Done
const CS_TE_EN:    u32 = 1 << 15;
const CS_LMONO:    u32 = 1 << 14;
const CS_LEN:      u32 = 1 << 13;
const CS_REN:      u32 = 1 << 12;
const CS_ADCS:     u32 = 1 << 11;
const CS_INTR:     u32 = 1 << 10;
const CS_INTD:     u32 = 1 << 9;
const CS_DMAEN:    u32 = 1 << 8;
const CS_TA:       u32 = 1 << 7;  // Transfer Active
const CS_CSPOL:    u32 = 1 << 6;
const CS_CLEAR_RX: u32 = 1 << 5;
const CS_CLEAR_TX: u32 = 1 << 4;
const CS_CPOL:     u32 = 1 << 3;  // Clock Polarity
const CS_CPHA:     u32 = 1 << 2;  // Clock Phase
const CS_CS_MASK:  u32 = 0b11;    // Chip Select (bits 1:0)

/// Initialize SPI0 hardware.
/// Sets up GPIO pins for ALT0 (SPI) function and configures the SPI controller.
pub fn init() {
    // ── Configure GPIO pins for SPI0 (ALT0) ──
    // GPIO 7  (CE1)  — ALT0
    // GPIO 8  (CE0)  — ALT0
    // GPIO 9  (MISO) — ALT0
    // GPIO 10 (MOSI) — ALT0
    // GPIO 11 (SCLK) — ALT0
    set_gpio_alt0(7);
    set_gpio_alt0(8);
    set_gpio_alt0(9);
    set_gpio_alt0(10);
    set_gpio_alt0(11);

    unsafe {
        // Clear FIFOs and reset
        mmio::write(SPI_CS, CS_CLEAR_RX | CS_CLEAR_TX);

        // Set clock divider for ~12.5MHz SPI clock
        // Core clock on RPi Zero 2 W is 400MHz (250MHz in QEMU)
        // Divider = 32 → 400MHz / 32 = 12.5MHz (within tft9341.dtbo's 16MHz max)
        mmio::write(SPI_CLK, 32);
    }

    crate::println!("[OK] SPI0: initialized, clock divider=32 (~12.5MHz)");
}

/// Transfer a single byte over SPI (full-duplex: sends and receives).
/// `cs` selects the chip: 0 = CE0 (display), 1 = CE1 (touch).
pub fn transfer_byte(cs: u8, tx: u8) -> u8 {
    unsafe {
        // Clear FIFOs
        mmio::write(SPI_CS, CS_CLEAR_RX | CS_CLEAR_TX);

        // Set chip select and start transfer
        let cs_val = (cs as u32) & CS_CS_MASK;
        mmio::write(SPI_CS, cs_val | CS_TA);

        // Wait for TX FIFO ready
        let mut timeout = 100_000u32;
        while mmio::read(SPI_CS) & CS_TXD == 0 {
            timeout -= 1;
            if timeout == 0 {
                mmio::write(SPI_CS, 0);
                return 0;
            }
            core::hint::spin_loop();
        }

        // Write byte to FIFO
        mmio::write(SPI_FIFO, tx as u32);

        // Wait for transfer complete
        timeout = 100_000;
        while mmio::read(SPI_CS) & CS_DONE == 0 {
            timeout -= 1;
            if timeout == 0 {
                break;
            }
            core::hint::spin_loop();
        }

        // Read received byte
        let rx = mmio::read(SPI_FIFO) as u8;

        // Deassert CS (clear TA)
        mmio::write(SPI_CS, 0);

        rx
    }
}

/// Send a buffer of bytes over SPI (TX only, draining RX FIFO continuously).
/// IMPORTANT: BCM2835/2710 SPI0 has a 64-byte RX FIFO. If RX FIFO becomes full
/// during a transfer, the hardware halts transmission until RX FIFO is read!
/// `cs` selects the chip: 0 = CE0 (display), 1 = CE1 (touch).
pub fn write_bytes(cs: u8, data: &[u8]) {
    unsafe {
        // Clear FIFOs
        mmio::write(SPI_CS, CS_CLEAR_RX | CS_CLEAR_TX);

        // Set chip select and start transfer
        let cs_val = (cs as u32) & CS_CS_MASK;
        mmio::write(SPI_CS, cs_val | CS_TA);

        for &byte in data {
            // Drain RX FIFO so it never fills up and stalls the SPI clock
            while mmio::read(SPI_CS) & CS_RXD != 0 {
                let _ = mmio::read(SPI_FIFO);
            }

            // Wait for TX FIFO space
            let mut timeout = 100_000u32;
            while mmio::read(SPI_CS) & CS_TXD == 0 {
                while mmio::read(SPI_CS) & CS_RXD != 0 {
                    let _ = mmio::read(SPI_FIFO);
                }
                timeout -= 1;
                if timeout == 0 {
                    mmio::write(SPI_CS, 0);
                    return;
                }
                core::hint::spin_loop();
            }
            mmio::write(SPI_FIFO, byte as u32);
        }

        // Wait for transfer complete, draining RX FIFO along the way
        let mut timeout = 100_000u32;
        while mmio::read(SPI_CS) & CS_DONE == 0 {
            while mmio::read(SPI_CS) & CS_RXD != 0 {
                let _ = mmio::read(SPI_FIFO);
            }
            timeout -= 1;
            if timeout == 0 {
                break;
            }
            core::hint::spin_loop();
        }

        // Drain any remaining bytes in RX FIFO
        while mmio::read(SPI_CS) & CS_RXD != 0 {
            let _ = mmio::read(SPI_FIFO);
        }

        // Deassert CS
        mmio::write(SPI_CS, 0);
    }
}

/// Repeat a 16-bit big-endian color word `count` times in a single CS-asserted SPI burst.
/// Drains RX FIFO continuously so transfers of any size (e.g. 320×240 = 76,800 pixels)
/// stream at full SPI speed without stalling or toggling CS.
pub fn write_repeat_u16(cs: u8, word: u16, count: usize) {
    let hi = (word >> 8) as u32;
    let lo = (word & 0xFF) as u32;

    unsafe {
        mmio::write(SPI_CS, CS_CLEAR_RX | CS_CLEAR_TX);

        let cs_val = (cs as u32) & CS_CS_MASK;
        mmio::write(SPI_CS, cs_val | CS_TA);

        for _ in 0..count {
            // Send high byte
            while mmio::read(SPI_CS) & CS_TXD == 0 {
                while mmio::read(SPI_CS) & CS_RXD != 0 {
                    let _ = mmio::read(SPI_FIFO);
                }
            }
            mmio::write(SPI_FIFO, hi);

            // Send low byte
            while mmio::read(SPI_CS) & CS_TXD == 0 {
                while mmio::read(SPI_CS) & CS_RXD != 0 {
                    let _ = mmio::read(SPI_FIFO);
                }
            }
            mmio::write(SPI_FIFO, lo);

            // Opportunistically drain RX FIFO
            while mmio::read(SPI_CS) & CS_RXD != 0 {
                let _ = mmio::read(SPI_FIFO);
            }
        }

        while mmio::read(SPI_CS) & CS_DONE == 0 {
            while mmio::read(SPI_CS) & CS_RXD != 0 {
                let _ = mmio::read(SPI_FIFO);
            }
        }

        while mmio::read(SPI_CS) & CS_RXD != 0 {
            let _ = mmio::read(SPI_FIFO);
        }

        mmio::write(SPI_CS, 0);
    }
}

/// Send 16-bit words as big-endian bytes over SPI, draining RX FIFO continuously.
/// Efficient for framebuffer pixel data (RGB565).
pub fn write_u16_be(cs: u8, data: &[u16]) {
    unsafe {
        mmio::write(SPI_CS, CS_CLEAR_RX | CS_CLEAR_TX);

        let cs_val = (cs as u32) & CS_CS_MASK;
        mmio::write(SPI_CS, cs_val | CS_TA);

        for &word in data {
            let hi = (word >> 8) as u8;
            let lo = (word & 0xFF) as u8;

            while mmio::read(SPI_CS) & CS_TXD == 0 {
                while mmio::read(SPI_CS) & CS_RXD != 0 {
                    let _ = mmio::read(SPI_FIFO);
                }
            }
            mmio::write(SPI_FIFO, hi as u32);

            while mmio::read(SPI_CS) & CS_TXD == 0 {
                while mmio::read(SPI_CS) & CS_RXD != 0 {
                    let _ = mmio::read(SPI_FIFO);
                }
            }
            mmio::write(SPI_FIFO, lo as u32);

            while mmio::read(SPI_CS) & CS_RXD != 0 {
                let _ = mmio::read(SPI_FIFO);
            }
        }

        while mmio::read(SPI_CS) & CS_DONE == 0 {
            while mmio::read(SPI_CS) & CS_RXD != 0 {
                let _ = mmio::read(SPI_FIFO);
            }
        }

        while mmio::read(SPI_CS) & CS_RXD != 0 {
            let _ = mmio::read(SPI_FIFO);
        }

        mmio::write(SPI_CS, 0);
    }
}

/// Full-duplex multi-byte SPI transfer at ~1.5MHz clock (`divider = 256`),
/// used for the ADS7846 / XPT2046 touch controller on CE1 (`cs = 1`),
/// then restores the fast 12.5MHz clock (`divider = 32`) for the ILI9341 display.
pub fn transfer_bytes_slow(cs: u8, tx: &[u8], rx: &mut [u8]) {
    let len = tx.len().min(rx.len());
    unsafe {
        mmio::write(SPI_CLK, 256); // ~1.56 MHz (ADS7846 max is 2 MHz)
        mmio::write(SPI_CS, CS_CLEAR_RX | CS_CLEAR_TX);

        let cs_val = (cs as u32) & CS_CS_MASK;
        mmio::write(SPI_CS, cs_val | CS_TA);

        for i in 0..len {
            let mut timeout = 100_000u32;
            while mmio::read(SPI_CS) & CS_TXD == 0 {
                timeout -= 1;
                if timeout == 0 {
                    break;
                }
            }
            mmio::write(SPI_FIFO, tx[i] as u32);

            timeout = 100_000;
            while mmio::read(SPI_CS) & CS_RXD == 0 {
                timeout -= 1;
                if timeout == 0 {
                    break;
                }
            }
            rx[i] = mmio::read(SPI_FIFO) as u8;
        }

        let mut timeout = 100_000u32;
        while mmio::read(SPI_CS) & CS_DONE == 0 {
            timeout -= 1;
            if timeout == 0 {
                break;
            }
        }

        mmio::write(SPI_CS, 0);
        mmio::write(SPI_CLK, 32); // Restore 12.5 MHz for ILI9341 LCD
    }
}

// ─── GPIO helpers ───

const GPIO_BASE: usize = mmio::PERIPHERAL_BASE + 0x0020_0000;

/// Set a GPIO pin to ALT0 function (SPI0)
fn set_gpio_alt0(pin: u32) {
    let reg = GPIO_BASE + ((pin / 10) * 4) as usize;
    let shift = ((pin % 10) * 3) as u32;
    unsafe {
        let val = mmio::read(reg);
        let val = (val & !(0b111 << shift)) | (0b100 << shift); // ALT0 = 0b100
        mmio::write(reg, val);
    }
}
