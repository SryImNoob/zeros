// ZerOS — Board Support Package (BSP): Raspberry Pi Zero 2 W (BCM2710A1)
//
// Centralizes physical memory layout and peripheral base addresses for
// BCM2710A1 (RP3A0-AU SiP, 512MB RAM) + Argon POD expansion module.

#![allow(dead_code)]

pub mod argon_pod;

/// BCM2837 / BCM2710A1 peripheral MMIO base address in physical memory
pub const PERIPHERAL_BASE: usize = 0x3F00_0000;

/// ARM local peripherals base (Core timers, mailbox routing, IRQ source)
pub const LOCAL_BASE: usize = 0x4000_0000;

/// End of kernel-allocatable physical RAM (256MB, safely below GPU memory on 512MB RPi Zero 2 W)
pub const RAM_END: usize = 0x1000_0000;

/// Detect if running inside QEMU `raspi3b` (CNTFRQ_EL0 == 62.5MHz vs 19.2MHz on real BCM2710)
#[inline]
pub fn is_qemu() -> bool {
    crate::arch::timer::is_qemu()
}
