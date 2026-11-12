// ZerOS — MMU (Memory Management Unit) Setup (`M9.5`)
//
// Configures AArch64 virtual memory with identity mapping (VA = PA).
// Uses 4KB granule with 2MB block mappings at L2 level.
//
// Address space (39-bit VA, T0SZ=25):
//   L1 table → 512 entries × 1GB each
//   L2 table → 512 entries × 2MB each
//
// Memory map:
//   0x00000000 - 0x3EFFFFFF : Normal memory (RAM, ~1008MB, Normal Non-Cacheable for warm-boot & DMA coherency)
//   0x3F000000 - 0x3FFFFFFF : Device memory (BCM2710 peripherals, Device-nGnRnE)
//   0x40000000 - 0x401FFFFF : Device memory (ARM local peripherals, Device-nGnRnE)

use crate::mm::page_alloc;

// ─── Page table entry flags ───

/// Block descriptor: [1:0] = 01
const BLOCK_DESC: u64 = 0b01;

/// Table descriptor: [1:0] = 11
const TABLE_DESC: u64 = 0b11;

/// Access Flag (bit 10) — must be set or access triggers a fault
const AF: u64 = 1 << 10;

/// Shareability: Inner Shareable (bits [9:8] = 11)
const SH_INNER: u64 = 0b11 << 8;

/// AttrIndx = 0 → MAIR Attr0 (Normal memory)
const ATTR_NORMAL: u64 = 0 << 2;

/// AttrIndx = 1 → MAIR Attr1 (Device memory)
const ATTR_DEVICE: u64 = 1 << 2;

/// Normal memory block flags (Inner Shareable)
const NORMAL_BLOCK: u64 = BLOCK_DESC | ATTR_NORMAL | SH_INNER | AF;

/// Device memory block flags (non-cacheable, non-shareable)
const DEVICE_BLOCK: u64 = BLOCK_DESC | ATTR_DEVICE | AF;

/// 2MB block size
const BLOCK_2MB: usize = 2 * 1024 * 1024;

/// Number of 2MB blocks for RAM (up to peripheral base at 0x3F000000)
const RAM_BLOCKS: usize = 504; // 504 × 2MB = 1008MB

/// Clean and invalidate L1/L2 data cache lines covering `[start_addr, start_addr + byte_len)`
/// to Point of Coherency (`dc civac`).
#[inline]
pub fn dma_clean_invalidate(start_addr: usize, byte_len: usize) {
    let start = start_addr & !63;
    let end = (start_addr + byte_len + 63) & !63;
    let mut addr = start;
    unsafe {
        while addr < end {
            core::arch::asm!("dc civac, {}", in(reg) addr, options(nostack, preserves_flags));
            addr += 64;
        }
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
    }
}

/// Invalidate L1/L2 data cache lines covering `[start_addr, start_addr + byte_len)` (`dc ivac`).
#[inline]
pub fn dma_invalidate(start_addr: usize, byte_len: usize) {
    let start = start_addr & !63;
    let end = (start_addr + byte_len + 63) & !63;
    let mut addr = start;
    unsafe {
        while addr < end {
            core::arch::asm!("dc ivac, {}", in(reg) addr, options(nostack, preserves_flags));
            addr += 64;
        }
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
    }
}

/// Initialize the MMU with identity mapping and enable Instruction Cache + Normal Memory access.
pub fn init() {
    // ── Allocate page tables from page allocator ──
    let l1_addr = page_alloc::alloc_page().expect("MMU: failed to alloc L1 table");
    let l2_0_addr = page_alloc::alloc_page().expect("MMU: failed to alloc L2 table (0-1GB)");
    let l2_1_addr = page_alloc::alloc_page().expect("MMU: failed to alloc L2 table (1-2GB)");

    let l1 = l1_addr as *mut u64;
    let l2_0 = l2_0_addr as *mut u64;
    let l2_1 = l2_1_addr as *mut u64;

    unsafe {
        // ── Fill L2_0: covers 0x00000000 - 0x3FFFFFFF (0-1GB) ──
        for i in 0..RAM_BLOCKS {
            let addr = (i * BLOCK_2MB) as u64;
            *l2_0.add(i) = addr | NORMAL_BLOCK;
        }

        // Peripherals: entries 504..511 → 0x3F000000 - 0x3FFFFFFF (device memory)
        for i in 504..512 {
            let addr = (i * BLOCK_2MB) as u64;
            *l2_0.add(i) = addr | DEVICE_BLOCK;
        }

        // ── Fill L2_1: covers 0x40000000 - 0x7FFFFFFF (1-2GB) ──
        // ARM local peripherals: entry 0 → 0x40000000 - 0x401FFFFF
        *l2_1.add(0) = 0x4000_0000u64 | DEVICE_BLOCK;

        // ── Fill L1: top-level table ──
        *l1.add(0) = (l2_0_addr as u64) | TABLE_DESC;
        *l1.add(1) = (l2_1_addr as u64) | TABLE_DESC;

        // ── Configure MMU registers ──

        // MAIR_EL1: Memory Attribute Indirection Register
        //   Attr0 [7:0]  = 0x44 → Normal, Inner/Outer Non-Cacheable (100% coherent with GPU/DMA & warm reset)
        //   Attr1 [15:8] = 0x00 → Device-nGnRnE (strongly ordered)
        let mair: u64 = 0x44;
        core::arch::asm!("msr mair_el1, {}", in(reg) mair);

        // TCR_EL1: Translation Control Register
        let tcr: u64 = 25
            | (0b00 << 8)   // IRGN0
            | (0b00 << 10)  // ORGN0
            | (0b11 << 12)  // SH0
            | (0b00 << 14)  // TG0
            | (1u64 << 23); // EPD1
        core::arch::asm!("msr tcr_el1, {}", in(reg) tcr);

        // TTBR0_EL1: Translation Table Base Register
        core::arch::asm!("msr ttbr0_el1, {}", in(reg) l1_addr);
        core::arch::asm!("isb");

        // ── Invalidate TLB and instruction cache ──
        core::arch::asm!(
            "tlbi vmalle1is",
            "dsb ish",
            "isb",
            "ic iallu",
            "dsb ish",
            "isb",
        );

        // ── Enable MMU + instruction cache (data is Normal Non-Cacheable for GPU/DMA & warm-boot coherency) ──
        let mut sctlr: u64;
        core::arch::asm!("mrs {}, sctlr_el1", out(reg) sctlr);
        sctlr &= !(1 << 2);  // Ensure C (Data cache) is off so stale warm-reboot lines never hit
        sctlr |= (1 << 0)    // M — MMU enable
               | (1 << 12);  // I — Instruction cache enable
        core::arch::asm!("msr sctlr_el1, {}", in(reg) sctlr);
        core::arch::asm!("isb");
    }

    crate::println!("[OK] MMU enabled: identity map, 4KB granule, 2MB blocks");
    crate::println!(
        "     L1=0x{:08x} L2[0-1GB]=0x{:08x} L2[1-2GB]=0x{:08x}",
        l1_addr,
        l2_0_addr,
        l2_1_addr
    );
}
