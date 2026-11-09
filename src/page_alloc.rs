// ZerOS — Physical Page Frame Allocator
//
// Uses a simple free-list stored in the free pages themselves (zero extra memory).
// Each free page stores a pointer to the next free page at its start.
// O(1) allocation and deallocation (LIFO stack).
//
// Memory layout:
//   0x00000000 - 0x0007FFFF : Stack
//   0x00080000 - __kernel_end : Kernel image
//   __kernel_end + HEAP_SIZE : Heap (managed by heap.rs)
//   After heap - 0x3EFFFFFF : Free pages (managed here)
//   0x3F000000+ : Peripherals (not managed)

use core::ptr;

/// Page size: 4 KB
pub const PAGE_SIZE: usize = 4096;

/// End of usable RAM (peripheral MMIO starts here)
const RAM_END: usize = 0x3F00_0000;

/// Free page list head (each free page stores a pointer to the next)
static mut FREE_HEAD: *mut usize = ptr::null_mut();

/// Number of free pages available
static mut FREE_COUNT: usize = 0;

/// Total number of pages ever registered
static mut TOTAL_PAGES: usize = 0;

extern "C" {
    static __kernel_end: u8;
}

/// Get the kernel end address (page-aligned)
pub fn kernel_end_addr() -> usize {
    unsafe { &__kernel_end as *const u8 as usize }
}

/// Initialize the page allocator.
/// `reserved_end` is the first address AFTER all reserved regions (kernel + heap).
/// All pages from `reserved_end` to `RAM_END` become free.
pub fn init(reserved_end: usize) {
    assert!(reserved_end % PAGE_SIZE == 0, "reserved_end must be page-aligned");
    assert!(reserved_end < RAM_END, "No free memory available");

    let mut count: usize = 0;
    let mut addr = reserved_end;

    unsafe {
        while addr + PAGE_SIZE <= RAM_END {
            // Store current free list head at the start of this page
            *(addr as *mut usize) = FREE_HEAD as usize;
            FREE_HEAD = addr as *mut usize;
            count += 1;
            addr += PAGE_SIZE;
        }

        FREE_COUNT = count;
        TOTAL_PAGES = count;
    }

    let free_mb = count * PAGE_SIZE / 1024 / 1024;
    crate::println!("[OK] Page allocator: {} pages free ({}MB), range 0x{:08x}-0x{:08x}",
        count, free_mb, reserved_end, RAM_END);
}

/// Allocate a single physical page (4KB, zeroed).
/// Returns the physical address, or None if out of memory.
pub fn alloc_page() -> Option<usize> {
    crate::interrupts::without_interrupts(|| unsafe {
        if FREE_HEAD.is_null() {
            return None;
        }

        let page = FREE_HEAD as usize;

        // Read next pointer BEFORE zeroing
        let next = *FREE_HEAD;
        FREE_HEAD = next as *mut usize;
        FREE_COUNT -= 1;

        // Zero the page
        ptr::write_bytes(page as *mut u8, 0, PAGE_SIZE);

        Some(page)
    })
}

/// Free a physical page back to the allocator.
/// The page address must be page-aligned and within managed range.
#[allow(dead_code)]
pub fn free_page(addr: usize) {
    assert!(addr % PAGE_SIZE == 0, "free_page: address not page-aligned");
    assert!(addr >= kernel_end_addr() && addr < RAM_END, "free_page: address out of range");

    crate::interrupts::without_interrupts(|| unsafe {
        *(addr as *mut usize) = FREE_HEAD as usize;
        FREE_HEAD = addr as *mut usize;
        FREE_COUNT += 1;
    })
}

/// Get the number of free pages
pub fn free_count() -> usize {
    unsafe { FREE_COUNT }
}

/// Get the total number of managed pages
#[allow(dead_code)]
pub fn total_count() -> usize {
    unsafe { TOTAL_PAGES }
}

/// Helper: align a value up to the given alignment
pub fn align_up(val: usize, align: usize) -> usize {
    (val + align - 1) & !(align - 1)
}
