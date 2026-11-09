// ZerOS — Kernel Heap Allocator
//
// A linked-list free-list allocator implementing GlobalAlloc.
// Enables the `alloc` crate (Vec, Box, String, etc.)
//
// Design:
//   - Heap occupies a fixed region after the kernel image
//   - Each free block stores { size, next_ptr } at its start (16 bytes)
//   - Allocated blocks store { size, back_ptr } in a 16-byte header
//   - Data is returned at header + padding, aligned to the requested alignment
//   - Back-pointer stored at data_ptr - 8 for O(1) deallocation

use core::alloc::{GlobalAlloc, Layout};
use core::ptr;

/// Heap size: 256 KB (backed by physical memory after kernel image)
const HEAP_SIZE: usize = 256 * 1024;

/// Minimum block size (must fit FreeNode = 16 bytes)
const MIN_BLOCK: usize = 32;

/// Free list node — stored at the start of each free block
#[repr(C)]
struct FreeNode {
    size: usize,          // Total block size (including this header)
    next: *mut FreeNode,  // Pointer to next free block (or null)
}

const FREE_NODE_SIZE: usize = core::mem::size_of::<FreeNode>(); // 16 bytes

/// The global kernel allocator
pub struct KernelAllocator;

unsafe impl Sync for KernelAllocator {}

#[global_allocator]
static ALLOCATOR: KernelAllocator = KernelAllocator;

/// Free list head
static mut FREE_HEAD: *mut FreeNode = ptr::null_mut();

/// Heap bounds (set during init)
static mut HEAP_START: usize = 0;
static mut HEAP_END: usize = 0;
static mut INITIALIZED: bool = false;

/// Initialize the heap allocator.
/// Returns the end address of the heap (for page allocator to start after).
pub fn init(start: usize) -> usize {
    assert!(start % 16 == 0, "Heap start must be 16-byte aligned");

    let end = start + HEAP_SIZE;

    unsafe {
        HEAP_START = start;
        HEAP_END = end;

        // Initialize with a single free block spanning the entire heap
        let node = start as *mut FreeNode;
        (*node).size = HEAP_SIZE;
        (*node).next = ptr::null_mut();
        FREE_HEAD = node;
        INITIALIZED = true;
    }

    crate::println!("[OK] Heap: {}KB at 0x{:08x}-0x{:08x}", HEAP_SIZE / 1024, start, end);

    end
}

fn align_up(val: usize, align: usize) -> usize {
    (val + align - 1) & !(align - 1)
}

unsafe impl GlobalAlloc for KernelAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if !INITIALIZED {
            return ptr::null_mut();
        }

        crate::interrupts::without_interrupts(|| {
            let align = layout.align().max(16);
            let data_size = layout.size().max(FREE_NODE_SIZE);

            let mut prev: *mut FreeNode = ptr::null_mut();
            let mut curr = FREE_HEAD;

            while !curr.is_null() {
                let block_start = curr as usize;
                let block_size = (*curr).size;

                // Calculate aligned data start (leave room for back-pointer at data-8)
                let data_start = align_up(block_start + 16, align);
                let total_needed = (data_start - block_start) + data_size;

                if block_size >= total_needed {
                    let remaining = block_size - total_needed;

                    if remaining >= MIN_BLOCK {
                        // Split: create a new free block after this allocation
                        let new_free = (block_start + total_needed) as *mut FreeNode;
                        (*new_free).size = remaining;
                        (*new_free).next = (*curr).next;

                        (*curr).size = total_needed;

                        // Update free list
                        if prev.is_null() {
                            FREE_HEAD = new_free;
                        } else {
                            (*prev).next = new_free;
                        }
                    } else {
                        // Use entire block (don't split — avoids tiny fragments)
                        if prev.is_null() {
                            FREE_HEAD = (*curr).next;
                        } else {
                            (*prev).next = (*curr).next;
                        }
                    }

                    // Store back-pointer at data_start - 8
                    // (block_start is recoverable from this)
                    *((data_start - 8) as *mut usize) = block_start;

                    return data_start as *mut u8;
                }

                prev = curr;
                curr = (*curr).next;
            }

            ptr::null_mut() // Out of memory
        })
    }

    unsafe fn dealloc(&self, ptr: *mut u8, _layout: Layout) {
        if ptr.is_null() {
            return;
        }

        crate::interrupts::without_interrupts(|| {
            let data_start = ptr as usize;

            // Read back-pointer to find block start
            let block_start = *((data_start - 8) as *const usize);

            // Recover the block as a free node
            let node = block_start as *mut FreeNode;
            // size field is still valid at block_start

            // Insert at head of free list (O(1))
            (*node).next = FREE_HEAD;
            FREE_HEAD = node;
        })
    }
}

/// Get heap usage statistics: (used_bytes, total_bytes)
#[allow(dead_code)]
pub fn stats() -> (usize, usize) {
    crate::interrupts::without_interrupts(|| unsafe {
        let mut free_bytes: usize = 0;
        let mut curr = FREE_HEAD;
        while !curr.is_null() {
            free_bytes += (*curr).size;
            curr = (*curr).next;
        }
        (HEAP_SIZE - free_bytes, HEAP_SIZE)
    })
}
