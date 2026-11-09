// ZerOS — Bare-Metal Synchronization Primitives
//
// On a single-core Cortex-A53 with Non-Cacheable Normal memory (where
// hardware exclusive monitors `ldxr`/`stxr` cannot be used), concurrency
// only arises between normal execution and interrupt handlers (IRQs).
//
// `IrqSafeLock<T>` provides safe interior mutability without `static mut`
// or `ldxr`/`stxr` by masking IRQs for the duration of a critical section.

use core::cell::UnsafeCell;

/// Interrupt-safe critical section lock for single-core bare-metal kernel state.
pub struct IrqSafeLock<T> {
    data: UnsafeCell<T>,
}

unsafe impl<T: Send> Sync for IrqSafeLock<T> {}
unsafe impl<T: Send> Send for IrqSafeLock<T> {}

impl<T> IrqSafeLock<T> {
    /// Create a new interrupt-safe lock wrapping `val`.
    pub const fn new(val: T) -> Self {
        Self {
            data: UnsafeCell::new(val),
        }
    }

    /// Execute a closure with exclusive mutable access to the protected data
    /// while hardware IRQs are masked on the current core.
    #[inline]
    pub fn with<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        let daif: u64;
        unsafe {
            core::arch::asm!(
                "mrs {0}, daif",
                "msr daifset, #2",
                out(reg) daif,
                options(nomem, nostack)
            );
        }

        let result = unsafe { f(&mut *self.data.get()) };

        unsafe {
            core::arch::asm!(
                "msr daif, {0}",
                in(reg) daif,
                options(nomem, nostack)
            );
        }

        result
    }
}
