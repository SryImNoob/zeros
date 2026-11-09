// ZerOS — AArch64 Exception Vector Table & Handlers
//
// The vector table has 16 entries (4 groups × 4 exception types).
// Each entry is 0x80 (128) bytes. Table must be aligned to 0x800.
//
// Groups:
//   - Current EL with SP0
//   - Current EL with SPx  ← our kernel runs here (EL1h)
//   - Lower EL using AArch64
//   - Lower EL using AArch32
//
// Exception types per group:
//   - Synchronous
//   - IRQ
//   - FIQ
//   - SError

use core::arch::global_asm;

/// Saved CPU context during an exception
#[repr(C)]
pub struct ExceptionContext {
    pub gpr: [u64; 31],   // x0-x30
    pub elr: u64,         // Exception Link Register
    pub spsr: u64,        // Saved Program Status Register
    pub esr: u64,         // Exception Syndrome Register
}

// ─── Assembly: vector table + save/restore macros ───

global_asm!(r#"

// ---- Context save macro ----
.macro EXCEPTION_SAVE
    sub     sp, sp, #272            // 31 regs + elr + spsr + esr = 34 * 8 = 272 (17 * 16, 16-byte aligned!)
    stp     x0,  x1,  [sp, #0]
    stp     x2,  x3,  [sp, #16]
    stp     x4,  x5,  [sp, #32]
    stp     x6,  x7,  [sp, #48]
    stp     x8,  x9,  [sp, #64]
    stp     x10, x11, [sp, #80]
    stp     x12, x13, [sp, #96]
    stp     x14, x15, [sp, #112]
    stp     x16, x17, [sp, #128]
    stp     x18, x19, [sp, #144]
    stp     x20, x21, [sp, #160]
    stp     x22, x23, [sp, #176]
    stp     x24, x25, [sp, #192]
    stp     x26, x27, [sp, #208]
    stp     x28, x29, [sp, #224]
    str     x30, [sp, #240]
    mrs     x0, elr_el1
    mrs     x1, spsr_el1
    mrs     x2, esr_el1
    stp     x0, x1, [sp, #248]
    str     x2, [sp, #264]
.endm

// ---- Context restore + eret macro ----
.macro EXCEPTION_RESTORE
    ldp     x0, x1, [sp, #248]
    msr     elr_el1, x0
    msr     spsr_el1, x1
    ldp     x0,  x1,  [sp, #0]
    ldp     x2,  x3,  [sp, #16]
    ldp     x4,  x5,  [sp, #32]
    ldp     x6,  x7,  [sp, #48]
    ldp     x8,  x9,  [sp, #64]
    ldp     x10, x11, [sp, #80]
    ldp     x12, x13, [sp, #96]
    ldp     x14, x15, [sp, #112]
    ldp     x16, x17, [sp, #128]
    ldp     x18, x19, [sp, #144]
    ldp     x20, x21, [sp, #160]
    ldp     x22, x23, [sp, #176]
    ldp     x24, x25, [sp, #192]
    ldp     x26, x27, [sp, #208]
    ldp     x28, x29, [sp, #224]
    ldr     x30, [sp, #240]
    add     sp, sp, #272
    eret
.endm

// ============================================================
//  Exception Vector Table — aligned to 0x800
// ============================================================
.balign 0x800
.global exception_vectors
exception_vectors:

// ---- Group 0: Current EL with SP0 ----
.balign 0x80
    b       exc_default_entry
.balign 0x80
    b       exc_default_entry
.balign 0x80
    b       exc_default_entry
.balign 0x80
    b       exc_default_entry

// ---- Group 1: Current EL with SPx (our kernel) ----
.balign 0x80
    b       exc_sync_entry          // Synchronous (e.g. data abort, SVC)
.balign 0x80
    b       exc_irq_entry           // IRQ ← timer interrupts arrive here
.balign 0x80
    b       exc_default_entry       // FIQ (unused)
.balign 0x80
    b       exc_default_entry       // SError

// ---- Group 2: Lower EL using AArch64 ----
.balign 0x80
    b       exc_default_entry
.balign 0x80
    b       exc_default_entry
.balign 0x80
    b       exc_default_entry
.balign 0x80
    b       exc_default_entry

// ---- Group 3: Lower EL using AArch32 ----
.balign 0x80
    b       exc_default_entry
.balign 0x80
    b       exc_default_entry
.balign 0x80
    b       exc_default_entry
.balign 0x80
    b       exc_default_entry

// ============================================================
//  Exception Entry Points
// ============================================================

exc_irq_entry:
    EXCEPTION_SAVE
    mov     x0, sp                  // arg0: pointer to ExceptionContext
    bl      irq_dispatch            // → interrupts::irq_dispatch()
    EXCEPTION_RESTORE

exc_sync_entry:
    EXCEPTION_SAVE
    mov     x0, sp
    bl      sync_exception_handler  // → exception::sync_exception_handler()
    EXCEPTION_RESTORE

exc_default_entry:
    EXCEPTION_SAVE
    mov     x0, sp
    bl      default_exception_handler
    EXCEPTION_RESTORE

"#);

// ─── Rust exception handlers ───

/// Handle synchronous exceptions (data abort, undefined instruction, etc.)
#[no_mangle]
pub extern "C" fn sync_exception_handler(ctx: &ExceptionContext) {
    let ec = (ctx.esr >> 26) & 0x3F;
    crate::println!("!!! SYNC EXCEPTION !!!");
    crate::println!("  EC=0x{:02x} ESR=0x{:016x}", ec, ctx.esr);
    crate::println!("  ELR=0x{:016x} SPSR=0x{:08x}", ctx.elr, ctx.spsr);
    // Halt
    loop {
        unsafe { core::arch::asm!("wfe"); }
    }
}

/// Default handler for unexpected exceptions
#[no_mangle]
pub extern "C" fn default_exception_handler(ctx: &ExceptionContext) {
    crate::println!(
        "\n!!! UNEXPECTED EXCEPTION !!!\n  ESR=0x{:08x}  ELR=0x{:016x}",
        ctx.esr, ctx.elr
    );
    loop {
        unsafe { core::arch::asm!("wfe"); }
    }
}
