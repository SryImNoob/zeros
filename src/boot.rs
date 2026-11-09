// ZerOS — boot.S equivalent in Rust inline assembly
// AArch64 boot stub for Raspberry Pi Zero 2 W
//
// Boot flow:
//   1. Park secondary cores (core > 0)
//   2. Drop from EL3/EL2 → EL1 (configure timer access, AArch64 mode)
//   3. Set exception vector table (VBAR_EL1)
//   4. Set up stack, clear BSS
//   5. Jump to kernel_main
//
// Note: Real RPi firmware drops to EL2, QEMU raspi3b starts at EL3.
//       We handle both cases.

use core::arch::global_asm;

global_asm!(
    r#"
.section .text.boot

.global _start

_start:
    // ---- Park secondary cores ----
    mrs     x0, mpidr_el1
    and     x0, x0, #3
    cbnz    x0, .L_park

    // ---- Check current Exception Level ----
    mrs     x0, CurrentEL
    lsr     x0, x0, #2         // x0 = current EL (1, 2, or 3)

    cmp     x0, #3
    b.eq    .L_from_el3

    cmp     x0, #2
    b.eq    .L_from_el2

    // Already at EL1
    b       .L_el1_entry

    // ---- EL3 → EL2 transition ----
.L_from_el3:
    // Set EL2 to AArch64
    mov     x1, #(1 << 10)     // SCR_EL3.RW = 1 (AArch64 at EL2)
    orr     x1, x1, #(1 << 0)  // SCR_EL3.NS = 1 (Non-secure)
    orr     x1, x1, #(1 << 3)  // SCR_EL3.EA = 0... actually let's route IRQ/FIQ normally
    msr     scr_el3, x1

    // Prepare SPSR for EL2: return to EL2h, all DAIF masked
    mov     x1, #0x3C9          // M[4:0]=01001 (EL2h), DAIF=1111
    msr     spsr_el3, x1

    // Set return address to EL2 setup
    adr     x1, .L_from_el2
    msr     elr_el3, x1

    eret                        // Drop to EL2

    // ---- EL2 → EL1 transition ----
.L_from_el2:
    // Enable AArch64 at EL1, don't trap WFE/WFI/SMC
    mov     x1, #(1 << 31)     // HCR_EL2.RW = 1 (AArch64 at EL1)
    msr     hcr_el2, x1

    // Do not trap FPU/SIMD accesses to EL2
    msr     cptr_el2, xzr

    // Allow EL1 to access physical timer & counter
    mrs     x1, cnthctl_el2
    orr     x1, x1, #3         // EL1PCTEN=1, EL1PCEN=1
    msr     cnthctl_el2, x1
    msr     cntvoff_el2, xzr   // Virtual timer offset = 0

    // Prepare SPSR for EL1: return to EL1h, all DAIF masked
    mov     x1, #0x3C5         // M[4:0]=00101 (EL1h), DAIF=1111
    msr     spsr_el2, x1

    // Set return address to EL1 entry point
    adr     x1, .L_el1_entry
    msr     elr_el2, x1

    eret                       // Drop to EL1

.L_el1_entry:
    // ---- Enable FPU/SIMD (CPACR_EL1.FPEN = 0b11) ----
    mov     x0, #(3 << 20)     // FPEN bits [21:20] = 11 (no trap)
    msr     cpacr_el1, x0
    isb

    // ---- Set exception vector table ----
    ldr     x0, =exception_vectors
    msr     vbar_el1, x0

    // ---- Set up stack (grows downward from 0x80000) ----
    ldr     x0, =_start
    mov     sp, x0

    // ---- Clear BSS section ----
    ldr     x0, =__bss_start
    ldr     x1, =__bss_end
.L_bss_loop:
    cmp     x0, x1
    b.ge    .L_bss_done
    str     xzr, [x0], #8
    b       .L_bss_loop
.L_bss_done:

    // ---- Jump to Rust kernel entry ----
    bl      kernel_main

    // Fallthrough: park if kernel_main returns
.L_park:
    wfe
    b       .L_park
"#
);
