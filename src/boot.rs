// ZerOS — boot.S equivalent in Rust inline assembly
// AArch64 boot stub for Raspberry Pi Zero 2 W

use core::arch::global_asm;

// Boot code: placed at 0x80000 by the linker script
global_asm!(
    r#"
.section .text.boot

.global _start

_start:
    // Check processor ID — only proceed on core 0
    mrs     x0, mpidr_el1
    and     x0, x0, #3
    cbnz    x0, .L_park

    // Set up stack pointer (growing downward from 0x80000)
    ldr     x0, =_start
    mov     sp, x0

    // Clear BSS section
    ldr     x0, =__bss_start
    ldr     x1, =__bss_end
.L_bss_loop:
    cmp     x0, x1
    b.ge    .L_bss_done
    str     xzr, [x0], #8
    b       .L_bss_loop
.L_bss_done:

    // Jump to Rust kernel_main
    bl      kernel_main

    // If kernel_main returns, park the core
.L_park:
    wfe
    b       .L_park
"#
);
