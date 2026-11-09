/* ZerOS Linker Script for Raspberry Pi Zero 2 W (AArch64) */
/* The RPi firmware loads kernel8.img at 0x80000 */

ENTRY(_start)

SECTIONS
{
    . = 0x80000;

    /* Boot code must come first */
    .text.boot : {
        KEEP(*(.text.boot))
    }

    .text : {
        *(.text*)
    }

    .rodata : ALIGN(8) {
        *(.rodata*)
    }

    .data : ALIGN(8) {
        *(.data*)
    }

    /* BSS (uninitialized data) */
    .bss (NOLOAD) : ALIGN(16) {
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(16);
        __bss_end = .;
    }

    /* Stack grows downward from _start (0x80000) */

    /DISCARD/ : {
        *(.comment*)
        *(.gnu*)
        *(.note*)
        *(.eh_frame*)
    }
}
