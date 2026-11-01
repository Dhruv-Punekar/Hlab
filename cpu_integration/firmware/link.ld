/* 16 KiB of BRAM at address 0 (cpu_subsystem RAM_WORDS = 4096) */
MEMORY { RAM (rwx) : ORIGIN = 0x00000000, LENGTH = 16K }
ENTRY(_start)
SECTIONS
{
    .text : {
        KEEP(*(.text.start))
        *(.text .text.*)
    } > RAM
    .rodata : { *(.rodata .rodata.* .srodata .srodata.*) } > RAM
    .data   : { *(.data .data.* .sdata .sdata.*) } > RAM
    .bss (NOLOAD) : {
        . = ALIGN(4);
        __bss_start = .;
        *(.bss .bss.* .sbss .sbss.* COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > RAM
    __image_end = .;
    __stack_top = ORIGIN(RAM) + LENGTH(RAM);
    /* keep at least 2 KiB between the image and the top of RAM for the stack */
    ASSERT(__image_end <= __stack_top - 2048, "firmware too large for 16 KiB RAM")
}
