/* The cross-check programs: linked in place at the bottom of RAM, with the stack 16 MiB up. Both QEMU's
   mcf5208evb and the MCF5441x's DDR have RAM there. */
ENTRY(_start)
SECTIONS
{
    . = 0x40000000;
    .text : { *(.text .text.*) }
    .rodata : { *(.rodata .rodata.*) }
    .data : { *(.data .data.*) }
    . = ALIGN(4);
    __bss_start = .;
    .bss : { *(.bss .bss.* COMMON) }
    . = ALIGN(4);
    _end = .;
    __stack = 0x40000000 + 0x01000000;
}
