/* coldfire-rt: the board's memory.x (found on the linker search path) defines MEMORY, the RAM region
   (REGION_ALIAS("REGION_RAM", …)) and __stack_top, and may add board sections (e.g. on-chip SRAM). */
INCLUDE memory.x
ENTRY(_start)
SECTIONS {
  .vectors ORIGIN(REGION_RAM) (NOLOAD) : { KEEP(*(.vectors)) } > REGION_RAM
  .text ORIGIN(REGION_RAM) + 0x400 : { KEEP(*(.text._start)) *(.text .text.*) } > REGION_RAM
  .rodata : { *(.rodata .rodata.*) . = ALIGN(4); __shandlers = .; KEEP(*(.coldfire_rt.handlers)) __ehandlers = .; } > REGION_RAM
  .data : { *(.data .data.*) } > REGION_RAM
  .bss (NOLOAD) : { . = ALIGN(4); __sbss = .; *(.bss .bss.* COMMON) . = ALIGN(4); __ebss = .; } > REGION_RAM
  /DISCARD/ : { *(.eh_frame*) }
}
/* a binary that never names coldfire_rt (no `use coldfire_rt as _;`) would link without a startup */
ASSERT(DEFINED(coldfire_rt_start), "coldfire-rt is not linked: add `use coldfire_rt as _;` to the binary")
/* VBR keeps only bits 31..20 (RM 3.2.8): the table must start on a 1 MB boundary */
ASSERT((ADDR(.vectors) & 0xFFFFF) == 0, "coldfire-rt: the vector table (ORIGIN(REGION_RAM)) must be 1 MB aligned (VBR bits 19..0 are not implemented)")
