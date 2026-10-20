/* coldfire-rt, cold layout: the chip's boot hardware starts this image from reset. The board's memory.x defines DDR,
   REGION_ALIAS("REGION_RAM", …), __stack_top and MEDIUM, the boot medium as the boot part sees it (serial: the image
   from 0x80000000; NOR: the flash at 0; NAND: none, the HAL supplies it). The HAL's boot-region.x, chosen by its boot
   feature, defines BOOT (where the boot part runs), HEAD/HEAD_LOAD/BOOT_LOAD, __boot_stack_top, __boot_budget and
   __boot_entry. Image offset 0 holds the initial SP and PC; the app part runs in DDR and loads from MEDIUM. memory.x
   may also set __app_part_max, the most app-part bytes the medium holds. */
INCLUDE memory.x
INCLUDE boot-region.x
ENTRY(_start)
SECTIONS {
  /* image offset 0: SP and PC (data, kept out of the code sections), then whatever runs in place before the boot
     part (NAND's relocator) */
  .boot.vectors : { __image_start = .; KEEP(*(.boot.vectors)) } > HEAD AT> HEAD_LOAD
  .boot.head : { KEEP(*(.boot.reloc .boot.reloc.*)) . = ALIGN(4); } > HEAD AT> HEAD_LOAD
  .boot : {
    KEEP(*(.boot.text._start))
    *(.boot.text .boot.text.*)
    . = ALIGN(4);
  } > BOOT AT> BOOT_LOAD
  /* the boot sequence and the rest of the boot part's constants: data, after its code */
  .boot.rodata : {
    KEEP(*(.boot.rodata.coldfire_rt.*))
    *(.boot.rodata .boot.rodata.*)
    . = ALIGN(4);
  } > BOOT AT> BOOT_LOAD
  __head_size = SIZEOF(.boot.vectors) + SIZEOF(.boot.head);
  __boot_vma = ADDR(.boot);
  __boot_lma = LOADADDR(.boot);
  __boot_vma_len = SIZEOF(.boot) + SIZEOF(.boot.rodata);
  __boot_size = __head_size + __boot_vma_len;
  /* the boot part runs before DDR: it can't keep writable data (it lives on its stack) */
  .boot.forbidden (NOLOAD) : { *(.boot.data .boot.data.* .boot.bss .boot.bss.*) } > BOOT

  .vectors ORIGIN(REGION_RAM) (NOLOAD) : { KEEP(*(.vectors)) } > REGION_RAM
  .text ORIGIN(REGION_RAM) + 0x400 : { __app_vma = .; *(.text .text.*) } > REGION_RAM AT> MEDIUM
  /* the boot part copies the app part as one block: each section loads at the same offset from .text as it runs,
     alignment gaps included (AT> alone would pack the load addresses) */
  .rodata : AT(LOADADDR(.text) + (ADDR(.rodata) - ADDR(.text))) {
    *(.rodata .rodata.*) . = ALIGN(4); __shandlers = .; KEEP(*(.coldfire_rt.handlers)) __ehandlers = .;
  } > REGION_RAM
  .data : AT(LOADADDR(.text) + (ADDR(.data) - ADDR(.text))) { *(.data .data.*) . = ALIGN(4); __app_vma_end = .; } > REGION_RAM
  __app_lma = LOADADDR(.text);
  __app_len = __app_vma_end - __app_vma;
  __app_offset = __app_lma - ORIGIN(MEDIUM);
  .bss (NOLOAD) : { . = ALIGN(4); __sbss = .; *(.bss .bss.* COMMON) . = ALIGN(4); __ebss = .; } > REGION_RAM
  /DISCARD/ : { *(.eh_frame*) }
}
ASSERT(DEFINED(coldfire_rt_start), "coldfire-rt is not linked: add `use coldfire_rt as _;` to the binary")
ASSERT(DEFINED(__coldfire_rt_boot), "cold layout without a boot part: enable a boot feature of the chip's HAL")
ASSERT(SIZEOF(.boot.forbidden) == 0, "coldfire-rt: the boot part has .data or .bss; it runs before DDR and keeps no writable statics")
ASSERT(__app_lma + __app_len <= ORIGIN(MEDIUM) + LENGTH(MEDIUM), "coldfire-rt: the app part doesn't fit in MEDIUM, the boot medium")
/* optional, from memory.x: how many app-part bytes the boot medium holds (NAND's MEDIUM numbers payload bytes, it
   doesn't know the device) */
ASSERT(__app_len <= (DEFINED(__app_part_max) ? __app_part_max : __app_len), "coldfire-rt: the app part is larger than memory.x's __app_part_max")
ASSERT(__boot_size <= __boot_budget, "coldfire-rt: the boot part is larger than the boot source allows")
ASSERT((ADDR(.vectors) & 0xFFFFF) == 0, "coldfire-rt: the vector table (ORIGIN(REGION_RAM)) must be 1 MB aligned (VBR bits 19..0 are not implemented)")
