/* The layout mcf5441x-emu's tests/hal.rs maps, as examples/memory.x has it: 128 MB of DDR at 0x40000000 with the
   stack at its top, and the 64 KB on-chip SRAM at 0x80000000 (RAMBAR1), the USB structures in its upper half. */
MEMORY { DDR (rwx) : ORIGIN = 0x40000000, LENGTH = 128M }
REGION_ALIAS("REGION_RAM", DDR);   /* code and data; the vector table at its start must be 1 MB aligned */
__stack_top = 0x48000000;
SECTIONS {
  /* embassy_mcf5441x::usb's dQH list, dTDs and buffers, NOLOAD: here the upper 32 KB of the on-chip SRAM, leaving the
     lower half to whatever the bootloader left there. DDR works too, with 64-byte IN packets. */
  .usb_sram 0x80008000 (NOLOAD) : {
    __susb = .;
    *(.usb_sram.qh) *(.usb_sram.td) *(.usb_sram.setup) *(.usb_sram.buf)
    __eusb = .;
  }
}
ASSERT(__eusb <= 0x80010000, "the USB structures must fit in the on-chip SRAM (it ends at 0x80010000)")
