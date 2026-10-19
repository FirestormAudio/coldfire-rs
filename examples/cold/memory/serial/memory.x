/* A board that boots from SPI flash. MEDIUM: the image as the boot part sees it, from 0x80000000 (where the SBF put
   its first 64 KB) on through the flash: the SPI flash's size, less the 7-byte SBF header and what sits before it. */
MEMORY {
  DDR (rwx) : ORIGIN = 0x40000000, LENGTH = 128M
  MEDIUM (r) : ORIGIN = 0x80000000, LENGTH = 16M
}
REGION_ALIAS("REGION_RAM", DDR);
__stack_top = 0x48000000;
