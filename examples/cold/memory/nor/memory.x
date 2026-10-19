/* A board that boots from NOR flash on FlexBus CS0. MEDIUM: the flash, at 0. */
MEMORY {
  DDR (rwx) : ORIGIN = 0x40000000, LENGTH = 128M
  MEDIUM (rx) : ORIGIN = 0x00000000, LENGTH = 16M
}
REGION_ALIAS("REGION_RAM", DDR);
__stack_top = 0x48000000;
