/* A board that boots from NAND flash: the HAL's boot region supplies MEDIUM (the app pages). */
MEMORY {
  DDR (rwx) : ORIGIN = 0x40000000, LENGTH = 128M
}
REGION_ALIAS("REGION_RAM", DDR);
__stack_top = 0x48000000;
