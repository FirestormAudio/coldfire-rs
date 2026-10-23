# TODO

## Drivers not written yet (embassy-mcf5441x), in priority order

Peripherals a known board uses come first; the rest wait for hardware to test them on.

1. A time driver on a DMA timer (DTIM0–3): one-shot alarms at the reference compare, for a tick rate above 1 kHz
   (`dtim::Timestamp` is the free-running counter).
2. Waiting for a board to test on: I2C (I2C0–I2C5); RTC; PWM (MCPWM); ADC and DAC; FlexCAN (FLEXCAN0/1);
   Ethernet (MACNET0/1 and the switch, through embassy-net); a NAND flash driver (the cold-boot part only reads
   boot-format pages); USB host mode.

## Cold boot (embassy-mcf5441x)

- The serial path's SPI fetch run on hardware, and the NOR and NAND paths on a board that boots from them (so far they
  are checked on the host and in an emulator).
- The NAND ECC status: NFC_CFG[ECCAD]'s unit (taken in 8-byte units; the manual says "byte address" with 9 bits),
  which byte of the slot holds CORFAIL, and whether every read rewrites it.
- Typed DDR geometry and timing helpers that produce the CR words, once the timing units (SD_CLK or controller
  cycles) are confirmed on hardware.
- Image tooling (SBF header, NAND page split, interleave and BCH), if the crates should ever carry it.

## Existing drivers (embassy-mcf5441x)

- EPORT: IRQ1, IRQ4 and IRQ7 once PAR_IRQ0H's layout is verified (they are the MCF54410's only IRQ pads); run on
  hardware, including whether GPIO PPDSDR reads a pad muxed to IRQn, which the driver's levels rely on.

- Settle the PAR layouts pinmux.json marks unverified: PAR_CS, PAR_IRQ0H and PAR_DSPIOWL's OWDAT by the reserved-bit
  readback on hardware.
- DSPI: an interrupt/eDMA-driven async driver when something needs one. Vectors: DSPI0_SR 95, DSPI1–3_SR 182–184.
  eDMA requests: RX/TX 12/13, 14/15, 28/29, 44/45. Also `Operation::DelayNs` in `PcsDevice` transactions (refused today:
  the blocking driver holds no clock reference, so it stays usable without a runtime).
- DSPI: frame sizes other than 8 (plus 16-bit reads), and several hardware-CS devices sharing one bus.
- eSDHC: writes and TRIM run on hardware (so far emulation-tested only);
  CMD23 reliable write, erase-group ERASE, secure erase and trim, discard, sanitize, write protection and RPMB; a TRIM
  busy bounded by EXT_CSD TRIM_MULT rather than the controller's data timeout.
- eSDHC: SD cards (CMD8/ACMD41, 3.3 V checks), card detect, 1-bit
  and 8-bit buses, the controller's own DMA/ADMA, an SD clock computed from the clock tree (PLL OUTDIV3,
  MISCCR SDHCSRC) instead of fixed dividers, and the RM's divider-change sequence (SDCLKEN off, wait SDSTB).
- eSDHC: bound the bring-up waits without a time driver (an attempt cap on the CMD1 loop); today only the time driver's
  clock bounds them, with a poll-count backstop that is very long for a missing card.
- SSI: AC97, the second FIFO pair, an async layer over the render callback; SSI0 run on hardware.
- USB: the on-chip SRAM base (0x80000000, RAMBAR1) is assumed in `usb::desc::in_sram`; RAMBAR1 can't be read back, so a
  board with SRAM elsewhere needs it passed in. Not supported: host mode, VBUS sensing (power is reported once, at
  start), remote wakeup, isochronous endpoints, more than one dTD per endpoint direction.
