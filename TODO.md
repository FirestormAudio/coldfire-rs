# TODO

## Full bring-up from reset (embassy-mcf5441x)

The HAL assumes a bootloader has already set the PLL, the DDR controller and the boot pin mux. Add an opt-in
bring-up path so a board without a bootloader can boot a HAL app: PLL programming (PLL_CR/PLL_DR, lock wait),
DDR controller init (timings from the board, the DDR_CR registers; one board's boot code is a proven reference),
and the boot pin mux. Needs hardware checks of every setting.
- Settle the PAR layouts pinmux.json marks unverified: PAR_CS, PAR_IRQ0H and PAR_DSPIOWL's OWDAT by the reserved-bit
  readback on hardware.
- DSPI: an interrupt/eDMA-driven async driver when something needs one. Vectors: DSPI0_SR 95, DSPI1–3_SR 182–184.
  eDMA requests: RX/TX 12/13, 14/15, 28/29, 44/45. Also `Operation::DelayNs` in `PcsDevice` transactions (refused today:
  the blocking driver holds no clock reference, so it stays usable without a runtime).
- DSPI: frame sizes other than 8 (plus 16-bit reads), and several hardware-CS devices sharing one bus.
- eSDHC: writes (CMD24/25, CMD23 reliable write), erase, SD cards (CMD8/ACMD41, 3.3 V checks), card detect, 1-bit
  and 8-bit buses, the controller's own DMA/ADMA, an SD clock computed from the clock tree (PLL OUTDIV3,
  MISCCR SDHCSRC) instead of fixed dividers, and the RM's divider-change sequence (SDCLKEN off, wait SDSTB).
- eSDHC: bound the bring-up waits without a time driver (an attempt cap on the CMD1 loop); today only the time driver's
  clock bounds them, with a poll-count backstop that is very long for a missing card.
- USB: the on-chip SRAM base (0x80000000, RAMBAR1) is assumed in `usb::desc::in_sram`; RAMBAR1 can't be read back, so a
  board with SRAM elsewhere needs it passed in. Not supported: host mode, VBUS sensing (power is reported once, at
  start), remote wakeup, isochronous endpoints, more than one dTD per endpoint direction.

## Before the first publish

The crates pass `cargo publish --dry-run --workspace` (`toolchain/tests/test_publish.py`). Each step below needs the
owner's go-ahead at the time:

1. The legal and trademark look: the PAC's SVD is derived from NXP's reference manual (names, offsets and encodings
   only, no manual prose).
2. Create the GitHub repository https://github.com/FirestormAudio/coldfire-rs (the crates already name it as their `repository`).
3. The first push: branch `main`, one squashed root commit of the reviewed tree. The development history stays
   private.
4. `cargo login`, then publish in dependency order, with the part feature that verification needs:
   `cargo publish -p coldfire`, `-p coldfire-rt-macros`, `-p coldfire-rt`, `-p mcf5441x-pac --features mcf54418`,
   `-p embassy-mcf5441x --features mcf54418`.

