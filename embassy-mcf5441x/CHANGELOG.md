# Changelog

## Unreleased

### Added

- `eport`: IRQ1 (PC1), IRQ4 (PC4) and IRQ7 (PC6), on every part. Their PAR_IRQ0H fields are measured on an MCF54415.
- `gpio`: the pads on PAR_CS (PB3–PB6), PAR_IRQ0H (PC1, PC4, PC6) and PAR_DSPIOWL's OWDAT (PD3) are muxed to GPIO
  when taken as GPIO; their layouts are measured on an MCF54415, so they are no longer left alone.

### Fixed

- `eport`: a pin's level is read through INTC0 IPR, with the pin level-sensitive for the read. GPIO PPDSDR, which
  0.3.0 read, gives 0 for a pad muxed to IRQn, so `is_high` was always false and `wait_for_low` always returned at
  once. `wait_for_low` now arms the level-sensitive mode.

## 0.3.0

### Breaking

- `clocks::Gate` is `#[non_exhaustive]` and has new variants (`Eport`, `Dtim0`–`Dtim3`): a `match` on it needs a
  wildcard arm.

### Added

- `eport`: the edge port's IRQ pads as async inputs. `eport::Input` implements `embedded-hal-async`'s `Wait` (edges
  and levels) and `embedded-hal`'s `InputPin`. The pads offered are IRQ2 (PC2), IRQ3 (PC3) and IRQ6 (PC5), on the
  MCF54415–MCF54418. The interrupt level is set per pad (`eport::Config::priority`).
- `dtim`: the DMA timers as free-running 32-bit timestamp counters (`dtim::Timestamp`), at the bus clock or bus
  clock / 16, through a prescaler of 1 to 256. `DTIM0`–`DTIM3` are in `Peripherals`.
- `esdhc`: `Esdhc::write` (multiple-block writes, CMD25) and `Esdhc::erase` (TRIM), each checking the card's status
  afterwards. The new `esdhc::Error::Card` carries the status word when it has error bits.
- `interrupt::typelevel`: `EPORT_EPF1`–`EPORT_EPF7`.
- A dependency on `embedded-hal-async` 1.0.

### Fixed

- `uart`: receive errors use block error mode (UMR1 ERR), so a framing error is seen when receive DMA takes the
  character.
- `uart`: a blocking read takes a character's error bits and RXRDY from one status read, so an error is reported with
  the character it came with, not on the next read.
- `usb`: an endpoint reads as disabled until the controller runs; its registers read all ones while the module's clock
  is gated.

### Not yet run on hardware

`eport`, `dtim`, eSDHC writes and TRIM, and SSI0 are tested on an emulated MCF5441x only.

## 0.2.0 and 0.1.0

Released before this changelog was kept.
