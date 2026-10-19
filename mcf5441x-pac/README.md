# mcf5441x-pac

Peripheral access crate for the NXP MCF5441x (ColdFire V4e). Part of [coldfire-rs](https://github.com/FirestormAudio/coldfire-rs). Most code wants the HAL,
[`embassy-mcf5441x`](https://crates.io/crates/embassy-mcf5441x), on top of this.

## What's in it

- **Register blocks:** one constant per peripheral instance (`UART0` … `UART9`, `EDMA`, `USBOTG`, …).
- **Registers and fields:** typed reads, writes and modifies. Every access goes through
  [`coldfire`](https://crates.io/crates/coldfire)'s width-fixed MMIO, so a 32-bit register is never touched with
  byte instructions.
- **The `Interrupt` enum:** every interrupt source, as a CPU vector number.
- **Read side effects:** registers whose read has one (a FIFO pop, or a status cleared by reading) are marked in the
  SVD.

```rust
use mcf5441x_pac::UART9;

fn put(c: u8) {
    while !UART9.usr().read().txrdy() {}
    UART9.utb().write(|w| w.set_tb(c));
}
```

## Features

- **The part:** `mcf54410`, `mcf54415`, `mcf54416`, `mcf54417` or `mcf54418`. Choose at least one; building with
  none is a compile error.
  - The peripherals a part doesn't have are compiled out, along with their `Interrupt` variants. For example, the
    MCF54410 has no `UART9`, `SSI1` or `USBHOST`.
  - Features are additive: with several on, the crate offers what any of those parts has.
- **`rt`:** the `#[interrupt]` attribute through `mcf5441x_pac::interrupt`, and a weak default for every
  interrupt handler, for [`coldfire-rt`](https://crates.io/crates/coldfire-rt). ColdFire builds only.

## How it is made

The crate is generated, and nothing in `src/` is edited by hand:
1. The reference manual is converted to an SVD (`svd/mcf5441x.svd`) of register and field names, offsets and widths.
2. The SVD is checked against independent address lists.
3. chiptool generates the crate.

The SVD carries no text from the manual, and the manual isn't in the repository. The generator is in the
[repository](https://github.com/FirestormAudio/coldfire-rs/tree/main/pac-gen).

## License

Licensed under either of [Apache License, Version 2.0](https://github.com/FirestormAudio/coldfire-rs/blob/main/LICENSE-APACHE) or
[MIT license](https://github.com/FirestormAudio/coldfire-rs/blob/main/LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as
defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
