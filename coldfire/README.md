# coldfire

Core support for NXP ColdFire V4e processors (the MCF5441x and its kin). Part of
[coldfire-rs](https://github.com/FirestormAudio/coldfire-rs), which also has the toolchain, the runtime, a PAC and an Embassy HAL.

## What's in it

- **`mmio`:** register reads and writes at a fixed width (8, 16 and 32 bits). A 32-bit register is never read or
  written with byte instructions, which matters for peripherals that latch or clear on access.
- **`interrupt`:**
  - `disable`, `restore` and `enable`;
  - `free`, which runs a closure with interrupts masked;
  - `wait_until`, which sleeps in `stop` until a condition holds.
- **`register`:** the status register and the control registers (VBR, CACR and others).
- **`asm`:** `stop`, `halt`, `nop`.
- **A `critical-section` implementation,** with feature `critical-section-single-core`. It masks interrupts through
  the status register.
- **The libcalls GCC emits** for atomics and byte swaps on ColdFire, which has no atomic instructions.
- **`emac`,** with feature `emac`: the Enhanced Multiply-Accumulate unit's accumulators, modes and multiply-accumulate
  operations.

```rust
use coldfire::{interrupt, mmio};

const PIT0_PCSR: usize = 0xFC08_0000;
let pcsr = unsafe { mmio::r16(PIT0_PCSR) };          // a 16-bit read, never split
interrupt::free(|| unsafe { mmio::w16(PIT0_PCSR, pcsr | 1) });
```

## Building

Building for ColdFire needs the coldfire-rs toolchain (`toolchain/setup.sh` and `toolchain/cargo-cf`); see the
[repository README](https://github.com/FirestormAudio/coldfire-rs#toolchain). On other targets the ColdFire-only modules are compiled out, so a host build has
`mmio` and, with the feature, `emac`.

## License

Licensed under either of [Apache License, Version 2.0](https://github.com/FirestormAudio/coldfire-rs/blob/main/LICENSE-APACHE) or
[MIT license](https://github.com/FirestormAudio/coldfire-rs/blob/main/LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as
defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
