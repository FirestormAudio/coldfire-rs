# coldfire-rt-macros

The attributes of [`coldfire-rt`](https://crates.io/crates/coldfire-rt). Part of [coldfire-rs](https://github.com/FirestormAudio/coldfire-rs). Use them
through `coldfire-rt`, or a PAC's `interrupt` re-export, rather than depending on this crate directly.

- **`#[entry]`** marks the function the reset handler calls; it must be `fn() -> !`.
- **`#[exception]`** marks a handler for a processor exception, named after the vector:
  `#[exception] fn Trap0() { … }`.
- **`#[interrupt]`** marks a handler for a peripheral interrupt, named after the PAC's `Interrupt` variant:
  `#[interrupt] fn PIT0() { … }`.

Each handler becomes a register-saving trampoline and a record in the `.coldfire_rt.handlers` section. The runtime
copies those records into the vector table at startup.

## License

Licensed under either of [Apache License, Version 2.0](https://github.com/FirestormAudio/coldfire-rs/blob/main/LICENSE-APACHE) or
[MIT license](https://github.com/FirestormAudio/coldfire-rs/blob/main/LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as
defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
