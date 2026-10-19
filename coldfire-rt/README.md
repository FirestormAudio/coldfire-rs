# coldfire-rt

Minimal runtime for NXP ColdFire V4e processors. Part of [coldfire-rs](https://github.com/FirestormAudio/coldfire-rs).

## What's in it

- **Reset:** `_start` masks interrupts, sets the stack, clears `.bss`, builds the vector table in RAM, points VBR at
  it and calls your `#[entry]` function.
- **The vector table:** every handler marked `#[interrupt]` or `#[exception]` goes into it at startup. Unhandled
  vectors panic with the vector number. `set_vector` replaces a vector at run time.
- **`exception_entry!`:** a register-saving trampoline, for handlers you install yourself.
- **`link.x`:** the linker script. The build script puts it on the linker search path, so link with `-Tlink.x`.
- **Re-exports:** the attributes from [`coldfire-rt-macros`](https://crates.io/crates/coldfire-rt-macros), and
  [`coldfire`](https://crates.io/crates/coldfire).

```rust
#![no_std]
#![no_main]

use coldfire_rt::entry;

#[entry]
fn main() -> ! {
    loop {}
}
```

## Your board's `memory.x`

`link.x` includes a `memory.x` from the linker search path. It must define:
- **`MEMORY`, and `REGION_ALIAS("REGION_RAM", …)`** for the region the program runs from. Its origin holds the
  vector table, and must be 1 MB aligned. Code and the entry point start at origin + 0x400.
- **`__stack_top`.**

```text
MEMORY { DDR (rwx) : ORIGIN = 0x40000000, LENGTH = 128M }
REGION_ALIAS("REGION_RAM", DDR);
__stack_top = 0x48000000;
```

The [repository README](https://github.com/FirestormAudio/coldfire-rs#your-boards-memoryx) covers the rest of the contract, including the sections the USB
driver needs.

## Cold boot

The `cold` feature (the chip's HAL turns it on with its cold boot features) links the image so the boot hardware can
start it from reset: SP and PC at image offset 0, then a boot part, then the app part, which runs in DDR and loads
from the boot medium. Startup calls the HAL's boot part (`__coldfire_rt_boot`) on a stack in on-chip SRAM, then
runs as usual in DDR. `#[boot_config]` links the board's description into the boot part. `memory.x` adds `MEDIUM`,
the boot medium as the boot part sees it; the HAL's own linker fragment says where the boot part runs.

## Features

- `critical-section-single-core` (default): installs `coldfire`'s `critical-section` implementation.
- `cold`: the cold-boot layout above.

## License

Licensed under either of [Apache License, Version 2.0](https://github.com/FirestormAudio/coldfire-rs/blob/main/LICENSE-APACHE) or
[MIT license](https://github.com/FirestormAudio/coldfire-rs/blob/main/LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as
defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
