# coldfire-rs

Rust for NXP ColdFire V4e microcontrollers, starting with the MCF5441x family (MCF54410, MCF54415–MCF54418). This
repository has:

- **a toolchain** that builds Rust for ColdFire;
- **core support and a runtime**;
- **a peripheral access crate (PAC)** generated from an SVD;
- **an [Embassy](https://embassy.dev) HAL** with async drivers.

The drivers run on hardware, on an MCF54415 board (SSI as SSI1 only: no board here wires SSI0). The API is young and may still change before 1.0.

## Crates

| Crate | What it is |
|---|---|
| [`coldfire`](coldfire) | Core support: width-fixed MMIO, interrupt masking, the status and control registers, a `critical-section` implementation, the libcalls GCC emits, and the EMAC unit. |
| [`coldfire-rt`](coldfire-rt) | Runtime: reset, the stack and `.bss`, a RAM vector table, exception trampolines, and the linker script. |
| [`coldfire-rt-macros`](coldfire-rt-macros) | The `#[entry]`, `#[exception]` and `#[interrupt]` attributes. Use them through `coldfire-rt`. |
| [`mcf5441x-pac`](mcf5441x-pac) | The MCF5441x PAC: register blocks, typed fields and the `Interrupt` enum, generated from an SVD with chiptool. |
| [`embassy-mcf5441x`](embassy-mcf5441x) | The Embassy HAL: `init`, clocks, interrupts and executors, an embassy-time driver, GPIO and pin mux, and drivers for eDMA, UART, DSPI, SSI, eSDHC and USB. |

`examples/` has three small programs to start a board from (see [examples/README.md](examples/README.md)).
`hal-tests/` holds the HAL's test programs, one per driver area, which `mcf5441x-emu`'s tests run on the emulated
chip (see [hal-tests/README.md](hal-tests/README.md)).

### HAL drivers

| Peripheral | Driver |
|---|---|
| GPIO, pin mux | `Output`, `Input` and `Flex` pins. Each driver muxes its own pads, from the reference manual's pin table. |
| PIT | The embassy-time driver, on one PIT at 1 kHz. |
| INTC | Interrupt levels, `bind_interrupts!`, software pends, and thread and interrupt executors. |
| EPORT | The IRQ pads as async inputs: `Wait` for edges and levels, `InputPin` (`embedded-hal-async`). IRQ2, IRQ3 and IRQ6 on MCF54415–MCF54418; emulation-tested only. |
| eDMA | Channels and transfer descriptors, as used by the drivers. |
| DMA timers | `Timestamp`: a free-running 32-bit counter at the bus clock or bus/16, through a prescaler. Emulation-tested only. |
| UART | Blocking `Uart`/`UartTx`/`UartRx`, and a `BufferedUart` with eDMA receive and interrupt-driven transmit (`embedded-io`, `embedded-io-async`). |
| DSPI | Blocking SPI: `SpiBus` and `SpiDevice` with hardware chip selects (`embedded-hal` 1.0). |
| eSDHC | eMMC block reads (read-only) through eDMA, async. |
| SSI | I2S or network-mode audio: full-duplex eDMA rings, a `render(input, output)` callback in a software interrupt at the level you pick. SSI1 runs on hardware; SSI0 is emulation-tested only. |
| USB OTG | Device mode at high speed through a ULPI PHY (`embassy-usb-driver`). MCF54415–MCF54418 only. |

Not there yet: eSDHC writes, EPORT, DMA timers, I2C, RTC, PWM, ADC/DAC, FlexCAN, Ethernet, NAND flash and
USB host.
[TODO.md](TODO.md) has the list.

## Quick start

1. **Build the toolchain** once (see [Toolchain](#toolchain)). It needs `rustup` and GNU binutils for `m68k-elf` on
   `PATH`:

       toolchain/setup.sh

2. **Build the examples:**

       toolchain/cargo-cf examples --features mcf54415   # or mcf54416, mcf54417, mcf54418

   The ELFs land in `examples/target/m68k-coldfire-none-elf/release/`.

3. **Load one** the way your board loads code, for example over BDM: load the ELF and start it at its entry
   point.

### How an app starts

`embassy-mcf5441x` needs exactly one boot feature:
- **Cold boot** (`boot-serial`, `boot-nor` or `boot-nand`): the image starts from reset. Its boot part sets up the
  PLL and DDR from your board's `BootConfig`, copies the app part into DDR from the boot medium, and runs `main`.
  See [Cold boot](embassy-mcf5441x/README.md#cold-boot) in the HAL's README.
- **`bootloader`:** something earlier has set up the PLL, the DDR controller and the boot pin mux (and RAMBAR1, if
  you use the on-chip SRAM), and loaded the app into DDR. The HAL reads the PLL to learn its clocks.

Either way, `init` never retunes the PLL: code running from DDR can't, since the DDR clock comes from it.

## Using the crates in a board project

Depend on the HAL and the runtime, and pick your part:

```toml
[dependencies]
embassy-mcf5441x = { version = "0.2", features = ["mcf54415", "bootloader", "executor-thread"] }   # or a cold boot feature
coldfire-rt = "0.1"
```

The HAL's features:
- **The part:** exactly one of `mcf54410`, `mcf54415`, `mcf54416`, `mcf54417`, `mcf54418`.
- **How it boots:** exactly one of `bootloader`, `boot-serial`, `boot-nor` or `boot-nand` (see above).
- **Executors:** `executor-thread` and `executor-interrupt`.
- **The time driver's PIT:** `time-driver-pit0` (the default), `time-driver-pit1`, `-pit2` or `-pit3`.

A minimal program:

```rust
#![no_std]
#![no_main]

use coldfire_rt::entry;
use embassy_mcf5441x::uart::{Config, Uart};

#[entry]
fn main() -> ! {
    // the PLL's reference clock in Hz: the HAL computes the core and bus clocks from it
    let p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(24_000_000));
    let mut uart = Uart::new(p.UART0, p.PF3, p.PF4, Config::new(115_200)).unwrap();
    uart.blocking_write(b"hello\r\n");
    loop {}
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
```

### Your board's `memory.x`

`coldfire-rt` ships the linker script, `link.x`. Its build script puts it on the linker search path, so binaries
link with `-Tlink.x`; `toolchain/cargo-cf` does this. `link.x` includes a `memory.x` that your board supplies. The
linker finds it in the crate being built, or in `CF_MEMORY_DIR` with `cargo-cf`.

`memory.x` defines:
- **`MEMORY`, with `REGION_ALIAS("REGION_RAM", …)`** for the region that holds the program. The vector table goes at
  its origin, which must be 1 MB aligned, because VBR keeps bits 31..20 only. Code starts at origin + 0x400, and the
  entry point `_start` is there.
- **`__stack_top`.**
- **The USB driver's buffers,** if you use USB. They are in the input sections `.usb_sram.qh`, `.usb_sram.td`,
  `.usb_sram.setup` and `.usb_sram.buf`; place them NOLOAD in the on-chip SRAM. They work from DDR too, with 64-byte
  IN packets.

[examples/memory.x](examples/memory.x) is a complete example.

`link.x` refuses to link a binary that never names `coldfire_rt`, because it would have no startup code. `#[entry]`
names it; otherwise add `use coldfire_rt as _;`. A board that brings its own `link.x` must keep the
`.coldfire_rt.handlers` section, which is how `#[interrupt]` and `#[exception]` handlers reach the vector table.

## Toolchain

LLVM's m68k backend emits instructions ColdFire doesn't have, so builds go through
[rustc_codegen_gcc](https://github.com/rust-lang/rustc_codegen_gcc) with a libgccjit for `m68k-elf`. Its default
CPU is the 54418.

- **`toolchain/setup.sh`** builds the backend under `build/rust-coldfire`; set `RCF=` to put it elsewhere.
  - The first run takes 30–60 minutes.
  - It is idempotent, and works offline once built.
  - It pins the nightly toolchain and the backend's revision.
- **`toolchain/cargo-cf <crate-dir> [cargo args]`** builds a crate for `m68k-coldfire-none-elf`, using
  `-Zbuild-std` and the target spec `toolchain/m68k-coldfire-none-elf.json`. It then runs `toolchain/cfgate.py`
  on every executable:
  - objdump must decode each instruction as ColdFire ISA C with EMAC, the MCF5441x core;
  - GNU as (`-mcpu=54418`) must accept it.

  `CF_OBJDUMP_ARCH` and `CF_CPU` select another ColdFire.

The crates also build on a host, with the ColdFire-only parts compiled out, so `cargo test` and `cargo doc` work
anywhere.

### Editors

rust-analyzer checks through stock rustc, whose LLVM rejects the data layout GCC uses for ColdFire. For editors
there's a second target spec, `toolchain/m68k-coldfire-none-elf-check.json`. It has LLVM's m68k layout and is
otherwise identical; a test keeps the two in step. Point rust-analyzer at it with:
- **`cargo.target`:** the absolute path of `toolchain/m68k-coldfire-none-elf-check.json`;
- **`cargo.extraArgs`:** `["-Zbuild-std=core,alloc", "-Zjson-target-spec"]`;
- **`cargo.features`:** `["mcf5441x-pac/mcf54418"]`;
- **`RUSTUP_TOOLCHAIN`:** the nightly that `setup.sh` pins;
- **`check.overrideCommand`:** `cargo check` with the same flags and its own `--target-dir`.

Those settings hold local paths, so `.vscode/` and `.zed/` are gitignored.

## How the PAC is made

`mcf5441x-pac` is generated, and nothing in its `src/` is edited by hand:

1. **The manual becomes an SVD.** [`pac-gen`](pac-gen) converts the MCF5441x reference manual into an SVD, keeping
   register and field names, offsets and widths only. It then applies the patches in
   `mcf5441x-pac/svd/patches/mcf5441x.yaml`.
2. **The SVD is checked** against independent address lists from U-Boot, Linux and hardware-proven code.
3. **chiptool generates the crate,** and `postgen.py` adapts it for ColdFire.

The reference manual itself isn't in the repository, and no text from it is. [pac-gen/README.md](pac-gen/README.md)
has the details.

## Development

- **Python tests** for the generator and toolchain. `RM=` points at a markdown copy of the reference manual, for the
  tests that need it; without it they skip:

      python -m pytest pac-gen/tests toolchain/tests

- **The HAL's host tests:**

      cargo test -p embassy-mcf5441x --features mcf54418,bootloader --lib

- **A ColdFire build of the HAL:**

      toolchain/cargo-cf embassy-mcf5441x --features mcf54415,bootloader,executor-thread,executor-interrupt

- **The HAL on the emulator:** `hal-tests/` and the examples, built with `toolchain/cargo-cf` and run on
  `mcf5441x-emu` against emulated peers (they skip without the ColdFire toolchain):

      cargo test -p mcf5441x-emu --test hal --test examples

The PAC needs a part feature, so a plain `cargo build` or `cargo test` leaves it out (`default-members`). Workspace
commands take `--features mcf5441x-pac/<part>`.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as
defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
