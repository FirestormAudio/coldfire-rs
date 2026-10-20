# embassy-mcf5441x

An [Embassy](https://embassy.dev) HAL for the NXP MCF5441x (ColdFire V4e: MCF54410, MCF54415–MCF54418). Part of
[coldfire-rs](https://github.com/FirestormAudio/coldfire-rs). The drivers run on hardware, on an MCF54415 board, except SSI (emulation-tested; its hardware run is pending).

## Drivers

| Module | What it does |
|---|---|
| `init`, `clocks` | Takes the peripherals, reads the core and bus clocks from the PLL, and starts the time driver. |
| `interrupt` | Interrupt levels (1–6), enable and mask, software pends, and `bind_interrupts!`. |
| `executor` | A thread executor that idles in `stop`, and an interrupt executor driven by a software interrupt. |
| time driver | embassy-time on one PIT, ticking at 1 kHz. |
| `gpio`, `pinmux` | `Output`, `Input` and `Flex` pins, pad slew rates. Each driver muxes its own pads. |
| `dma` | eDMA channels and transfer descriptors. |
| `uart` | Blocking `Uart`, `UartTx` and `UartRx`, and a `BufferedUart` with eDMA receive and interrupt-driven transmit (`embedded-io`, `embedded-io-async`). |
| `spi` | DSPI: `SpiBus` and `SpiDevice` with hardware chip selects (`embedded-hal` 1.0, blocking). |
| `ssi` | SSI0/SSI1: I2S or network mode, full-duplex eDMA rings, `render(input, output)` in a software interrupt at a level you choose; a clocks-only start for codec bring-up. |
| `esdhc` | eMMC block reads (read-only) through eDMA, async. |
| `usb` | USB OTG in device mode at high speed through a ULPI PHY (`embassy-usb-driver`); MCF54415–MCF54418. |

## Example

```rust
#![no_std]
#![no_main]

use coldfire_rt::entry;
use embassy_executor::Spawner;
use embassy_mcf5441x::executor::Executor;
use embassy_mcf5441x::uart::{Config, UartTx};
use embassy_time::Timer;
use static_cell::StaticCell;

#[embassy_executor::task]
async fn tick(mut tx: UartTx<'static>) {
    loop {
        Timer::after_secs(1).await;
        tx.blocking_write(b"tick\r\n");
    }
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    let p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(24_000_000)); // the PLL's reference clock
    let tx = UartTx::new(p.UART0, p.PF3, Config::new(115_200)).unwrap();
    unsafe { coldfire::interrupt::enable() };
    EXECUTOR.init(Executor::new()).run(|spawner: Spawner| spawner.spawn(tick(tx).unwrap()))
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
```

Interrupt-driven drivers take their interrupts through `bind_interrupts!`, as in embassy:

```rust
embassy_mcf5441x::bind_interrupts!(struct Irqs { USBOTG => embassy_mcf5441x::usb::InterruptHandler; });
```

The [repository](https://github.com/FirestormAudio/coldfire-rs/tree/main/examples) has complete examples: `uart-echo`, `ticker` and `usb-cdc`.

## Features

- **The part:** exactly one of `mcf54410`, `mcf54415`, `mcf54416`, `mcf54417`, `mcf54418`.
- **How it boots:** exactly one of `bootloader` (a bootloader has set up the PLL and DDR), or cold boot from reset
  through `boot-serial`, `boot-nor` or `boot-nand`.
- **Executors:** `executor-thread` and `executor-interrupt`.
- **The time driver:** `time-driver-pit0` (the default), `time-driver-pit1`, `time-driver-pit2` or
  `time-driver-pit3`. The PIT it uses leaves `Peripherals`.

## Cold boot

With `boot-serial`, `boot-nor` or `boot-nand`, the app is one image the chip's boot hardware starts from reset. It
has two parts, which coldfire-rt's cold layout links together:
- **The boot part** runs where the boot source starts the core. It sets up the chip from your `BootConfig`, copies the
  app part into DDR, and hands over to the usual startup.
- **The app part** runs in DDR and loads from the boot medium, right after the boot part.

| Feature | The boot part runs | Room | What the boot medium holds |
|---|---|---|---|
| `boot-serial` | in SRAM at 0x80000000, where the serial boot facility copied it | 60 KB | the SBF header (BLDIV, BLL, SBF_RCON), then the image: SP/PC, boot part, app part |
| `boot-nor` | in place from NOR on FlexBus CS0, at 0 | the flash | the image at 0 |
| `boot-nand` | in the NFC's boot buffer at 0, then (relocated) in SRAM | 3968 bytes (whole 8-byte chunks of the 3984 the NFC shows) | four boot pages holding the image's boot part, interleaved 8 bytes at a time; then the app part in the same boot-page format (1056-byte pages: 996 payload bytes, 60 inverted 32-bit BCH bytes) |

Making that image (the SBF header, NAND's page split, interleave and ECC) is your board's tooling: the crates stop
at the link layout. Your `memory.x` defines `MEDIUM`, the boot medium as the boot part sees it (serial: the image
from 0x80000000 on; NOR: the flash at 0; NAND: leave it to the HAL).

Describe the board with `#[boot_config]`:

```rust,ignore
use embassy_mcf5441x::boot::*;

/// 25 MHz × 20: VCO 500 MHz, core 250 MHz, bus 125 MHz.
const PLL: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };
/// Your DRAM's controller words (RM 21.4), from its datasheet at your SD_CLK, START clear; DDR_PADCR; pad drive.
const DDR: Ddr = Ddr { cr: &[/* (index, value), ... */], padcr: 1 << 9 | 1, full_strength: false };

#[boot_config]
static BOOT: BootConfig = BootConfig::builder(RefClock::Crystal(25_000_000), Pll::Set(PLL), DDR)
    .app(AppSource::SpiFlash { offset: 7, ctar: 7 << 27 | 1 })   // the image starts after the 7-byte SBF header
    .build();
```

- **`Pll::Set`** programs the PLL in a limp-mode window (one 32-bit PLL_DR write, then the lock wait). **`Pll::Keep`**
  is for a PLL the boot hardware already set, such as a serial-boot header's SBF_RCON: the settings say what it set,
  and the boot part halts if PLL_CR/PLL_DR disagree. Either way the settings are checked.
- **The checks are compile errors:** the reference manual's limits (reference 14–50 MHz, VCO 240–500 MHz, core at
  most 250 MHz, bus exactly half the core, NFC at most 80 MHz), START clear, PADCR's mandatory bits, and an app
  source the boot feature can use. `.out_of_spec(OutOfSpec::IKnowWhatImDoing)` lifts the frequency limits only.
- **Optional parts:** `.ddr2clk(...)`, `.outdiv3(...)` and `.outdiv4(...)` (set in the limp window); `.extra(...)`
  for board register writes at a named point (after the PLL, or before the DDR controller); `.cs0(...)` for
  `boot-nor`'s flash timing (it stays at 0).
- **A failure halts** with a marker in D0: `0xB007_0001` PLL mismatch, `_0002` PLL lock, `_0003` DDR init, `_0004`
  SPI, `_0005` uncorrectable NAND page, `_0006` NAND busy. A CPU exception during the boot part isn't caught.
- **Limits to keep in `memory.x`:** with `boot-serial` and `AppSource::Mapped` in SRAM, the whole image must fit below
  the boot part's 4 KB stack at the top of SRAM (and within what your SBF header's BLL loads); NAND boards set
  `__app_part_max` to what their NAND holds for the app part.
- **`#[boot_config]` is coldfire-rt's attribute** (re-exported here): the app depends on `coldfire-rt` directly, as
  every app does for its startup.

## What it assumes

- **Under `bootloader`, a bootloader has set up the PLL, the DDR controller and the boot pin mux.** The HAL reads
  the clocks; it doesn't program them. Under cold boot, the boot part did that.
- **Pads with an uncertain pin-mux layout are left alone by GPIO.** That covers PAR_CS, PAR_IRQ0H and PAR_DSPIOWL's
  OWDAT field, the serial-boot flash chip select (PC0), and the PAR_FEC group-mode pads.
- **Data cache:** the DMA drivers assume it is off.

## Building

Building for ColdFire needs the coldfire-rs toolchain, and the board supplies a `memory.x`; see the
[repository README](https://github.com/FirestormAudio/coldfire-rs#quick-start).

## License

Licensed under either of [Apache License, Version 2.0](https://github.com/FirestormAudio/coldfire-rs/blob/main/LICENSE-APACHE) or
[MIT license](https://github.com/FirestormAudio/coldfire-rs/blob/main/LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as
defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
