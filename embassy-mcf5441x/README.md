# embassy-mcf5441x

An [Embassy](https://embassy.dev) HAL for the NXP MCF5441x (ColdFire V4e: MCF54410, MCF54415–MCF54418). Part of
[coldfire-rs](https://github.com/FirestormAudio/coldfire-rs). Every driver runs on hardware, on an MCF54415 board.

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

## What it assumes

- **A bootloader has set up the PLL, the DDR controller and the boot pin mux.** The HAL reads the clocks; it doesn't
  program them.
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
