# examples

Starting points for an MCF5441x board on `embassy-mcf5441x`:

| Example | What it does |
|---|---|
| `uart-echo` | Blocking UART0 (TX PF3, RX PF4) at 115,200 baud, echoing every byte. |
| `ticker` | The thread executor and embassy-time: a line on UART0 every second. |
| `usb-cdc` | A USB CDC-ACM device on the OTG port and its ULPI PHY, echoing what the host sends. |

Build them with the repository's toolchain (see the top-level README):

    toolchain/cargo-cf examples --features mcf54415   # or mcf54416, mcf54417, mcf54418

The ELFs land in `examples/target/m68k-coldfire-none-elf/release/`.

They assume a bootloader has set up the PLL and DDR, as the HAL does. Before running them on your board, edit:
- **`memory.x`:** where RAM is, where the stack goes, and the on-chip SRAM the USB driver may use;
- **the pads,** in the sources;
- **`REF_HZ`:** your PLL reference clock.

Then load an ELF the way your board loads code, for example over BDM: load it and start it at its entry point.

The `usb-cdc` example uses crates.io's `embassy-usb`, which doesn't yet describe a high-speed device in its
descriptors. The device still runs at high speed.
