# hal-tests

`embassy-mcf5441x`'s drivers as small ColdFire programs, one per driver area, for `mcf5441x-emu`'s tests to run on
the emulated chip. It isn't published, and it stays out of the workspace as `examples/` does.

| Program | What it checks |
|---|---|
| `gpio` | Outputs set, cleared and toggled; inputs read with each pull and driven by the host; a `Flex` turned from input to output. |
| `uart` | The blocking UART: echo, a framing error reported before its character, an overrun. |
| `uart-buffered` | `BufferedUart`: writes and flushes, `write_atomic` commands kept whole, receive DMA across the ring's wrap. |
| `spi` | DSPI transactions and transfers, wide reads through CTAR1, the bare bus with a GPIO select. |
| `dma` | eDMA copies at each size, a source ring, SLAST/DLAST, the major-loop interrupt, a channel link. |
| `esdhc` | eMMC bring-up, the sector count and EXT_CSD, multi-block reads. |
| `ssi` | An I2S stream whose render adds 1 to each sample, with no block late or skipped. |
| `time` | embassy-time timers and a ticker, against `Instant` and the host's clock. |
| `executor-interrupt` | Interrupt executors at levels 2 and 5 preempting each other and the thread executor. |

Build them with the repository's toolchain:

    toolchain/cargo-cf hal-tests

`mcf5441x-emu/tests/hal.rs` builds them into `hal-tests/target/emu` and runs each with the peers it needs: a UART
peer, an SPI part, an eMMC, SSI samples, GPIO levels. It skips without the ColdFire toolchain.

A program reports through libgloss m68k semihosting: it writes lines to stdout and ends with an exit status.

| Status | Meaning |
|---|---|
| 0 | Every check passed. |
| 1 | A check failed: `FAIL file:line: ...` on stdout. |
| 101 | A panic: `PANIC file:line: message` on stdout. |

`src/lib.rs` has the support the programs share: the semihosting trap, `say!`, `check!`, `check_eq!`, `pass` and
the panic handler. The programs also run under `mcf5441x-run`, except where they need a host peer.
