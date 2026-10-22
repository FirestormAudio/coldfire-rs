# mcf5441x-emu

An exact emulator of the MCF5441x (ColdFire V4e) core, for running firmware and tests on the host. It has no
dependencies. The core executes the integer instruction set exactly; models of the chip's peripherals sit at their
addresses (see [Peripherals](#peripherals)), and a board model adds its memory and the parts around the chip through
a small device interface and decides what a stop means.

## What is exact

- The integer instruction set of ISA_A, ISA_B and ISA_C, with condition codes, the hardware divider (DIVU/DIVS/REMU/REMS)
  and the EMAC (CFPRM ch. 6, RM ch. 5): four accumulators, MACSR, the mask register, fractional and saturating modes.
- Values measured on an MCF54415 are reproduced: MVS, MVZ, REMU, FF1, BITREV, BYTEREV, MOV3Q, SATS and MAC; BRA.L
  on ISA_C; the fault status of address-error frames (0100 for an odd JMP, BRA, BSR, JSR or RTS target or an odd PC
  in the frame an RTE would load, 1100 for a bad index extension word) and what each leaves on the stack (JSR pushes
  its return address first, BSR pushes nothing, RTS and RTE pop nothing); a MOVEC to 0xC04, which changes nothing;
  a WDEBUG write of 0x00020000 to CSR, which runs on; the ACCext01 and ACCext23 layouts; and the EMAC's overflow
  behaviour (a product overflow without OMC adds the product's low 40 bits, signed or unsigned; fractional -1 * -1 is
  no product overflow; PAVn is recomputed by each MAC without OMC; with OMC a set PAVn freezes the accumulator).
- Exceptions are vectored through VBR with ColdFire stack frames (CFPRM ch. 11). An [`ExceptionPolicy`] decides which
  vectors end the run instead of being taken; the default stops on every vector except TRAP #0-15 and interrupts.
- Interrupts are sampled at instruction boundaries, so a request raised by a device write is taken before the next
  instruction.

Where the manuals do not say what the hardware does and it was not measured, the code carries a `guess:` comment with
the reason, and so does its test. The EMAC overflow cases and the address-error frames of jumps, branches, calls and
returns are all measured; open points include a PC made odd from outside (it faults at the fetch with FS 0100) and
which CSR bits a WDEBUG may write without stopping the run (see below).

## What is not modelled

- The FPU. This chip has none: FPU opcodes take the line-F exception (vector 11).
- The CAU (cryptographic acceleration unit): its instructions stop the run with `Stop::Unmodelled` and the detail `CAU`.
- MMU translation: MOVEC accepts a write to MMUBAR, the memory-mapped MMU registers are not mapped, and addresses are
  never translated.
- Caches: there is no cache model. CACR\[EUSP\] selects the separate user stack pointer; the rest of CACR and the ACRs
  accept writes and have no effect.
- Cycle timing: one instruction is one tick of the machine clock.
- The debug module (RM ch. 43). WDEBUG is decoded (the command word `0x2C80 | DRc`, then the data): a write to CSR
  that sets only bits a program cannot observe (BSTAT, FOF, TRG, HALT, BKPT, HRL, PCD and IPW; only PCD is measured)
  changes nothing; any other WDEBUG stops the run with `Stop::Unmodelled` and a detail naming the register and the
  value.

## Speed

Instructions are decoded once into blocks and run from a cache; a write over cached code is seen by the next
instruction fetched. On a bitwise CRC-32 loop the interpreter ran at about 200 MIPS in a release build on a desktop
x86-64 machine. That is a measurement of that benchmark (`tests/speed.rs`), not a guarantee.

## Attaching a board

A [`Machine`] holds a [`Cpu`] (`m.cpu`), a [`Memory`] (`m.mem`) and the devices. Map RAM with `m.mem.map_ram`, then add
the chip's peripherals ([Peripherals](#peripherals)) and the board's own:

- [`Device`] is the trait a peripheral implements: `read` and `write` at an offset in its window, plus optional
  `next_event` and `advance` for things it does by itself on the machine clock.
- `add_device(base, size, device)` maps a device window; `add_timer(device)` adds one with no registers.
- `add_interrupt_controller(base, size, controller)` maps an [`InterruptController`]. Devices drive request lines with
  `cx.irq.set(controller, source, on)`; the controller turns them into a level and a vector.
- A device ends a run with `DeviceCx::halt(kind, detail)`; the instruction making the access completes first. With
  `DeviceCx::refuse(kind, detail)` it refuses the access instead: the instruction does not complete, PC stays on it
  and the run stops with that kind and detail.
- A misaligned word or longword access to a device is split into the aligned pieces of RM Table 3-12, each a device
  access of its own. Instruction fetches from a device window are neither traced nor poll-counted.
- A device with events can list the interrupt sources they could assert (`wake_sources`), so that a STOP nothing can
  wake halts at once; an interrupt controller hears which of its requests the core took (`acknowledge`).
- `m.device::<T>(id)` and `m.device_mut::<T>(id)` give typed access to a device after the fact.

`Machine::run(Limit)` returns a [`StopReason`] with a [`Stop`] kind, the program counter and a detail string.
A [`Limit`] bounds a run by `until_pc` and `max_instructions`; `set_time_limit` bounds it by the clock. Stops include
`UntilPc`, `InstructionLimit`, `TimeLimit`, `HaltedForever` (STOP with nothing that can wake the core), `Illegal`,
`Unmapped`, `Poll`, `Unmodelled`, `Exception(vector)` and `Halt(reason)`. A faulting instruction does not complete: PC
stays on it, and a later run can resume.

For debugging a board model: `m.mem.trace(true)` records device accesses (`take_trace`), a poll detector stops a loop
that reads one device register over and over with no write in between (`set_poll_limit`), and `m.profile(true)` counts
block entries (`take_profile`).

## Peripherals

[`Mcf5441x::build`](soc::Mcf5441x::build) maps the chip's modules into a machine at their RM addresses (RM Table 1-3,
Table 1-4), each wired to its interrupt source (RM Table 17-15, Table 17-16, Table 17-17) and DMA request line (RM Table
19-6), and returns their ids in a [`Chip`](soc::Chip). What the chip does not decide comes from
[`ChipOptions`](soc::ChipOptions): the parts behind the chip (the SPI NOR flash and the DSPI and chip select it is on,
the eMMC, the NAND array), the PLL_CR and PLL_DR values the reset configuration left, and the time scale of every clock
in machine ticks (each UART's character time, each SSI's frame period, ticks per microsecond for USB, the eSDHC's data
latency and CMD6 busy time), the clock gates at the start, and the rate of each DMA timer's DT*n*IN pin. Nothing in the
models assumes a clock frequency. `ChipOptions::default()` is the chip with nothing on it (no parts, the PLL values of
the default reset configuration, UARTs with no character time (0 ticks: sending and receiving are instant), one tick per
frame and microsecond, every clock gate open, no DT*n*IN pin toggling); the `with_nor`, `with_card`, `with_nand`,
`with_pll`, `with_uart_ticks`, `with_ssi_ticks`, `with_usb_ticks_per_us`, `with_sd_timing`, `with_gates` and `with_dtin`
builders change one part of it. `Chip::nor`, `nor_mut`, `card` and `card_mut` give the parts back from the machine.

```
use mcf5441x_emu::Machine;
use mcf5441x_emu::soc::{ChipOptions, Gates, Mcf5441x, Uart};

let mut m = Machine::new();
m.mem.map_ram(0x4000_0000, 0x0100_0000); // DDR: its size is the board's
let chip = Mcf5441x::build(
    &mut m,
    ChipOptions {
        nor: None,
        card: None,
        nand: None,
        pll: (0x19, 0x0043_0C41), // test values
        uart_ticks_per_byte: [100; 10],
        ssi_ticks_per_frame: [500; 2],
        usb_ticks_per_us: 100,
        sd_data_latency: 1000,
        sd_cmd6_busy: 2000,
        gates: Gates::Open,
        dtin: [None; 4],
    },
);
// UART0: receiver enable, a character on the line at tick 0, in URB one character time later.
m.access(0xFC06_0008, mcf5441x_emu::Size::Byte, Some(0x01)).unwrap();
m.device_mut::<Uart>(chip.uart[0]).receive(&[(0, Some(b'x'))]);
assert_eq!(m.idle_until(100), None);
assert_eq!(m.access(0xFC06_000C, mcf5441x_emu::Size::Byte, None), Ok(u32::from(b'x')));
```

### The chip map

| Base | Block | Interrupt (controller, source) | DMA request lines |
|---|---|---|---|
| 0xFC04_8000, 0xFC04_C000, 0xFC05_0000 | INTC0, INTC1, INTC2 | | |
| 0xFC05_4000 | global IACK registers | | |
| 0xFC04_4000 (8 KB) | eDMA, a bus master | channels 0-15: INTC0 8-23; 16-55: INTC1 8-47; 56-63: INTC2 0 | software: 30, 31, 42, 43, 46, 47 |
| 0xFC08_0000 + 0x4000 n | PIT0-PIT3 | INTC2 13-16 | |
| 0xFC07_0000 + 0x4000 n | DMA timers 0-3 | INTC0 32-35 | |
| 0xFC09_0000 | Edge port 0 | INTC0 1-7 (pins IRQ1-IRQ7) | |
| 0xFC06_0000 + 0x4000 n | UART0-UART3 | INTC0 26-29 | receive: 2, 4, 6, 16 |
| 0xEC06_0000 + 0x4000 n | UART4-UART7 | INTC1 48-51 | receive: 18, 20, 22, 32 |
| 0xEC07_0000, 0xEC07_4000 | UART8, UART9 | INTC1 52, 53 | receive: 34, 36 |
| 0xFC0B_C000, 0xFC0C_8000 | SSI0, SSI1 | | TX0: 50, 54; RX0: 48, 52 |
| 0xFC05_C000, 0xFC03_C000, 0xEC03_8000, 0xEC03_C000 | DSPI0-DSPI3 (RM Table 40-2) | | |
| 0xFC0C_C000 | eSDHC | INTC2 31 | 59 |
| 0xFC0B_0000 (4 KB) | USB OTG, a bus master | INTC2 17 | |
| 0xFC0B_8000 | DDR controller | | |
| 0xFC0C_0000 | PLL | | |
| 0xFC0F_C000 | NFC (when the options give a NAND array) | | |

Every window is the module's 16 KB slot unless the table says otherwise. The interrupt controllers are added in
order, so INTC0 wins over INTC1 and INTC1 over INTC2 at an equal level (RM 17.3.2).

Stub blocks, which accept every access and read back the last value written at each offset
([`StubBlock`](soc::StubBlock)): the crossbar switch (0xFC00_4000), the FlexBus registers (0xFC00_8000), the SCM
(0xFC04_0000), the CCM with the reset controller and power management (0xEC09_0000) and pin multiplexing and GPIO
(0xEC09_4000). Every other module in RM Table 1-3 and Table 1-4 (the FlexCANs, the I2Cs, the ADC and DACs, the RTC, the
SIM, the USB host, the RNG, the Ethernet MACs and switch, 1-Wire and the mcPWM) and RGPIO (0x8C00_0000) is an
[`Unmodelled`](soc::Unmodelled) block: an access stops the run with `Stop::Unmodelled` and the module's name and offset
as the detail, with PC on the instruction. The SRAM (where the board's RAMBAR puts it, RM ch. 7) and the DDR (its size
is the board's) are not mapped by `build`; the reserved slots fault as unmapped.

### Clock gates

The SCM's peripheral power management registers (RM 9.2.2-9.2.4) are modelled inside the SCM stub: PPMSR0/1 and
PPMCR0/1 set and clear one CDn bit, or all of them with SAMCD and CAMCD, and PPMHR0/1 and PPMLR0/1 hold the gates
(bit set: the module's clock is off). Every module in a peripheral slot with a CDn bit (RM Tables 9-5 to 9-8) is tied
to its gate; the SCM, the crossbar switch, the global IACK slot and RGPIO have none, and neither has RAM. While a
module's gate is closed:

- a read of it returns all ones at every size, for the core and for bus masters alike (measured on an MCF54415);
- a write to it is dropped (measured on an MCF54415);
- its events are held: it is not advanced or serviced, and it lists no wake sources; the PIT, the DMA timers, the
  SSI and the eSDHC stop their clocks and continue where they stopped when the gate opens, while the other models
  with events (the UARTs, USB) catch up when it opens (guess: not measured);
- its interrupt and DMA request lines stay as it left them (guess: not measured), and an interrupt controller's
  requests still reach the core.

[`Gates`](soc::Gates) chooses the gates at the start: `Open` (the default: every module clocked, as before gates were
modelled), `Reset` (the values measured after a soft reset on an MCF54415: PPMHR0 `0x7F9EFBFF`, PPMLR0
`0xFFDEF300`, PPMHR1 `0xFFFFFFCF`, PPMLR1 `0x3F00C0F4`; RM Table 9-1 gives PPMHR0 `0x7E1EFBFF`, without CD56 and
CD55) or `Set` with given values. A power-on reset was not measured. `Chip::gates` reads them back. The gate check
sits on the device path only: RAM accesses never see it. The machine's own API (`Machine::set_clock_gate`,
`set_clock_gates`, `clock_gates`, and `Device::gate` for a model that keeps time) serves any board's models.

### What each block models

- **INTC0-2 and the global IACK** (RM ch. 17): IPR, IMR, the forcing registers, ICRs, SIMR/CIMR, the software and
  level IACK registers, and ICONFIG, CLMASK and SLMASK shared by the three. A forced source delivers even when IMR
  masks it, and IPR never shows forced sources (both measured on an MCF54415).
- **PIT0-3** (RM ch. 38): the prescaler, the modulus, reload and free-running modes, PIF and PIE; longword
  accesses read all ones and write nothing (measured on an MCF54415).
- **DMA timers 0-3** (RM ch. 39): the bus clock (one per tick, as the PIT's), the bus clock / 16 or the DT*n*IN pin
  (`ChipOptions::with_dtin`; by default it never toggles), the prescaler, MODE16, the reference with REF, ORRI and
  free-run or restart, and DTCN cleared by any write. Capture mode, the output modes, DMA requests (DMAEN), the
  1588 timebase and programmable delay stop the run with `Stop::Unmodelled`.
- **Edge port 0** (RM ch. 18): per-pin level or edge detection, EPIER and EPFR on INTC0 sources 1-7; the host drives
  pins IRQ1-IRQ7 with `Eport::set_pin` inside `Machine::with_device`, and a change asserts or drops the interrupt at
  once. All pins start high (guess). EPDDR, EPDR and EPPDR, which RM Table 18-2 leaves out, are where other ColdFire
  edge ports have them (guess).
- **eDMA** (RM ch. 19): 64 channels served at their peripheral's request or by START/SSRT, 8- and 32-bit
  transfers, SMOD/DMOD rings, SLAST/DLAST, INT_HALF/INT_MAJOR and D_REQ; misaligned addresses are configuration
  errors with ES as measured on an MCF54415. On the built chip SERQ is taken only on the request lines a model
  drives (the table's last column) and the channels RM Table 19-6 leaves to software (30, 31, 42, 43, 46, 47);
  START and SSRT work on every channel.
- **UART0-9** (RM ch. 41): a one-deep receiver and a transmitter with a holding and a shift register, one character
  per character time (a status read 3 ticks after a write to an idle transmitter sees TXRDY again: measured on an
  MCF54415 as 2 to 9 instructions, the model takes the shortest); RXRDY/TXRDY interrupts; overrun and framing
  errors; the receive DMA request. A [`SerialPeer`](soc::SerialPeer) attached with `Uart::attach` stands for whatever
  is on the far end of the lines: it is called with each character as it leaves the shift register and at its own
  events, and sends characters back (or ends the run) through a [`PeerCx`](soc::PeerCx); `Uart::peer` and `peer_mut`
  give it back by type.
- **SSI0-1** (RM ch. 35): I2S master or network mode with transmit and receive FIFO 0, one frame per frame period,
  the watermark DMA requests; 24-bit words send bits 23:0 of a FIFO word (measured on an MCF54415); an underrun sends
  silence and empties the FIFO (where the RM re-transmits the previous data). The host supplies the received samples
  and reads what was sent.
- **DSPI0-3** (RM ch. 40): master mode, 8- and 16-bit frames through CTAR0/1, the FIFOs, EOQ and the transfer counter;
  frames take no time. [`SpiNor`](parts::SpiNor) is a serial NOR flash for a chip select.
- **eSDHC** (RM ch. 25): commands, responses and block reads through DATPORT, drained by eDMA channel 59, with an
  [`Emmc`](parts::Emmc) (read-only) behind it; the reset values and the RSTA/RSTD behaviour measured on an MCF54415.
- **USB OTG** (RM ch. 24): device mode with dQHs and dTDs in memory, endpoints 0-3, and host primitives (setup, IN
  and OUT packets, bus resets, scripts on the machine's clock). A high-speed IN of more than 64 bytes from the DDR
  window (0x4000_0000-0x7FFF_FFFF) never completes (measured on an MCF54415); from SRAM it does.
- **DDR controller** (RM ch. 21): the registers, with the reset values and the INTSTATUS after START measured on an
  MCF54415; the DDR itself is RAM. `Ddrmc::set_init_completes(false)` models a DRAM that never finishes initializing.
- **PLL** (RM ch. 8): PLL_CR and PLL_DR from the options, locked unless the host says otherwise
  (`Pll::set_locked(false)` drops LOCK and LOCKS, and sets LOLF when PLL_CR enables it; LOCKS stays clear when the
  lock returns, as RM 8.3.4 has it). With LOLEN in PLL_CR clear the loss is not detected and LOCK stays set (RM Table
  8-5). A loss with LOLEN and LOLRE or LOLIRQ set stops the next run with `Stop::Unmodelled`: the loss-of-lock reset
  and interrupt request are not modelled.
- **NFC** (RM ch. 22): page reads into the buffer from a [`NandArray`](soc::NandArray), with the ECC status byte
  as the RM describes it; the reset values measured on an MCF54415.

### What refuses

A feature a model leaves out is a tripwire, not a silent difference: the access that reaches it is refused (the
instruction does not complete, PC stays on it) and the run stops with `Stop::Unmodelled`, the detail naming the
block and the feature. Among them: eDMA scatter/gather, channel linking, other transfer sizes, error interrupts
and SERQ on a channel nothing drives; UART FIFO-full receive requests, multidrop and loopback modes; SSI configurations outside I2S master or
network mode with FIFO 0, and its own interrupts; DSPI slave mode, CTAR2-7, interrupt and DMA requests, a push to a
full FIFO; eSDHC write commands, its internal DMA and a clock divider change with the clock on; USB host mode, little-
endian buffers and full-speed bulk packets over 64 bytes; NFC commands other than page reads, and its interrupt; access
sizes a register does not take. A tripwire reached by a device on its own (an eDMA request that never drops, a USB DMA
access that fails) halts the run with the same kind after the instruction. Each model's documentation lists its own.

### Bus masters and DMA

The eDMA and the USB controller are bus masters ([`Machine::add_bus_master`]): during any call into them they reach
the address space through a [`Bus`] that leaves themselves out. A peripheral drives the level of its DMA request line
([`DmaLines`]); the machine serves the bus masters at every instruction boundary, after due device events and before
interrupts are sampled, until the lines settle, and after every host-side call. A transfer takes no time: a request is
served in the tick it is raised. A bus-master write over cached code is seen by the next fetch, as a CPU write is.

### From the host

- `Machine::access(addr, size, write)` makes an access as the CPU would at the current tick, then serves the bus
  masters; when a device refuses it, `Machine::take_refusal` gives the kind and the detail.
- `Machine::idle_until(tick)` moves the clock, running due device events and the bus masters but no instructions; it
  returns the stop when a device halts first.
- `Machine::with_device::<T, R>(id, f)` calls into a device with a full context (for the USB host primitives, say),
  then serves the bus masters; `device` and `device_mut` give plain typed access (a UART's line, an SSI's capture, a
  card's command log).
- [`soc::UsbHost`](soc::UsbHost) (in `soc::usb::host`) is a USB host on the OTG port that runs the machine itself
  between its transactions: `connect`, `enumerate`, `control_in` and `control_out`, and a CDC-ACM port's `cdc_open`,
  `write` and `read`. Each operation runs the machine in short slices until the device answers or a timeout passes
  (`set_timeouts`), and fails with a [`HostError`](soc::HostError). With `set_urb` a bulk IN is held as a read URB
  of that size is (until a short packet or a full URB, as Linux's cdc_acm does); with none every packet reaches
  `read` at once. A core waiting in STOP for the host (`Stop::HaltedForever`) is not an error: the host moves the
  clock on and keeps going.
- Logs: the models that keep one (the UART's sent characters, register writes of the DDR controller, PLL, eSDHC, USB
  and stub blocks, the DSPI's frames, the SSI's capture, the NOR's and the card's command logs) drain it with a
  `take_*`, and `record(false)` stops the logging; that changes only the log, never the registers.

### Known differences

- Clock gates: `build` starts with every gate open unless the options say otherwise; the chip resets with most of
  them closed. Reserved bits of PPM{H,L}Rn are stored like the others (guess: not measured).
- Time: one instruction is one tick, so code runs at one instruction per tick of whatever clock the board's options
  scale to; transfers, frames and register accesses take no bus time.
- Interrupts the models do not raise are never asserted. Setting the enables of the SSI's, the DSPI's and the NFC's
  interrupts and of the eDMA error interrupt is refused (see each model); the DDR controller's INTMASK and the PLL's
  LOCIRQ and LOLIRQ are stored, but neither model ever raises its interrupt, and neither is wired to an interrupt
  controller; a PLL loss of lock that would request the interrupt (or a reset) stops the run instead.
- DMA requests nothing drives: the UART transmit requests (and every request line of a block that is not modelled,
  or whose model does not request DMA) are never asserted. On the built chip, SERQ on such a channel is refused,
  naming the channel; a software start (START, SSRT) works on any channel.
- Bus masters' accesses are not split: a misaligned access from the eDMA or USB reaches a device whole. The chip's
  models refuse one that crosses a register.

Open guesses (each a `guess:` comment in the code and its test): the PIT's expiry one count after PCNTR reads 0 and
the prescaler's partial count on a PRE change; the UART hand-over to the shift register taking the shortest of the
measured 2 to 9 instructions; the DSPI's FIFO counters with 16 entries and the deselect of a part reached by an
idle-low select; the eSDHC's RSTA reset of the registers not measured and a data timeout one tick after the data
phase; USB detach 100 µs after RS clears; the SSI's underrun sending silence and emptying the FIFO, where RM Table
35-8 re-transmits the previous data; the word and longword accesses over the INTC's 8-bit SIMR, CIMR, CLMASK and
SLMASK; the DDR controller's PADCR bit 9; the NFC's address increment after a single-page read and its ECC status
byte, which follows the RM and is not measured (in ECC bypass mode, a guess); the PLL relocking at once (LOCKS stays
clear after a lost lock); the NOR flash's WEL and WIP timing around a failed program.

## Running a program

[`load_elf`](elf::load_elf) reads a 32-bit big-endian m68k ELF executable: each `PT_LOAD` segment goes to its physical
address with the rest of its memory size zeroed (a `.bss` segment with no file data is zeroed too), and the entry
point is returned. Anything else is an [`elf::ElfError`].

```no_run
// Needs a linked program on disk, so this is compiled but not run.
use mcf5441x_emu::{Limit, Machine, elf::load_elf};

let mut m = Machine::new();
m.mem.map_ram(0x4010_0000, 0x0100_0000);
let elf = std::fs::read("firmware.elf").unwrap();
m.cpu.pc = load_elf(&mut m, &elf).unwrap();
m.cpu.a[7] = 0x4100_0000;

let r = m.run(Limit { max_instructions: Some(1_000_000), ..Limit::default() });
println!("stopped: {:?} at {:#x} {}", r.kind, r.pc, r.detail);
```

A program can also be placed by hand, which needs no files:

```
use mcf5441x_emu::{Limit, Machine, Stop};

let mut m = Machine::new();
m.mem.map_ram(0x4010_0000, 0x1000);
// moveq #5,%d0 ; addq.l #1,%d0 ; nop
m.mem.load(0x4010_0000, &[0x70, 0x05, 0x52, 0x80, 0x4e, 0x71]).unwrap();
m.cpu.pc = 0x4010_0000;

let r = m.run(Limit { until_pc: Some(0x4010_0006), ..Limit::default() });
assert_eq!(r.kind, Stop::UntilPc);
assert_eq!(m.cpu.d[0], 6);
```

### Booting from reset

[`boot`](boot::boot) starts a program as the chip does from reset in each boot mode, doing what the boot hardware
has done before the core fetches its reset vector: the serial boot facility copies the boot image from the SPI flash
on DSPI0 PCS1 into the SRAM (RM ch. 11), FlexBus CS0 decodes a NOR flash at address 0 (RM 20.4.1.3), or the NFC
shows the four boot pages at address 0 (RM 22.4.4). It then loads SP and PC from the image's first two longwords.
[`sbf_image`](boot::sbf_image), [`nand_pages`](boot::nand_pages) and [`nand_boot_view`](boot::nand_boot_view) lay
out an image as each medium holds it, and [`elf::sections`] and [`elf::symbols`] read the sections (with their run
and load addresses) and symbols of a linked program to build it from.

```
use mcf5441x_emu::boot::{BootSource, boot, sbf_image};
use mcf5441x_emu::parts::SpiNor;
use mcf5441x_emu::soc::{ChipOptions, Mcf5441x};

let mut m = Machine::new();
m.mem.map_ram(0x8000_0000, 0x1_0000); // the SRAM, where the board's RAMBAR puts it
let mut chip = Mcf5441x::build(&mut m, ChipOptions::default());
// SP, PC, then `bra.s .`
let image = [0x80, 0x01, 0x00, 0x00, 0x80, 0x00, 0x00, 0x08, 0x60, 0xFE, 0x4E, 0x71];
let flash = SpiNor::new(1 << 20, [0x01, 0x20, 0x18, 0x4D, 0x01, 0x80], &sbf_image(&image, image.len(), 0));
let pc = boot(&mut m, &mut chip, BootSource::Serial { flash, sram_base: 0x8000_0000 }).unwrap();
assert_eq!((pc, m.cpu.a[7]), (0x8000_0008, 0x8001_0000));
assert!(chip.nor(&m).is_some()); // the flash stays on DSPI0 PCS1
# use mcf5441x_emu::Machine;
```

## License

MIT OR Apache-2.0.
