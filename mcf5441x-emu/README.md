# mcf5441x-emu

An exact emulator of the MCF5441x (ColdFire V4e) core, for running firmware and tests on the host. It has no
dependencies. The core executes the integer instruction set exactly; a board model attaches its own peripherals
through a small device interface and decides what a stop means.

## What is exact

- The integer instruction set of ISA_A, ISA_B and ISA_C, with condition codes, the hardware divider (DIVU/DIVS/REMU/REMS)
  and the EMAC (CFPRM ch. 6, RM ch. 5): four accumulators, MACSR, the mask register, fractional and saturating modes.
- Values measured on an MCF54415 are reproduced: MVS, MVZ, REMU, FF1, BITREV, BYTEREV, MOV3Q, SATS and MAC; BRA.L
  on ISA_C; the fault status of address-error frames (0100 for an odd jump target, 1100 for a bad index extension
  word); the ACCext01 layout; and the EMAC's overflow behaviour (a product overflow without OMC keeps the low 48 bits
  of the product, PAVn is recomputed by each MAC without OMC, and with OMC a set PAVn freezes the accumulator).
- Exceptions are vectored through VBR with ColdFire stack frames (CFPRM ch. 11). An [`ExceptionPolicy`] decides which
  vectors end the run instead of being taken; the default stops on every vector except TRAP #0-15 and interrupts.
- Interrupts are sampled at instruction boundaries, so a request raised by a device write is taken before the next
  instruction.

Where the manuals do not say what the hardware does and it was not measured, the code carries a `guess:` comment with
the reason, and so does its test. The notable open points: the fault status for an odd RTS, JSR or BSR target (taken
as 0100, like the measured odd jump), ACCext23 (taken to mirror the measured ACCext01), and the parts of a product
overflow without OMC the measured case does not settle (bits 47-40 of the product kept rather than sign-extended
from bit 39, unsigned integers, and the scale factor).

## What is not modelled

- The FPU. This chip has none: FPU opcodes take the line-F exception (vector 11).
- The CAU (cryptographic acceleration unit): its instructions stop the run with `Stop::Unmodelled` and the detail `CAU`.
- MMU translation: MOVEC accepts a write to MMUBAR, the memory-mapped MMU registers are not mapped, and addresses are
  never translated.
- Caches: there is no cache model. CACR\[EUSP\] selects the separate user stack pointer; the rest of CACR and the ACRs
  accept writes and have no effect.
- Cycle timing: one instruction is one tick of the machine clock.

## Speed

Instructions are decoded once into blocks and run from a cache; a write over cached code is seen by the next
instruction fetched. On a bitwise CRC-32 loop the interpreter ran at about 200 MIPS in a release build on a desktop
x86-64 machine. That is a measurement of that benchmark (`tests/speed.rs`), not a guarantee.

## Attaching a board

A [`Machine`] holds a [`Cpu`] (`m.cpu`), a [`Memory`] (`m.mem`) and the devices. Map RAM with `m.mem.map_ram`, then add
peripherals:

- [`Device`] is the trait a peripheral implements: `read` and `write` at an offset in its window, plus optional
  `next_event` and `advance` for things it does by itself on the machine clock.
- `add_device(base, size, device)` maps a device window; `add_timer(device)` adds one with no registers.
- `add_interrupt_controller(base, size, controller)` maps an [`InterruptController`]. Devices drive request lines with
  `cx.irq.set(controller, source, on)`; the controller turns them into a level and a vector.
- A device ends a run with `DeviceCx::halt(kind, detail)`; the instruction making the access completes first.
- `m.device::<T>(id)` and `m.device_mut::<T>(id)` give typed access to a device after the fact.

`Machine::run(Limit)` returns a [`StopReason`] with a [`Stop`] kind, the program counter and a detail string.
A [`Limit`] bounds a run by `until_pc` and `max_instructions`; `set_time_limit` bounds it by the clock. Stops include
`UntilPc`, `InstructionLimit`, `TimeLimit`, `HaltedForever` (STOP with nothing that can wake the core), `Illegal`,
`Unmapped`, `Poll`, `Unmodelled`, `Exception(vector)` and `Halt(reason)`. A faulting instruction does not complete: PC
stays on it, and a later run can resume.

For debugging a board model: `m.mem.trace(true)` records device accesses (`take_trace`), a poll detector stops a loop
that reads one device register over and over with no write in between (`set_poll_limit`), and `m.profile(true)` counts
block entries (`take_profile`).

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

## License

MIT OR Apache-2.0.
