//! `mcf5441x-run`: run a bare ColdFire ELF on the emulated MCF5441x, with its console and exit status through
//! libgloss m68k semihosting (see `mcf5441x_emu::semihost`).

use std::process::ExitCode;

use mcf5441x_emu::elf::{load_elf, symbols};
use mcf5441x_emu::semihost::{Outcome, Semihost};
use mcf5441x_emu::soc::{ChipOptions, Gates, Mcf5441x};
use mcf5441x_emu::{Limit, Machine, Stop};

const USAGE: &str = "\
usage: mcf5441x-run [--max-ms N] [--max-instructions N] [--clock-hz N] [--epoch S] ELF

Run ELF on an emulated MCF5441x (DDR at 0x40000000, 128 MiB; SRAM at 0x80000000, 64 KiB), serving libgloss m68k
semihosting on this process's stdin, stdout and stderr.

  --max-ms N            stop after N ms of emulated time (default 60000)
  --max-instructions N  stop after N instructions (default: no limit)
  --clock-hz N          the emulated clock, for time (default 132000000)
  --epoch S             gettimeofday's seconds at the start (default 0)

Exit status: the program's exit(n) & 0xFF; 124 when a limit is reached; 125 when the emulator stops (an illegal
instruction, a fault, a plain HALT, a refused call or a bad pointer); 2 for bad arguments or an unloadable ELF.
";

const DDR: (u32, u32) = (0x4000_0000, 0x0800_0000);
const SRAM: (u32, u32) = (0x8000_0000, 0x0001_0000);

struct Args {
    max_ms: u64,
    max_instructions: Option<u64>,
    clock_hz: u64,
    epoch: u64,
    elf: String,
}

/// The arguments, or the exit code to leave with.
fn parse() -> Result<Args, ExitCode> {
    let mut args = Args {
        max_ms: 60_000,
        max_instructions: None,
        clock_hz: 132_000_000,
        epoch: 0,
        elf: String::new(),
    };
    let mut elf = None;
    let mut it = std::env::args().skip(1);
    let usage = |why: &str| {
        eprintln!("mcf5441x-run: {why}\n{USAGE}");
        ExitCode::from(2)
    };
    while let Some(a) = it.next() {
        let flag = match a.as_str() {
            "-h" | "--help" => {
                print!("{USAGE}");
                return Err(ExitCode::SUCCESS);
            }
            "--max-ms" | "--max-instructions" | "--clock-hz" | "--epoch" => a,
            _ if a.starts_with('-') => return Err(usage(&format!("unknown option {a}"))),
            _ if elf.is_none() => {
                elf = Some(a);
                continue;
            }
            _ => return Err(usage("more than one ELF")),
        };
        let Some(n) = it.next().and_then(|v| v.parse::<u64>().ok()) else {
            return Err(usage(&format!("{flag} needs a number")));
        };
        match flag.as_str() {
            "--max-ms" => args.max_ms = n,
            "--max-instructions" => args.max_instructions = Some(n),
            "--clock-hz" if n > 0 => args.clock_hz = n,
            "--clock-hz" => return Err(usage("--clock-hz must be above 0")),
            _ => args.epoch = n,
        }
    }
    args.elf = elf.ok_or_else(|| usage("no ELF"))?;
    Ok(args)
}

fn main() -> ExitCode {
    let args = match parse() {
        Ok(a) => a,
        Err(code) => return code,
    };
    let bytes = match std::fs::read(&args.elf) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("mcf5441x-run: {}: {e}", args.elf);
            return ExitCode::from(2);
        }
    };
    let mut m = Machine::new();
    Mcf5441x::build(&mut m, ChipOptions::default().with_gates(Gates::Open));
    m.mem.map_ram(DDR.0, DDR.1);
    m.mem.map_ram(SRAM.0, SRAM.1);
    let entry = match load_elf(&mut m, &bytes) {
        Ok(e) => e,
        Err(e) => {
            eprintln!("mcf5441x-run: {}: {e}", args.elf);
            return ExitCode::from(2);
        }
    };
    let syms = symbols(&bytes).unwrap_or_default();
    m.cpu.set_sr(0x2700);
    m.cpu.pc = entry;
    m.cpu.a[7] = syms
        .get("__stack")
        .or(syms.get("_stack"))
        .copied()
        .unwrap_or(DDR.0 + DDR.1);
    let ticks = (args.max_ms as u128 * args.clock_hz as u128 / 1000).min(u64::MAX as u128) as u64;
    m.set_time_limit(Some(ticks));

    let mut host = Semihost::new(
        Box::new(std::io::stdin()),
        Box::new(std::io::stdout()),
        Box::new(std::io::stderr()),
    );
    host.epoch = args.epoch;
    host.clock_hz = args.clock_hz;
    let outcome = host.run(
        &mut m,
        Limit {
            max_instructions: args.max_instructions,
            ..Limit::default()
        },
    );
    match outcome {
        Outcome::Exit(n) => return ExitCode::from((n & 0xFF) as u8),
        Outcome::Stopped(s) if s.kind == Stop::TimeLimit => {
            eprintln!(
                "mcf5441x-run: time limit ({} ms) reached at PC 0x{:08X}",
                args.max_ms, s.pc
            );
            return ExitCode::from(124);
        }
        Outcome::Stopped(s) if s.kind == Stop::InstructionLimit => {
            let n = args.max_instructions.unwrap_or_default();
            eprintln!(
                "mcf5441x-run: instruction limit ({n} instructions) reached at PC 0x{:08X}",
                s.pc
            );
            return ExitCode::from(124);
        }
        Outcome::Stopped(s) => {
            eprintln!(
                "mcf5441x-run: {:?} at PC 0x{:08X}, tick {}: {}",
                s.kind,
                s.pc,
                m.now(),
                s.detail
            )
        }
        Outcome::Refused { call, name, pc } => {
            eprintln!(
                "mcf5441x-run: semihosting call {call} ({name}) not supported, at PC 0x{pc:08X}"
            )
        }
        Outcome::BadPointer { call, addr, pc } => {
            eprintln!("mcf5441x-run: semihosting {call}: bad buffer 0x{addr:08X}, at PC 0x{pc:08X}")
        }
    }
    ExitCode::from(125)
}
