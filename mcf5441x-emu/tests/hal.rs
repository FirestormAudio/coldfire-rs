//! embassy-mcf5441x's drivers on the emulated chip: the programs of coldfire-rs's `hal-tests` crate, built with
//! `toolchain/cargo-cf`, each run with the peers it needs under libgloss semihosting ([`Semihost`]). A program
//! passes by exiting 0; a failed check prints `FAIL file:line: ...` and exits 1, a panic exits 101. The tests are
//! skipped (with a message) when the ColdFire toolchain is not built.

use std::cell::RefCell;
use std::io::{self, Write};
use std::path::PathBuf;
use std::process::Command;
use std::rc::Rc;
use std::sync::{Mutex, OnceLock};

use mcf5441x_emu::elf::load_elf;
use mcf5441x_emu::semihost::{Outcome, Semihost};
use mcf5441x_emu::soc::{Chip, ChipOptions, Mcf5441x, PinChange};
use mcf5441x_emu::{Limit, Machine};

/// DDR as hal-tests/memory.x has it: 128 MB at 0x40000000, the stack at its top.
const DDR: (u32, u32) = (0x4000_0000, 0x0800_0000);
const STACK_TOP: u32 = 0x4800_0000;
/// The SRAM where the programs' RAMBAR puts it.
const SRAM: (u32, u32) = (0x8000_0000, 0x1_0000);
/// The bus clock the HAL finds: the chip's reset PLL (FBKDIV 9, REFDIV ÷2, OUTDIV2 3) from the programs' 24 MHz
/// reference, a 120 MHz VCO over 4. One tick is one bus clock (the PIT counts one per tick).
const BUS_HZ: u64 = 30_000_000;
const TICKS_PER_MS: u64 = BUS_HZ / 1000;

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..")
}

/// The directory of the built programs, or `None` (with a message) when the ColdFire toolchain is not built.
fn built() -> Option<PathBuf> {
    static BUILT: OnceLock<Option<PathBuf>> = OnceLock::new();
    static BUILD: Mutex<()> = Mutex::new(());
    let _one = BUILD.lock().unwrap_or_else(|e| e.into_inner());
    BUILT
        .get_or_init(|| {
            let root = root();
            let rcf = std::env::var_os("RCF")
                .map_or_else(|| root.join("build/rust-coldfire"), PathBuf::from);
            if !rcf
                .join("rustc_codegen_gcc/target/debug/librustc_codegen_gcc.so")
                .exists()
            {
                return None;
            }
            let crate_dir = root.join("hal-tests");
            let mut cmd = Command::new(root.join("toolchain/cargo-cf"));
            // the outer cargo's settings are not the ColdFire build's
            for (k, _) in std::env::vars_os() {
                let k = k.to_string_lossy().into_owned();
                if (k.starts_with("CARGO_") && k != "CARGO_HOME")
                    || k.starts_with("RUSTC")
                    || k.starts_with("RUSTFLAGS")
                    || k == "RUSTDOCFLAGS"
                    || k == "RUSTUP_TOOLCHAIN"
                {
                    cmd.env_remove(k);
                }
            }
            // cargo-cf runs in the crate's directory: the target directory is relative to it
            let out = cmd
                .arg(&crate_dir)
                .args(["--target-dir", "target/emu"])
                .env("RCF", &rcf)
                .output()
                .expect("toolchain/cargo-cf runs");
            let err = String::from_utf8_lossy(&out.stderr);
            assert!(
                out.status.success(),
                "cargo-cf hal-tests failed:\n{}",
                &err[err.len().saturating_sub(4000)..]
            );
            Some(crate_dir.join("target/emu/m68k-coldfire-none-elf/release"))
        })
        .clone()
}

/// Program `name` as ELF bytes; `None` (with a message) when the ColdFire toolchain is not built.
fn elf(name: &str) -> Option<Vec<u8>> {
    let Some(dir) = built() else {
        eprintln!("skipped: the ColdFire toolchain is not built (toolchain/setup.sh, or set RCF)");
        return None;
    };
    Some(std::fs::read(dir.join(name)).unwrap_or_else(|e| panic!("{name} built: {e}")))
}

/// A machine with the chip (`options`), DDR and SRAM.
fn machine(options: ChipOptions) -> (Machine, Chip) {
    let mut m = Machine::new();
    m.mem.map_ram(DDR.0, DDR.1);
    m.mem.map_ram(SRAM.0, SRAM.1);
    let chip = Mcf5441x::build(&mut m, options);
    (m, chip)
}

/// A writer the test reads back.
#[derive(Clone, Default)]
struct Sink(Rc<RefCell<Vec<u8>>>);

impl Write for Sink {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.0.borrow_mut().extend_from_slice(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl Sink {
    fn text(&self) -> String {
        String::from_utf8_lossy(&self.0.borrow()).into_owned()
    }
}

/// How a program's run ended.
struct Ran {
    outcome: Outcome,
    stdout: String,
    stderr: String,
}

/// Load `elf` as a boot stage leaves a program (supervisor mode, interrupts masked, the stack at the top of DDR)
/// and run it under semihosting for at most `max_ms` of emulated time.
fn run(m: &mut Machine, elf: &[u8], max_ms: u64) -> Ran {
    m.cpu.pc = load_elf(m, elf).unwrap();
    m.cpu.set_sr(0x2700);
    m.cpu.a[7] = STACK_TOP;
    m.set_time_limit(Some(m.now() + max_ms * TICKS_PER_MS));
    let (out, err) = (Sink::default(), Sink::default());
    let mut host = Semihost::new(
        Box::new(io::empty()),
        Box::new(out.clone()),
        Box::new(err.clone()),
    );
    host.clock_hz = BUS_HZ;
    let outcome = host.run(m, Limit::default());
    Ran {
        outcome,
        stdout: out.text(),
        stderr: err.text(),
    }
}

/// The program passed: it exited 0.
fn passed(r: &Ran) {
    assert_eq!(
        r.outcome,
        Outcome::Exit(0),
        "stdout:\n{}\nstderr:\n{}",
        r.stdout,
        r.stderr
    );
}

/// Port `port`'s logged (PDDR, PODR) pairs, masked to `mask`, without repeats.
fn port_states(changes: &[PinChange], port: u8, mask: u8) -> Vec<(u8, u8)> {
    let mut states: Vec<(u8, u8)> = Vec::new();
    for c in changes.iter().filter(|c| c.port == port) {
        let s = (c.pddr & mask, c.podr & mask);
        if states.last() != Some(&s) {
            states.push(s);
        }
    }
    states
}

#[test]
fn gpio_drives_reads_and_pulls() {
    let Some(elf) = elf("gpio") else { return };
    let (mut m, chip) = machine(ChipOptions::default());
    // PD0 driven high for the whole run; PC2 left to its pull
    chip.gpio_mut(&mut m).drive(3, 0, Some(true));
    let r = run(&mut m, &elf, 100);
    passed(&r);
    assert_eq!(r.stdout.lines().count(), 5, "{}", r.stdout);
    let changes = chip.gpio_mut(&mut m).take_changes();
    // PA0: high, then an output driving it, then low, then high again
    assert_eq!(
        port_states(&changes, 0, 1),
        [(0, 1), (1, 1), (1, 0), (1, 1)]
    );
    // PB0: low as it becomes an output, so no change of the latch
    assert_eq!(port_states(&changes, 1, 1), [(1, 0)]);
    // PC2: low, then an output
    assert_eq!(port_states(&changes, 2, 1 << 2), [(1 << 2, 0)]);
    assert_eq!(chip.gpio(&m).output(2, 2), Some(false));
    // PD0 stays an input
    assert_eq!(chip.gpio(&m).output(3, 0), None);
}
