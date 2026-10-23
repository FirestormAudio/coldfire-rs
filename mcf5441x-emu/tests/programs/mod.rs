//! ColdFire programs built with `toolchain/cargo-cf` and run on the chip: the build (skipped, with a message, when
//! the ColdFire toolchain is not built), the machine the programs' `memory.x` describes, and the clocks they run at.
#![allow(dead_code)]

use std::collections::HashMap;
use std::path::PathBuf;
use std::process::Command;
use std::sync::Mutex;

use mcf5441x_emu::Machine;
use mcf5441x_emu::elf::load_elf;
use mcf5441x_emu::soc::{Chip, ChipOptions, Mcf5441x};

/// DDR as the programs' memory.x has it: 128 MB at 0x40000000, the stack at its top.
pub const DDR: (u32, u32) = (0x4000_0000, 0x0800_0000);
pub const STACK_TOP: u32 = 0x4800_0000;
/// The SRAM where the programs' RAMBAR puts it.
pub const SRAM: (u32, u32) = (0x8000_0000, 0x1_0000);
/// The bus clock the HAL finds: the chip's reset PLL (FBKDIV 9, REFDIV 0, OUTDIV2 3) from the programs' 24 MHz
/// reference, a 240 MHz VCO over 4. One tick is one bus clock (the PIT counts one per tick).
pub const BUS_HZ: u64 = 60_000_000;
pub const TICKS_PER_MS: u64 = BUS_HZ / 1000;
/// 115,200 baud in ticks: ten bits a character.
pub const CHAR_TICKS: u64 = BUS_HZ * 10 / 115_200;
/// 48 kHz frames in ticks.
pub const FRAME_TICKS: u64 = BUS_HZ / 48_000;

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..")
}

/// Build coldfire-rs's crate `dir` (`hal-tests`, `examples`) once per test process into its `target/emu`, and
/// return the directory of its executables; `None` (with a message) when the ColdFire toolchain is not built.
pub fn build(dir: &str) -> Option<PathBuf> {
    static BUILT: Mutex<Option<HashMap<String, Option<PathBuf>>>> = Mutex::new(None);
    let mut built = BUILT.lock().unwrap_or_else(|e| e.into_inner());
    let out = built
        .get_or_insert_with(HashMap::new)
        .entry(dir.to_string())
        .or_insert_with(|| cargo_cf(dir))
        .clone();
    if out.is_none() {
        eprintln!("skipped: the ColdFire toolchain is not built (toolchain/setup.sh, or set RCF)");
    }
    out
}

fn cargo_cf(dir: &str) -> Option<PathBuf> {
    let root = root();
    let rcf =
        std::env::var_os("RCF").map_or_else(|| root.join("build/rust-coldfire"), PathBuf::from);
    if !rcf
        .join("rustc_codegen_gcc/target/debug/librustc_codegen_gcc.so")
        .exists()
    {
        return None;
    }
    let crate_dir = root.join(dir);
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
        "cargo-cf {dir} failed:\n{}",
        &err[err.len().saturating_sub(4000)..]
    );
    Some(crate_dir.join("target/emu/m68k-coldfire-none-elf/release"))
}

/// Executable `name` of crate `dir` as ELF bytes; `None` (with a message) when the toolchain is not built.
pub fn elf(dir: &str, name: &str) -> Option<Vec<u8>> {
    let out = build(dir)?;
    Some(std::fs::read(out.join(name)).unwrap_or_else(|e| panic!("{dir} {name} built: {e}")))
}

/// A machine with the chip (`options`), DDR and SRAM.
pub fn machine(options: ChipOptions) -> (Machine, Chip) {
    let mut m = Machine::new();
    m.mem.map_ram(DDR.0, DDR.1);
    m.mem.map_ram(SRAM.0, SRAM.1);
    let chip = Mcf5441x::build(&mut m, options);
    (m, chip)
}

/// Load `elf` as a boot stage leaves a program: supervisor mode, interrupts masked, the stack at the top of DDR,
/// PC at its entry; and stop the run `max_ms` of emulated time from now.
pub fn start(m: &mut Machine, elf: &[u8], max_ms: u64) {
    m.cpu.pc = load_elf(m, elf).unwrap();
    m.cpu.set_sr(0x2700);
    m.cpu.a[7] = STACK_TOP;
    m.set_time_limit(Some(m.now() + max_ms * TICKS_PER_MS));
}

/// An SSI line's frame `n`: two 16-bit slots.
pub fn ssi_frame(n: u64) -> Vec<i32> {
    let x = (n & 0x7FFF) as i32;
    vec![x, -x]
}

/// From the first frame `capture` sends that isn't silence, each frame is the line's ([`ssi_frame`]) a fixed
/// delay earlier plus `add` (16-bit), for `frames` frames; returns the delay.
pub fn ssi_follows(capture: &[(u64, Vec<i32>)], add: i32, frames: usize) -> usize {
    let first = capture
        .iter()
        .position(|(_, f)| f.iter().any(|&w| w != 0))
        .expect("the stream sends something");
    let x = (capture[first].1[0] as i16).wrapping_sub(add as i16) as i64;
    let delay = first - x as usize;
    assert!(
        capture.len() >= first + frames,
        "{} frames captured",
        capture.len()
    );
    for (k, (_, f)) in capture[first..first + frames].iter().enumerate() {
        let want: Vec<i16> = ssi_frame((first + k - delay) as u64)
            .iter()
            .map(|&w| (w as i16).wrapping_add(add as i16))
            .collect();
        let got: Vec<i16> = f.iter().map(|&w| w as i16).collect();
        assert_eq!(got, want, "frame {}", first + k);
    }
    delay
}
