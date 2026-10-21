#![allow(dead_code)]
use std::path::PathBuf;
use std::process::Command;
use std::sync::atomic::{AtomicUsize, Ordering};

static COUNTER: AtomicUsize = AtomicUsize::new(0);

/// Assemble `src` for the MCF54418 and link it at `base`; returns the flat binary.
pub fn assemble(src: &str, base: u32) -> Vec<u8> {
    assemble_cpu(src, base, "54418")
}

/// Assemble `src` with `m68k-elf-as -mcpu=<cpu>` and link it at `base`; returns the flat binary.
pub fn assemble_cpu(src: &str, base: u32, cpu: &str) -> Vec<u8> {
    let dir: PathBuf = std::env::temp_dir().join(format!(
        "mcf5441x-emu-{}-{}",
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::create_dir_all(&dir).unwrap();
    let result = (|| {
        let (s, o, b) = (dir.join("x.s"), dir.join("x.o"), dir.join("x.bin"));
        std::fs::write(&s, src).unwrap();
        let a = Command::new("m68k-elf-as")
            .arg(format!("-mcpu={cpu}"))
            .arg("-o")
            .arg(&o)
            .arg(&s)
            .output()
            .expect("m68k-elf-as");
        if !a.status.success() {
            return Err(String::from_utf8_lossy(&a.stderr).into_owned());
        }
        let l = Command::new("m68k-elf-ld")
            .arg(format!("-Ttext={base:#x}"))
            .args(["--oformat", "binary", "-o"])
            .arg(&b)
            .arg(&o)
            .output()
            .expect("m68k-elf-ld");
        if !l.status.success() {
            return Err(String::from_utf8_lossy(&l.stderr).into_owned());
        }
        Ok(std::fs::read(&b).unwrap())
    })();
    let _ = std::fs::remove_dir_all(&dir);
    result.unwrap_or_else(|e| panic!("assembler failed:\n{e}"))
}

pub const BASE: u32 = 0x4010_0000;
pub const STACK: u32 = 0x4100_0000;
/// A scratch longword in RAM; A1 points at it in [`run`].
pub const MEM: u32 = 0x4020_0000;

pub const X: u8 = 0x10;
pub const N: u8 = 0x08;
pub const Z: u8 = 0x04;
pub const V: u8 = 0x02;
pub const C: u8 = 0x01;

/// Assemble `src`, load it at `BASE` with RAM at `0x40100000..0x41100000` and A7 = `STACK`, call `setup` and run
/// to the end of the text or to the first stop. Returns the machine and why it stopped.
pub fn run(
    src: &str,
    setup: impl FnOnce(&mut mcf5441x_emu::Machine),
) -> (mcf5441x_emu::Machine, mcf5441x_emu::StopReason) {
    use mcf5441x_emu::{Limit, Machine};
    let code = assemble(&format!("{src}\n.balign 2\nend:\n.long end\n"), BASE);
    let (code, tail) = code.split_at(code.len() - 4);
    let end = u32::from_be_bytes(tail.try_into().unwrap());
    let mut m = Machine::new();
    m.mem.map_ram(0x4010_0000, 0x0100_0000);
    m.mem.load(BASE, code).unwrap();
    m.cpu.pc = BASE;
    m.cpu.a[7] = STACK;
    m.cpu.a[1] = MEM;
    setup(&mut m);
    let r = m.run(Limit {
        until_pc: Some(end),
        ..Limit::default()
    });
    (m, r)
}

/// As [`run`], and the run must reach the end of the text.
pub fn run_ok(src: &str, setup: impl FnOnce(&mut mcf5441x_emu::Machine)) -> mcf5441x_emu::Machine {
    let (m, r) = run(src, setup);
    assert_eq!(r.kind, mcf5441x_emu::Stop::UntilPc, "{src}: {r:?}");
    m
}

/// The vector table [`prepare`] sets up.
pub const VBR: u32 = 0x4020_0000;

/// Assemble `src` at `BASE` into a machine with RAM at `0x40100000..0x41100000`, A7 = `STACK` (supervisor mode, the
/// reset SR 0x2700) and VBR = [`VBR`]; returns it and the addresses of `labels`.
pub fn prepare(src: &str, labels: &[&str]) -> (mcf5441x_emu::Machine, Vec<u32>) {
    let tail: String = labels.iter().map(|l| format!(".long {l}\n")).collect();
    let code = assemble(&format!("{src}\n.balign 2\n{tail}"), BASE);
    let (code, tail) = code.split_at(code.len() - 4 * labels.len());
    let addrs = tail
        .chunks(4)
        .map(|c| u32::from_be_bytes(c.try_into().unwrap()))
        .collect();
    let mut m = mcf5441x_emu::Machine::new();
    m.mem.map_ram(0x4010_0000, 0x0100_0000);
    m.mem.load(BASE, code).unwrap();
    m.cpu.pc = BASE;
    m.cpu.a[7] = STACK;
    m.cpu.vbr = VBR;
    (m, addrs)
}

/// A run bounded only by `pc`.
pub fn until(pc: u32) -> mcf5441x_emu::Limit {
    mcf5441x_emu::Limit {
        until_pc: Some(pc),
        ..Default::default()
    }
}

/// Point vector `v` of the table at [`VBR`] to `handler`.
pub fn set_vector(m: &mut mcf5441x_emu::Machine, v: u32, handler: u32) {
    m.mem
        .write(VBR + 4 * v, mcf5441x_emu::Size::Long, handler)
        .unwrap();
}
