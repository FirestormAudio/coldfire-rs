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
