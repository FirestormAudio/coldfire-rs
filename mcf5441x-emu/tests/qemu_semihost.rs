//! `mcf5441x-run` against QEMU's semihosting: the C programs in `fixtures/semihost`, built with `m68k-elf-gcc` for
//! the MCF5208 (a core QEMU's `mcf5208evb` and the MCF5441x share), run with the same input under both, and their
//! console output and exit status must match. Without QEMU or the compiler the checks print why and pass.
//!
//! QEMU runs as `qemu-system-m68k -M mcf5208evb -cpu any -nographic -monitor none -serial none
//! -semihosting-config enable=on,target=native,chardev=s0 -chardev stdio,id=s0,mux=off -kernel ELF` (QEMU 11.1).
//! Known differences, none of them the runner's to copy:
//! - QEMU writes fds 0-2 to one console (the chardev; stderr without one), so stdout and stderr can't be told
//!   apart there. QEMU's console is compared with the runner's stdout and stderr merged in order.
//! - QEMU's console never reports end of input: a READ there blocks. The programs that read stop at a NUL byte
//!   or after one byte, and the runner's 0 at EOF is checked in `run_cli.rs` and `semihost.rs`.
//! - QEMU's console holds 1 KiB of input and drops what arrives past it, so no program is given more.
//! - INIT_SIM in QEMU moves SP and D1 to its RAM size, which is not RAM on the mcf5208evb; `crt0.S` sets SP again.
//! - QEMU writes errno 9999 (GDB's EUNKNOWN) when a call succeeds, where the runner writes 0; `isatty.c` prints
//!   errno only for a failed call.

use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::OnceLock;
use std::time::{Duration, Instant};

const PROGRAMS: [&str; 5] = ["hello", "exits", "echo", "printf", "isatty"];

/// The directory of built programs, or why there is none.
fn built() -> &'static Result<PathBuf, String> {
    static BUILT: OnceLock<Result<PathBuf, String>> = OnceLock::new();
    BUILT.get_or_init(|| {
        for tool in ["qemu-system-m68k", "m68k-elf-gcc"] {
            if Command::new(tool).arg("--version").output().is_err() {
                return Err(format!("no {tool}"));
            }
        }
        let src = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/semihost");
        let out =
            std::env::temp_dir().join(format!("mcf5441x-qemu-semihost-{}", std::process::id()));
        std::fs::create_dir_all(&out).unwrap();
        for p in PROGRAMS {
            let o = Command::new("m68k-elf-gcc")
                .args([
                    "-mcpu=5208",
                    "-O1",
                    "-ffreestanding",
                    "-nostdlib",
                    "-fno-builtin",
                ])
                .arg("-I")
                .arg(&src)
                .arg("-T")
                .arg(src.join("link.ld"))
                .args(["crt0.S", "semi.S", "mini.c", &format!("{p}.c")].map(|f| src.join(f)))
                .arg("-o")
                .arg(out.join(format!("{p}.elf")))
                .output()
                .unwrap();
            assert!(
                o.status.success(),
                "building {p}:\n{}",
                String::from_utf8_lossy(&o.stderr)
            );
        }
        Ok(out)
    })
}

/// Run `cmd` with `stdin`, its stdout and stderr both into one file; returns that output and the exit code.
fn run(mut cmd: Command, stdin: &[u8], name: &str) -> (Vec<u8>, Option<i32>) {
    let dir = built().as_ref().unwrap();
    let path = dir.join(format!("{name}.out"));
    let file = File::create(&path).unwrap();
    let mut child = cmd
        .stdin(Stdio::piped())
        .stdout(file.try_clone().unwrap())
        .stderr(file)
        .spawn()
        .unwrap();
    child.stdin.take().unwrap().write_all(stdin).unwrap();
    let deadline = Instant::now() + Duration::from_secs(20);
    let status = loop {
        if let Some(s) = child.try_wait().unwrap() {
            break s;
        }
        if Instant::now() > deadline {
            child.kill().unwrap();
            panic!(
                "{name} did not finish:\n{}",
                String::from_utf8_lossy(&std::fs::read(&path).unwrap())
            );
        }
        std::thread::sleep(Duration::from_millis(10));
    };
    (std::fs::read(&path).unwrap(), status.code())
}

/// Run program `p` on `stdin` under QEMU and the runner, and check they agree; returns the output.
fn check(p: &str, stdin: &[u8], tag: &str) -> Option<(Vec<u8>, Option<i32>)> {
    let dir = match built() {
        Ok(d) => d,
        Err(why) => {
            println!("skipped: {why}");
            return None;
        }
    };
    let elf = dir.join(format!("{p}.elf"));
    let mut qemu = Command::new("qemu-system-m68k");
    qemu.args([
        "-M",
        "mcf5208evb",
        "-cpu",
        "any",
        "-nographic",
        "-monitor",
        "none",
        "-serial",
        "none",
    ])
    .args([
        "-semihosting-config",
        "enable=on,target=native,chardev=s0",
        "-chardev",
        "stdio,id=s0,mux=off",
    ])
    .arg("-kernel")
    .arg(&elf);
    let mut ours = Command::new(env!("CARGO_BIN_EXE_mcf5441x-run"));
    ours.arg(&elf);
    let q = run(qemu, stdin, &format!("{p}-{tag}-qemu"));
    let r = run(ours, stdin, &format!("{p}-{tag}-run"));
    assert_eq!(
        String::from_utf8_lossy(&r.0),
        String::from_utf8_lossy(&q.0),
        "{p} ({tag}): the runner's output (left) against QEMU's"
    );
    assert_eq!(r.1, q.1, "{p} ({tag}): exit codes");
    Some(r)
}

#[test]
fn hello() {
    if let Some((out, code)) = check("hello", b"", "") {
        assert_eq!(out, b"hello, world\na line on stderr\n");
        assert_eq!(code, Some(0));
    }
}

#[test]
fn exits() {
    for c in [0u8, 1, 65, 255] {
        if let Some((_, code)) = check("exits", &[c], &c.to_string()) {
            assert_eq!(code, Some(c as i32));
        }
    }
}

#[test]
fn echo() {
    // every byte value but NUL, then the NUL that ends it, within QEMU's 1 KiB of input
    let mut input: Vec<u8> = (0..1000u32).map(|i| (i % 255 + 1) as u8).collect();
    let text = input.clone();
    input.extend_from_slice(b"\0not echoed");
    if let Some((out, code)) = check("echo", &input, "") {
        assert_eq!(out, text);
        assert_eq!(code, Some(0));
    }
}

#[test]
fn printf() {
    if let Some((out, code)) = check("printf", b"", "") {
        assert_eq!(out.iter().filter(|&&b| b == b'\n').count(), 200);
        assert_eq!(code, Some(0));
    }
}

#[test]
fn isatty() {
    if let Some((out, _)) = check("isatty", b"", "") {
        assert!(String::from_utf8_lossy(&out).contains("isatty(3) = 0, errno 9"));
    }
}
