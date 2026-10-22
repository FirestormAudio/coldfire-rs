//! The `mcf5441x-run` binary on small assembled ELFs: exit codes, the console and the limits.

use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Output, Stdio};
use std::sync::atomic::{AtomicUsize, Ordering};

static COUNTER: AtomicUsize = AtomicUsize::new(0);

/// The trap macro, and `block`, which fills a parameter block at A1 (pointed at 0x40100000 by `_start`).
const PRELUDE: &str = "
.macro block x, y, z
    move.l \\x,%d2
    move.l %d2,(%a1)
    move.l \\y,%d2
    move.l %d2,4(%a1)
    move.l \\z,%d2
    move.l %d2,8(%a1)
.endm
.macro semi code, arg
    move.l #\\code,%d0
    move.l \\arg,%d1
    .balignw 4,0x4e71
    nop
    halt
    .long 0x4e7bf000
.endm
.global _start
_start:
    lea 0x40100000,%a1
";

/// A scratch directory for one test.
fn scratch() -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "mcf5441x-run-{}-{}",
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Assemble `src` after the prelude and link it into an ELF at 0x40000000, with any `extra` linker arguments.
fn elf(src: &str, extra: &[&str]) -> PathBuf {
    let dir = scratch();
    let (s, o, e) = (dir.join("x.s"), dir.join("x.o"), dir.join("x.elf"));
    std::fs::write(&s, format!("{PRELUDE}\n{src}\n")).unwrap();
    let a = Command::new("m68k-elf-as")
        .args(["-mcpu=54418", "-o"])
        .arg(&o)
        .arg(&s)
        .output()
        .expect("m68k-elf-as");
    assert!(a.status.success(), "{}", String::from_utf8_lossy(&a.stderr));
    let l = Command::new("m68k-elf-ld")
        .args(["-N", "-Ttext=0x40000000", "-e", "_start"])
        .args(extra)
        .arg("-o")
        .arg(&e)
        .arg(&o)
        .output()
        .expect("m68k-elf-ld");
    assert!(l.status.success(), "{}", String::from_utf8_lossy(&l.stderr));
    e
}

fn run(args: &[&str], stdin: &[u8]) -> Output {
    let mut child = Command::new(env!("CARGO_BIN_EXE_mcf5441x-run"))
        .args(args)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    child.stdin.take().unwrap().write_all(stdin).unwrap();
    child.wait_with_output().unwrap()
}

fn run_elf(src: &str, args: &[&str]) -> Output {
    let e = elf(src, &[]);
    let mut all = args.to_vec();
    all.push(e.to_str().unwrap());
    run(&all, b"")
}

fn text(b: &[u8]) -> String {
    String::from_utf8_lossy(b).into_owned()
}

#[test]
fn exit_code_is_the_programs() {
    for n in [0, 3, 255] {
        let o = run_elf(&format!("semi 0, #{n}"), &[]);
        assert_eq!(o.status.code(), Some(n), "{}", text(&o.stderr));
        assert!(o.stdout.is_empty() && o.stderr.is_empty());
    }
}

#[test]
fn hello_on_stdout_and_stderr() {
    let o = run_elf(
        "
        lea msg,%a0
        block #1, %a0, #6
        semi 5, %a1
        lea warn,%a0
        block #2, %a0, #5
        semi 5, %a1
        semi 0, #0
    msg: .ascii \"hello\\n\"
    warn: .ascii \"warn\\n\"
        ",
        &[],
    );
    assert_eq!(o.status.code(), Some(0));
    assert_eq!(text(&o.stdout), "hello\n");
    assert_eq!(text(&o.stderr), "warn\n");
}

#[test]
fn stdin_is_echoed() {
    // read up to 64 bytes at a time into 0x40100100 and write them back, until EOF
    let e = elf(
        "
    loop:
        block #0, #0x40100100, #64
        semi 4, %a1
        move.l (%a1),%d3
        beq done
        block #1, #0x40100100, %d3
        semi 5, %a1
        bra loop
    done:
        semi 0, #0
        ",
        &[],
    );
    let input: Vec<u8> = (0..1000u32).map(|i| b'a' + (i % 26) as u8).collect();
    let o = run(&[e.to_str().unwrap()], &input);
    assert_eq!(o.status.code(), Some(0), "{}", text(&o.stderr));
    assert_eq!(o.stdout, input);
}

#[test]
fn time_limit_is_124() {
    let o = run_elf("here: bra here", &["--max-ms", "5"]);
    assert_eq!(o.status.code(), Some(124));
    assert!(
        text(&o.stderr).contains("time limit (5 ms) reached at PC 0x4000"),
        "{}",
        text(&o.stderr)
    );
}

#[test]
fn instruction_limit_is_124() {
    let o = run_elf("here: bra here", &["--max-instructions", "1000"]);
    assert_eq!(o.status.code(), Some(124));
    assert!(
        text(&o.stderr).contains("instruction limit (1000 instructions) reached"),
        "{}",
        text(&o.stderr)
    );
}

#[test]
fn illegal_is_125_with_the_pc() {
    let o = run_elf("illegal", &[]);
    assert_eq!(o.status.code(), Some(125));
    assert!(
        text(&o.stderr).contains("Illegal at PC 0x40000006"),
        "{}",
        text(&o.stderr)
    );
}

#[test]
fn a_refused_call_is_125() {
    let o = run_elf("semi 2, %a1", &[]);
    assert_eq!(o.status.code(), Some(125));
    assert!(
        text(&o.stderr).contains("semihosting call 2 (open) not supported"),
        "{}",
        text(&o.stderr)
    );
}

#[test]
fn a_plain_halt_is_125() {
    let o = run_elf("halt", &[]);
    assert_eq!(o.status.code(), Some(125));
    assert!(
        text(&o.stderr).contains("Halt(\"halt\") at PC 0x40000008"),
        "{}",
        text(&o.stderr)
    );
}

#[test]
fn no_elf_is_2() {
    let o = run(&[], b"");
    assert_eq!(o.status.code(), Some(2));
    assert!(text(&o.stderr).contains("usage:"));
    let o = run(&["/nonexistent/x.elf"], b"");
    assert_eq!(o.status.code(), Some(2));
    let dir = scratch();
    std::fs::write(dir.join("junk"), b"not an elf").unwrap();
    let o = run(&[dir.join("junk").to_str().unwrap()], b"");
    assert_eq!(o.status.code(), Some(2));
}

#[test]
fn an_unknown_flag_is_2() {
    let o = run(&["--bogus", "x.elf"], b"");
    assert_eq!(o.status.code(), Some(2));
    assert!(text(&o.stderr).contains("unknown option --bogus"));
}

#[test]
fn help_is_0() {
    let o = run(&["--help"], b"");
    assert_eq!(o.status.code(), Some(0));
    assert!(text(&o.stdout).starts_with("usage: mcf5441x-run"));
}

#[test]
fn output_before_a_fault_reaches_stdout() {
    let o = run_elf(
        "
        lea msg,%a0
        block #1, %a0, #7
        semi 5, %a1
        move.l 0x10000000,%d0
    msg: .ascii \"partial\"
        ",
        &[],
    );
    assert_eq!(o.status.code(), Some(125));
    assert_eq!(text(&o.stdout), "partial");
    assert!(text(&o.stderr).contains("Unmapped"), "{}", text(&o.stderr));
}

#[test]
fn the_stack_symbol_sets_sp() {
    // exit with the stack pointer's low byte: __stack = 0x40200044
    let src = "move.l %sp,%d1\nmove.l #0,%d0\n.balignw 4,0x4e71\nnop\nhalt\n.long 0x4e7bf000";
    let e = elf(src, &["--defsym", "__stack=0x40200044"]);
    let o = run(&[e.to_str().unwrap()], b"");
    assert_eq!(o.status.code(), Some(0x44));
    // without it, the top of DDR
    let e = elf(src, &[]);
    let o = run(&[e.to_str().unwrap()], b"");
    assert_eq!(o.status.code(), Some(0));
}
