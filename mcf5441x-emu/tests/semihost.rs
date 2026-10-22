//! libgloss m68k semihosting: the trap sequence and the calls served (`mcf5441x_emu::semihost`).

mod common;

use std::cell::RefCell;
use std::io::{self, Read, Write};
use std::rc::Rc;

use common::{BASE, MEM, STACK, assemble};
use mcf5441x_emu::semihost::{Outcome, Semihost};
use mcf5441x_emu::{Limit, Machine, Size, Stop};

/// The trap: a 4-byte-aligned NOP, HALT, then the sentinel longword; `block` fills a parameter block at A1.
const SEMI: &str = "
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
";

/// A writer the test can read back, counting flushes.
#[derive(Clone, Default)]
struct Sink {
    bytes: Rc<RefCell<Vec<u8>>>,
    flushes: Rc<RefCell<usize>>,
}

impl Write for Sink {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.bytes.borrow_mut().extend_from_slice(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        *self.flushes.borrow_mut() += 1;
        Ok(())
    }
}

impl Sink {
    fn text(&self) -> String {
        String::from_utf8(self.bytes.borrow().clone()).unwrap()
    }
}

struct Rig {
    m: Machine,
    host: Semihost,
    out: Sink,
    err: Sink,
}

/// Assemble `src` (with the `semi` macro), load it at `BASE` with RAM over `0x40100000..0x41100000`, and attach a
/// semihosting handler reading `stdin`.
fn rig(src: &str, stdin: &[u8]) -> Rig {
    let code = assemble(&format!("{SEMI}\n{src}\n"), BASE);
    let mut m = Machine::new();
    m.mem.map_ram(0x4010_0000, 0x0100_0000);
    m.mem.load(BASE, &code).unwrap();
    m.cpu.pc = BASE;
    m.cpu.a[7] = STACK;
    m.cpu.a[1] = MEM;
    let (out, err) = (Sink::default(), Sink::default());
    let input: Box<dyn Read> = Box::new(io::Cursor::new(stdin.to_vec()));
    let host = Semihost::new(input, Box::new(out.clone()), Box::new(err.clone()));
    Rig { m, host, out, err }
}

impl Rig {
    fn run(&mut self) -> Outcome {
        self.host.run(&mut self.m, Limit::default())
    }
    fn long(&mut self, addr: u32) -> u32 {
        self.m.mem.read(addr, Size::Long).unwrap()
    }
}

/// A parameter block at MEM (A1), buffers at MEM + 0x100.
const BLOCK: u32 = MEM;
const BUF: u32 = MEM + 0x100;

#[test]
fn exit_returns_its_status() {
    for code in [0i32, 1, 255, -1] {
        let mut r = rig(&format!("semi 0, #{code}"), b"");
        assert_eq!(r.run(), Outcome::Exit(code), "exit {code}");
    }
}

#[test]
fn write_to_stdout_and_stderr() {
    let mut r = rig(
        "
        lea msg,%a0
        block #1, %a0, #6
        semi 5, %a1
        move.l (%a1),%d6
        move.l 4(%a1),%d7
        block #2, %a0, #3
        semi 5, %a1
        semi 0, #0
    msg: .ascii \"hello\\nerr\"
        ",
        b"",
    );
    assert_eq!(r.run(), Outcome::Exit(0));
    assert_eq!(r.out.text(), "hello\n");
    assert_eq!(r.err.text(), "hel");
    assert_eq!((r.m.cpu.d[6], r.m.cpu.d[7]), (6, 0), "count, errno");
    assert_eq!((r.long(BLOCK), r.long(BLOCK + 4)), (3, 0));
}

#[test]
fn write_to_another_fd_is_ebadf() {
    let mut r = rig(
        "
        block #7, %a1, #1
        semi 5, %a1
        semi 0, #0
        ",
        b"",
    );
    assert_eq!(r.run(), Outcome::Exit(0));
    assert_eq!(
        (r.long(BLOCK), r.long(BLOCK + 4)),
        (u32::MAX, 9),
        "-1, EBADF"
    );
    assert!(r.out.text().is_empty() && r.err.text().is_empty());
}

#[test]
fn read_takes_stdin_and_returns_zero_at_eof() {
    let mut r = rig(
        &format!(
            "
        block #0, #{BUF}, #16
        semi 4, %a1
        move.l (%a1),%d6
        block #0, #{BUF}, #16
        semi 4, %a1
        move.l (%a1),%d7
        semi 0, #0
        "
        ),
        b"abc",
    );
    assert_eq!(r.run(), Outcome::Exit(0));
    assert_eq!(r.m.cpu.d[6], 3, "three bytes");
    assert_eq!(r.m.cpu.d[7], 0, "EOF");
    assert_eq!(r.m.mem.peek(BUF, 3).unwrap(), b"abc");
}

#[test]
fn isatty_for_stdio() {
    let mut r = rig(
        "
        move.l #1,(%a1)
        semi 12, %a1
        move.l (%a1),%d6
        move.l #5,(%a1)
        semi 12, %a1
        move.l (%a1),%d7
        semi 0, #0
        ",
        b"",
    );
    assert_eq!(r.run(), Outcome::Exit(0));
    // guess: fds 0-2 are terminals and every other fd is EBADF, as a console host reports.
    assert_eq!(r.m.cpu.d[6], 1);
    assert_eq!(r.m.cpu.d[7], 0);
    assert_eq!(r.long(BLOCK + 4), 9, "EBADF");
}

#[test]
fn gettimeofday_follows_the_emulated_clock() {
    let mut r = rig(
        &format!(
            "
        move.l #{BUF},(%a1)
        semi 11, %a1
        semi 0, #0
        "
        ),
        b"",
    );
    r.host.epoch = 1000;
    r.host.clock_hz = 1000; // one tick is a millisecond
    assert_eq!(r.run(), Outcome::Exit(0));
    let ticks = r.m.now();
    let tv = r.m.mem.peek(BUF, 12).unwrap();
    let sec = u32::from_be_bytes(tv[0..4].try_into().unwrap()) as u64;
    let usec = u64::from_be_bytes(tv[4..12].try_into().unwrap());
    assert_eq!(r.long(BLOCK), 0, "return status");
    // the timeval is read at the call, a few ticks before the run ends
    assert!(sec == 1000, "sec {sec}");
    assert!(
        usec <= ticks * 1000 && usec >= (ticks - 10) * 1000,
        "usec {usec} ticks {ticks}"
    );
}

#[test]
fn init_sim_changes_nothing() {
    let mut r = rig("semi 1, #0x41000000\nsemi 0, %d1", b"");
    assert_eq!(r.run(), Outcome::Exit(0x4100_0000));
    assert_eq!(r.m.cpu.a[7], STACK);
}

#[test]
fn open_is_refused() {
    let mut r = rig("semi 2, %a1\nsemi 0, #0", b"");
    match r.run() {
        Outcome::Refused {
            call: 2,
            name: "open",
            pc,
        } => assert!(pc > BASE),
        o => panic!("{o:?}"),
    }
}

#[test]
fn an_unknown_code_is_refused() {
    let mut r = rig("semi 99, %a1\nsemi 0, #0", b"");
    assert!(matches!(
        r.run(),
        Outcome::Refused {
            call: 99,
            name: "unknown",
            ..
        }
    ));
}

#[test]
fn a_halt_without_the_sentinel_is_a_plain_stop() {
    let mut r = rig(".balignw 4,0x4e71\nnop\nhalt\nnop\nnop\nsemi 0, #0", b"");
    match r.run() {
        Outcome::Stopped(s) => assert_eq!(s.kind, Stop::Halt("halt")),
        o => panic!("{o:?}"),
    }
}

#[test]
fn a_wrong_sentinel_is_a_plain_stop() {
    let mut r = rig(
        ".balignw 4,0x4e71\nnop\nhalt\n.long 0x4e7bf001\nsemi 0, #0",
        b"",
    );
    assert!(matches!(r.run(), Outcome::Stopped(s) if s.kind == Stop::Halt("halt")));
}

#[test]
fn a_nop_that_isnt_4_aligned_is_a_plain_stop() {
    let mut r = rig(
        ".balignw 4,0x4e71\nnop\nnop\nhalt\n.long 0x4e7bf000\nsemi 0, #0",
        b"",
    );
    assert!(matches!(r.run(), Outcome::Stopped(s) if s.kind == Stop::Halt("halt")));
}

#[test]
fn a_buffer_outside_ram_is_a_bad_pointer() {
    let mut r = rig(
        "
        block #1, #0x10000000, #4
        semi 5, %a1
        semi 0, #0
        ",
        b"",
    );
    assert!(matches!(
        r.run(),
        Outcome::BadPointer {
            call: "write",
            addr: 0x1000_0000,
            ..
        }
    ));
    assert!(r.out.text().is_empty());
}

#[test]
fn a_parameter_block_outside_ram_is_a_bad_pointer() {
    let mut r = rig("semi 5, #0x10000000\nsemi 0, #0", b"");
    assert!(matches!(
        r.run(),
        Outcome::BadPointer {
            call: "write",
            addr: 0x1000_0000,
            ..
        }
    ));
}

#[test]
fn output_is_flushed_before_a_fault() {
    let mut r = rig(
        "
        lea msg,%a0
        block #1, %a0, #2
        semi 5, %a1
        illegal
    msg: .ascii \"ok\"
        ",
        b"",
    );
    match r.run() {
        Outcome::Stopped(s) => assert_eq!(s.kind, Stop::Illegal),
        o => panic!("{o:?}"),
    }
    assert_eq!(r.out.text(), "ok");
    assert!(*r.out.flushes.borrow() >= 1 && *r.err.flushes.borrow() >= 1);
}

#[test]
fn the_instruction_limit_spans_served_calls() {
    // a loop of writes: the limit stops it, counted across the runs between calls
    let mut r = rig(
        "
        lea msg,%a0
    loop:
        block #1, %a0, #1
        semi 5, %a1
        bra loop
    msg: .ascii \"x\"
        ",
        b"",
    );
    let before = r.m.instructions();
    match r.host.run(
        &mut r.m,
        Limit {
            max_instructions: Some(100),
            ..Limit::default()
        },
    ) {
        Outcome::Stopped(s) => assert_eq!(s.kind, Stop::InstructionLimit),
        o => panic!("{o:?}"),
    }
    assert_eq!(r.m.instructions() - before, 100);
    assert!(r.out.text().len() > 5, "several writes served");
}
