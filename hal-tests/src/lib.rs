//! What every program shares: semihosting output and exit (libgloss m68k: a 4-byte-aligned NOP, HALT and the
//! sentinel 0x4E7BF000, the call in D0 and its parameter block or argument in D1), checks that fail with the file
//! and line, and a panic handler. The host side is mcf5441x-emu's `tests/hal.rs`: a program passes by exiting 0,
//! a failed check exits 1 and a panic 101.
#![no_std]
#![feature(asm_experimental_arch)]

use core::arch::asm;
use core::fmt::{self, Write};

const EXIT: u32 = 0;
const WRITE: u32 = 5;
const GETTIMEOFDAY: u32 = 11;

/// The trap: call `op` with D1 = `arg`.
#[inline(never)]
fn trap(op: u32, arg: u32) {
    // SAFETY: the host serves the call and resumes after the sentinel; the call writes only the block D1 names
    unsafe {
        asm!(
            ".balignw 4,0x4e71",
            "nop",
            "halt",
            ".long 0x4e7bf000",
            inout("d0") op => _,
            in("d1") arg,
            options(nostack)
        );
    }
}

/// Write `bytes` to fd `fd` (1 stdout, 2 stderr).
pub fn write(fd: u32, bytes: &[u8]) {
    let block = [fd, bytes.as_ptr() as u32, bytes.len() as u32, 0];
    trap(WRITE, block.as_ptr() as u32);
}

/// End the program with `status`.
pub fn exit(status: u32) -> ! {
    trap(EXIT, status);
    loop {
        coldfire::asm::stop();
    }
}

/// The host's clock (the emulated one) in microseconds, through GETTIMEOFDAY.
pub fn gettimeofday() -> u64 {
    // GDB's timeval: a 32-bit tv_sec, then a 64-bit tv_usec
    let tv = [0u32; 3];
    let block = [tv.as_ptr() as u32, 0, 0, 0];
    trap(GETTIMEOFDAY, block.as_ptr() as u32);
    // SAFETY: the host wrote the timeval; read it back past the compiler's view of `tv`
    let [sec, hi, lo] = unsafe { core::ptr::read_volatile(&tv) };
    u64::from(sec) * 1_000_000 + (u64::from(hi) << 32 | u64::from(lo))
}

/// Formatted text on stdout ([`say!`]).
pub struct Stdout;

impl Write for Stdout {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        write(1, s.as_bytes());
        Ok(())
    }
}

/// Every check passed.
pub fn pass() -> ! {
    exit(0)
}

/// A failure: `FAIL` and `msg` on stdout, exit 1.
pub fn fail(msg: &str) -> ! {
    let _ = writeln!(Stdout, "FAIL {msg}");
    exit(1)
}

/// A line on stdout.
#[macro_export]
macro_rules! say {
    ($($t:tt)*) => {{
        use core::fmt::Write as _;
        let _ = writeln!($crate::Stdout, $($t)*);
    }};
}

/// Fail with the file and line unless `cond` holds.
#[macro_export]
macro_rules! check {
    ($c:expr) => {
        $crate::check!($c, "{}", stringify!($c))
    };
    ($c:expr, $($t:tt)*) => {
        if !$c {
            $crate::say!("FAIL {}:{}: {}", file!(), line!(), format_args!($($t)*));
            $crate::exit(1);
        }
    };
}

/// Fail with the file, the line and both values unless they are equal.
#[macro_export]
macro_rules! check_eq {
    ($a:expr, $b:expr) => {{
        let (a, b) = (&$a, &$b);
        $crate::check!(a == b, "{} == {}: {:?} != {:?}", stringify!($a), stringify!($b), a, b);
    }};
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = match info.location() {
        Some(l) => writeln!(Stdout, "PANIC {}:{}: {}", l.file(), l.line(), info.message()),
        None => writeln!(Stdout, "PANIC: {}", info.message()),
    };
    exit(101)
}
