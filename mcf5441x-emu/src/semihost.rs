//! Semihosting with libgloss's m68k protocol (`libgloss/m68k/m68k-semi.txt`), the one QEMU serves: a program
//! reaches the host through a trap, a `HALT` after a 4-byte-aligned `NOP` and before the sentinel longword
//! `0x4E7BF000`. D0 holds the call number; D1 holds the address of the call's parameter block, or for `exit` and
//! `init_sim` the argument itself. A call returns its result in the block's first longword and errno (GDB's
//! File-I/O values) in the second.
//!
//! [`Semihost::run`] runs a [`Machine`] and serves the calls as their traps stop it. The console calls are served:
//! `exit`, `write` to fds 1 and 2, `read` from fd 0, `isatty` and `gettimeofday`, whose time comes from the
//! emulated clock so that a run does not depend on the host. `init_sim` changes nothing. Every file or system call
//! ends the run as [`Outcome::Refused`].

use std::io::{Read, Write};

use crate::machine::Machine;
use crate::stop::{Limit, Stop, StopReason};

/// The trap: `NOP`, `HALT`, then the sentinel.
const TRAP: [u8; 8] = [0x4E, 0x71, 0x4A, 0xC8, 0x4E, 0x7B, 0xF0, 0x00];

// GDB File-I/O errno values.
const EBADF: u32 = 9;
const EINVAL: u32 = 22;

/// The clock [`Semihost::new`] assumes: 125 MHz, the MCF5441x's top bus clock (a 250 MHz core, RM ch. 8), whose
/// ticks the machine counts.
pub const DEFAULT_CLOCK_HZ: u64 = 125_000_000;

/// How a [`Semihost::run`] ended.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Outcome {
    /// The program called `exit` with this status.
    Exit(i32),
    /// The machine stopped for anything but a trap (a limit, a fault, a plain `HALT`, ...).
    Stopped(StopReason),
    /// The program made a call that is not served; `pc` is past its `HALT`.
    Refused {
        call: u32,
        name: &'static str,
        pc: u32,
    },
    /// A call's parameter block or buffer at `addr` is not in mapped RAM; the host was not touched.
    BadPointer {
        call: &'static str,
        addr: u32,
        pc: u32,
    },
}

/// The host side: the program's console and its clock.
pub struct Semihost {
    pub stdin: Box<dyn Read>,
    pub stdout: Box<dyn Write>,
    pub stderr: Box<dyn Write>,
    /// `gettimeofday` at tick 0, in seconds.
    pub epoch: u64,
    /// The machine's ticks per second, for `gettimeofday`.
    pub clock_hz: u64,
}

/// Whether the run stopped just past the `HALT` of a trap: `pc_after_halt` is the stop's PC.
pub fn is_trap(m: &mut Machine, pc_after_halt: u32) -> bool {
    let nop = pc_after_halt.wrapping_sub(4);
    nop & 3 == 0 && m.mem.peek(nop, 8).is_some_and(|w| w == TRAP)
}

/// A call's name, for refusals and bad pointers.
fn name(call: u32) -> &'static str {
    match call {
        0 => "exit",
        1 => "init_sim",
        2 => "open",
        3 => "close",
        4 => "read",
        5 => "write",
        6 => "lseek",
        7 => "rename",
        8 => "unlink",
        9 => "stat",
        10 => "fstat",
        11 => "gettimeofday",
        12 => "isatty",
        13 => "system",
        _ => "unknown",
    }
}

/// What serving a call left to do.
enum Served {
    Continue,
    End(Outcome),
}

impl Semihost {
    /// A host on these streams, with the epoch at 0 and the clock at [`DEFAULT_CLOCK_HZ`].
    pub fn new(stdin: Box<dyn Read>, stdout: Box<dyn Write>, stderr: Box<dyn Write>) -> Semihost {
        Semihost {
            stdin,
            stdout,
            stderr,
            epoch: 0,
            clock_hz: DEFAULT_CLOCK_HZ,
        }
    }

    /// Run `m` within `limit`, serving every trap, until the program exits or the machine stops otherwise.
    /// `limit.max_instructions` counts the instructions of the whole call, across the traps served. stdout and
    /// stderr are flushed before it returns.
    pub fn run(&mut self, m: &mut Machine, limit: Limit) -> Outcome {
        let start = m.instructions();
        let outcome = loop {
            let left = limit
                .max_instructions
                .map(|n| n.saturating_sub(m.instructions() - start));
            let stop = m.run(Limit {
                max_instructions: left,
                ..limit
            });
            if stop.kind != Stop::Halt("halt") || !is_trap(m, stop.pc) {
                break Outcome::Stopped(stop);
            }
            match self.serve(m, stop.pc) {
                Served::Continue => m.cpu.pc = stop.pc + 4,
                Served::End(o) => break o,
            }
        };
        // Nothing is left to report a flush failure to: the outcome stands.
        let _ = self.stdout.flush();
        let _ = self.stderr.flush();
        outcome
    }

    /// Serve the call of the trap whose `HALT` ends at `pc`.
    fn serve(&mut self, m: &mut Machine, pc: u32) -> Served {
        let (call, d1) = (m.cpu.d[0], m.cpu.d[1]);
        let what = name(call);
        // The block's longwords this call reads; every block holds at least the result and errno.
        let args = match call {
            0 => return Served::End(Outcome::Exit(d1 as i32)),
            1 => return Served::Continue,
            4 | 5 => 3,
            11 => 2,
            12 => 1,
            _ => {
                return Served::End(Outcome::Refused {
                    call,
                    name: what,
                    pc,
                });
            }
        };
        let bad = |addr| {
            Served::End(Outcome::BadPointer {
                call: what,
                addr,
                pc,
            })
        };
        let Some(block) = m.mem.peek(d1, args.max(2) * 4) else {
            return bad(d1);
        };
        let arg = |i: usize| u32::from_be_bytes(block[i * 4..i * 4 + 4].try_into().unwrap());
        let (result, errno) = match call {
            4 => {
                let (fd, buf, len) = (arg(0), arg(1), arg(2));
                if m.mem.peek(buf, len).is_none() {
                    return bad(buf);
                }
                if fd != 0 {
                    (u32::MAX, EBADF)
                } else {
                    let mut bytes = vec![0; len as usize];
                    match self.stdin.read(&mut bytes) {
                        Ok(n) => {
                            m.mem.load(buf, &bytes[..n]).unwrap();
                            (n as u32, 0)
                        }
                        // guess: a host error is reported as EINVAL, the catch-all GDB File-I/O offers.
                        Err(_) => (u32::MAX, EINVAL),
                    }
                }
            }
            5 => {
                let (fd, buf, len) = (arg(0), arg(1), arg(2));
                let Some(bytes) = m.mem.peek(buf, len) else {
                    return bad(buf);
                };
                let out = match fd {
                    1 => &mut self.stdout,
                    2 => &mut self.stderr,
                    _ => return self.reply(m, d1, u32::MAX, EBADF),
                };
                match out.write_all(&bytes) {
                    Ok(()) => (len, 0),
                    // guess: as for read.
                    Err(_) => (u32::MAX, EINVAL),
                }
            }
            11 => {
                let tv = arg(0);
                if m.mem.peek(tv, 12).is_none() {
                    return bad(tv);
                }
                // GDB's timeval: a 32-bit tv_sec, then a 64-bit tv_usec. The timezone is ignored.
                let (ticks, hz) = (m.now() as u128, self.clock_hz.max(1) as u128);
                let sec = self.epoch.wrapping_add((ticks / hz) as u64) as u32;
                let usec = ((ticks % hz) * 1_000_000 / hz) as u64;
                let mut val = sec.to_be_bytes().to_vec();
                val.extend_from_slice(&usec.to_be_bytes());
                m.mem.load(tv, &val).unwrap();
                (0, 0)
            }
            // fds 0-2 are terminals and every other fd is EBADF, as QEMU reports (tests/qemu_semihost.rs).
            12 if arg(0) <= 2 => (1, 0),
            12 => (0, EBADF),
            _ => unreachable!(),
        };
        self.reply(m, d1, result, errno)
    }

    /// Write a call's result and errno into its block at `block` (already checked to be in RAM).
    fn reply(&mut self, m: &mut Machine, block: u32, result: u32, errno: u32) -> Served {
        let mut val = result.to_be_bytes().to_vec();
        val.extend_from_slice(&errno.to_be_bytes());
        m.mem.load(block, &val).unwrap();
        Served::Continue
    }
}
