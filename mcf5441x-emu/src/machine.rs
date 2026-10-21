//! A core, its memory and the run loop.

use crate::cpu::exception::{stacks_next_pc, vector};
use crate::cpu::{self, Cpu, DecodeError, ExceptionPolicy, Flow};
use crate::isa::Isa;
use crate::mem::{Fault, Memory};
use crate::stop::{Limit, Stop, StopReason};

/// A core and its address space. One instruction is one tick.
#[derive(Debug, Clone, Default)]
pub struct Machine {
    pub cpu: Cpu,
    pub mem: Memory,
    /// Which exceptions stop the run instead of being taken.
    pub policy: ExceptionPolicy,
    ticks: u64,
    instructions: u64,
}

impl Machine {
    /// A core in its reset state and an empty address space.
    pub fn new() -> Machine {
        Machine::default()
    }

    /// The clock, in ticks since creation.
    pub fn now(&self) -> u64 {
        self.ticks
    }

    /// Instructions completed since creation.
    pub fn instructions(&self) -> u64 {
        self.instructions
    }

    /// Execute until a limit is reached or something stops the core. `until_pc` is checked before each
    /// instruction (so a run that starts there stops at once); `max_instructions` counts this run's instructions.
    ///
    /// An instruction that faults or stops does not complete: PC stays on it, the registers are as they were before
    /// it (RM 3.3.4.16) and it is not counted. An exception the [`ExceptionPolicy`] stops on ends the run the same
    /// way; one it does not is taken (frame, vector, handler) and the instruction that took it counts as one.
    pub fn run(&mut self, limit: Limit) -> StopReason {
        let end = limit
            .max_instructions
            .map(|n| self.instructions.saturating_add(n));
        loop {
            let pc = self.cpu.pc;
            if limit.until_pc == Some(pc) {
                return self.stop(Stop::UntilPc, String::new());
            }
            if end.is_some_and(|end| self.instructions >= end) {
                return self.stop(Stop::InstructionLimit, String::new());
            }
            if pc & 1 != 0 {
                // guess: jumps, branches and returns check their own targets (RM 3.3.4.2); a PC that is odd
                // anyway (loaded by RTE, or set from outside) faults at the fetch, with itself as the stacked PC.
                if let Some(r) = self.take(vector::ADDRESS_ERROR, pc) {
                    return r;
                }
                self.tick();
                continue;
            }
            let traced = self.cpu.tracing();
            let saved = self.cpu.a;
            let (d, form) = match cpu::decode(&mut self.mem, pc) {
                Ok(x) => x,
                Err(DecodeError::Fault(f)) => return self.fault(f),
                Err(DecodeError::Vector(v)) => {
                    if let Some(r) = self.take(v, pc) {
                        return r;
                    }
                    self.tick();
                    continue;
                }
            };
            let next = pc.wrapping_add(d.len as u32);
            match cpu::exec::execute(&mut self.cpu, &mut self.mem, &d) {
                Ok(Flow::Next) => self.cpu.pc = next,
                Ok(Flow::Jump) => {}
                Ok(Flow::Exception(v)) => {
                    self.cpu.a = saved;
                    self.cpu.pc = pc;
                    if let Some(r) = self.take(v, if stacks_next_pc(v) { next } else { pc }) {
                        return r;
                    }
                    self.tick();
                    continue;
                }
                Ok(Flow::ExceptionAfterPush(v)) => {
                    self.cpu.pc = pc;
                    if let Some(r) = self.take(v, pc) {
                        return r;
                    }
                    self.tick();
                    continue;
                }
                Ok(Flow::Wait) => {
                    self.cpu.pc = next;
                    self.tick();
                    // RM 3.3.4.6: a STOP that starts traced, or whose immediate sets T, takes the trace exception
                    // with PC after it instead of waiting.
                    if traced || self.cpu.tracing() {
                        if let Some(r) = self.take(vector::TRACE, next) {
                            return r;
                        }
                        continue;
                    }
                    return self.stop(Stop::HaltedForever, String::new());
                }
                Ok(Flow::StopAfter(kind)) => {
                    self.cpu.pc = next;
                    self.tick();
                    return self.stop(kind, String::new());
                }
                Ok(Flow::Stop(kind)) => {
                    self.cpu.a = saved;
                    self.cpu.pc = pc;
                    let detail = match kind {
                        Stop::Unmodelled if form.isa == Isa::Cau => "CAU".to_string(),
                        Stop::Unmodelled => form.mnemonic.to_string(),
                        _ => String::new(),
                    };
                    return self.stop(kind, detail);
                }
                Err(f) => {
                    self.cpu.a = saved;
                    self.cpu.pc = pc;
                    return self.fault(f);
                }
            }
            self.tick();
            // RM 3.3.4.6: with T set when it began, a completed instruction is followed by a trace exception.
            if traced && let Some(r) = self.take(vector::TRACE, self.cpu.pc) {
                return r;
            }
        }
    }

    /// One instruction completed.
    #[inline(always)]
    fn tick(&mut self) {
        self.instructions += 1;
        self.ticks += 1;
    }

    /// Exception `v` with `return_pc` for the frame: stop on it if the policy says so (PC is on the instruction
    /// that took it, or past it for trace), else take it. `Some` ends the run.
    fn take(&mut self, v: u8, return_pc: u32) -> Option<StopReason> {
        if self.policy.stop_on[v as usize] {
            return Some(self.exception(v));
        }
        self.cpu
            .raise(&mut self.mem, v, return_pc)
            .err()
            .map(|f| self.fault(f))
    }

    fn stop(&self, kind: Stop, detail: String) -> StopReason {
        StopReason {
            kind,
            pc: self.cpu.pc,
            detail,
        }
    }

    fn fault(&self, f: Fault) -> StopReason {
        match f {
            Fault::Unmapped { addr, size } => self.stop(
                Stop::Unmapped,
                format!("unmapped {size:?} access at {addr:#010x}"),
            ),
            Fault::Device(kind) => self.stop(kind, String::new()),
        }
    }

    /// The run stops on exception `vector`.
    fn exception(&self, vector: u8) -> StopReason {
        let opword = self
            .mem
            .peek(self.cpu.pc, 2)
            .map(|b| u16::from_be_bytes([b[0], b[1]]));
        let detail = opword.map_or(String::new(), |w| format!("opword {w:#06x}"));
        match vector {
            4 => self.stop(Stop::Illegal, detail),
            v => self.stop(Stop::Exception(v), detail),
        }
    }
}
