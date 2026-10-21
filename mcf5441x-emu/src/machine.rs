//! A core, its memory and the run loop.

use crate::cpu::{self, Cpu, DecodeError, Flow};
use crate::mem::{Fault, Memory};
use crate::stop::{Limit, Stop, StopReason};

/// A core and its address space. One instruction is one tick.
#[derive(Debug, Clone, Default)]
pub struct Machine {
    pub cpu: Cpu,
    pub mem: Memory,
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
    /// An instruction that faults or stops does not complete: PC stays on it and it is not counted.
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
            let (d, form) = match cpu::decode(&mut self.mem, pc) {
                Ok(x) => x,
                Err(DecodeError::Fault(f)) => return self.fault(f),
                Err(DecodeError::Vector(v)) => return self.exception(v),
            };
            match cpu::exec::execute(&mut self.cpu, &mut self.mem, &d) {
                Ok(Flow::Next) => self.cpu.pc = pc.wrapping_add(d.len as u32),
                Ok(Flow::Jump) => {}
                Ok(Flow::Stop(kind)) => {
                    self.cpu.pc = pc;
                    let detail = match kind {
                        Stop::Unmodelled => form.mnemonic.to_string(),
                        _ => String::new(),
                    };
                    return self.stop(kind, detail);
                }
                Err(f) => {
                    self.cpu.pc = pc;
                    return self.fault(f);
                }
            }
            self.instructions += 1;
            self.ticks += 1;
        }
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

    /// An exception the core does not vector yet: the run stops on it.
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
