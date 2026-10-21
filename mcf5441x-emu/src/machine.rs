//! A core, its memory and devices, and the run loop.
//!
//! Time is the instruction count: one instruction is one tick, interrupt entry takes none, and STOP jumps the
//! clock to the next device event. The loop executes instructions back to back until the clock reaches
//! `Memory::horizon`: the earliest of the next device event and the time limit, or the end of any instruction
//! that touched a device (or the next instruction while a request waits on the interrupt mask). There it
//! *services* the machine: the due device events, a halt a device asked for, the time limit, and interrupts, in
//! that order; then it sets the next horizon. RAM-only code pays one comparison per instruction for all of it.
//!
//! The level-7 line is sampled at each service, which follows every instruction that could change it (a device
//! access or event), so its edges are seen per instruction, as the core samples them (RM 17.1.1). Lines changed
//! from outside the devices between runs are sampled once, when the next run starts.

use crate::cpu::exception::{stacks_next_pc, vector};
use crate::cpu::{self, Cpu, DecodeError, ExceptionPolicy, Flow};
use crate::device::{Access, Device, DeviceId};
use crate::irq::{InterruptController, Irq};
use crate::isa::Isa;
use crate::mem::{Fault, Memory};
use crate::stop::{Limit, Stop, StopReason};

/// A core, its address space and its devices. One instruction is one tick.
#[derive(Debug, Default)]
pub struct Machine {
    pub cpu: Cpu,
    pub mem: Memory,
    /// Which exceptions stop the run instead of being taken. Interrupt vectors are covered too: a request whose
    /// vector the policy stops on ends the run before entry, with PC on the instruction it would interrupt.
    pub policy: ExceptionPolicy,
    instructions: u64,
    /// Stop when the clock reaches this tick.
    time_limit: Option<u64>,
    /// After STOP: the core waits for an interrupt.
    waiting: bool,
    /// The level-7 line as last sampled.
    level7_line: bool,
    /// The level-7 line rose since a level-7 interrupt was last taken.
    level7_edge: bool,
}

impl Machine {
    /// A core in its reset state and an empty address space.
    pub fn new() -> Machine {
        Machine::default()
    }

    /// The clock, in ticks since creation.
    pub fn now(&self) -> u64 {
        self.mem.now
    }

    /// Instructions completed since creation.
    pub fn instructions(&self) -> u64 {
        self.instructions
    }

    /// Map `dev` at `base..base + size` (see [`Memory::add_device`]).
    pub fn add_device(&mut self, base: u32, size: u32, dev: Box<dyn Device>) -> DeviceId {
        self.mem.add_device(base, size, dev)
    }

    /// Map interrupt controller `c` at `base..base + size`; the core takes the requests it reports.
    pub fn add_interrupt_controller(
        &mut self,
        base: u32,
        size: u32,
        c: Box<dyn InterruptController>,
    ) -> DeviceId {
        self.mem.add_interrupt_controller(base, size, c)
    }

    /// Add a device with events and no registers.
    pub fn add_timer(&mut self, dev: Box<dyn Device>) -> DeviceId {
        self.mem.add_timer(dev)
    }

    /// Device `id`. Panics when it is not a `T`.
    pub fn device<T: Device>(&self, id: DeviceId) -> &T {
        self.mem.device(id)
    }

    /// Device `id`, mutably. Panics when it is not a `T`.
    pub fn device_mut<T: Device>(&mut self, id: DeviceId) -> &mut T {
        self.mem.device_mut(id)
    }

    /// Stop with [`Stop::TimeLimit`] when the clock reaches `ticks` (an absolute tick, not one per run); `None`
    /// removes the limit.
    pub fn set_time_limit(&mut self, ticks: Option<u64>) {
        self.time_limit = ticks;
        self.mem.horizon = 0;
    }

    /// Record every device access, or stop recording.
    pub fn trace(&mut self, on: bool) {
        self.mem.trace(on);
    }

    /// The device accesses recorded since tracing started or the trace was last taken.
    pub fn take_trace(&mut self) -> Vec<Access> {
        self.mem.take_trace()
    }

    /// Stop with [`Stop::Poll`] after more than `reads` reads of one device register by one instruction with no
    /// device write in between (default 10\_000). The count starts again with each run.
    pub fn set_poll_limit(&mut self, reads: u32) {
        self.mem.set_poll_limit(reads);
    }

    /// Execute until a limit is reached or something stops the core. `until_pc` is checked before each
    /// instruction, after any interrupt due there is taken (so a run that starts there with nothing to service
    /// stops at once); `max_instructions` counts this run's instructions.
    ///
    /// An instruction that faults or stops does not complete: PC stays on it, the registers are as they were before
    /// it (RM 3.3.4.16) and it is not counted. An exception the [`ExceptionPolicy`] stops on ends the run the same
    /// way; one it does not is taken (frame, vector, handler) and the instruction that took it counts as one.
    ///
    /// A device access completes even when the device asks to halt; the run then stops before the next
    /// instruction. Interrupts are sampled before every instruction that follows a device access or a device
    /// event, so a request raised by a write is taken before the next instruction. Changes made between runs
    /// (lines, device state, the time limit) are seen when the next run starts.
    pub fn run(&mut self, limit: Limit) -> StopReason {
        let end = limit
            .max_instructions
            .map_or(u64::MAX, |n| self.instructions.saturating_add(n));
        self.mem.stop = None;
        self.mem.reset_polls();
        self.mem.horizon = 0;
        loop {
            if self.mem.now >= self.mem.horizon {
                match self.service() {
                    Ok(false) => {}
                    // An interrupt was taken: sample again before the handler's first instruction.
                    Ok(true) => continue,
                    Err(r) => return r,
                }
            }
            let pc = self.cpu.pc;
            if limit.until_pc == Some(pc) {
                return self.stop(Stop::UntilPc, String::new());
            }
            if self.instructions >= end {
                return self.stop(Stop::InstructionLimit, String::new());
            }
            self.mem.pc = pc;
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
                    // The push stands only when the exception is taken; a run that stops on it leaves the
                    // registers as before the instruction, so it can resume (the pushed longword stays in memory).
                    if self.policy.stop_on[v as usize] {
                        self.cpu.a = saved;
                    }
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
                    self.waiting = true;
                    self.mem.horizon = 0;
                    continue;
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

    /// At an instruction boundary: advance the devices whose events are due, report a halt a device asked for,
    /// stop at the time limit, and take the highest deliverable interrupt. While the core waits after STOP, the
    /// clock jumps from event to event until an interrupt is taken. `Ok(true)` when an interrupt was taken.
    fn service(&mut self) -> Result<bool, StopReason> {
        loop {
            self.mem.advance_due();
            if let Some(r) = self.mem.stop.take() {
                return Err(self.stop(r.kind, r.detail));
            }
            if let Some(limit) = self.time_limit
                && self.mem.now >= limit
            {
                return Err(self.stop(Stop::TimeLimit, format!("{} ticks", self.mem.now)));
            }
            let (irq, held) = self.deliverable();
            if let Some(irq) = irq {
                if self.policy.stop_on[irq.vector as usize] {
                    return Err(self.exception(irq.vector));
                }
                if let Err(f) =
                    self.cpu
                        .interrupt(&mut self.mem, irq.vector, irq.level, self.cpu.pc)
                {
                    return Err(self.fault(f));
                }
                if irq.level == 7 {
                    // Taken: the line has to fall and rise again for another.
                    self.level7_edge = false;
                }
                self.waiting = false;
                return Ok(true);
            }
            let event = self.mem.next_event();
            if !self.waiting {
                let now = self.mem.now;
                let mut h = event.unwrap_or(u64::MAX);
                if let Some(limit) = self.time_limit {
                    h = h.min(limit);
                }
                if held {
                    // A request waits on SR[I]: any instruction may lower it.
                    h = now + 1;
                }
                self.mem.horizon = h.max(now + 1);
                return Ok(false);
            }
            // STOP: nothing to take yet, so jump the clock to the next event (or the time limit before it).
            match event {
                Some(t) if self.mem.has_controllers() => {
                    let t = t.max(self.mem.now);
                    self.mem.now = self.time_limit.map_or(t, |limit| t.min(limit));
                }
                _ => {
                    return Err(self.stop(
                        Stop::HaltedForever,
                        format!(
                            "waiting with SR {:#06x} and nothing able to interrupt",
                            self.cpu.sr()
                        ),
                    ));
                }
            }
        }
    }

    /// The request the core takes now, if any, and whether another request is held off by SR\[I\]. Samples the
    /// level-7 line for its rising edge.
    fn deliverable(&mut self) -> (Option<Irq>, bool) {
        let line = self.mem.level7();
        if line != self.level7_line {
            self.level7_line = line;
            self.level7_edge = line;
        }
        let Some(irq) = self.mem.pending(self.level7_edge) else {
            return (None, false);
        };
        let ipl = (self.cpu.sr() >> 8) as u8 & 7;
        if irq.level == 7 || irq.level > ipl {
            (Some(irq), false)
        } else {
            (None, true)
        }
    }

    /// One instruction completed.
    #[inline(always)]
    fn tick(&mut self) {
        self.instructions += 1;
        self.mem.now += 1;
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
