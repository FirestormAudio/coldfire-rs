//! A core, its memory and devices, and the run loop.
//!
//! Time is the instruction count: one instruction is one tick, interrupt entry takes none, and STOP jumps the
//! clock to the next device event. The loop executes instructions back to back until the clock reaches
//! `Memory::horizon`: the earliest of the next device event, the time limit and the instruction limit, or the end
//! of any instruction that touched a device, wrote SR (a request held off by the interrupt mask may now be taken)
//! or wrote over decoded code. There it drops the blocks over written code and *services* the machine: the due
//! device events, a halt a device asked for, the time limit, and interrupts, in that order; then it sets the next
//! horizon. RAM-only code pays one comparison per instruction for all of it. Bus masters are serviced there too,
//! after the device events (see [`crate::bus`]).
//!
//! Instructions run from decoded blocks: straight runs decoded once, ending at a branch, a jump, a write to SR or a
//! page boundary. Execution enters a block only at its start, so a run that stops in the middle of one (at the
//! instruction limit, `until_pc` or the horizon) resumes in a block that starts there.
//!
//! The level-7 line is sampled at each service, which follows every instruction that could change it (a device
//! access or event), so its edges are seen per instruction, as the core samples them (RM 17.1.1). Lines changed
//! from outside the devices between runs are sampled once, when the next run starts.

use crate::bus::DmaLines;
use crate::cpu::block::Blocks;
use crate::cpu::exception::{
    ADDRESS_ERROR_FETCH_FS, ADDRESS_ERROR_OPERAND_FS, stacks_next_pc, vector,
};
use crate::cpu::{self, Cpu, DecodeError, ExceptionPolicy, Flow};
use crate::device::{Access, Device, DeviceCx, DeviceId};
use crate::irq::{InterruptController, Irq};
use crate::isa::Isa;
use crate::mem::{Fault, Memory, Size};
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
    /// Decoded blocks.
    blocks: Blocks,
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

    /// Map bus master `dev` at `base..base + size` (see [`Memory::add_bus_master`] and [`crate::bus`]).
    pub fn add_bus_master(&mut self, base: u32, size: u32, dev: Box<dyn Device>) -> DeviceId {
        self.mem.add_bus_master(base, size, dev)
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

    /// Call `f` on device `id` with a full context at the current tick (with a [`Bus`](crate::bus::Bus) when it
    /// is a bus master), then service the bus masters. Panics when it is not a `T`. A halt asked for here (or by
    /// the servicing) stays pending and is reported by the next run, before its first instruction.
    pub fn with_device<T: Device, R>(
        &mut self,
        id: DeviceId,
        f: impl FnOnce(&mut T, &mut DeviceCx) -> R,
    ) -> R {
        let r = self.mem.with_device(id, f);
        self.mem.service_masters();
        r
    }

    /// Make an access as the core would at the current tick, by the instruction at PC: RAM, or a device access
    /// that is traced but not counted by the poll detector (a misaligned one split as the core splits it, RM
    /// Table 3-12); then service the bus masters. Returns the value read, or 0 for a write (`write` holds the
    /// value to write). A halt a device asks for stays pending and is reported by the next run, before its first
    /// instruction. An access a device refuses ([`DeviceCx::refuse`]) returns `Err(Fault::Device(kind))` and
    /// leaves nothing pending: there is no instruction to stop on. Its kind and detail are kept for
    /// [`Machine::take_refusal`].
    pub fn access(&mut self, addr: u32, size: Size, write: Option<u32>) -> Result<u32, Fault> {
        self.mem.pc = self.cpu.pc;
        let r = self.mem.host_access(addr, size, write);
        self.mem.service_masters();
        r
    }

    /// The kind and detail of the last access [`Machine::access`] had refused, cleared by this call; `None` when
    /// there is none. `pc` is the PC at the time.
    pub fn take_refusal(&mut self) -> Option<StopReason> {
        self.mem.host_refusal.take()
    }

    /// Move the clock to `tick` without executing instructions or taking interrupts: advance the devices at each
    /// of their events up to and including `tick`, in order, servicing the bus masters after each. `None` when the
    /// clock reached `tick`. When a device halts (or its event is not moved past the clock), the clock stays at that
    /// event and the halt is returned, not reported again. A halt still pending from [`Machine::access`] or
    /// [`Machine::with_device`] is left for the next run. Panics when `tick` is before the clock.
    pub fn idle_until(&mut self, tick: u64) -> Option<StopReason> {
        assert!(
            tick >= self.mem.now,
            "idle_until({tick}) is before the clock ({})",
            self.mem.now
        );
        self.mem.horizon = 0;
        let pending = self.mem.stop.take();
        let r = self.idle(tick);
        self.mem.stop = pending;
        r
    }

    /// [`Machine::idle_until`] with no halt pending.
    fn idle(&mut self, tick: u64) -> Option<StopReason> {
        loop {
            self.mem.advance_due();
            self.mem.service_masters();
            if let Some(r) = self.mem.stop.take() {
                return Some(self.stop(r.kind, r.detail));
            }
            match self.mem.next_event() {
                Some(t) if t <= self.mem.now => {
                    return Some(self.stop(
                        Stop::Halt("device event not advanced"),
                        format!(
                            "a device's next event stays at tick {t}, not after the clock ({}); the device must \
                             move its event forward in `advance`",
                            self.mem.now
                        ),
                    ));
                }
                Some(t) if t <= tick => self.mem.now = t,
                _ => break,
            }
        }
        self.mem.now = tick;
        None
    }

    /// The DMA request lines.
    pub fn dma_lines(&self) -> &DmaLines {
        self.mem.dma_lines()
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
    /// instruction. Interrupts are sampled before every instruction that follows a device access, a device
    /// event or a write to SR, so a request raised by a write is taken before the next instruction. Changes made
    /// between runs (lines, device state, the time limit) are seen when the next run starts.
    ///
    /// Instructions are decoded once into blocks and run from there; a write over decoded code is seen by the
    /// next instruction fetched, as if every instruction were decoded where it is executed.
    pub fn run(&mut self, limit: Limit) -> StopReason {
        let r = self.run_cached(limit);
        self.end();
        r
    }

    fn run_cached(&mut self, limit: Limit) -> StopReason {
        let end = self.begin(limit);
        loop {
            if self.mem.now >= self.mem.horizon {
                match self.boundary() {
                    Ok(false) => {}
                    // An interrupt was taken: sample again before the handler's first instruction.
                    Ok(true) => continue,
                    Err(r) => return r,
                }
                // Blocks look only at the horizon, so it covers the instruction limit too.
                let left = end.saturating_sub(self.instructions);
                self.mem.horizon = self.mem.horizon.min(self.mem.now.saturating_add(left));
            }
            let pc = self.cpu.pc;
            if limit.until_pc == Some(pc) {
                return self.stop(Stop::UntilPc, String::new());
            }
            if self.instructions >= end {
                return self.stop(Stop::InstructionLimit, String::new());
            }
            let block = if pc & 1 == 0 && !self.cpu.tracing() {
                self.blocks.get(&mut self.mem, pc)
            } else {
                None
            };
            let r = match block {
                Some(slot) => self.run_block(slot, limit.until_pc),
                None => self.step(pc),
            };
            if let Some(r) = r {
                return r;
            }
        }
    }

    /// [`Machine::run`] decoding every instruction where it is executed, without the block cache: the reference
    /// the cache is checked against.
    #[doc(hidden)]
    pub fn run_uncached(&mut self, limit: Limit) -> StopReason {
        let r = self.run_each(limit);
        self.end();
        r
    }

    fn run_each(&mut self, limit: Limit) -> StopReason {
        let end = self.begin(limit);
        loop {
            if self.mem.now >= self.mem.horizon {
                match self.boundary() {
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
            if let Some(r) = self.step(pc) {
                return r;
            }
        }
    }

    /// Count block entries from now on, starting from zero (`true`), or stop counting (`false`, the default).
    pub fn profile(&mut self, on: bool) {
        self.blocks.profile(on);
    }

    /// How many times execution entered each block since profiling started or the profile was last taken, by the
    /// block's start address, sorted by it; empty when not profiling. A block is a straight run of instructions
    /// ending at a branch, a jump, a write to SR or a 4 KiB page boundary. Instructions run one at a time (while
    /// tracing, or fetched from outside RAM) are not counted.
    pub fn take_profile(&mut self) -> Vec<(u32, u64)> {
        self.blocks.take_profile()
    }

    /// Start a run: the instruction count to stop at. A halt asked for between runs is kept, for the first
    /// boundary to report.
    fn begin(&mut self, limit: Limit) -> u64 {
        self.mem.reset_polls();
        self.mem.refused = None;
        self.mem.horizon = 0;
        limit
            .max_instructions
            .map_or(u64::MAX, |n| self.instructions.saturating_add(n))
    }

    /// End a run: a halt asked for by an access that then faulted is dropped.
    fn end(&mut self) {
        self.mem.stop = None;
    }

    /// At the horizon: drop the blocks over written code, then service the machine, then drop the blocks over
    /// code that bus masters wrote while serviced.
    fn boundary(&mut self) -> Result<bool, StopReason> {
        self.drop_written();
        let r = self.service();
        self.drop_written();
        r
    }

    /// Drop the blocks over written code.
    fn drop_written(&mut self) {
        while let Some(page) = self.mem.space.written.pop() {
            self.blocks.drop_page(page);
        }
    }

    /// Execute the block in `slot` from its start, up to the horizon, `until_pc` or the first instruction that
    /// does not fall through to the next. `Some` ends the run.
    fn run_block(&mut self, slot: u32, until_pc: Option<u32>) -> Option<StopReason> {
        let block = self.blocks.block_mut(slot);
        block.count += 1;
        let mut n = block.insns.len();
        if let Some(u) = until_pc
            && u.wrapping_sub(block.start) < block.end.wrapping_sub(block.start)
        {
            n = block.index_of(u);
        }
        let (cpu, mem) = (&mut self.cpu, &mut self.mem);
        let mut pc = block.start;
        let mut done = 0;
        let mut other = None;
        for d in &block.insns[..n] {
            mem.pc = pc;
            let saved = cpu.a;
            match cpu::exec::execute(cpu, mem, d) {
                Ok(Flow::Next) => {
                    pc = pc.wrapping_add(d.len as u32);
                    cpu.pc = pc;
                    done += 1;
                    mem.now += 1;
                    // A device access, a write to SR or to decoded code, an event or the instruction limit.
                    if mem.now >= mem.horizon {
                        break;
                    }
                }
                Ok(Flow::Jump) => {
                    done += 1;
                    mem.now += 1;
                    break;
                }
                r => {
                    other = Some((r, pc, d.len, saved));
                    break;
                }
            }
        }
        self.instructions += done;
        let (r, pc, len, saved) = other?;
        self.finish(r, pc, len, saved, false)
    }

    /// Decode and execute the instruction at `pc` on its own. `Some` ends the run.
    fn step(&mut self, pc: u32) -> Option<StopReason> {
        self.mem.pc = pc;
        if pc & 1 != 0 {
            // guess: jumps, branches and returns check their own targets (RM 3.3.4.2); a PC that is odd
            // anyway (loaded by RTE, or set from outside) faults at the fetch, with itself as the stacked PC and FS
            // 0100 (by analogy with the odd RTS measured on an MCF54415).
            if let Some(r) = self.take(vector::ADDRESS_ERROR, pc) {
                return Some(r);
            }
            self.tick();
            return None;
        }
        let traced = self.cpu.tracing();
        let saved = self.cpu.a;
        let d = match cpu::decode(&mut self.mem, pc) {
            Ok((d, _)) => d,
            Err(DecodeError::Fault(f)) => return Some(self.fault(f)),
            Err(DecodeError::Vector(v)) => {
                // The only address error decoding raises is a bad index extension word: an operand-read error
                // (measured on an MCF54415).
                let fs = if v == vector::ADDRESS_ERROR {
                    ADDRESS_ERROR_OPERAND_FS
                } else {
                    0
                };
                if let Some(r) = self.take_with_fs(v, fs, pc) {
                    return Some(r);
                }
                self.tick();
                return None;
            }
        };
        let r = cpu::exec::execute(&mut self.cpu, &mut self.mem, &d);
        self.finish(r, pc, d.len, saved, traced)
    }

    /// Complete the instruction at `pc` (`len` bytes) after its handler returned `r`: set PC, count it, take the
    /// exception or stop it asked for, and the trace exception when T was set as it began (`traced`). `saved` is
    /// A0-A7 before it. `Some` ends the run.
    fn finish(
        &mut self,
        r: Result<Flow, Fault>,
        pc: u32,
        len: u8,
        saved: [u32; 8],
        traced: bool,
    ) -> Option<StopReason> {
        let next = pc.wrapping_add(len as u32);
        match r {
            Ok(Flow::Next) => self.cpu.pc = next,
            Ok(Flow::Jump) => {}
            Ok(Flow::Exception(v)) => {
                self.cpu.a = saved;
                self.cpu.pc = pc;
                if let Some(r) = self.take(v, if stacks_next_pc(v) { next } else { pc }) {
                    return Some(r);
                }
                self.tick();
                return None;
            }
            Ok(Flow::ExceptionAfterPush(v)) => {
                self.cpu.pc = pc;
                // The push stands only when the exception is taken; a run that stops on it leaves the
                // registers as before the instruction, so it can resume (the pushed longword stays in memory).
                if self.policy.stop_on[v as usize] {
                    self.cpu.a = saved;
                }
                if let Some(r) = self.take(v, pc) {
                    return Some(r);
                }
                self.tick();
                return None;
            }
            Ok(Flow::Wait) => {
                self.cpu.pc = next;
                self.tick();
                // RM 3.3.4.6: a STOP that starts traced, or whose immediate sets T, takes the trace exception
                // with PC after it instead of waiting.
                if traced || self.cpu.tracing() {
                    return self.take(vector::TRACE, next);
                }
                self.waiting = true;
                self.mem.horizon = 0;
                return None;
            }
            Ok(Flow::StopAfter(kind)) => {
                self.cpu.pc = next;
                self.tick();
                return Some(self.stop(kind, String::new()));
            }
            Ok(Flow::Stop(kind)) => {
                self.cpu.a = saved;
                self.cpu.pc = pc;
                let detail = match kind {
                    Stop::Unmodelled => self.form_name(pc),
                    _ => String::new(),
                };
                return Some(self.stop(kind, detail));
            }
            Err(f) => {
                self.cpu.a = saved;
                self.cpu.pc = pc;
                return Some(self.fault(f));
            }
        }
        self.tick();
        // RM 3.3.4.6: with T set when it began, a completed instruction is followed by a trace exception.
        if traced {
            return self.take(vector::TRACE, self.cpu.pc);
        }
        None
    }

    /// The mnemonic of the form at `pc` (it was just decoded there), or "CAU" for the CAU's forms.
    fn form_name(&self, pc: u32) -> String {
        let form = self
            .mem
            .peek(pc, 2)
            .and_then(|b| crate::isa::decode(u16::from_be_bytes([b[0], b[1]])));
        match form {
            Some(f) if f.isa == Isa::Cau => "CAU".to_string(),
            Some(f) => f.mnemonic.to_string(),
            None => String::new(),
        }
    }

    /// At an instruction boundary: advance the devices whose events are due, service the bus masters, report a
    /// halt a device asked for, stop at the time limit, and take the highest deliverable interrupt. While the core
    /// waits after STOP, the clock jumps from event to event until an interrupt is taken. `Ok(true)` when an
    /// interrupt was taken.
    fn service(&mut self) -> Result<bool, StopReason> {
        loop {
            self.mem.advance_due();
            self.mem.service_masters();
            if let Some(r) = self.mem.stop.take() {
                return Err(self.stop(r.kind, r.detail));
            }
            if let Some(limit) = self.time_limit
                && self.mem.now >= limit
            {
                return Err(self.stop(Stop::TimeLimit, format!("{} ticks", self.mem.now)));
            }
            if let Some((controller, irq)) = self.deliverable() {
                if self.policy.stop_on[irq.vector as usize] {
                    return Err(self.exception(irq.vector));
                }
                if let Err(f) =
                    self.cpu
                        .interrupt(&mut self.mem, irq.vector, irq.level, self.cpu.pc)
                {
                    return Err(self.fault(f));
                }
                self.mem.acknowledge(controller, irq);
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
                self.mem.horizon = h.max(now + 1);
                return Ok(false);
            }
            // STOP: nothing to take yet, so jump the clock to the next event (or the time limit before it).
            match event {
                Some(t) if self.can_wake() => {
                    if t <= self.mem.now {
                        // Every due event was just advanced; one still due would be advanced forever.
                        return Err(self.stop(
                            Stop::Halt("device event not advanced"),
                            format!(
                                "a device's next event stays at tick {t}, not after the clock ({}); the device must move its event forward in `advance`",
                                self.mem.now
                            ),
                        ));
                    }
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

    /// Whether any request a device could raise would be taken while the core waits after STOP: some controller,
    /// with the lines as they could become, reports a level above SR\[I\], or level 7 when the level-7 line is
    /// not already held (a held line gives no new edge).
    ///
    /// The lines as they could become are the asserted lines plus the sources every device with a pending event
    /// lists from [`Device::wake_sources`]; when one of those devices cannot say, every source of every controller
    /// is taken as asserted. While the core waits, SR is fixed and no instruction runs, so no controller register
    /// is written: a device event can only change the request lines. The probe is exact as long as a controller's
    /// masks and levels change only through its registers, and the sources a device lists cover its events.
    // guess: an interrupt controller that changes its own masks or levels in `advance` (none is modelled: the chip's
    // INTC has no timed behaviour) could wake a core this reports as unable to wake.
    fn can_wake(&self) -> bool {
        let ipl = (self.cpu.sr() >> 8) as u8 & 7;
        let lines = self
            .mem
            .wake_lines()
            .unwrap_or_else(crate::irq::IrqLines::all);
        self.mem
            .pending_with(&lines, !self.level7_line)
            .is_some_and(|irq| irq.level == 7 || irq.level > ipl)
    }

    /// The request the core takes now, if any, and the controller making it. Samples the level-7 line for its
    /// rising edge. A request held off by SR\[I\] is looked at again after the next write to SR (MOVE to SR,
    /// STRLDSR, RTE and STOP end the run of uninterrupted execution).
    fn deliverable(&mut self) -> Option<(usize, Irq)> {
        let line = self.mem.level7();
        if line != self.level7_line {
            self.level7_line = line;
            self.level7_edge = line;
        }
        let (controller, irq) = self.mem.pending(self.level7_edge)?;
        let ipl = (self.cpu.sr() >> 8) as u8 & 7;
        (irq.level == 7 || irq.level > ipl).then_some((controller, irq))
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
        let fs = if v == vector::ADDRESS_ERROR {
            ADDRESS_ERROR_FETCH_FS
        } else {
            0
        };
        self.take_with_fs(v, fs, return_pc)
    }

    /// As [`Machine::take`], with the fault status `fs` for the frame.
    fn take_with_fs(&mut self, v: u8, fs: u32, return_pc: u32) -> Option<StopReason> {
        if self.policy.stop_on[v as usize] {
            return Some(self.exception(v));
        }
        self.cpu
            .raise_with_fs(&mut self.mem, v, fs, return_pc)
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

    /// The stop for fault `f`; a device's refusal brings its detail.
    fn fault(&mut self, f: Fault) -> StopReason {
        match f {
            Fault::Unmapped { addr, size } => self.stop(
                Stop::Unmapped,
                format!("unmapped {size:?} access at {addr:#010x}"),
            ),
            Fault::Device(kind) => {
                let detail = self.mem.refused.take().unwrap_or_default();
                self.stop(kind, detail)
            }
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
