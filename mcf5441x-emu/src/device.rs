//! Memory-mapped device models: the trait they implement, the context each call gets, and the access trace.

use std::any::Any;

use crate::bus::{Bus, DmaLines};
use crate::irq::IrqLines;
use crate::mem::{Size, Space};
use crate::stop::{Stop, StopReason};

/// A peripheral model.
///
/// A device mapped with [`Machine::add_device`](crate::Machine::add_device) sees the accesses that fall wholly
/// inside its window, with the offset from its base; an access that runs past either end of the window is
/// unmapped. A timer added with [`Machine::add_timer`](crate::Machine::add_timer) has no registers and only gets
/// [`Device::advance`].
///
/// Time is the machine's clock: one tick per instruction. A device that needs to act at a tick reports it from
/// [`Device::next_event`]; the core stops at exactly that instruction boundary and calls [`Device::advance`]
/// before the next instruction runs, then checks for interrupts.
///
/// A device added with [`Machine::add_bus_master`](crate::Machine::add_bus_master) is a *bus master*: every call
/// into it gets a [`Bus`] from [`DeviceCx::bus`], and [`Device::service`] is called at every instruction boundary
/// (see [`crate::bus`]).
pub trait Device: Any {
    /// A read of `size` at `offset`. Only the low `size` bytes of the result are used.
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32;
    /// A write of `size` at `offset`; `value` holds only the low `size` bytes.
    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx);
    /// The absolute tick of the next thing this device does by itself, if any. It is asked again after every
    /// call into this device (a register access, [`Device::advance`], [`Device::service`], [`Device::gate`], an
    /// interrupt acknowledge, [`Machine::with_device`](crate::Machine::with_device)), when its gated state
    /// changes (its clock gate closing or opening), and after the host had it through
    /// [`Machine::device_mut`](crate::Machine::device_mut); the answer is kept in between. So the event may depend only on the device's own state, changed only by those calls: not on
    /// state it shares with another device (through a cell, say), which a call into that device could change.
    fn next_event(&self) -> Option<u64> {
        None
    }
    /// Called at an instruction boundary once the clock has reached [`Device::next_event`]. `now` may be past
    /// the event when the event was set in the past. Afterwards the event should lie after `now`; a device whose
    /// event stays due is advanced again at the next instruction boundary.
    fn advance(&mut self, _now: u64, _cx: &mut DeviceCx) {}
    /// Bus masters only: serve the DMA requests in `cx.dma`. Called at every instruction boundary after the due
    /// events, and after every host-side call, on every bus master in the order they were added, and called again
    /// while a round of calls changed the request lines; more than 64 rounds that all change them stop the run
    /// with [`Stop::Unmodelled`]. Not called while [`Device::serves`] is `false`.
    ///
    /// A master serves each asserted request until it drops or no further progress is possible (e.g. it runs
    /// minor loops while the line stays asserted): a request left asserted is not serviced again until the next
    /// boundary, which RAM-only code may never reach.
    fn service(&mut self, _cx: &mut DeviceCx) {}
    /// Bus masters only: whether [`Device::service`] could do anything with the DMA request lines as they are in
    /// `dma`. The machine skips the call when this is `false`; the default, `true`, has it made every time. A
    /// master overrides it only with an exact answer: `false` must mean that the call would change nothing.
    fn serves(&self, _dma: &DmaLines) -> bool {
        true
    }
    /// Whether the poll detector counts reads of this device's registers (see
    /// [`Machine::set_poll_limit`](crate::Machine::set_poll_limit)). A model whose registers change with time or
    /// with state the core cannot see returns `false`.
    fn poll_counted(&self) -> bool {
        true
    }
    /// The interrupt sources (controller, source) that this device's future events, or the DMA it requests, could
    /// assert: pushed to `out`, returning `true`; `false` (the default) when the device cannot say, and any source
    /// might be asserted. A bus master lists the sources the requests it would serve could assert (the eDMA: the
    /// routes of its enabled channels with interrupts); it is asked whether or not it has an event of its own.
    /// While the core waits after STOP, a device without a pending event that cannot say is left out.
    fn wake_sources(&self, _out: &mut Vec<(usize, u8)>) -> bool {
        false
    }
    /// Called at tick `now` when this device's clock gate closes (`closed`) or opens again (see
    /// [`Machine::set_clock_gate`](crate::Machine::set_clock_gate)). While the gate is closed the device is not
    /// called for accesses (a read returns all ones and a write is dropped), its events are held (it is neither
    /// advanced nor serviced and [`Device::next_event`] is not asked) and it lists no wake sources; the request
    /// lines it drives stay as it left them. A model whose state follows the clock moves it here by the time the
    /// gate was closed, so that it resumes where it stopped; one that does not (the default) is advanced when the
    /// gate opens with `now` past its held event, as for an event set in the past.
    fn gate(&mut self, _closed: bool, _now: u64) {}
}

/// What a device can reach during an access or an [`Device::advance`].
pub struct DeviceCx<'a> {
    /// The clock: the tick of the instruction making the access, or the tick being advanced to.
    pub now: u64,
    /// The interrupt request lines into the interrupt controllers.
    pub irq: &'a mut IrqLines,
    /// The DMA request lines into the eDMA.
    pub dma: &'a mut DmaLines,
    pub(crate) stop: &'a mut Option<StopReason>,
    /// The address space without the device being called, when that device is a bus master.
    pub(crate) space: Option<&'a mut Space>,
    /// The refusal of the access in flight, if the device made one.
    pub(crate) refused: Option<Refusal>,
    /// The clock gates, as [`DeviceCx::clock_gates`] reads them and [`DeviceCx::set_clock_gates`] leaves them.
    pub(crate) gates: u128,
}

/// A refused access: the kind of stop and its detail.
pub(crate) type Refusal = (Stop, String);

impl DeviceCx<'_> {
    /// The clock gates (see [`Machine::set_clock_gates`](crate::Machine::set_clock_gates)): bit n is set while
    /// gate n is closed.
    pub fn clock_gates(&self) -> u128 {
        self.gates
    }

    /// Set the clock gates, for the device that holds the gate registers. The devices whose gated state changes
    /// see it when this call into the device returns, at the same tick.
    pub fn set_clock_gates(&mut self, closed: u128) {
        self.gates = closed;
    }

    /// The address space, for a bus master; `None` for any other device. The bus borrows this context: the
    /// lines a device reached through it drives are seen in `irq` and `dma` once it is dropped.
    pub fn bus(&mut self) -> Option<Bus<'_>> {
        Bus::of(self)
    }

    /// End the run. The access (and the instruction making it) completes; the run stops at the next instruction
    /// boundary, with PC on the next instruction, so a later run continues from there. The first request in an
    /// instruction wins. When the instruction then faults, the fault is reported and the request is dropped.
    pub fn halt(&mut self, kind: Stop, detail: impl Into<String>) {
        if self.stop.is_none() {
            *self.stop = Some(StopReason {
                kind,
                pc: 0,
                detail: detail.into(),
            });
        }
    }

    /// Refuse the access in flight: it fails, and the instruction making it does not complete. The run stops with
    /// `kind` and `detail`, PC on that instruction and the registers as they were before it, as for an access to
    /// unmapped memory; a later run tries the instruction again. This is how a model stops on a register access
    /// to a feature it does not model. The first refusal in a call wins. Only the instruction is rolled back: a
    /// device that refuses a register write may already have acted on it (run minor loops, set an ERQ bit), and
    /// those effects stay.
    ///
    /// A bus master's access refused this way fails with [`Fault::Device`](crate::Fault::Device) and also asks the
    /// run to stop as [`DeviceCx::halt`] does, so the detail is kept whatever the master does with the error. A
    /// host-side [`Machine::access`](crate::Machine::access) refused this way returns that error and leaves no stop
    /// pending, including one a bus master's refused access inside it asked for. Called outside an access (from [`Device::advance`], [`Device::service`],
    /// [`InterruptController::acknowledge`](crate::InterruptController::acknowledge) or
    /// [`Machine::with_device`](crate::Machine::with_device)), it acts as [`DeviceCx::halt`].
    pub fn refuse(&mut self, kind: Stop, detail: impl Into<String>) {
        if self.refused.is_none() {
            self.refused = Some((kind, detail.into()));
        }
    }
}

/// Ask the run to stop with `refusal` as [`DeviceCx::halt`] does, unless a halt is already asked for.
pub(crate) fn halt_on(stop: &mut Option<StopReason>, refusal: Option<Refusal>) {
    if let Some((kind, detail)) = refusal
        && stop.is_none()
    {
        *stop = Some(StopReason {
            kind,
            pc: 0,
            detail,
        });
    }
}

/// A device added to a [`Machine`](crate::Machine), for [`Machine::device`](crate::Machine::device).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DeviceId(pub(crate) usize);

/// One device access, recorded while [`Machine::trace`](crate::Machine::trace) is on. RAM accesses are not
/// recorded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Access {
    /// The clock during the access.
    pub t: u64,
    /// The instruction making it.
    pub pc: u32,
    pub addr: u32,
    pub size: Size,
    pub write: bool,
    /// The value read or written, truncated to `size`.
    pub value: u32,
}
