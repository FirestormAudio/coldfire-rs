//! Memory-mapped device models: the trait they implement, the context each call gets, and the access trace.

use std::any::Any;

use crate::irq::IrqLines;
use crate::mem::Size;
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
pub trait Device: Any {
    /// A read of `size` at `offset`. Only the low `size` bytes of the result are used.
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32;
    /// A write of `size` at `offset`; `value` holds only the low `size` bytes.
    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx);
    /// The absolute tick of the next thing this device does by itself, if any. It is asked again after every
    /// access to any device and every [`Device::advance`].
    fn next_event(&self) -> Option<u64> {
        None
    }
    /// Called at an instruction boundary once the clock has reached [`Device::next_event`]. `now` may be past
    /// the event when the event was set in the past. Afterwards the event should lie after `now`; a device whose
    /// event stays due is advanced again at the next instruction boundary.
    fn advance(&mut self, _now: u64, _cx: &mut DeviceCx) {}
}

/// What a device can reach during an access or an [`Device::advance`].
pub struct DeviceCx<'a> {
    /// The clock: the tick of the instruction making the access, or the tick being advanced to.
    pub now: u64,
    /// The interrupt request lines into the interrupt controllers.
    pub irq: &'a mut IrqLines,
    pub(crate) stop: &'a mut Option<StopReason>,
}

impl DeviceCx<'_> {
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
