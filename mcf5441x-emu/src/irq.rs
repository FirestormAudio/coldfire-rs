//! Interrupt request lines and the interface of an interrupt controller (RM 3.3.4.13, CFPRM ch. 11).
//!
//! Devices drive request lines into the controllers ([`IrqLines`]); each controller turns its asserted, unmasked
//! sources into a priority level and a vector ([`InterruptController::pending`]). The core samples at instruction
//! boundaries: it takes the highest pending request when its level is above SR\[I\], or is 7. Level 7 is
//! non-maskable and edge-triggered on one line shared by all controllers: it is taken once each time that line
//! rises (RM 17.1.1, RM 3.2.9 SR\[I\]).

use std::cell::Cell;
use std::rc::Rc;

use crate::device::{Device, DeviceCx};

/// The request lines into each interrupt controller: up to 64 sources per controller.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct IrqLines {
    asserted: Vec<u64>,
    /// Every source of every controller reads as asserted: the lines as they could become.
    all: bool,
}

impl IrqLines {
    /// Assert (`on`) or negate source `source` (0-63) of controller `controller` (the controller's own number,
    /// which its model uses to read [`IrqLines::asserted`]).
    pub fn set(&mut self, controller: usize, source: u8, on: bool) {
        assert!(source < 64, "interrupt source {source} out of range");
        if controller >= self.asserted.len() {
            if !on {
                return;
            }
            self.asserted.resize(controller + 1, 0);
        }
        let bit = 1u64 << source;
        if on {
            self.asserted[controller] |= bit;
        } else {
            self.asserted[controller] &= !bit;
        }
    }

    /// The asserted sources of `controller`, one bit per source.
    pub fn asserted(&self, controller: usize) -> u64 {
        if self.all {
            return !0;
        }
        self.asserted.get(controller).copied().unwrap_or(0)
    }

    /// Lines with every source of every controller asserted.
    pub(crate) fn all() -> IrqLines {
        IrqLines {
            asserted: Vec::new(),
            all: true,
        }
    }
}

/// A request the core may take: its priority level (1-7) and vector (64-255 for device interrupts).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Irq {
    pub level: u8,
    pub vector: u8,
}

/// An interrupt controller: a device (its registers) that also arbitrates requests for the core.
pub trait InterruptController: Device {
    /// The highest-priority asserted, unmasked request, ignoring level-7 sources unless `allow_level7`.
    fn pending(&self, lines: &IrqLines, allow_level7: bool) -> Option<Irq>;
    /// Whether an asserted, unmasked level-7 source holds the core's level-7 line up.
    fn level7(&self, lines: &IrqLines) -> bool;
    /// The core took `irq`, this controller's request: called at interrupt entry, once the exception frame is
    /// written and the core is about to run the handler, on the controller whose request was taken (the one
    /// [`InterruptController::pending`] reported it). Requests are sampled again before the handler's first
    /// instruction, so a request this negates is not taken twice.
    fn acknowledge(&mut self, _irq: Irq, _cx: &mut DeviceCx) {}
    /// A counter the controller moves on whenever what [`InterruptController::pending`] and
    /// [`InterruptController::level7`] report for the same request lines could change, whatever changed it (a call
    /// into this controller or into a device sharing its state); `None` (the default) when it cannot say. Asked
    /// when the controller is added, and again after the host had any device mutably
    /// ([`Machine::device_mut`](crate::Machine::device_mut), [`Machine::with_device`](crate::Machine::with_device));
    /// controllers sharing their state may share one counter.
    ///
    /// While every controller has a counter, the machine asks `pending` and `level7` again only when a counter or
    /// the request lines changed since it last asked them, and reuses their answers otherwise.
    fn generation(&self) -> Option<Rc<Cell<u64>>> {
        None
    }
}
