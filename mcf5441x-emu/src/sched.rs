//! The device table: address windows, the interrupt controllers and bus masters among the devices, and the events
//! they schedule.
//!
//! Everything here walks the devices in the order they were added, so the order of advances, of bus master
//! servicing and the choice between equal interrupt requests never depend on anything but that order.
//!
//! A bus master is taken out of its slot while it is called (it gets the rest of the address space through a
//! [`Bus`](crate::bus::Bus)) and put back afterwards; while it is out, its window reads as unmapped.
//!
//! The table also holds the clock gates: 128 gate bits, set when the gate is closed, and the gate each device is
//! tied to, if any. A device whose gate is closed is *gated*: its accesses do not reach it, its events are held
//! (it is neither advanced nor, for a bus master, serviced) and it lists no wake sources. Its request lines stay
//! as it left them. The gated state is kept as one flag per device, so a device access pays one check.
//!
//! The table keeps each device's [`Device::next_event`] as it last answered, and asks it again only after a call
//! into that device (an access, an advance, a service, an acknowledge, a gate change or a host-side call) and
//! when its gated state changes, so finding the due events and the earliest one costs no call into any device.

use std::cell::Cell;
use std::rc::Rc;

use crate::device::{Device, DeviceId};
use crate::irq::{InterruptController, Irq, IrqLines};

/// A device, keeping the controller interface of the ones that have it.
enum Model {
    Plain(Box<dyn Device>),
    /// A device that may access the address space.
    Master(Box<dyn Device>),
    Controller(Box<dyn InterruptController>),
    /// A bus master taken out of its slot for the duration of a call into it.
    Out,
}

/// What a slot holds, for [`Devices::kind`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Kind {
    /// A device that is not a bus master (a controller is one of these).
    Plain,
    Master,
    /// A bus master that is being called.
    Out,
}

/// A device's address window: `base..=last`.
#[derive(Debug, Clone, Copy)]
struct Window {
    base: u32,
    last: u32,
    index: usize,
}

impl Default for Window {
    /// A window holding no address.
    fn default() -> Self {
        Window {
            base: 1,
            last: 0,
            index: 0,
        }
    }
}

/// The devices of a [`Memory`](crate::Memory).
#[derive(Default)]
pub(crate) struct Devices {
    /// In the order they were added; a [`DeviceId`] indexes this.
    models: Vec<Model>,
    /// Sorted by base, non-overlapping.
    windows: Vec<Window>,
    /// The window [`Devices::find`] last found, tried first: device accesses come in runs to one device.
    last_window: Cell<Window>,
    /// Indices of the interrupt controllers in `models`.
    controllers: Vec<usize>,
    /// The controllers' generation counters ([`InterruptController::generation`]), one for each shared one.
    generations: Vec<Rc<Cell<u64>>>,
    /// A controller without a generation counter was added.
    ungenerated: bool,
    /// Moves on whenever the controllers or their counters may have changed: a controller was added, or the host
    /// had a device mutably.
    epoch: u64,
    /// Indices of the bus masters in `models`.
    masters: Vec<usize>,
    /// The clock gates: bit n is set while gate n is closed.
    closed: u128,
    /// Per device: the gate it is tied to, if any.
    gate: Vec<Option<u8>>,
    /// Per device: its gate is closed.
    gated: Vec<bool>,
    /// Per device: the gated state it was last told of ([`Device::gate`]).
    told: Vec<bool>,
    /// Per device: its [`Device::next_event`] when it was last asked, `None` while it is gated.
    events: Vec<Option<u64>>,
    /// The earliest of `events`.
    soonest: Option<u64>,
    /// The host had a device mutably ([`Devices::get_mut`], or a host-side call) since every event and generation
    /// counter was last asked for: until [`Devices::refresh_all`], each event is asked when it is wanted and the
    /// controllers' answers are not reused.
    pub(crate) stale: bool,
    /// Counts register accesses (the core's and bus masters') and clock gate settings, wrapping: unchanged, no
    /// device was reached through its registers and no gate changed.
    pub(crate) touched: u64,
}

impl Devices {
    pub(crate) fn len(&self) -> usize {
        self.models.len()
    }

    /// Whether `base..=last` overlaps a device window.
    pub(crate) fn overlaps(&self, base: u32, last: u32) -> bool {
        self.windows
            .iter()
            .any(|w| base <= w.last && w.base <= last)
    }

    fn add(&mut self, window: Option<(u32, u32)>, model: Model) -> DeviceId {
        let index = self.models.len();
        if let Some((base, last)) = window {
            assert!(
                !self.overlaps(base, last),
                "device overlaps another at {base:#x}"
            );
            let at = self.windows.partition_point(|w| w.base < base);
            self.windows.insert(at, Window { base, last, index });
        }
        match model {
            Model::Controller(_) => self.controllers.push(index),
            Model::Master(_) => self.masters.push(index),
            _ => {}
        }
        self.models.push(model);
        if matches!(self.models[index], Model::Controller(_)) {
            self.regenerate();
        }
        self.gate.push(None);
        self.gated.push(false);
        self.told.push(false);
        self.events.push(None);
        self.refresh(index);
        DeviceId(index)
    }

    pub(crate) fn add_device(
        &mut self,
        window: Option<(u32, u32)>,
        dev: Box<dyn Device>,
    ) -> DeviceId {
        self.add(window, Model::Plain(dev))
    }

    pub(crate) fn add_master(&mut self, window: (u32, u32), dev: Box<dyn Device>) -> DeviceId {
        self.add(Some(window), Model::Master(dev))
    }

    pub(crate) fn add_controller(
        &mut self,
        window: (u32, u32),
        c: Box<dyn InterruptController>,
    ) -> DeviceId {
        self.add(Some(window), Model::Controller(c))
    }

    /// The device whose window holds all of `[addr, addr + len)`, and the offset of `addr` in it.
    #[inline]
    pub(crate) fn find(&self, addr: u32, len: u32) -> Option<(usize, u32)> {
        let last = self.last_window.get();
        let w = if last.base <= addr && addr <= last.last {
            last
        } else {
            let w = *self.windows[..self.windows.partition_point(|w| w.base <= addr)].last()?;
            self.last_window.set(w);
            w
        };
        let off = addr - w.base;
        (off as u64 + len as u64 <= (w.last - w.base) as u64 + 1).then_some((w.index, off))
    }

    /// Device `i`, unless it is a bus master being called.
    pub(crate) fn try_get(&self, i: usize) -> Option<&dyn Device> {
        match &self.models[i] {
            Model::Plain(d) | Model::Master(d) => Some(&**d),
            Model::Controller(c) => Some(&**c),
            Model::Out => None,
        }
    }

    pub(crate) fn get(&self, i: usize) -> &dyn Device {
        self.try_get(i)
            .unwrap_or_else(|| panic!("device {i} is being called"))
    }

    /// Device `i`, mutably, for the host: its event is asked again whenever it is wanted until the next
    /// [`Devices::refresh_all`].
    pub(crate) fn get_mut(&mut self, i: usize) -> &mut dyn Device {
        self.stale = true;
        self.call_mut(i)
    }

    /// Device `i`, mutably, for a call whose end is followed by [`Devices::refresh`].
    pub(crate) fn call_mut(&mut self, i: usize) -> &mut dyn Device {
        match &mut self.models[i] {
            Model::Plain(d) | Model::Master(d) => &mut **d,
            Model::Controller(c) => &mut **c,
            Model::Out => panic!("device {i} is being called"),
        }
    }

    pub(crate) fn kind(&self, i: usize) -> Kind {
        match self.models[i] {
            Model::Plain(_) | Model::Controller(_) => Kind::Plain,
            Model::Master(_) => Kind::Master,
            Model::Out => Kind::Out,
        }
    }

    /// Take bus master `i` out of its slot; panics unless it is one and is in its slot.
    pub(crate) fn take(&mut self, i: usize) -> Box<dyn Device> {
        match std::mem::replace(&mut self.models[i], Model::Out) {
            Model::Master(d) => d,
            _ => panic!("device {i} is not a bus master in its slot"),
        }
    }

    /// Put bus master `i` back after [`Devices::take`].
    pub(crate) fn restore(&mut self, i: usize, dev: Box<dyn Device>) {
        debug_assert!(matches!(self.models[i], Model::Out));
        self.models[i] = Model::Master(dev);
    }

    /// The bus masters, in the order they were added.
    pub(crate) fn masters(&self) -> &[usize] {
        &self.masters
    }

    /// Ask device `i` for its event again, after a call into it or a change of its gated state; a gated device has
    /// none, and a bus master being called is asked when it is put back.
    #[inline]
    pub(crate) fn refresh(&mut self, i: usize) {
        let event = if self.gated[i] {
            None
        } else if let Some(d) = self.try_get(i) {
            d.next_event()
        } else {
            return;
        };
        let was = std::mem::replace(&mut self.events[i], event);
        if event == was {
            return;
        }
        // `None` is no event: it sorts after every tick here.
        let before = |a: Option<u64>, b: Option<u64>| a.is_some_and(|a| b.is_none_or(|b| a < b));
        if before(event, self.soonest) {
            self.soonest = event;
        } else if was.is_some() && was == self.soonest {
            self.soonest = self.events.iter().filter_map(|&t| t).min();
        }
    }

    /// Ask every device for its event again, and every controller for its generation counter, if the host had a
    /// device mutably since they were last all asked.
    pub(crate) fn refresh_all(&mut self) {
        if self.stale {
            self.stale = false;
            for i in 0..self.models.len() {
                self.refresh(i);
            }
            self.regenerate();
        }
    }

    /// Ask every controller for its generation counter ([`InterruptController::generation`]).
    fn regenerate(&mut self) {
        self.epoch = self.epoch.wrapping_add(1);
        self.generations.clear();
        self.ungenerated = false;
        for k in 0..self.controllers.len() {
            let Model::Controller(c) = &self.models[self.controllers[k]] else {
                unreachable!("controller index of a device that is not a controller");
            };
            match c.generation() {
                Some(g) if self.generations.iter().any(|a| Rc::ptr_eq(a, &g)) => {}
                Some(g) => self.generations.push(g),
                None => self.ungenerated = true,
            }
        }
    }

    /// Device `i`'s event; `None` when it is gated.
    #[inline]
    fn event(&self, i: usize) -> Option<u64> {
        if !self.stale {
            return self.events[i];
        }
        if self.gated[i] {
            return None;
        }
        self.try_get(i)?.next_event()
    }

    /// Whether device `i` has an event due at `now`; a gated device has none.
    pub(crate) fn due(&self, i: usize, now: u64) -> bool {
        self.event(i).is_some_and(|t| t <= now)
    }

    /// The earliest event any device that is not gated has scheduled.
    pub(crate) fn next_event(&self) -> Option<u64> {
        if !self.stale {
            return self.soonest;
        }
        (0..self.models.len()).filter_map(|i| self.event(i)).min()
    }

    /// The earliest event any bus master that is not gated has scheduled.
    pub(crate) fn masters_event(&self) -> Option<u64> {
        self.masters.iter().filter_map(|&i| self.event(i)).min()
    }

    /// Whether device `i`'s clock gate is closed.
    #[inline(always)]
    pub(crate) fn gated(&self, i: usize) -> bool {
        self.gated[i]
    }

    /// The clock gates: bit n is set while gate n is closed.
    pub(crate) fn closed(&self) -> u128 {
        self.closed
    }

    /// Set the clock gates at tick `now`, telling every device in its slot whose gated state changed.
    // guess: a gated module's events are held and its interrupt and DMA request lines stay as it left them; an
    // interrupt controller's requests still reach the core while it is gated (none of this is measured).
    pub(crate) fn set_closed(&mut self, closed: u128, now: u64) {
        self.touched = self.touched.wrapping_add(1);
        self.closed = closed;
        for i in 0..self.models.len() {
            self.gated[i] = self.gate[i].is_some_and(|g| closed >> g & 1 != 0);
            self.tell(i, now);
            self.refresh(i);
        }
    }

    /// Tie device `i` to gate `gate` (0-127), or to none, at tick `now`. Panics when `gate` is above 127.
    pub(crate) fn set_gate(&mut self, i: usize, gate: Option<u8>, now: u64) {
        assert!(
            gate.is_none_or(|g| g < 128),
            "no clock gate {gate:?}: 0-127"
        );
        self.gate[i] = gate;
        self.gated[i] = gate.is_some_and(|g| self.closed >> g & 1 != 0);
        self.tell(i, now);
        self.refresh(i);
    }

    /// Tell device `i` its gated state ([`Device::gate`]) if it changed since it was last told and the device is
    /// in its slot (a bus master being called is told when it is put back).
    pub(crate) fn tell(&mut self, i: usize, now: u64) {
        let gated = self.gated[i];
        if self.told[i] != gated && !matches!(self.models[i], Model::Out) {
            self.told[i] = gated;
            self.call_mut(i).gate(gated, now);
            self.refresh(i);
        }
    }

    /// The interrupt controllers with their indices, in the order they were added.
    fn controllers(&self) -> impl Iterator<Item = (usize, &dyn InterruptController)> + '_ {
        self.controllers.iter().map(|&i| match &self.models[i] {
            Model::Controller(c) => (i, &**c),
            _ => unreachable!("controller index of a device that is not a controller"),
        })
    }

    /// Controller `i`, mutably, for a call whose end is followed by [`Devices::refresh`]; panics when device `i` is
    /// not an interrupt controller.
    pub(crate) fn controller_mut(&mut self, i: usize) -> &mut dyn InterruptController {
        match &mut self.models[i] {
            Model::Controller(c) => &mut **c,
            _ => panic!("device {i} is not an interrupt controller"),
        }
    }

    /// The highest request over all controllers, and the index of the controller making it. At equal levels the
    /// controller added first wins: RM 17.3.2 gives INTC0 the highest priority and INTC2 the lowest, so the
    /// chip's controllers are added in that order.
    pub(crate) fn pending(&self, lines: &IrqLines, allow_level7: bool) -> Option<(usize, Irq)> {
        self.controllers()
            .filter_map(|(i, c)| Some((i, c.pending(lines, allow_level7)?)))
            .fold(None, |best: Option<(usize, Irq)>, (i, irq)| match best {
                Some(b) if b.1.level >= irq.level => Some(b),
                _ => Some((i, irq)),
            })
    }

    /// Whether any controller holds the level-7 line up.
    pub(crate) fn level7(&self, lines: &IrqLines) -> bool {
        self.controllers().any(|(_, c)| c.level7(lines))
    }

    /// Whether every controller has a generation counter ([`InterruptController::generation`]) and the counters
    /// read `gens`, with the controllers and their counters as they were at `epoch`.
    #[inline]
    pub(crate) fn generations_are(&self, epoch: u64, gens: &[u64]) -> bool {
        let c = &self.generations;
        !self.ungenerated
            && !self.stale
            && epoch == self.epoch
            && c.len() == gens.len()
            && c.iter().zip(gens).all(|(c, &g)| c.get() == g)
    }

    /// Set `gens` to what the controllers' generation counters read and `epoch` to the epoch; `false` when a
    /// controller has none or the host had a device mutably since the counters were last asked for.
    pub(crate) fn generations(&self, epoch: &mut u64, gens: &mut Vec<u64>) -> bool {
        *epoch = self.epoch;
        gens.clear();
        gens.extend(self.generations.iter().map(|c| c.get()));
        !self.ungenerated && !self.stale
    }

    /// Push to `out` the interrupt sources every device that is not gated lists ([`Device::wake_sources`]), in the
    /// order the devices were added, whether it has a pending event or not (a bus master lists the routes of the
    /// requests it would serve); `false` when a device with a pending event cannot say.
    pub(crate) fn wake_sources(&self, out: &mut Vec<(usize, u8)>) -> bool {
        let mut known = true;
        for d in (0..self.models.len())
            .filter(|&i| !self.gated[i])
            .filter_map(|i| self.try_get(i))
        {
            let mark = out.len();
            if !d.wake_sources(out) {
                out.truncate(mark);
                if d.next_event().is_some() {
                    known = false;
                }
            }
        }
        known
    }
}
