//! The edge port (RM ch. 18): its IRQ pads as async inputs ([`Input`]: embedded-hal-async `Wait`, embedded-hal
//! `InputPin`).
//!
//! A wait arms its pin: the pin's EPPAR field is set to the sense, its flag (EPFR) cleared and its interrupt (EPIER)
//! enabled before anything else, and the handler disables it again and wakes the waiter. Dropping a wait disarms
//! its pin. The edge waits arm their edge. `wait_for_low` arms the level-sensitive mode, whose request is the pin
//! being low, so it fires at once on a low pin. `wait_for_high` arms the rising edge, then reads the level and
//! returns at once if the pin is high or the edge already came.
//!
//! The edge port has no data register (RM Table 18-2), and a pad muxed to IRQn reads 0 in its GPIO port's PPDSDR
//! (measured on an MCF54415). The level is read through the interrupt controller instead: with the pin
//! level-sensitive and its EPIER bit set, its INTC0 IPR bit is set while the pin is low, whether or not the source
//! is masked (measured on an MCF54415). [`Input::is_high`] sets that up for the read and puts the pin's EPPAR and
//! EPIER back, with the source masked meanwhile. A rising edge in the few instructions between that read and the
//! rising edge's return to EPPAR is not seen, so `wait_for_high` can miss a pin that rises then and stays high.
//!
//! Each pad's source on INTC0 (source n for IRQn, RM Table 17-15) takes its level from its ICR, as every source
//! does: [`Config::priority`]. The pads offered are those whose PAR field is settled (`pins.rs`): IRQ1-IRQ7 on the
//! 256-pin parts.
use core::convert::Infallible;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Poll;

use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

use crate::clocks::{self, Gate};
use crate::gpio::{self, Pull};
use crate::interrupt::typelevel::{Binding, Handler, Interrupt};
use crate::interrupt::Priority;
use crate::pinmux;
use crate::pac;

mod pins;

const EPPAR: usize = 0x0;
const EPIER: usize = 0x3;
const EPFR: usize = 0x6;

fn base() -> usize {
    crate::pac::EPORT.as_ptr() as usize
}

/// What an armed pin waits for (EPPAR modes, RM Table 18-3).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u16)]
enum Edge {
    /// Level-sensitive: the pin being low.
    Low = 0b00,
    Rising = 0b01,
    Falling = 0b10,
    Both = 0b11,
}

/// (mask, value) of pin `n`'s EPPAR field for `edge`.
const fn eppar_field(n: u8, edge: Edge) -> (u16, u16) {
    (0b11 << (2 * n), (edge as u16) << (2 * n))
}

/// Driver configuration.
#[non_exhaustive]
#[derive(Copy, Clone, Debug)]
pub struct Config {
    /// Interrupt level of the pad's source: level 4 by default.
    pub priority: Priority,
}

impl Default for Config {
    fn default() -> Self {
        Self { priority: Priority::P4 }
    }
}

pub(crate) mod sealed {
    use crate::pinmux::{Function, ParField};

    pub trait IrqPin {
        /// The pad's EPF source.
        type Interrupt: crate::interrupt::typelevel::Interrupt;
        /// n of IRQn.
        const IRQ: u8;
        const PAR: ParField;
        const FUNCTION: Function;
    }
}

/// A pad that carries an edge port input.
#[allow(private_bounds)]
pub trait IrqPin: sealed::IrqPin + gpio::Pin + PeripheralType {}

static WAKERS: [AtomicWaker; 8] = [const { AtomicWaker::new() }; 8];
/// Set by the handler when pin n's edge came; cleared when it is armed.
static FIRED: [AtomicBool; 8] = [const { AtomicBool::new(false) }; 8];

/// The edge port's interrupt handler, bound to each pad's source (`EPORT_EPFn => eport::InterruptHandler`).
pub struct InterruptHandler;

impl<I: Interrupt> Handler<I> for InterruptHandler {
    unsafe fn on_interrupt() {
        // EPORT_EPFn is vector 64 + n
        let n = (I::IRQ as u16 - 64) as u8;
        disarm(n);
        FIRED[n as usize].store(true, Ordering::Release);
        WAKERS[n as usize].wake();
    }
}

/// The pad of pin `n`'s level, and whether its flag was set (an armed edge came). The pin is level-sensitive with
/// its interrupt enabled for the read of its IPR bit, its source masked meanwhile; its flag is cleared, as a set flag
/// would show in IPR too (RM 18.4.2); its EPPAR field and EPIER bit are put back.
fn sample(n: u8) -> (bool, bool) {
    let (mask, _) = eppar_field(n, Edge::Low);
    critical_section::with(|_| unsafe {
        let unmasked = pac::INTC0.imrl().read().int_mask() & 1 << n == 0;
        pac::INTC0.simr().write(|w| w.set_simr(n));
        let (par, ier, fr) = (base() + EPPAR, base() + EPIER, base() + EPFR);
        let (p, e) = (coldfire::mmio::r16(par), coldfire::mmio::r8(ier));
        let latched = coldfire::mmio::r8(fr) & 1 << n != 0;
        coldfire::mmio::w8(fr, 1 << n);
        coldfire::mmio::w16(par, p & !mask);
        coldfire::mmio::w8(ier, e | 1 << n);
        // guess: two reads back give the pad's synchronizer the bus clocks it needs; only measured over BDM
        let _ = (coldfire::mmio::r16(par), coldfire::mmio::r8(ier));
        let low = pac::INTC0.iprl().read().int() & 1 << n != 0;
        coldfire::mmio::w8(ier, e);
        coldfire::mmio::w16(par, p);
        if unmasked {
            pac::INTC0.cimr().write(|w| w.set_cimr(n));
        }
        (!low, latched)
    })
}

/// Pin `n` waits for `edge`: its flag cleared, its mode set, then its interrupt enabled.
fn arm(n: u8, edge: Edge) {
    FIRED[n as usize].store(false, Ordering::Release);
    let (mask, v) = eppar_field(n, edge);
    critical_section::with(|_| unsafe {
        coldfire::mmio::w8(base() + EPFR, 1 << n);
        let par = base() + EPPAR;
        coldfire::mmio::w16(par, (coldfire::mmio::r16(par) & !mask) | v);
        let ier = base() + EPIER;
        coldfire::mmio::w8(ier, coldfire::mmio::r8(ier) | 1 << n);
    });
}

/// Pin `n`'s interrupt disabled and its flag cleared.
fn disarm(n: u8) {
    critical_section::with(|_| unsafe {
        let ier = base() + EPIER;
        coldfire::mmio::w8(ier, coldfire::mmio::r8(ier) & !(1 << n));
        coldfire::mmio::w8(base() + EPFR, 1 << n);
    });
}

/// Disarms its pin when dropped: a wait cut short leaves nothing armed.
struct Armed(u8);

impl Drop for Armed {
    fn drop(&mut self) {
        disarm(self.0);
    }
}

/// An edge port input.
pub struct Input<'d> {
    n: u8,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> Input<'d> {
    /// Set the pad's pull, mux it to IRQn, ungate the edge port and enable the pad's source at
    /// `config.priority`. The pin is armed only while a wait runs.
    pub fn new<P: IrqPin>(
        pin: Peri<'d, P>,
        pull: Pull,
        _irq: impl Binding<P::Interrupt, InterruptHandler> + 'd,
        config: Config,
    ) -> Self {
        let n = P::IRQ;
        gpio::set_pull(gpio::SealedPin::port(&*pin), gpio::SealedPin::bit(&*pin), pull);
        pinmux::set_function(P::PAR, P::FUNCTION);
        clocks::ungate(Gate::Eport);
        disarm(n);
        P::Interrupt::set_priority(config.priority);
        // SAFETY: the binding proves the handler is installed; the pin stays disarmed until a wait arms it
        unsafe { P::Interrupt::enable() };
        Self { n, _p: PhantomData }
    }

    /// The pad's level, read through INTC0 IPR (see the module docs).
    pub fn is_high(&self) -> bool {
        sample(self.n).0
    }

    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Wait for `edge`; with `high`, return at once if the pin is high or the edge already came.
    async fn wait(&mut self, edge: Edge, high: bool) {
        let n = self.n;
        arm(n, edge);
        let _armed = Armed(n);
        if high {
            let (level, latched) = sample(n);
            if level || latched {
                return;
            }
        }
        poll_fn(|cx| {
            WAKERS[n as usize].register(cx.waker());
            if FIRED[n as usize].load(Ordering::Acquire) { Poll::Ready(()) } else { Poll::Pending }
        })
        .await
    }
}

impl embedded_hal::digital::ErrorType for Input<'_> {
    type Error = Infallible;
}

impl embedded_hal::digital::InputPin for Input<'_> {
    fn is_high(&mut self) -> Result<bool, Infallible> {
        Ok(Input::is_high(self))
    }

    fn is_low(&mut self) -> Result<bool, Infallible> {
        Ok(Input::is_low(self))
    }
}

impl embedded_hal_async::digital::Wait for Input<'_> {
    async fn wait_for_high(&mut self) -> Result<(), Infallible> {
        self.wait(Edge::Rising, true).await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Infallible> {
        self.wait(Edge::Low, false).await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Infallible> {
        self.wait(Edge::Rising, false).await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Infallible> {
        self.wait(Edge::Falling, false).await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Infallible> {
        self.wait(Edge::Both, false).await;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eppar_fields_are_two_bits_per_pin() {
        // RM Table 18-3: 00 level, 01 rising, 10 falling, 11 both
        assert_eq!(eppar_field(1, Edge::Low), (0b11 << 2, 0));
        assert_eq!(eppar_field(2, Edge::Rising), (0b11 << 4, 0b01 << 4));
        assert_eq!(eppar_field(6, Edge::Falling), (0b11 << 12, 0b10 << 12));
        assert_eq!(eppar_field(7, Edge::Both), (0b11 << 14, 0b11 << 14));
    }

    #[test]
    fn the_default_level_is_4() {
        assert_eq!(Config::default().priority, Priority::P4);
    }
}
