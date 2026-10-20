//! SSI (synchronous serial interface, RM ch. 35): I2S or network-mode audio, streamed by eDMA in both directions.
use embassy_hal_internal::PeripheralType;

use crate::pac;
use crate::pac::ssi0::regs;
use crate::pinmux::set_function;

mod config;
mod pins;
mod stream;
pub use stream::*;
pub use config::*;

pub(crate) mod sealed {
    use crate::pinmux::{Function, ParField};

    pub trait Instance {
        const REGS: crate::pac::ssi0::Ssi0;
        const GATE: crate::clocks::Gate;
        /// 0 for SSI0, 1 for SSI1 (MISCCR's clock-source field).
        const INDEX: u8;
        fn state() -> &'static super::stream::State;
        /// The render interrupt's entry (an exception trampoline into `State::run_isr`).
        #[cfg(target_arch = "m68k")]
        fn render_entry() -> unsafe extern "C" fn();
    }
    pub trait BclkPin<T> {
        const PAR: ParField;
        const FUNCTION: Function;
    }
    pub trait FsPin<T> {
        const PAR: ParField;
        const FUNCTION: Function;
    }
    pub trait TxdPin<T> {
        const PAR: ParField;
        const FUNCTION: Function;
    }
    pub trait RxdPin<T> {
        const PAR: ParField;
        const FUNCTION: Function;
    }
    pub trait MclkPin<T> {
        const PAR: ParField;
        const FUNCTION: Function;
    }
    pub trait ClkinPin {
        const PAR: ParField;
        const FUNCTION: Function;
    }
}

/// An SSI, with the eDMA channels its transmit-0 and receive-0 requests drive (RM Table 19-6).
#[allow(private_bounds)]
pub trait Instance: sealed::Instance + PeripheralType + 'static {
    type TxDma: crate::dma::ChannelInstance;
    type RxDma: crate::dma::ChannelInstance;
}
/// A pad that can carry `T`'s bit clock.
#[allow(private_bounds)]
pub trait BclkPin<T: Instance>: sealed::BclkPin<T> + PeripheralType {}
/// A pad that can carry `T`'s frame sync.
#[allow(private_bounds)]
pub trait FsPin<T: Instance>: sealed::FsPin<T> + PeripheralType {}
/// A pad that can carry `T`'s transmit data.
#[allow(private_bounds)]
pub trait TxdPin<T: Instance>: sealed::TxdPin<T> + PeripheralType {}
/// A pad that can carry `T`'s receive data.
#[allow(private_bounds)]
pub trait RxdPin<T: Instance>: sealed::RxdPin<T> + PeripheralType {}
/// A pad that can carry `T`'s master clock out.
#[allow(private_bounds)]
pub trait MclkPin<T: Instance>: sealed::MclkPin<T> + PeripheralType {}
/// The pad that can carry SSI_CLKIN, the external audio clock both SSIs may run from.
#[allow(private_bounds)]
pub trait ClkinPin: sealed::ClkinPin + PeripheralType {}

macro_rules! instance {
    ($(#[$m:meta])* $name:ident, $gate:ident, $index:expr, $tx:ident, $rx:ident, $entry:ident, $isr:ident) => {
        #[cfg(target_arch = "m68k")]
        $(#[$m])*
        coldfire_rt::exception_entry!($entry, $isr);
        #[cfg(target_arch = "m68k")]
        $(#[$m])*
        #[unsafe(no_mangle)]
        extern "C" fn $isr(_frame: u32) {
            <crate::peripherals::$name as sealed::Instance>::state().run_isr();
        }
        $(#[$m])*
        impl sealed::Instance for crate::peripherals::$name {
            const REGS: crate::pac::ssi0::Ssi0 = crate::pac::$name;
            const GATE: crate::clocks::Gate = crate::clocks::Gate::$gate;
            const INDEX: u8 = $index;
            fn state() -> &'static stream::State {
                static STATE: stream::State = stream::State::new();
                &STATE
            }
            #[cfg(target_arch = "m68k")]
            fn render_entry() -> unsafe extern "C" fn() {
                $entry
            }
        }
        $(#[$m])*
        impl Instance for crate::peripherals::$name {
            type TxDma = crate::peripherals::$tx;
            type RxDma = crate::peripherals::$rx;
        }
    };
}

instance!(SSI0, Ssi0, 0, DMA_CH50, DMA_CH48, ssi0_render_entry, embassy_mcf5441x_ssi0_render);
instance!(#[cfg(any(feature = "mcf54415", feature = "mcf54416", feature = "mcf54417", feature = "mcf54418"))] SSI1, Ssi1, 1, DMA_CH54, DMA_CH52, ssi1_render_entry, embassy_mcf5441x_ssi1_render);

pub(crate) const CR_SSI_EN: u32 = 1 << 0;
pub(crate) const CR_TE: u32 = 1 << 1;
pub(crate) const CR_RE: u32 = 1 << 2;
/// Depth of each FIFO, in words (RM 35.1.2).
pub(crate) const FIFO_DEPTH: usize = 15;

/// A register the start sequence writes.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Reg {
    Cr,
    Tx0,
}

/// How many zero words to prime the transmit FIFO with: whole frames only, or every later frame would start a slot
/// late (left and right swapped).
pub(crate) const fn prime_words(slots: usize) -> usize {
    if slots > FIFO_DEPTH { 0 } else { FIFO_DEPTH - FIFO_DEPTH % slots }
}

/// The clocks-only start, in order: enable, prime the transmit FIFO with silent frames, then transmit (the bit clock
/// and frame sync run while TE is set; RM 35.4.1). The first `n + 2` entries are the sequence.
pub(crate) fn start_sequence(cr: u32, slots: usize) -> ([(Reg, u32); FIFO_DEPTH + 2], usize) {
    let n = prime_words(slots);
    let mut seq = [(Reg::Tx0, 0); FIFO_DEPTH + 2];
    seq[0] = (Reg::Cr, cr | CR_SSI_EN);
    seq[n + 1] = (Reg::Cr, cr | CR_SSI_EN | CR_TE);
    (seq, n)
}

/// An SSI, configured but not yet streaming.
pub struct Ssi<'d, T: Instance> {
    pub(crate) cr: u32,
    pub(crate) slots: usize,
    pub(crate) tx_watermark: u8,
    pub(crate) rx_watermark: u8,
    _p: core::marker::PhantomData<&'d mut T>,
}

impl<'d, T: Instance> Ssi<'d, T> {
    /// Ungate `T`, mux its pins, select its clock source and write its configuration (transmit, receive and the
    /// SSI itself stay off).
    pub fn new<B: BclkPin<T>, F: FsPin<T>, X: TxdPin<T>, R: RxdPin<T>>(
        _ssi: crate::Peri<'d, T>,
        _bclk: crate::Peri<'d, B>,
        _fs: crate::Peri<'d, F>,
        _txd: crate::Peri<'d, X>,
        _rxd: crate::Peri<'d, R>,
        config: Config,
    ) -> Self {
        let w = words(&config);
        crate::clocks::ungate(T::GATE);
        set_function(<B as sealed::BclkPin<T>>::PAR, <B as sealed::BclkPin<T>>::FUNCTION);
        set_function(<F as sealed::FsPin<T>>::PAR, <F as sealed::FsPin<T>>::FUNCTION);
        set_function(<X as sealed::TxdPin<T>>::PAR, <X as sealed::TxdPin<T>>::FUNCTION);
        set_function(<R as sealed::RxdPin<T>>::PAR, <R as sealed::RxdPin<T>>::FUNCTION);
        let (mask, value) = misccr_src(T::INDEX, config.clock);
        let m = pac::CCM.misccr().read().0;
        pac::CCM.misccr().write_value(pac::ccm::regs::Misccr((m & !mask) | value));
        if let ClockSource::Pll { div } = config.clock {
            let (mask, value) = cdrh_div(T::INDEX, div);
            let d = pac::CCM.cdrh().read().0;
            pac::CCM.cdrh().write_value(pac::ccm::regs::Cdrh((d & !mask) | value));
        }
        let r = T::REGS;
        r.cr().write_value(regs::Cr(w.cr));
        r.tcr().write_value(regs::Tcr(w.tcr));
        r.rcr().write_value(regs::Rcr(w.rcr));
        r.ccr().write_value(regs::Ccr(w.ccr));
        r.fcsr().write_value(regs::Fcsr(w.fcsr));
        r.ier().write_value(regs::Ier(w.ier));
        Self {
            cr: w.cr,
            slots: config.slots as usize,
            tx_watermark: config.tx_watermark,
            rx_watermark: config.rx_watermark,
            _p: core::marker::PhantomData,
        }
    }

    /// Put SSI_CLKIN (the external audio clock) on its pad, for `ClockSource::Clkin`.
    pub fn with_clkin<C: ClkinPin>(self, _clkin: crate::Peri<'d, C>) -> Self {
        set_function(<C as sealed::ClkinPin>::PAR, <C as sealed::ClkinPin>::FUNCTION);
        self
    }

    /// Run the bit clock and frame sync with silence going out, without streaming: for a codec that needs its clocks
    /// before it is configured. `stream` keeps them running.
    pub fn start_clocks(&mut self) {
        let r = T::REGS;
        let (seq, n) = start_sequence(self.cr, self.slots);
        for &(reg, v) in &seq[..n + 2] {
            match reg {
                Reg::Cr => r.cr().write_value(regs::Cr(v)),
                Reg::Tx0 => r.tx0().write_value(regs::Tx0(v)),
            }
        }
    }

    /// Transmit, receive and the SSI off.
    pub fn stop(&mut self) {
        T::REGS.cr().write_value(regs::Cr(self.cr));
    }

    /// The SSI is enabled (clocks running).
    pub fn is_running(&self) -> bool {
        T::REGS.cr().read().0 & CR_SSI_EN != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clocks::{bank_slot, Gate};
    use crate::pinmux::{par_update, Function};

    #[test]
    fn ssi_gates() {
        assert_eq!(bank_slot(Gate::Ssi0), (0, 47));
        assert_eq!(bank_slot(Gate::Ssi1), (0, 50));
        assert_eq!(<crate::peripherals::SSI0 as sealed::Instance>::GATE, Gate::Ssi0);
        assert_eq!(<crate::peripherals::SSI1 as sealed::Instance>::GATE, Gate::Ssi1);
    }

    #[test]
    fn ssi1_pins_are_par_uart2_alt2() {
        use crate::peripherals::{PE3, PE4, PE5, PE6, SSI1};
        let fields = [
            (<PE6 as sealed::BclkPin<SSI1>>::PAR, <PE6 as sealed::BclkPin<SSI1>>::FUNCTION),
            (<PE5 as sealed::FsPin<SSI1>>::PAR, <PE5 as sealed::FsPin<SSI1>>::FUNCTION),
            (<PE3 as sealed::TxdPin<SSI1>>::PAR, <PE3 as sealed::TxdPin<SSI1>>::FUNCTION),
            (<PE4 as sealed::RxdPin<SSI1>>::PAR, <PE4 as sealed::RxdPin<SSI1>>::FUNCTION),
        ];
        assert!(fields.iter().all(|(_, f)| *f == Function::Alt2));
        let v = fields.iter().fold(0u8, |v, &(p, f)| par_update(v, p, f));
        assert_eq!(v, 0x55);
    }

    #[test]
    fn clkin_is_ph4_alt1() {
        use crate::peripherals::PH4;
        assert_eq!(par_update(0, <PH4 as sealed::ClkinPin>::PAR, <PH4 as sealed::ClkinPin>::FUNCTION), 0x02);
    }

    #[test]
    fn start_clocks_sets_en_and_te_and_primes_silence() {
        let cr = 1 << 4 | 0b01 << 5;
        let (seq, n) = start_sequence(cr, 2);
        assert_eq!(seq[0], (Reg::Cr, cr | CR_SSI_EN));
        assert!(seq[1..1 + n].iter().all(|&w| w == (Reg::Tx0, 0)));
        assert_eq!(seq[1 + n], (Reg::Cr, cr | CR_SSI_EN | CR_TE));
    }

    #[test]
    fn priming_fills_whole_frames_only() {
        // an odd word count would shift every later frame by a slot (left and right swapped)
        assert_eq!(prime_words(2), 14);
        assert_eq!(prime_words(4), 12);
        assert_eq!(prime_words(1), 15);
        assert_eq!(prime_words(16), 0);
        let (_, n) = start_sequence(0, 2);
        assert_eq!(n, 14);
    }

    #[test]
    fn ssi_dma_channels() {
        use crate::dma::ChannelInstance;
        use crate::peripherals::{SSI0, SSI1};
        // RM Table 19-6: SSIn transmit 0 / receive 0
        assert_eq!(<<SSI0 as Instance>::TxDma as ChannelInstance>::NUMBER, 50);
        assert_eq!(<<SSI0 as Instance>::RxDma as ChannelInstance>::NUMBER, 48);
        assert_eq!(<<SSI1 as Instance>::TxDma as ChannelInstance>::NUMBER, 54);
        assert_eq!(<<SSI1 as Instance>::RxDma as ChannelInstance>::NUMBER, 52);
    }
}
