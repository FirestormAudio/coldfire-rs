//! SSI (synchronous serial interface, RM ch. 35): I2S or network-mode audio, streamed by eDMA in both directions.
use embassy_hal_internal::PeripheralType;

mod pins;

pub(crate) mod sealed {
    use crate::pinmux::{Function, ParField};

    pub trait Instance {
        const REGS: crate::pac::ssi0::Ssi0;
        const GATE: crate::clocks::Gate;
        /// 0 for SSI0, 1 for SSI1 (MISCCR's clock-source field).
        const INDEX: u8;
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
    ($(#[$m:meta])* $name:ident, $gate:ident, $index:expr, $tx:ident, $rx:ident) => {
        $(#[$m])*
        impl sealed::Instance for crate::peripherals::$name {
            const REGS: crate::pac::ssi0::Ssi0 = crate::pac::$name;
            const GATE: crate::clocks::Gate = crate::clocks::Gate::$gate;
            const INDEX: u8 = $index;
        }
        $(#[$m])*
        impl Instance for crate::peripherals::$name {
            type TxDma = crate::peripherals::$tx;
            type RxDma = crate::peripherals::$rx;
        }
    };
}

instance!(SSI0, Ssi0, 0, DMA_CH50, DMA_CH48);
instance!(#[cfg(any(feature = "mcf54415", feature = "mcf54416", feature = "mcf54417", feature = "mcf54418"))] SSI1, Ssi1, 1, DMA_CH54, DMA_CH52);

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
