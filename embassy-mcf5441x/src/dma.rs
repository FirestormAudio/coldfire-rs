//! eDMA (RM ch. 19): channel tokens and transfer control descriptors.
//!
//! The request map is fixed: channel n serves DMA request n (UART9 RX is 36, UART8 RX 34, …), so a driver
//! takes the token of the channel its peripheral requests on. The controller stays at its reset
//! configuration (CR, DCHPRI: fixed priority, no minor-loop mapping); only the per-channel set/clear
//! registers and the TCDs are written.
use core::marker::PhantomData;

use embassy_hal_internal::{Peri, PeripheralType};

use crate::clocks::{self, Gate};
use crate::interrupt::typelevel;
use crate::pac;

/// A DMA channel token.
pub trait ChannelInstance: PeripheralType + 'static {
    const NUMBER: u8;
    /// The interrupt this channel raises (channels 56..=63 share one).
    type Interrupt: typelevel::Interrupt;
}

macro_rules! impl_dma_channel {
    ($name:ident, $n:expr, $irq:ident) => {
        impl $crate::dma::ChannelInstance for $crate::peripherals::$name {
            const NUMBER: u8 = $n;
            type Interrupt = $crate::interrupt::typelevel::$irq;
        }
    };
}
pub(crate) use impl_dma_channel;

/// Transfer size (TCD ATTR SSIZE/DSIZE).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Size {
    Bits8 = 0,
    Bits16 = 1,
    Bits32 = 2,
    Bytes16 = 4,
}

/// TCD ATTR: source size and modulo (2^`smod`-byte ring, 0 = off), destination size and modulo.
pub const fn attr(ssize: Size, smod: u8, dsize: Size, dmod: u8) -> u16 {
    ((smod as u16 & 0x1F) << 11) | ((ssize as u16) << 8) | ((dmod as u16 & 0x1F) << 3) | dsize as u16
}

/// TCD CSR bits.
pub mod csr {
    pub const START: u16 = 1 << 0;
    pub const INT_MAJOR: u16 = 1 << 1;
    pub const INT_HALF: u16 = 1 << 2;
    /// Clear the channel's request enable when the major loop completes.
    pub const D_REQ: u16 = 1 << 3;
    /// The channel is executing a minor loop.
    pub const ACTIVE: u16 = 1 << 6;
    pub const DONE: u16 = 1 << 7;
}

/// A transfer control descriptor, field by field (no scatter-gather, no channel linking).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Tcd {
    pub saddr: u32,
    pub soff: i16,
    pub attr: u16,
    pub nbytes: u32,
    pub slast: i32,
    pub daddr: u32,
    pub doff: i16,
    pub citer: u16,
    pub dlast_sga: i32,
    pub biter: u16,
    pub csr: u16,
}

/// An owned DMA channel.
pub struct Channel<'d> {
    n: u8,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> Channel<'d> {
    pub fn new<C: ChannelInstance>(_ch: Peri<'d, C>) -> Self {
        clocks::ungate(Gate::Edma);
        Self { n: C::NUMBER, _p: PhantomData }
    }

    pub fn number(&self) -> u8 {
        self.n
    }

    /// Write the TCD, in the order SADDR, SOFF, ATTR, NBYTES, SLAST, DADDR, DOFF, CITER, DLAST_SGA, BITER, CSR.
    ///
    /// # Safety
    /// Once requests are enabled the eDMA reads and writes the addresses in `tcd`: they must stay valid
    /// for as long as the channel may run.
    pub unsafe fn configure(&mut self, tcd: &Tcd) {
        use pac::edma::regs;
        let t = pac::EDMA.tcd(self.n as usize);
        t.saddr().write_value(regs::Saddr(tcd.saddr));
        t.soff().write_value(regs::Soff(tcd.soff as u16));
        t.attr().write_value(regs::Attr(tcd.attr));
        t.nbytes().write_value(regs::Nbytes(tcd.nbytes));
        t.slast().write_value(regs::Slast(tcd.slast as u32));
        t.daddr().write_value(regs::Daddr(tcd.daddr));
        t.doff().write_value(regs::Doff(tcd.doff as u16));
        t.citer().write_value(regs::Citer(tcd.citer));
        t.dlast_sga().write_value(regs::DlastSga(tcd.dlast_sga as u32));
        t.biter().write_value(regs::Biter(tcd.biter));
        t.csr().write_value(regs::Csr(tcd.csr));
    }

    /// Let the peripheral's requests start transfers (SERQ).
    pub fn enable_request(&mut self) {
        pac::EDMA.serq().write(|w| w.set_serq(self.n));
    }

    /// Stop the peripheral's requests (CERQ).
    pub fn disable_request(&mut self) {
        pac::EDMA.cerq().write(|w| w.set_cerq(self.n));
    }

    pub fn clear_interrupt(&mut self) {
        clear_interrupt(self.n)
    }

    pub fn clear_error(&mut self) {
        pac::EDMA.cerr().write(|w| w.set_cerr(self.n));
    }

    pub fn clear_done(&mut self) {
        pac::EDMA.cdne().write(|w| w.set_cdne(self.n));
    }

    /// The destination address the next transfer writes to.
    pub fn daddr(&self) -> u32 {
        daddr(self.n)
    }

    pub fn csr(&self) -> u16 {
        pac::EDMA.tcd(self.n as usize).csr().read().0
    }
}

/// Clear channel `n`'s interrupt request (CINT), for handlers that hold only the number.
pub(crate) fn clear_interrupt(n: u8) {
    pac::EDMA.cint().write(|w| w.set_cint(n));
}

/// Channel `n`'s current DADDR.
pub(crate) fn daddr(n: u8) -> u32 {
    pac::EDMA.tcd(n as usize).daddr().read().0
}

/// Channel `n`'s current source address.
pub(crate) fn saddr(n: u8) -> u32 {
    pac::EDMA.tcd(n as usize).saddr().read().0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn attr_packs_sizes_and_modulos() {
        // 8-bit transfers into a 2^10-byte destination ring: the UART receive TCD (was ATTR 0x0050)
        assert_eq!(attr(Size::Bits8, 0, Size::Bits8, 10), 0x0050);
        // 32-bit both ways, no modulo (was ATTR_32BIT 0x0202)
        assert_eq!(attr(Size::Bits32, 0, Size::Bits32, 0), 0x0202);
        // a proven UART TX TCD: 4 KB source ring, 8-bit (ATTR 0x6000)
        assert_eq!(attr(Size::Bits8, 12, Size::Bits8, 0), 0x6000);
    }
}
