//! Interrupt/eDMA-driven UART, in a scheme proven on hardware: eDMA copies each received
//! byte into a 1 KB ring (one interrupt per byte), and the TXRDY interrupt feeds UTB from a 64-byte ring.
//!
//! The read position comes from the TCD's DADDR, not from counting interrupts: while interrupts are masked,
//! several completions merge into one pending interrupt. A reader that falls a whole ring behind can't be
//! detected. The DMA ring assumes the data cache is off.
//!
//! Several `BufferedUartTx` handles may share a port (it's `Clone`): enqueueing runs in a short critical
//! section, and `write_atomic` / `try_write_all` queue a whole command or nothing. The port has two TX wakers:
//! handles from `split()` (and their clones) use one, handles from `second()` the other, so one of each kind
//! may wait (`write`, `write_atomic`, `flush`) at the same time.
use core::cell::UnsafeCell;
use core::convert::Infallible;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering::{Acquire, Relaxed, Release}};
use core::task::Poll;

use embassy_hal_internal::Peri;
use embassy_sync::waitqueue::AtomicWaker;
use embedded_io_async::{ErrorType, Read, Write};

use super::{check_baud, enable_rx, line_setup, mux_rx, mux_tx, take_error, Config, ConfigError, Error, Instance, RxPin, TxPin};
use crate::dma::{self, ChannelInstance};
use crate::interrupt::typelevel::{Binding, Handler, Interrupt as _};
use crate::pac;

pub(crate) const RX_LEN: usize = 1024;
pub(crate) const TX_LEN: usize = 64;
const UIMR_TXRDY: u8 = 1 << 0;

/// A port's DMA receive ring, 1 KB aligned for DMOD = 10.
#[repr(C, align(1024))]
pub struct RxRing(UnsafeCell<[u8; RX_LEN]>);

// SAFETY: written only by the DMA, read only by the port's single `BufferedUartRx`.
unsafe impl Sync for RxRing {}

impl RxRing {
    pub(crate) const fn new() -> Self {
        Self(UnsafeCell::new([0; RX_LEN]))
    }
}

/// One port's state.
pub struct State {
    tx_ring: UnsafeCell<[u8; TX_LEN]>,
    rx_waker: AtomicWaker,
    /// One waker per TX handle kind (`split()` and `second()`), so two handles can wait at once.
    tx_wakers: [AtomicWaker; 2],
    tx_head: AtomicUsize, // advanced by Tx (under a critical section)
    tx_tail: AtomicUsize, // advanced by the TXRDY handler
    /// UIMR is write-only (reads return UISR), so keep a copy.
    uimr: AtomicU8,
    taken: AtomicBool,
}

// SAFETY: the TX ring is filled by Tx handles under a critical section and drained by the TXRDY handler,
// published through tx_head/tx_tail.
unsafe impl Sync for State {}

impl State {
    pub(crate) const fn new() -> Self {
        Self {
            tx_ring: UnsafeCell::new([0; TX_LEN]),
            rx_waker: AtomicWaker::new(),
            tx_wakers: [AtomicWaker::new(), AtomicWaker::new()],
            tx_head: AtomicUsize::new(0),
            tx_tail: AtomicUsize::new(0),
            uimr: AtomicU8::new(0),
            taken: AtomicBool::new(false),
        }
    }

    fn set_txrdy_irq(&self, r: pac::uart0::Uart0, on: bool) {
        critical_section::with(|_| {
            let v = self.uimr.load(Relaxed);
            let v = if on { v | UIMR_TXRDY } else { v & !UIMR_TXRDY };
            self.uimr.store(v, Relaxed);
            r.uimr().write_value(pac::uart0::regs::Uimr(v));
        });
    }
}

/// The port's TXRDY interrupt: feed UTB from the TX ring.
pub struct BufferedInterruptHandler<T: Instance> {
    _t: PhantomData<T>,
}

impl<T: Instance> Handler<T::Interrupt> for BufferedInterruptHandler<T> {
    unsafe fn on_interrupt() {
        let s = T::buffered();
        let tail = s.tx_tail.load(Relaxed);
        if tail == s.tx_head.load(Acquire) {
            s.set_txrdy_irq(T::REGS, false);
            return;
        }
        let b = unsafe { (*s.tx_ring.get())[tail % TX_LEN] };
        T::REGS.utb().write(|w| w.set_tb(b));
        s.tx_tail.store(tail.wrapping_add(1), Release);
        s.tx_wakers[0].wake();
        s.tx_wakers[1].wake();
    }
}

/// The receive channel's interrupt (one per byte): acknowledge it and wake the reader.
pub struct RxDmaInterruptHandler<T: Instance> {
    _t: PhantomData<T>,
}

impl<T: Instance> Handler<<T::RxDma as ChannelInstance>::Interrupt> for RxDmaInterruptHandler<T> {
    unsafe fn on_interrupt() {
        dma::clear_interrupt(<T::RxDma as ChannelInstance>::NUMBER);
        T::buffered().rx_waker.wake();
    }
}

/// A started port.
pub struct BufferedUart<'d> {
    rx: BufferedUartRx<'d>,
    tx: BufferedUartTx<'d>,
}

impl<'d> BufferedUart<'d> {
    /// Mux the pins, set the line up (`Config`), point the receive channel at the port's ring and start it.
    /// Panics if this UART was already started.
    pub fn new<T: Instance>(
        _uart: Peri<'d, T>,
        rx_dma: Peri<'d, T::RxDma>,
        tx: Peri<'d, impl TxPin<T>>,
        rx: Peri<'d, impl RxPin<T>>,
        _irqs: impl Binding<T::Interrupt, BufferedInterruptHandler<T>>
            + Binding<<T::RxDma as ChannelInstance>::Interrupt, RxDmaInterruptHandler<T>>
            + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        check_baud(config.baudrate)?; // before anything is touched: a rejected rate leaves the port free
        let s = T::buffered();
        assert!(!s.taken.swap(true, Relaxed), "UART already started");
        mux_tx::<T, _>(&tx);
        mux_rx::<T, _>(&rx);
        line_setup::<T>(config.baudrate)?;
        let ring = T::rx_ring();
        let mut ch = dma::Channel::new(rx_dma);
        let urb = T::REGS.as_ptr() as u32 + 0x0C;
        // SAFETY: the ring is a static, so the DMA's destination stays valid forever.
        unsafe {
            ch.configure(&dma::Tcd {
                saddr: urb,
                soff: 0,
                attr: dma::attr(dma::Size::Bits8, 0, dma::Size::Bits8, 10),
                nbytes: 1,
                slast: 0,
                daddr: ring.0.get() as u32,
                doff: 1,
                citer: 1,
                dlast_sga: 0,
                biter: 1,
                csr: dma::csr::INT_MAJOR,
            })
        };
        <<T::RxDma as ChannelInstance>::Interrupt>::set_priority(config.priority);
        T::Interrupt::set_priority(config.priority);
        T::REGS.uimr().write_value(pac::uart0::regs::Uimr(0));
        // SAFETY: the handlers only touch this port's state and registers, all set up above.
        unsafe {
            <<T::RxDma as ChannelInstance>::Interrupt>::enable();
            T::Interrupt::enable();
        }
        ch.enable_request();
        enable_rx(T::REGS);
        let n = <T::RxDma as ChannelInstance>::NUMBER;
        Ok(Self {
            rx: BufferedUartRx { state: s, ring, r: T::REGS, ch: n, offset: 0, _p: PhantomData },
            tx: BufferedUartTx { state: s, r: T::REGS, w: 0, _p: PhantomData },
        })
    }

    pub fn split(self) -> (BufferedUartRx<'d>, BufferedUartTx<'d>) {
        (self.rx, self.tx)
    }
}

impl ErrorType for BufferedUart<'_> {
    type Error = Error;
}

impl Read for BufferedUart<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        self.rx.read(buf).await
    }
}

impl Write for BufferedUart<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Error> {
        let Ok(n) = self.tx.write(buf).await;
        Ok(n)
    }

    async fn flush(&mut self) -> Result<(), Error> {
        let Ok(()) = self.tx.flush().await;
        Ok(())
    }
}

pub struct BufferedUartRx<'d> {
    state: &'static State,
    ring: &'static RxRing,
    r: pac::uart0::Uart0,
    ch: u8,
    offset: usize,
    _p: PhantomData<&'d mut ()>,
}

impl BufferedUartRx<'_> {
    fn available(&self) -> usize {
        let base = self.ring.0.get() as usize;
        let daddr = dma::daddr(self.ch) as usize;
        daddr.wrapping_sub(base).wrapping_sub(self.offset) % RX_LEN
    }

    /// Drop everything received so far: the next read returns only bytes the DMA writes after this.
    pub fn discard(&mut self) {
        let base = self.ring.0.get() as usize;
        let daddr = dma::daddr(self.ch) as usize;
        self.offset = daddr.wrapping_sub(base) % RX_LEN;
    }
}

impl ErrorType for BufferedUartRx<'_> {
    type Error = Error;
}

impl Read for BufferedUartRx<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        poll_fn(|cx| {
            if let Some(e) = take_error(self.r) {
                return Poll::Ready(Err(e));
            }
            let mut n = self.available();
            if n == 0 {
                self.state.rx_waker.register(cx.waker());
                n = self.available();
                if n == 0 {
                    return Poll::Pending;
                }
            }
            let n = n.min(buf.len());
            let ring = self.ring.0.get().cast::<u8>();
            for (i, b) in buf[..n].iter_mut().enumerate() {
                *b = unsafe { ring.add((self.offset + i) % RX_LEN).read_volatile() }; // written by DMA
            }
            self.offset = (self.offset + n) % RX_LEN;
            Poll::Ready(Ok(n))
        })
        .await
    }
}

#[derive(Clone)]
pub struct BufferedUartTx<'d> {
    state: &'static State,
    r: pac::uart0::Uart0,
    w: usize, // which TX waker this handle waits on
    _p: PhantomData<&'d mut ()>,
}

impl<'d> BufferedUartTx<'d> {
    /// A handle that waits on the port's second TX waker, so it and this one (or its clones) can both wait for
    /// ring space at the same time without stealing each other's wakeup.
    pub fn second(&self) -> BufferedUartTx<'d> {
        BufferedUartTx { state: self.state, r: self.r, w: 1, _p: PhantomData }
    }

    fn free(&self) -> usize {
        TX_LEN - self.state.tx_head.load(Relaxed).wrapping_sub(self.state.tx_tail.load(Acquire))
    }

    /// Queue `buf` (which fits); call inside a critical section.
    fn enqueue(&self, buf: &[u8]) {
        let head = self.state.tx_head.load(Relaxed);
        for (i, &b) in buf.iter().enumerate() {
            unsafe { (*self.state.tx_ring.get())[(head + i) % TX_LEN] = b };
        }
        self.state.tx_head.store(head.wrapping_add(buf.len()), Release);
        self.state.set_txrdy_irq(self.r, true);
    }

    /// Queue as much of `buf` as fits without waiting; returns how many bytes were queued.
    pub fn try_write(&mut self, buf: &[u8]) -> usize {
        critical_section::with(|_| {
            let n = self.free().min(buf.len());
            if n > 0 {
                self.enqueue(&buf[..n]);
            }
            n
        })
    }

    /// Queue all of `buf` or nothing, without waiting.
    pub fn try_write_all(&mut self, buf: &[u8]) -> bool {
        critical_section::with(|_| {
            let fits = self.free() >= buf.len();
            if fits {
                self.enqueue(buf);
            }
            fits
        })
    }

    /// Queue all of `buf` (at most 64 bytes) at once, waiting until it fits. Cancel-safe: nothing is queued
    /// unless all of it is.
    pub async fn write_atomic(&mut self, buf: &[u8]) {
        assert!(buf.len() <= TX_LEN);
        poll_fn(|cx| {
            if self.try_write_all(buf) {
                return Poll::Ready(());
            }
            self.state.tx_wakers[self.w].register(cx.waker());
            if self.try_write_all(buf) { Poll::Ready(()) } else { Poll::Pending }
        })
        .await
    }
}

impl ErrorType for BufferedUartTx<'_> {
    type Error = Infallible;
}

impl Write for BufferedUartTx<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Infallible> {
        if buf.is_empty() {
            return Ok(0);
        }
        poll_fn(|cx| {
            let n = self.try_write(buf);
            if n > 0 {
                return Poll::Ready(Ok(n));
            }
            self.state.tx_wakers[self.w].register(cx.waker());
            let n = self.try_write(buf);
            if n > 0 { Poll::Ready(Ok(n)) } else { Poll::Pending }
        })
        .await
    }

    async fn flush(&mut self) -> Result<(), Infallible> {
        poll_fn(|cx| {
            if self.free() != TX_LEN {
                self.state.tx_wakers[self.w].register(cx.waker());
                if self.free() != TX_LEN {
                    return Poll::Pending;
                }
            }
            Poll::Ready(())
        })
        .await;
        // the last byte is still shifting out (at most one byte time): yield rather than spin
        poll_fn(|cx| {
            if self.r.usr().read().txemp() {
                Poll::Ready(())
            } else {
                cx.waker().wake_by_ref();
                Poll::Pending
            }
        })
        .await;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::interrupt::typelevel;
    use crate::peripherals::{DMA_CH36, PB0, PC7, UART9};

    #[derive(Copy, Clone)]
    struct Irqs;
    unsafe impl Binding<typelevel::UART9, BufferedInterruptHandler<UART9>> for Irqs {}
    unsafe impl Binding<typelevel::EDMA_INTR_INT36, RxDmaInterruptHandler<UART9>> for Irqs {}

    #[test]
    fn a_rejected_baud_rate_leaves_the_port_free() {
        crate::clocks::set(crate::clocks::Clocks { vco_hz: 500_000_000, core_hz: 250_000_000, bus_hz: 125_000_000 });
        // twice: the first failure must not leave UART9 marked as started (and must touch no register)
        for _ in 0..2 {
            let r = unsafe {
                BufferedUart::new(UART9::steal(), DMA_CH36::steal(), PB0::steal(), PC7::steal(), Irqs, Config::new(0))
            };
            assert!(matches!(r, Err(ConfigError::BaudrateTooLow)));
        }
    }

    #[test]
    fn an_unsplit_port_reads_and_writes() {
        fn rw<T: Read + Write>() {}
        rw::<BufferedUart<'static>>();
    }
}
