//! Full-duplex streaming: TX and RX rings of two blocks each, run by eDMA in lockstep off one frame sync. Each TX
//! half or full transfer pends a software interrupt at the app's level; there the newest `FRAMES` received frames are
//! copied out and handed to `render` with the TX half the DMA isn't reading.
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU32, AtomicUsize, Ordering::Relaxed};

use super::{Instance, Ssi, CR_RE, CR_SSI_EN, CR_TE};
use crate::dma::{self, attr, csr, Channel, ChannelInstance, Size, Tcd};
use crate::interrupt::typelevel::{Binding, Handler, Interrupt as _};
use crate::interrupt::{InterruptExt, Priority, SoftIrq};
use crate::pac::ssi0::regs;
use crate::Peri;

/// `FRAMES` frames of `SLOTS` raw FIFO words (received words are sign-extended when the config asks for it).
pub type Block<const FRAMES: usize, const SLOTS: usize> = [[u32; SLOTS]; FRAMES];

/// A block's words as signed samples.
pub fn as_i32<const F: usize, const S: usize>(b: &Block<F, S>) -> &[[i32; S]; F] {
    // SAFETY: u32 and i32 have the same size and alignment
    unsafe { &*(b as *const Block<F, S> as *const [[i32; S]; F]) }
}

/// A block's words as signed samples, to fill.
pub fn as_i32_mut<const F: usize, const S: usize>(b: &mut Block<F, S>) -> &mut [[i32; S]; F] {
    // SAFETY: as `as_i32`
    unsafe { &mut *(b as *mut Block<F, S> as *mut [[i32; S]; F]) }
}

/// The two rings the eDMA runs, two blocks each. The app owns them (`static mut`) and hands them over for as long as
/// the stream runs. The data cache must be off.
#[repr(C, align(16))]
pub struct Rings<const FRAMES: usize, const SLOTS: usize> {
    tx: [Block<FRAMES, SLOTS>; 2],
    rx: [Block<FRAMES, SLOTS>; 2],
}

impl<const F: usize, const S: usize> Rings<F, S> {
    pub const fn new() -> Self {
        Self { tx: [[[0; S]; F]; 2], rx: [[[0; S]; F]; 2] }
    }
}

impl<const F: usize, const S: usize> Default for Rings<F, S> {
    fn default() -> Self {
        Self::new()
    }
}

/// Counters since the stream started.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    /// Blocks rendered.
    pub blocks: u32,
    /// Blocks the DMA started reading before `render` finished them.
    pub late: u32,
    /// Blocks skipped: two DMA interrupts were served by one render.
    pub skipped: u32,
    /// TX DMA half/full interrupts taken.
    pub dma_irqs: u32,
}

/// One SSI's stream state, shared by its two interrupts.
pub struct State {
    streaming: AtomicBool,
    isr: AtomicUsize,
    render: AtomicUsize,
    rings: AtomicUsize,
    soft: AtomicU16,
    tx_ch: AtomicU8,
    rx_ch: AtomicU8,
    last_half: AtomicU8,
    blocks: AtomicU32,
    late: AtomicU32,
    skipped: AtomicU32,
    dma_irqs: AtomicU32,
}

impl State {
    pub const fn new() -> Self {
        Self {
            streaming: AtomicBool::new(false),
            isr: AtomicUsize::new(0),
            render: AtomicUsize::new(0),
            rings: AtomicUsize::new(0),
            soft: AtomicU16::new(0),
            tx_ch: AtomicU8::new(0),
            rx_ch: AtomicU8::new(0),
            last_half: AtomicU8::new(NONE),
            blocks: AtomicU32::new(0),
            late: AtomicU32::new(0),
            skipped: AtomicU32::new(0),
            dma_irqs: AtomicU32::new(0),
        }
    }

    fn soft_irq(&self) -> SoftIrq {
        let v = self.soft.load(Relaxed) - 64;
        SoftIrq::new((v / 64) as u8, (v % 64) as u8)
    }

    /// The render interrupt's body (called from the instance's trampoline).
    #[cfg_attr(not(target_arch = "m68k"), allow(dead_code))]
    pub(crate) fn run_isr(&self) {
        let f = self.isr.load(Relaxed);
        if f != 0 {
            // SAFETY: set by `stream` to a `fn()` before the interrupt is enabled
            let f: fn() = unsafe { core::mem::transmute(f) };
            f();
        }
    }
}

impl Default for State {
    fn default() -> Self {
        Self::new()
    }
}

const NONE: u8 = 0xFF;

/// The TX descriptor: the ring into the transmit FIFO, one frame per request, interrupts at half and end.
pub(crate) const fn tx_tcd(ring: u32, fifo: u32, frames: usize, slots: usize) -> Tcd {
    let bytes = (2 * frames * slots * 4) as i32;
    Tcd {
        saddr: ring,
        soff: 4,
        attr: attr(Size::Bits32, 0, Size::Bits32, 0),
        nbytes: (slots * 4) as u32,
        slast: -bytes,
        daddr: fifo,
        doff: 0,
        citer: (2 * frames) as u16,
        dlast_sga: 0,
        biter: (2 * frames) as u16,
        csr: csr::INT_HALF | csr::INT_MAJOR,
    }
}

/// The RX descriptor: the receive FIFO into the ring, one frame per request, no interrupts (read by position).
pub(crate) const fn rx_tcd(fifo: u32, ring: u32, frames: usize, slots: usize) -> Tcd {
    let bytes = (2 * frames * slots * 4) as i32;
    Tcd {
        saddr: fifo,
        soff: 0,
        attr: attr(Size::Bits32, 0, Size::Bits32, 0),
        nbytes: (slots * 4) as u32,
        slast: 0,
        daddr: ring,
        doff: 4,
        citer: (2 * frames) as u16,
        dlast_sga: -bytes,
        biter: (2 * frames) as u16,
        csr: 0,
    }
}

/// The TX half to fill: the one the DMA (at `saddr`) isn't reading.
pub(crate) fn tx_fill_half(saddr: u32, ring: u32, half_bytes: u32) -> usize {
    ((saddr.wrapping_sub(ring) / half_bytes) & 1) as usize ^ 1
}

/// Where the newest `frames` complete frames start in the `2 × frames` RX ring the DMA writes at `daddr`, and how
/// many of them come before the wrap.
pub(crate) fn rx_newest(daddr: u32, ring: u32, frames: usize, slots: usize) -> (usize, usize) {
    let len = 2 * frames;
    let pos = (daddr.wrapping_sub(ring) as usize / (4 * slots)) % len;
    let start = (pos + len - frames) % len;
    (start, frames.min(len - start))
}

/// (late, skipped) for a render that filled `fill` after the previous one filled `prev`, with the DMA reading
/// `reading_after` once it was done.
pub(crate) fn note(prev: Option<usize>, fill: usize, reading_after: usize) -> (bool, bool) {
    (reading_after == fill, prev == Some(fill))
}

/// Copy `out.len()` frames from `ring` starting at frame `start`, wrapping.
pub(crate) fn copy_window<const S: usize>(ring: &[[u32; S]], start: usize, out: &mut [[u32; S]]) {
    for (i, f) in out.iter_mut().enumerate() {
        *f = ring[(start + i) % ring.len()];
    }
}

/// The TX DMA interrupt: bind it with `bind_interrupts!` to the instance's TX channel interrupt (SSI1's is
/// `EDMA_INTR_INT54`).
pub struct TxDmaInterruptHandler<T: Instance> {
    _p: core::marker::PhantomData<T>,
}

impl<T: Instance> Handler<<T::TxDma as ChannelInstance>::Interrupt> for TxDmaInterruptHandler<T> {
    unsafe fn on_interrupt() {
        let st = T::state();
        dma::clear_interrupt(<T::TxDma as ChannelInstance>::NUMBER);
        st.dma_irqs.fetch_add(1, Relaxed);
        st.soft_irq().pend();
    }
}

fn render_isr<T: Instance, const F: usize, const S: usize>() {
    let st = T::state();
    st.soft_irq().unpend();
    // SAFETY: `stream` stored these for this monomorphisation and keeps them valid while streaming
    let rings = unsafe { &mut *(st.rings.load(Relaxed) as *mut Rings<F, S>) };
    let render: fn(&Block<F, S>, &mut Block<F, S>) = unsafe { core::mem::transmute(st.render.load(Relaxed)) };
    let tx_base = rings.tx.as_ptr() as u32;
    let half = (F * S * 4) as u32;
    let tx_ch = st.tx_ch.load(Relaxed);
    let fill = tx_fill_half(dma::saddr(tx_ch), tx_base, half);
    let prev = st.last_half.swap(fill as u8, Relaxed);
    let (start, _) = rx_newest(dma::daddr(st.rx_ch.load(Relaxed)), rings.rx.as_ptr() as u32, F, S);
    let mut input: Block<F, S> = [[0; S]; F];
    // SAFETY: the two RX blocks are 2 × F contiguous frames
    let rx = unsafe { core::slice::from_raw_parts(rings.rx.as_ptr() as *const [u32; S], 2 * F) };
    copy_window(rx, start, &mut input);
    render(&input, &mut rings.tx[fill]);
    let reading_after = tx_fill_half(dma::saddr(tx_ch), tx_base, half) ^ 1;
    let (late, skipped) = note(if prev == NONE { None } else { Some(prev as usize) }, fill, reading_after);
    if late {
        st.late.fetch_add(1, Relaxed);
    }
    if skipped {
        st.skipped.fetch_add(1, Relaxed);
    }
    st.blocks.fetch_add(1, Relaxed);
}

/// A running stream; `stop` gives the `Ssi` back.
pub struct Stream<'d, T: Instance, const FRAMES: usize, const SLOTS: usize> {
    ssi: Ssi<'d, T>,
    tx: Channel<'d>,
    rx: Channel<'d>,
}

impl<'d, T: Instance> Ssi<'d, T> {
    /// Stream both directions. `render(input, output)` runs in `render_irq` at `render_level` after each TX half or
    /// full transfer (whose DMA interrupt runs at `dma_level`): `input` holds the newest `FRAMES` received frames,
    /// `output` the TX half to fill. Panics if `T` is already streaming.
    #[allow(clippy::too_many_arguments)]
    pub fn stream<const F: usize, const S: usize>(
        self,
        tx: Peri<'d, T::TxDma>,
        rx: Peri<'d, T::RxDma>,
        _irqs: impl Binding<<T::TxDma as ChannelInstance>::Interrupt, TxDmaInterruptHandler<T>>,
        rings: &'static mut Rings<F, S>,
        dma_level: Priority,
        render_irq: SoftIrq,
        render_level: Priority,
        render: fn(&Block<F, S>, &mut Block<F, S>),
    ) -> Stream<'d, T, F, S> {
        let st = T::state();
        assert!(!st.streaming.swap(true, Relaxed), "ssi: this SSI is already streaming");
        *rings = Rings::new();
        st.isr.store(render_isr::<T, F, S> as fn() as usize, Relaxed);
        st.render.store(render as usize, Relaxed);
        st.rings.store(rings as *mut Rings<F, S> as usize, Relaxed);
        st.soft.store(render_irq.vector(), Relaxed);
        st.tx_ch.store(<T::TxDma as ChannelInstance>::NUMBER, Relaxed);
        st.rx_ch.store(<T::RxDma as ChannelInstance>::NUMBER, Relaxed);
        st.last_half.store(NONE, Relaxed);
        for c in [&st.blocks, &st.late, &st.skipped, &st.dma_irqs] {
            c.store(0, Relaxed);
        }
        let r = T::REGS;
        let fifo_tx = r.tx0().as_ptr() as u32;
        let fifo_rx = r.rx0().as_ptr() as u32;
        let mut txc = Channel::new(tx);
        let mut rxc = Channel::new(rx);
        // SAFETY: the rings are 'static and the FIFOs are the SSI's own; both stay valid until `stop`
        unsafe {
            txc.configure(&tx_tcd(rings.tx.as_ptr() as u32, fifo_tx, F, S));
            rxc.configure(&rx_tcd(fifo_rx, rings.rx.as_ptr() as u32, F, S));
        }
        #[cfg(target_arch = "m68k")]
        // SAFETY: the vector is the app's chosen software interrupt, handled by this instance's trampoline
        unsafe {
            coldfire_rt::set_vector(render_irq.vector() as usize, <T as super::sealed::Instance>::render_entry());
        }
        render_irq.set_priority(render_level);
        <T::TxDma as ChannelInstance>::Interrupt::set_priority(dma_level);
        // SAFETY: both handlers are in place (the TX DMA's through the app's binding)
        unsafe {
            render_irq.enable();
            <T::TxDma as ChannelInstance>::Interrupt::enable();
        }
        let mut ssi = self;
        if r.cr().read().0 & CR_SSI_EN == 0 {
            r.cr().write_value(regs::Cr(ssi.cr | CR_SSI_EN));
            for _ in 0..super::prime_words(S) {
                r.tx0().write_value(regs::Tx0(0));
            }
        }
        rxc.enable_request();
        txc.enable_request();
        r.cr().write_value(regs::Cr(ssi.cr | CR_SSI_EN | CR_TE | CR_RE));
        ssi.cr &= !(CR_SSI_EN | CR_TE | CR_RE);
        Stream { ssi, tx: txc, rx: rxc }
    }
}

impl<'d, T: Instance, const F: usize, const S: usize> Stream<'d, T, F, S> {
    pub fn stats(&self) -> Stats {
        let st = T::state();
        Stats {
            blocks: st.blocks.load(Relaxed),
            late: st.late.load(Relaxed),
            skipped: st.skipped.load(Relaxed),
            dma_irqs: st.dma_irqs.load(Relaxed),
        }
    }

    /// Stop both directions and the SSI, mask the render interrupt, and give the `Ssi` back (`stream` can run again).
    pub fn stop(mut self) -> Ssi<'d, T> {
        let st = T::state();
        self.tx.disable_request();
        self.rx.disable_request();
        <T::TxDma as ChannelInstance>::Interrupt::disable();
        st.soft_irq().disable();
        self.ssi.stop();
        st.isr.store(0, Relaxed);
        st.streaming.store(false, Relaxed);
        self.ssi
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dma::csr;

    const RING: u32 = 0x8000_1000;
    const FIFO: u32 = 0xFC0C_8000;

    #[test]
    fn tx_tcd_for_32_frames_2_slots() {
        let t = tx_tcd(RING, FIFO, 32, 2);
        assert_eq!((t.saddr, t.soff, t.nbytes, t.slast), (RING, 4, 8, -512));
        assert_eq!((t.daddr, t.doff, t.dlast_sga), (FIFO, 0, 0));
        assert_eq!((t.citer, t.biter), (64, 64));
        assert_eq!(t.csr, csr::INT_HALF | csr::INT_MAJOR);
    }

    #[test]
    fn rx_tcd_mirrors_tx_without_interrupts() {
        let t = rx_tcd(FIFO + 8, RING, 32, 2);
        assert_eq!((t.saddr, t.soff, t.slast), (FIFO + 8, 0, 0));
        assert_eq!((t.daddr, t.doff, t.dlast_sga, t.nbytes), (RING, 4, -512, 8));
        assert_eq!((t.citer, t.biter, t.csr), (64, 64, 0));
    }

    #[test]
    fn fill_half_is_the_other_one() {
        assert_eq!(tx_fill_half(RING, RING, 256), 1);
        assert_eq!(tx_fill_half(RING + 252, RING, 256), 1);
        assert_eq!(tx_fill_half(RING + 256, RING, 256), 0);
        assert_eq!(tx_fill_half(RING + 511, RING, 256), 0);
    }

    #[test]
    fn rx_newest_without_wrap() {
        // DADDR at frame 40 of the 64-frame ring: the newest 32 complete frames are 8..40
        assert_eq!(rx_newest(RING + 40 * 8, RING, 32, 2), (8, 32));
    }

    #[test]
    fn rx_newest_across_the_wrap() {
        // DADDR at frame 10: frames 42..64, then 0..10
        assert_eq!(rx_newest(RING + 10 * 8, RING, 32, 2), (42, 22));
        // mid-frame (the minor loop half done): only complete frames count
        assert_eq!(rx_newest(RING + 10 * 8 + 4, RING, 32, 2), (42, 22));
    }

    #[test]
    fn stats_count_late_and_skipped() {
        // the same half twice: an interrupt was merged and a block skipped
        assert_eq!(note(Some(1), 1, 0), (false, true));
        // the DMA reached the half before it was finished
        assert_eq!(note(Some(0), 1, 1), (true, false));
        assert_eq!(note(None, 1, 0), (false, false));
    }

    #[test]
    fn copy_window_wraps() {
        let mut ring = [[0u32; 2]; 64];
        for (i, f) in ring.iter_mut().enumerate() {
            *f = [i as u32, 100 + i as u32];
        }
        let mut out = [[0u32; 2]; 32];
        copy_window(&ring, 42, &mut out);
        assert_eq!(out[0], [42, 142]);
        assert_eq!(out[21], [63, 163]);
        assert_eq!(out[22], [0, 100]);
        assert_eq!(out[31], [9, 109]);
    }
}
