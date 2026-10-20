//! Full-duplex streaming: TX and RX rings of two blocks each, run by eDMA in lockstep off one frame sync. Each TX
//! half or full transfer pends a software interrupt at the app's level; there the newest `FRAMES` received frames are
//! copied out and handed to `render` with the TX half the DMA isn't reading.
use core::ptr::NonNull;
use core::sync::atomic::{compiler_fence, AtomicBool, AtomicU8, AtomicU16, AtomicU32, AtomicUsize, Ordering::{Relaxed, SeqCst}};

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

/// The render vector each instance's stream holds (0: none), so two streams can't share one.
static CLAIMED: [AtomicU16; 2] = [AtomicU16::new(0), AtomicU16::new(0)];

/// `vector` is held by another instance's stream than `index`.
pub(crate) fn claimed_elsewhere(claimed: &[u16; 2], index: usize, vector: u16) -> bool {
    claimed.iter().enumerate().any(|(i, &v)| i != index && v == vector)
}

/// Streaming moves a frame (`S` words) per DMA request: it needs the `Config`'s slot count, room for a frame in the
/// FIFO, and watermarks of at least a frame (or TX writes past a full FIFO and RX reads an empty one, and the slots
/// rotate).
pub(crate) const fn check_geometry(slots: usize, tx_watermark: u8, rx_watermark: u8, s: usize) {
    if s != slots {
        panic!("ssi: the Block's slots differ from the Config's");
    }
    if s > super::FIFO_DEPTH {
        panic!("ssi: a frame of more than 15 words doesn't fit the FIFO");
    }
    if (tx_watermark as usize) < s || (rx_watermark as usize) < s {
        panic!("ssi: FIFO watermarks must be at least a frame (slots) to stream");
    }
}
/// The end-of-frame wait when the stream takes over running clocks: well over a frame at any rate the SSI's clocks
/// reach (the wait only bounds a stalled external clock).
const TFRC_SPINS: u32 = 1_000_000;

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

/// Copy `out.len()` frames from the `len`-frame ring at `ring` starting at frame `start`, wrapping. Volatile: the
/// DMA writes the ring meanwhile (never the frames in the window).
///
/// # Safety
/// `ring` must point to `len` readable frames.
pub(crate) unsafe fn copy_window<const S: usize>(ring: *const [u32; S], len: usize, start: usize, out: &mut [[u32; S]]) {
    for (i, f) in out.iter_mut().enumerate() {
        // SAFETY: the index is below `len`
        *f = unsafe { ring.add((start + i) % len).read_volatile() };
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
    // Raw pointers only: the DMA reads one TX half and writes the RX ring throughout, so no reference to a whole ring
    // is formed. SAFETY: `stream` stored these for this monomorphisation and keeps them valid while streaming
    let rings = st.rings.load(Relaxed) as *mut Rings<F, S>;
    let render: fn(&Block<F, S>, &mut Block<F, S>) = unsafe { core::mem::transmute(st.render.load(Relaxed)) };
    let tx = unsafe { core::ptr::addr_of_mut!((*rings).tx) } as *mut Block<F, S>;
    let rx = unsafe { core::ptr::addr_of!((*rings).rx) } as *const [u32; S];
    let tx_base = tx as u32;
    let half = (F * S * 4) as u32;
    let tx_ch = st.tx_ch.load(Relaxed);
    let fill = tx_fill_half(dma::saddr(tx_ch), tx_base, half);
    let prev = st.last_half.swap(fill as u8, Relaxed);
    let (start, _) = rx_newest(dma::daddr(st.rx_ch.load(Relaxed)), rx as u32, F, S);
    let mut input: Block<F, S> = [[0; S]; F];
    // SAFETY: the two RX blocks are 2 × F contiguous frames
    unsafe { copy_window(rx, 2 * F, start, &mut input) };
    // SAFETY: the half the DMA isn't reading; only this interrupt writes it
    render(&input, unsafe { &mut *tx.add(fill) });
    compiler_fence(SeqCst); // the block is written before the DMA's position is read again
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

/// A running stream; `stop` gives the `Ssi` and the rings back, and dropping it stops it too.
#[must_use = "dropping the stream stops it"]
pub struct Stream<'d, T: Instance, const FRAMES: usize, const SLOTS: usize> {
    ssi: Ssi<'d, T>,
    tx: Channel<'d>,
    rx: Channel<'d>,
    rings: NonNull<Rings<FRAMES, SLOTS>>,
}

impl<'d, T: Instance> Ssi<'d, T> {
    /// Stream both directions. `render(input, output)` runs in `render_irq` at `render_level` after each TX half or
    /// full transfer (whose DMA interrupt runs at `dma_level`): `input` holds the newest `FRAMES` received frames,
    /// `output` the TX half to fill. `render_irq`'s vector becomes this stream's: nothing else may use that source.
    /// Panics if `T` is already streaming or another SSI's stream holds `render_irq`.
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
        check_geometry(self.slots, self.tx_watermark, self.rx_watermark, S);
        let st = T::state();
        assert!(!st.streaming.swap(true, Relaxed), "ssi: SSI{} is already streaming", T::INDEX);
        let index = T::INDEX as usize;
        let claimed = [CLAIMED[0].load(Relaxed), CLAIMED[1].load(Relaxed)];
        if claimed_elsewhere(&claimed, index, render_irq.vector()) {
            st.streaming.store(false, Relaxed);
            panic!("ssi: another SSI's stream renders on this SoftIrq");
        }
        CLAIMED[index].store(render_irq.vector(), Relaxed);
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
        compiler_fence(SeqCst); // the state above is in place before either interrupt can run
        render_irq.set_priority(render_level);
        <T::TxDma as ChannelInstance>::Interrupt::set_priority(dma_level);
        // SAFETY: both handlers are in place (the TX DMA's through the app's binding)
        unsafe {
            render_irq.enable();
            <T::TxDma as ChannelInstance>::Interrupt::enable();
        }
        let mut ssi = self;
        let cr = r.cr().read().0;
        if cr & CR_SSI_EN == 0 {
            r.cr().write_value(regs::Cr(ssi.cr | CR_SSI_EN));
        } else if cr & CR_TE != 0 {
            // The clocks are running (`start_clocks`) and the transmitter has long run dry. Its FIFO has no slot tags,
            // so frames fed to it now would start in whichever slot comes next: stop it at the end of this frame (the
            // clocks keep running with TCD clear) and let TE start it again on a frame boundary.
            r.cr().write_value(regs::Cr(ssi.cr | CR_SSI_EN));
            let mut spins = 0;
            while !r.isr().read().tfrc() && spins < TFRC_SPINS {
                spins += 1;
            }
        }
        let queued = r.fcsr().read().tfcnt0() as usize;
        for _ in queued..super::prime_words(S) {
            r.tx0().write_value(regs::Tx0(0));
        }
        rxc.enable_request();
        txc.enable_request();
        r.cr().write_value(regs::Cr(ssi.cr | CR_SSI_EN | CR_TE | CR_RE));
        ssi.cr &= !(CR_SSI_EN | CR_TE | CR_RE);
        Stream { ssi, tx: txc, rx: rxc, rings: NonNull::from(rings) }
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

    /// Stop both directions and the SSI, mask and clear both interrupts, and give the `Ssi` and the rings back
    /// (`stream` can run again).
    pub fn stop(self) -> (Ssi<'d, T>, &'static mut Rings<F, S>) {
        let mut s = core::mem::ManuallyDrop::new(self);
        s.halt();
        // SAFETY: `s` is never dropped or used again: its fields move out once. The rings came in as `&'static mut`
        // and nothing else refers to them now that the DMA and the render interrupt are off.
        unsafe {
            let ssi = core::ptr::read(&s.ssi);
            core::ptr::drop_in_place(&mut s.tx);
            core::ptr::drop_in_place(&mut s.rx);
            (ssi, &mut *s.rings.as_ptr())
        }
    }

    fn halt(&mut self) {
        let st = T::state();
        self.tx.disable_request();
        self.rx.disable_request();
        <T::TxDma as ChannelInstance>::Interrupt::disable();
        st.soft_irq().disable();
        self.ssi.stop();
        // a half or end that landed during the above stays pending: clear it, or a restart takes a stray block
        self.tx.clear_interrupt();
        st.soft_irq().unpend();
        st.isr.store(0, Relaxed);
        CLAIMED[T::INDEX as usize].store(0, Relaxed);
        compiler_fence(SeqCst);
        st.streaming.store(false, Relaxed);
    }
}

impl<'d, T: Instance, const F: usize, const S: usize> Drop for Stream<'d, T, F, S> {
    fn drop(&mut self) {
        self.halt();
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::dma::csr;

    const RING: u32 = 0x4010_0000;
    const FIFO: u32 = 0xFC0C_8000;

    #[test]
    fn tx_tcd_for_48_frames_2_slots() {
        let t = tx_tcd(RING, FIFO, 48, 2);
        assert_eq!((t.saddr, t.soff, t.nbytes, t.slast), (RING, 4, 8, -768));
        assert_eq!((t.daddr, t.doff, t.dlast_sga), (FIFO, 0, 0));
        assert_eq!((t.citer, t.biter), (96, 96));
        assert_eq!(t.csr, csr::INT_HALF | csr::INT_MAJOR);
    }

    #[test]
    fn rx_tcd_mirrors_tx_without_interrupts() {
        let t = rx_tcd(FIFO + 8, RING, 48, 2);
        assert_eq!((t.saddr, t.soff, t.slast), (FIFO + 8, 0, 0));
        assert_eq!((t.daddr, t.doff, t.dlast_sga, t.nbytes), (RING, 4, -768, 8));
        assert_eq!((t.citer, t.biter, t.csr), (96, 96, 0));
    }

    fn geometry_refusal(slots: usize, tx_wm: u8, rx_wm: u8, s: usize) -> std::string::String {
        let r = std::panic::catch_unwind(|| check_geometry(slots, tx_wm, rx_wm, s));
        std::string::String::from(*r.unwrap_err().downcast::<&str>().unwrap())
    }

    #[test]
    fn a_render_vector_is_held_by_one_stream() {
        assert!(!claimed_elsewhere(&[0, 0], 1, 64 + 63));
        assert!(!claimed_elsewhere(&[64 + 63, 0], 0, 64 + 63));               // its own earlier claim (a restart)
        assert!(claimed_elsewhere(&[64 + 63, 0], 1, 64 + 63));                // SSI0 holds it
        assert!(!claimed_elsewhere(&[64 + 62, 0], 1, 64 + 63));
    }

    #[test]
    fn stream_geometry_must_fit_a_frame_per_request() {
        check_geometry(2, 2, 2, 2);
        check_geometry(4, 6, 4, 4);                                            // more room than a frame is fine
        assert!(geometry_refusal(2, 2, 2, 4).contains("slots"));              // Block's S against the Config
        assert!(geometry_refusal(4, 2, 4, 4).contains("watermark"));          // TX: 4 words into 2 free
        assert!(geometry_refusal(4, 4, 2, 4).contains("watermark"));          // RX: 4 words out of 2
        assert!(geometry_refusal(16, 15, 15, 16).contains("FIFO"));
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
        // SAFETY: the ring is 64 frames
        unsafe { copy_window(ring.as_ptr(), ring.len(), 42, &mut out) };
        assert_eq!(out[0], [42, 142]);
        assert_eq!(out[21], [63, 163]);
        assert_eq!(out[22], [0, 100]);
        assert_eq!(out[31], [9, 109]);
    }
}
