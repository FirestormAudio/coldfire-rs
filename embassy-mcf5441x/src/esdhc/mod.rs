//! eSDHC (RM ch. 25) with one eMMC behind it, read-only: a 4-bit high-speed bus, CMD18 multi-block reads that the
//! eDMA moves from DATPORT (the controller's request is fixed to channel 59), and completion on the eSDHC
//! interrupt. Bring-up is polled with bounded waits (the time driver's 1 kHz clock, plus a poll-count backstop);
//! reads are async and cancel-safe.
//!
//! Read-only by construction: the driver can send only the commands in `cmd::Cmd`, and none of them programs,
//! erases or reconfigures the card.
//!
//! Data lands as the controller delivers 32-bit DATPORT words in its reset (address-invariant) endian mode:
//! EXT_CSD byte `i` is at `i ^ 3` of the received block, which [`Esdhc::ext_csd`] undoes. The eDMA assumes the data
//! cache is off, and buffers must be memory it can write.
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, AtomicU32, Ordering::*};
use core::task::Poll;

use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

use crate::clocks::{self, Gate};
use crate::dma::{self, Channel, Size, Tcd};
use crate::interrupt::typelevel::{self, Binding, Handler, Interrupt as _};
use crate::interrupt::Priority;
use crate::pac::{self, esdhc::regs};
use crate::{peripherals, pinmux};

mod cmd;
#[rustfmt::skip]
mod pins;

use cmd::*;
pub use cmd::{Divider, Error};

/// The eDMA channel the eSDHC's buffer requests on.
const CH: u8 = 59;
/// Reads go out in pieces of at most this many bytes, a size proven on hardware.
const MAX_PIECE: usize = 64 * 1024;
/// Most register waits, in milliseconds.
const WAIT_MS: u64 = 10;
/// The card's power-up (CMD1 busy) may take up to a second.
const OCR_MS: u64 = 1000;
/// How long a CMD6 SWITCH may keep the card busy (DAT0 low). EXT_CSD's GENERIC_CMD6_TIME isn't read yet, so this is
/// generous.
const SWITCH_MS: u64 = 500;
/// A poll-count backstop, should the time base not be running (interrupts masked, or no time driver).
const MAX_POLLS: u32 = 50_000_000;

const R: pac::esdhc::Esdhc = pac::ESDHC;
/// 4-word read and write watermarks: one 16-byte eDMA minor loop.
const WATERMARKS: u32 = 0x0004_0004;

static WAKER: AtomicWaker = AtomicWaker::new();
/// The IRQSTAT bits the interrupt saw (0: not yet).
static STATUS: AtomicU32 = AtomicU32::new(0);

/// Settings.
#[non_exhaustive]
#[derive(Copy, Clone, Debug)]
pub struct Config {
    /// The SD clock during identification (must be at most 400 kHz).
    pub ident_clock: Divider,
    /// The SD clock once the card is in high-speed timing.
    pub data_clock: Divider,
    /// Interrupt level of the eSDHC interrupt.
    pub priority: Priority,
}

impl Config {
    /// The two SD clocks, interrupt level 4. The dividers depend on the board's eSDHC base clock (PLL OUTDIV3 or the
    /// oscillator, MISCCR SDHCSRC): identification must stay at or below 400 kHz, and the data clock within what the
    /// card's timing allows (eMMC high-speed: 52 MHz; SD cards: 25 MHz).
    pub const fn new(ident_clock: Divider, data_clock: Divider) -> Self {
        Self { ident_clock, data_clock, priority: Priority::P4 }
    }
}

pub(crate) mod sealed {
    use crate::pinmux::{Function, ParField};

    pub trait ClkPin {
        const PAR: ParField;
        const FUNCTION: Function;
    }
    pub trait CmdPin {
        const PAR: ParField;
        const FUNCTION: Function;
    }
    pub trait Dat0Pin {
        const PAR: ParField;
        const FUNCTION: Function;
    }
    pub trait Dat1Pin {
        const PAR: ParField;
        const FUNCTION: Function;
    }
    pub trait Dat2Pin {
        const PAR: ParField;
        const FUNCTION: Function;
    }
    pub trait Dat3Pin {
        const PAR: ParField;
        const FUNCTION: Function;
    }
}

/// A pad that can carry SDHC_CLK.
#[allow(private_bounds)]
pub trait ClkPin: sealed::ClkPin + PeripheralType {}
/// A pad that can carry SDHC_CMD.
#[allow(private_bounds)]
pub trait CmdPin: sealed::CmdPin + PeripheralType {}
/// A pad that can carry SDHC_DAT0.
#[allow(private_bounds)]
pub trait Dat0Pin: sealed::Dat0Pin + PeripheralType {}
/// A pad that can carry SDHC_DAT1.
#[allow(private_bounds)]
pub trait Dat1Pin: sealed::Dat1Pin + PeripheralType {}
/// A pad that can carry SDHC_DAT2.
#[allow(private_bounds)]
pub trait Dat2Pin: sealed::Dat2Pin + PeripheralType {}
/// A pad that can carry SDHC_DAT3.
#[allow(private_bounds)]
pub trait Dat3Pin: sealed::Dat3Pin + PeripheralType {}

/// Select the six pads (each its own PAR field; the registers' other pads keep theirs).
fn mux<C: ClkPin, M: CmdPin, D0: Dat0Pin, D1: Dat1Pin, D2: Dat2Pin, D3: Dat3Pin>(
    _: &Peri<'_, C>,
    _: &Peri<'_, M>,
    _: &Peri<'_, D0>,
    _: &Peri<'_, D1>,
    _: &Peri<'_, D2>,
    _: &Peri<'_, D3>,
) {
    pinmux::set_function(<C as sealed::ClkPin>::PAR, <C as sealed::ClkPin>::FUNCTION);
    pinmux::set_function(<M as sealed::CmdPin>::PAR, <M as sealed::CmdPin>::FUNCTION);
    pinmux::set_function(<D0 as sealed::Dat0Pin>::PAR, <D0 as sealed::Dat0Pin>::FUNCTION);
    pinmux::set_function(<D1 as sealed::Dat1Pin>::PAR, <D1 as sealed::Dat1Pin>::FUNCTION);
    pinmux::set_function(<D2 as sealed::Dat2Pin>::PAR, <D2 as sealed::Dat2Pin>::FUNCTION);
    pinmux::set_function(<D3 as sealed::Dat3Pin>::PAR, <D3 as sealed::Dat3Pin>::FUNCTION);
}

/// Milliseconds from the time driver, or 0 without one (then only `MAX_POLLS` bounds a wait).
fn now_ms() -> u64 {
    #[cfg(all(target_arch = "m68k", feature = "_time-driver"))]
    {
        embassy_time_driver::now()
    }
    #[cfg(not(all(target_arch = "m68k", feature = "_time-driver")))]
    {
        0
    }
}

/// A read buffer at `addr` must take 32-bit eDMA writes: an unaligned destination is an eDMA configuration error, the
/// controller's buffer then never drains and the read would wait forever.
const fn check_buffer(addr: usize) -> Result<(), Error> {
    if addr % 4 == 0 { Ok(()) } else { Err(Error::Misaligned) }
}

/// Poll `done` until it holds or `ms` milliseconds pass.
fn spin(ms: u64, mut done: impl FnMut() -> bool) -> Result<(), Error> {
    let end = now_ms() + ms + 1;
    for _ in 0..MAX_POLLS {
        if done() {
            return Ok(());
        }
        if now_ms() >= end {
            break;
        }
    }
    Err(Error::Timeout)
}

fn prsstat() -> u32 {
    R.prsstat().read().0
}

fn irqstat() -> u32 {
    R.irqstat().read().0
}

fn sysctl() -> u32 {
    R.sysctl().read().0
}

fn set_sysctl(v: u32) {
    R.sysctl().write_value(regs::Sysctl(v));
}

fn clear_irqstat() {
    R.irqstat().write_value(regs::Irqstat(0xFFFF_FFFF));
}

fn set_irqsigen(v: u32) {
    R.irqsigen().write_value(regs::Irqsigen(v));
}

/// Wait for the command and data lines, then issue `c`.
fn issue(c: Cmd) -> Result<(), Error> {
    let (x, arg) = c.encode();
    spin(WAIT_MS, || prsstat() & (CIHB | CDIHB) == 0)?;
    clear_irqstat();
    R.cmdarg().write_value(regs::Cmdarg(arg));
    R.xfertyp().write_value(regs::Xfertyp(x));
    Ok(())
}

/// Issue `c` and poll until it completes (`TC` too when `data`). Returns CMDRSP0.
fn cmd_polled(c: Cmd, data: bool) -> Result<u32, Error> {
    issue(c)?;
    let want = if data { CC | TC } else { CC };
    spin(WAIT_MS, || irqstat() & ERR != 0 || irqstat() & want == want)?;
    check(irqstat())?;
    Ok(R.cmdrsp0().read())
}

/// Point channel 59 at `dst`: 32-bit words from DATPORT, 16 bytes (the 4-word watermark) a request, the request
/// enable dropped when the major loop ends. No eDMA interrupt: the eSDHC's TC reports completion.
fn arm(ch: &mut Channel<'_>, dst: &mut [u8]) {
    let n = (dst.len() / 16) as u16;
    let tcd = Tcd {
        saddr: R.as_ptr() as usize as u32 + 0x20, // DATPORT
        soff: 0,
        attr: dma::attr(Size::Bits32, 0, Size::Bits32, 0),
        nbytes: 16,
        slast: 0,
        daddr: dst.as_mut_ptr() as u32,
        doff: 4,
        citer: n,
        dlast_sga: 0,
        biter: n,
        csr: dma::csr::D_REQ,
    };
    // SAFETY: the callers keep `dst` borrowed until the request is cleared (`abort`, or the end of the transfer)
    unsafe { ch.configure(&tcd) };
    ch.enable_request();
}

/// Has channel 59's major loop completed? Its last write can trail the controller's TC by a few cycles, so this gives
/// it a moment before saying no. The fence keeps the compiler from moving reads of the buffer before this check.
fn dma_done(ch: &Channel<'_>) -> bool {
    let done = (0..1000).any(|_| ch.csr() & dma::csr::DONE != 0);
    compiler_fence(SeqCst);
    done
}

/// Wait (boundedly) for a minor loop channel 59 may still be running: clearing its request stops new ones only.
fn dma_idle(ch: &Channel<'_>) {
    let _ = spin(WAIT_MS, || ch.csr() & dma::csr::ACTIVE == 0);
    compiler_fence(SeqCst);
}

/// Abandon a CMD18: stop channel 59, reset the controller's command and data sides, and send CMD12 to take the card
/// out of its data state. CMD12's own outcome is ignored: a card that had already finished answers it as an illegal
/// command.
fn abort() {
    set_irqsigen(0);
    pac::EDMA.cerq().write(|w| w.set_cerq(CH));
    set_sysctl(sysctl() | RSTC | RSTD);
    let _ = spin(WAIT_MS, || sysctl() & (RSTC | RSTD) == 0);
    // RSTD also resets PROCTL (to a 1-bit bus) and WML on this part, though the RM's list leaves them out: on hardware
    // every read after an abort then failed its CRC (2026-10-18)
    R.proctl().write_value(regs::Proctl(R.proctl().read().0 | DTW_4BIT));
    R.wml().write_value(regs::Wml(WATERMARKS));
    let _ = cmd_polled(Cmd::StopTransmission, false);
    let _ = spin(WAIT_MS, || prsstat() & DLA == 0);
    clear_irqstat();
}

/// Aborts the transfer unless forgotten: covers early returns and a future dropped mid-transfer.
struct AbortOnDrop;

impl Drop for AbortOnDrop {
    fn drop(&mut self) {
        abort();
    }
}

/// The eSDHC interrupt: records the signalled status and wakes the reader.
pub struct InterruptHandler;

impl Handler<typelevel::ESDHC> for InterruptHandler {
    unsafe fn on_interrupt() {
        let s = irqstat() & R.irqsigen().read().0;
        set_irqsigen(0); // drop the request; the reader clears IRQSTAT
        if s != 0 {
            STATUS.fetch_or(s, Release);
            WAKER.wake();
        }
    }
}

/// One 512-byte block the eDMA can fill: its 32-bit writes need a word-aligned destination (else a DAE configuration
/// error, and the controller's buffer never drains).
#[repr(C, align(4))]
struct Block([u8; 512]);

/// The eMMC, brought up and ready to read.
pub struct Esdhc<'d> {
    ch: Channel<'d>,
    ext: Block,
    sectors: u32,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> Esdhc<'d> {
    /// Mux the pads, ungate, reset the controller and bring the card up: identification at `config.ident_clock`,
    /// RCA 2, high-speed timing, `config.data_clock`, a 4-bit bus checked with CMD19/CMD14, 512-byte blocks, EXT_CSD
    /// read. Then the interrupt is enabled. Polled; takes up to about a second (the card's power-up). The waits are
    /// bounded by the time driver's clock; without a running time driver only a poll count bounds them, which for a
    /// missing card can take very long.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        _sdhc: Peri<'d, peripherals::ESDHC>,
        dma: Peri<'d, peripherals::DMA_CH59>,
        clk: Peri<'d, impl ClkPin>,
        cmd: Peri<'d, impl CmdPin>,
        dat0: Peri<'d, impl Dat0Pin>,
        dat1: Peri<'d, impl Dat1Pin>,
        dat2: Peri<'d, impl Dat2Pin>,
        dat3: Peri<'d, impl Dat3Pin>,
        _irq: impl Binding<typelevel::ESDHC, InterruptHandler>,
        config: Config,
    ) -> Result<Self, Error> {
        mux(&clk, &cmd, &dat0, &dat1, &dat2, &dat3);
        clocks::ungate(Gate::Esdhc);
        let mut ch = Channel::new(dma);

        set_sysctl(RSTA);
        spin(WAIT_MS, || sysctl() & RSTA == 0)?;
        set_sysctl(ident_sysctl(config.ident_clock));
        R.wml().write_value(regs::Wml(WATERMARKS));
        spin(WAIT_MS, || prsstat() & (CIHB | CDIHB) == 0)?;
        set_sysctl(sysctl() | INITA);
        spin(WAIT_MS, || sysctl() & INITA == 0)?;
        set_sysctl(with_timeout(sysctl()));
        R.irqstaten().write_value(0xFFFF_FFFF);
        set_irqsigen(0);

        cmd_polled(Cmd::GoIdle, false)?;
        let mut ocr = 0;
        spin(OCR_MS, || match cmd_polled(Cmd::SendOpCond, false) {
            Ok(r) => {
                ocr = r;
                r & (1 << 31) != 0
            }
            Err(_) => false,
        })?;
        if ocr & (3 << 29) != 2 << 29 {
            return Err(Error::ByteAddressed);
        }
        for c in [Cmd::AllSendCid, Cmd::SetRca, Cmd::SendCid, Cmd::SendCsd, Cmd::Select, Cmd::SwitchHsTiming] {
            cmd_polled(c, false)?;
        }
        // as proven on hardware: wait out the switch's busy, then change the divider with the clock
        // gated, wait for it to settle, and ungate it
        spin(SWITCH_MS, || prsstat() & DAT0 != 0)?;
        set_sysctl(sysctl() & !SDCLKEN);
        set_sysctl(with_clock(sysctl(), config.data_clock));
        spin(WAIT_MS, || prsstat() & SDSTB != 0)?;
        set_sysctl(sysctl() | SDCLKEN);
        cmd_polled(Cmd::SwitchBusWidth4, false)?;
        R.proctl().write_value(regs::Proctl(R.proctl().read().0 | DTW_4BIT));
        spin(SWITCH_MS, || prsstat() & DAT0 != 0)?; // the bus test must not start while the card is still switching

        R.blkattr().write_value(regs::Blkattr(1 << 16 | 4));
        cmd_polled(Cmd::BusTestW, false)?;
        spin(WAIT_MS, || prsstat() & BWEN != 0)?; // the response comes before the buffer
        R.datport().write_value(regs::Datport(0x5A));
        spin(WAIT_MS, || irqstat() & TC != 0)?;
        cmd_polled(Cmd::BusTestR, false)?;
        spin(WAIT_MS, || prsstat() & BREN != 0)?;
        let back = R.datport().read().0;
        spin(WAIT_MS, || irqstat() & TC != 0)?;
        if (back ^ 0xA5) & 0xFF != 0 {
            return Err(Error::BusTest);
        }
        cmd_polled(Cmd::SetBlockLen, false)?;

        let mut ext = Block([0; 512]);
        arm(&mut ch, &mut ext.0);
        R.blkattr().write_value(regs::Blkattr(1 << 16 | 512));
        let r = cmd_polled(Cmd::SendExtCsd, true);
        ch.disable_request(); // before any return: the request must not outlive `ext`
        if r.is_err() {
            dma_idle(&ch);
        }
        r?;
        if !dma_done(&ch) {
            return Err(Error::Dma);
        }
        let e = &ext.0;
        let sectors = u32::from_le_bytes([e[212 ^ 3], e[213 ^ 3], e[214 ^ 3], e[215 ^ 3]]); // SEC_COUNT

        clear_irqstat(); // no stale TC for the first read to find
        typelevel::ESDHC::set_priority(config.priority);
        // SAFETY: the handler's state (WAKER, STATUS) is ready, and IRQSIGEN is 0 until a read enables it
        unsafe { typelevel::ESDHC::enable() };
        Ok(Self { ch, ext, sectors, _p: PhantomData })
    }

    /// User-area size in sectors (EXT_CSD SEC_COUNT).
    pub fn sectors(&self) -> u32 {
        self.sectors
    }

    /// EXT_CSD byte `i`, as the card numbers it. Panics if `i` is 512 or more.
    pub fn ext_csd(&self, i: usize) -> u8 {
        self.ext.0[i ^ 3]
    }

    /// Read `buf.len() / 512` sectors from `lba`, in transfers of up to 64 KiB. A transfer that fails is aborted
    /// (controller reset, CMD12) and retried once. Dropping the future mid-transfer aborts it too, so the eDMA never
    /// writes into a buffer whose borrow has ended. A read past the end of the card is refused before anything is sent.
    ///
    /// `buf` must start on a 4-byte boundary (else `Error::Misaligned`, nothing sent). Panics unless `buf.len()` is a
    /// non-zero multiple of 512.
    pub async fn read(&mut self, lba: u32, buf: &mut [u8]) -> Result<(), Error> {
        assert!(!buf.is_empty() && buf.len() % 512 == 0, "eMMC read of {} bytes", buf.len());
        let n = (buf.len() / 512) as u32;
        if lba.checked_add(n).is_none_or(|end| end > self.sectors) {
            return Err(Error::OutOfRange);
        }
        check_buffer(buf.as_ptr() as usize)?;
        for (i, piece) in buf.chunks_mut(MAX_PIECE).enumerate() {
            let at = lba + (i * MAX_PIECE / 512) as u32;
            match self.read_piece(at, piece).await {
                Err(Error::Crc | Error::Timeout | Error::Dma) => self.read_piece(at, piece).await?,
                r => r?,
            }
        }
        Ok(())
    }

    async fn read_piece(&mut self, lba: u32, piece: &mut [u8]) -> Result<(), Error> {
        arm(&mut self.ch, piece);
        let guard = AbortOnDrop;
        STATUS.store(0, SeqCst);
        R.blkattr().write_value(regs::Blkattr(((piece.len() / 512) as u32) << 16 | 512));
        issue(Cmd::ReadMultiple(lba))?;
        // after issue(), which cleared IRQSTAT: the request is a level, so a TC already set still fires
        set_irqsigen(TC | ERR);
        let s = poll_fn(|cx| {
            let s = STATUS.load(Acquire);
            if s != 0 {
                return Poll::Ready(s);
            }
            WAKER.register(cx.waker());
            match STATUS.load(Acquire) {
                0 => Poll::Pending,
                s => Poll::Ready(s),
            }
        })
        .await;
        check(s)?;
        if !dma_done(&self.ch) {
            return Err(Error::Dma);
        }
        core::mem::forget(guard);
        set_irqsigen(0);
        clear_irqstat();
        self.ch.disable_request();
        Ok(())
    }
}

/// Masks the eSDHC interrupt. The clock gate and the pads are left as they are.
impl Drop for Esdhc<'_> {
    fn drop(&mut self) {
        typelevel::ESDHC::disable();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_takes_both_clocks_explicitly_at_level_4() {
        let c = Config::new(Divider::new(0x10, 5), Divider::new(0x01, 0));
        assert_eq!((c.ident_clock, c.data_clock), (Divider::new(0x10, 5), Divider::new(0x01, 0)));
        assert!(matches!(c.priority, Priority::P4));
    }

    #[test]
    fn the_ext_csd_block_takes_word_writes() {
        // a plain [u8; 512] local landed unaligned on hardware: eDMA DAE, CMD8 never completed
        assert!(core::mem::align_of::<Block>() >= 4);
        let b = Block([0; 512]);
        assert_eq!(check_buffer(b.0.as_ptr() as usize), Ok(()));
    }

    #[test]
    fn buffers_must_be_word_aligned() {
        assert_eq!(check_buffer(0x4000_0000), Ok(()));
        assert_eq!(check_buffer(0x4000_0001), Err(Error::Misaligned));
        assert_eq!(check_buffer(0x4000_0002), Err(Error::Misaligned));
    }
}
