//! embassy-usb-driver for the USB OTG controller in device mode (a ChipIdea core; RM ch. 24) behind a ULPI PHY,
//! proven on hardware at high speed.
//!
//! One dTD per endpoint direction (embassy-usb has at most one transfer in flight per endpoint), each with its own
//! bounce buffer. The dQH list, dTDs and buffers are statics in sections `.usb_sram.qh`, `.usb_sram.td`,
//! `.usb_sram.setup` and `.usb_sram.buf`, which the board's linker script must place (NOLOAD) in the 64 KB on-chip
//! SRAM at 0x80000000: DDR can't feed 512-byte IN packets. A board that leaves them in DDR still works, with IN packets
//! of 64 bytes (see [`desc::packet_size`]). SRAM and DDR are uncached here, so both are coherent with the
//! controller's DMA.
//!
//! Nothing touches the hardware until embassy-usb enables the bus: then the pads, clock and controller are set up and
//! the `USBOTG` interrupt (bind it with `bind_interrupts!`) is enabled. [`shutdown`] undoes that.
pub mod desc;

use core::cell::UnsafeCell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::ptr::{addr_of_mut, read_volatile, write_volatile};
use core::sync::atomic::{compiler_fence, AtomicBool, AtomicU32, AtomicU8, Ordering::{Acquire, Relaxed, Release}};
use core::task::Poll;

use desc::{bit, cap, packet_size, epcr_disable, epcr_enable, failed, index, pages, remaining, setup_bytes, token, Dqh, Dtd, T};
use embassy_hal_internal::Peri;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo, EndpointType, Event, Unsupported};

use crate::clocks::{self, Gate};
use crate::gpio::SealedPin;
use crate::interrupt::typelevel::{self, Binding, Handler, Interrupt as _};
use crate::interrupt::Priority;
use crate::pac;
use crate::peripherals::{PC5, PD2, PE7, PJ0, PK0, PK1, PK2, PK3, PK4, PK5, PK6, PK7, USBOTG};
use crate::pinmux::{self, Function, ParField, Slew, SlewGroup};
pub use crate::pinmux::UlpiMode;

/// The controller's registers: their PAC addresses, accessed as whole 32-bit words (the driver's values are whole
/// words, as proven on hardware).
macro_rules! reg {
    ($f:ident) => {
        pac::USBOTG.$f().as_ptr() as usize
    };
}

unsafe fn r32(a: usize) -> u32 {
    unsafe { coldfire::mmio::r32(a) }
}

unsafe fn w32(a: usize, v: u32) {
    unsafe { coldfire::mmio::w32(a, v) }
}

/// EPCRn (EPCR0..3 are consecutive words).
fn epcr_reg(n: usize) -> usize {
    reg!(epcr0) + 4 * n
}

mod cmd {
    pub const RS: u32 = 1;
    pub const RST: u32 = 1 << 1;
    pub const SUTW: u32 = 1 << 13;
    pub const ITC_MASK: u32 = 0x00FF_0000;
}

mod sts {
    pub const UI: u32 = 1;
    pub const PCI: u32 = 1 << 2;
    /// System error: a DMA bus error; the controller has stopped.
    pub const SEI: u32 = 1 << 4;
    pub const URI: u32 = 1 << 6;
    pub const SLI: u32 = 1 << 8;
}

mod portsc {
    pub const PTS_MASK: u32 = 0xC000_0000;
    pub const PTS_ULPI: u32 = 0x8000_0000;
    pub const PFSC: u32 = 1 << 24;
    pub const PR: u32 = 1 << 8;
    /// Current connect status (device mode: attached to a host).
    pub const CCS: u32 = 1;
    /// PSPD 27:26: 00 full, 01 low, 10 high speed (read-only, valid after the bus reset).
    pub const PSPD_SHIFT: u32 = 26;
    pub const PSPD_HIGH: u32 = 0b10;
}

mod epcr {
    pub const RXS: u32 = 1;
    pub const RXR: u32 = 1 << 6;
    pub const RXE: u32 = 1 << 7;
    pub const TXS: u32 = 1 << 16;
    pub const TXR: u32 = 1 << 22;
    pub const TXE: u32 = 1 << 23;
}

/// UE, UEE, PCE, SEE, URE, SLE (proven on hardware).
const USBINTR_VALUE: u32 = 0x157;
/// CM = device, ES = 1 (big-endian buffers; required on ColdFire), SLOM, SDIS (proven on hardware).
const USBMODE_DEVICE: u32 = 0x1E;
/// TX burst 2 words, RX burst 4 words (proven on hardware).
const BURSTSIZE_VALUE: u32 = 0x204;

const EPS: usize = 4;
/// The largest packet: high-speed bulk.
pub const MAX_PACKET: usize = 512;

struct Shared<T>(UnsafeCell<T>);
unsafe impl<T> Sync for Shared<T> {}

#[repr(C, align(2048))]
struct QhList([Dqh; 2 * EPS]);

/// 8 × 512 bytes in one 4 KB page, so no buffer crosses a page.
#[repr(C, align(4096))]
struct Buffers([[u8; MAX_PACKET]; 2 * EPS]);

// NOLOAD on the target: `Driver::new` zeroes what the controller reads.
#[cfg_attr(target_arch = "m68k", unsafe(link_section = ".usb_sram.qh"))]
static QH: Shared<QhList> = Shared(UnsafeCell::new(QhList([const { Dqh::ZERO }; 2 * EPS])));
#[cfg_attr(target_arch = "m68k", unsafe(link_section = ".usb_sram.td"))]
static TD: Shared<[Dtd; 2 * EPS]> = Shared(UnsafeCell::new([const { Dtd::ZERO }; 2 * EPS]));
#[cfg_attr(target_arch = "m68k", unsafe(link_section = ".usb_sram.buf"))]
static BUF: Shared<Buffers> = Shared(UnsafeCell::new(Buffers([[0; MAX_PACKET]; 2 * EPS])));
#[cfg_attr(target_arch = "m68k", unsafe(link_section = ".usb_sram.setup"))]
static SETUP: Shared<[u8; 8]> = Shared(UnsafeCell::new([0; 8]));

const EV_RESET: u32 = 1;
const EV_SUSPEND: u32 = 2;
const EV_RESUME: u32 = 4;
const EV_POWER: u32 = 8;
static EVENTS: AtomicU32 = AtomicU32::new(0);
/// EPCOMPLETE bits the ISR has seen that no transfer has taken yet.
static COMPLETE: AtomicU32 = AtomicU32::new(0);
static SETUP_READY: AtomicBool = AtomicBool::new(false);
/// Bumped by every setup packet: a control stage started before it is stale.
static SETUP_SEQ: AtomicU32 = AtomicU32::new(0);
static SUSPENDED: AtomicBool = AtomicBool::new(false);
/// Bumped by every bus reset: an endpoint transfer started before it is stale, whether or not the
/// controller cleared the endpoint's EPCR enable (the RM doesn't say it does).
static RESET_SEQ: AtomicU32 = AtomicU32::new(0);
/// Bumped whenever an endpoint is (re)configured or disabled: its transfers started before are stale.
static EP_SEQ: [AtomicU32; 2 * EPS] = [const { AtomicU32::new(0) }; 2 * EPS];
/// The bus is enabled (clock on, controller started): `shutdown` has something to undo.
static RUNNING: AtomicBool = AtomicBool::new(false);
static CTRL_MPS: AtomicU32 = AtomicU32::new(64);
static MPS: [AtomicU32; 2 * EPS] = [const { AtomicU32::new(0) }; 2 * EPS];
static TYPES: [AtomicU8; 2 * EPS] = [const { AtomicU8::new(0) }; 2 * EPS];
/// The packet size in use: `desc::packet_size` of the endpoint's MPS.
static EFF_MPS: [AtomicU32; 2 * EPS] = [const { AtomicU32::new(0) }; 2 * EPS];
static BUS_WAKER: AtomicWaker = AtomicWaker::new();
static EP_WAKERS: [AtomicWaker; 2 * EPS] = [const { AtomicWaker::new() }; 2 * EPS];

fn qh(i: usize) -> *mut Dqh {
    unsafe { addr_of_mut!((*QH.0.get()).0[i]) }
}

fn td(i: usize) -> *mut Dtd {
    unsafe { addr_of_mut!((*TD.0.get())[i]) }
}

fn buf(i: usize) -> *mut u8 {
    unsafe { addr_of_mut!((*BUF.0.get()).0[i]) as *mut u8 }
}

fn addr<T>(p: *const T) -> u32 {
    p as usize as u32
}

unsafe fn vw(p: *mut u32, v: u32) {
    unsafe { write_volatile(p, v) }
}

unsafe fn vr(p: *const u32) -> u32 {
    unsafe { read_volatile(p) }
}

/// Bounded wait on a register condition (the controller clears these within microseconds).
fn spin_until(mut done: impl FnMut() -> bool) {
    for _ in 0..1_000_000 {
        if done() {
            return;
        }
    }
}

fn event(e: u32) {
    EVENTS.fetch_or(e, Relaxed);
    BUS_WAKER.wake();
}

fn wake_all() {
    for w in &EP_WAKERS {
        w.wake();
    }
}

/// The USB OTG interrupt (INTC2 source 17).
pub fn on_interrupt() {
    unsafe {
        let sts = r32(reg!(usbsts)) & r32(reg!(usbintr));
        w32(reg!(usbsts), sts);
        if sts & sts::SEI != 0 {
            // a DMA bus error stops the controller (RM USBSTS SEI): restart it; the host sees a reconnect
            init_controller();
            forget_transfers();
            return;
        }
        if sts & sts::URI != 0 {
            bus_reset();
        }
        if sts & sts::UI != 0 {
            if r32(reg!(epsetupsr)) & 1 != 0 {
                read_setup();
            }
            let c = r32(reg!(epcomplete));
            if c != 0 {
                w32(reg!(epcomplete), c);
                COMPLETE.fetch_or(c, Relaxed);
                for i in 0..2 * EPS {
                    if c & bit(i / 2, i % 2 == 1) != 0 {
                        EP_WAKERS[i].wake();
                    }
                }
            }
        }
        if sts & sts::SLI != 0 && !SUSPENDED.swap(true, Relaxed) {
            event(EV_SUSPEND);
        }
        if sts & sts::PCI != 0 && SUSPENDED.swap(false, Relaxed) {
            event(EV_RESUME);
        }
    }
}

/// Bus reset clean-up (RM §24.5.3.2.1): it must finish while the port is still in reset, so it
/// runs here in the ISR.
unsafe fn bus_reset() {
    unsafe {
        w32(reg!(epsetupsr), r32(reg!(epsetupsr)));
        w32(reg!(epcomplete), r32(reg!(epcomplete)));
        spin_until(|| r32(reg!(epprime)) == 0);
        w32(reg!(epflush), 0xFFFF_FFFF);
        spin_until(|| r32(reg!(epflush)) == 0);
        if r32(reg!(portsc1)) & portsc::PR == 0 {
            // The reset ended before this clean-up. The RM's answer (step 4) is a controller reset and a
            // fresh start; the host sees a reconnect and resets again.
            init_controller();
        }
    }
    forget_transfers();
}

/// After a bus reset or a restart: nothing in flight survives; tell embassy-usb it was reset.
fn forget_transfers() {
    COMPLETE.store(0, Relaxed);
    SETUP_READY.store(false, Relaxed);
    SUSPENDED.store(false, Relaxed);
    RESET_SEQ.fetch_add(1, Relaxed);
    event(EV_RESET);
    wake_all();
}

/// Copy the setup packet under the setup tripwire (RM §24.5.3.4.4 with SLOM = 1).
unsafe fn read_setup() {
    unsafe {
        w32(reg!(epsetupsr), 1);
        let q = qh(0);
        let (mut w0, mut w1);
        loop {
            w32(reg!(usbcmd), r32(reg!(usbcmd)) | cmd::SUTW);
            w0 = vr(addr_of_mut!((*q).setup[0]));
            w1 = vr(addr_of_mut!((*q).setup[1]));
            if r32(reg!(usbcmd)) & cmd::SUTW != 0 {
                break;
            }
        }
        w32(reg!(usbcmd), r32(reg!(usbcmd)) & !cmd::SUTW);
        *SETUP.0.get() = setup_bytes(w0, w1);
    }
    SETUP_SEQ.fetch_add(1, Relaxed);
    SETUP_READY.store(true, Relaxed);
    EP_WAKERS[0].wake();
    EP_WAKERS[1].wake();
}

/// The PAR fields of the three ULPI pads that have their own (DIR, NXT, USB_CLKIN), from the pins' table entries.
#[derive(Copy, Clone)]
struct Pads {
    dir: ParField,
    nxt: ParField,
    clkin: ParField,
}

/// Pads and clock for the OTG controller and its ULPI PHY, in an order proven on hardware. The other fields of
/// PAR_TIMER and PAR_IRQ0L keep their functions.
fn bring_up(pads: Pads, mode: UlpiMode) {
    // Before the pads are muxed: at the power-on slew rate a cold USB3300 answered register reads with RX CMDs, or
    // wedged until a power cycle with the cable out. "Low" for both pad groups (they carry ULPI_STP and
    // ULPI_DATA[7:0]) works.
    pinmux::set_slew(SlewGroup::Rmii0, Slew::Low);
    pinmux::set_slew(SlewGroup::Rmii1, Slew::Low);
    clocks::ungate(Gate::UsbOtg);
    pinmux::set_function(pads.dir, Function::Alt2);
    pinmux::set_function(pads.nxt, Function::Alt2);
    pinmux::set_fec_mode(mode);
    pinmux::set_function(pads.clkin, Function::Alt2);
    // USB clock from the USB_CLKIN pin (60 MHz from the PHY), and the OTG controller owns the ULPI pins
    pac::CCM.misccr().modify(|w| w.set_usbsrc(false));
    pac::CCM.misccr2().modify(|w| w.set_ulpi(false));
}

/// Reset the controller and start it in device mode (RM §24.5.3.1, with values proven on hardware).
unsafe fn init_controller() {
    unsafe {
        w32(reg!(usbcmd), cmd::RST);
        spin_until(|| r32(reg!(usbcmd)) & cmd::RST == 0);
        // ULPI PHY; PFSC = 0 allows the high-speed chirp
        w32(reg!(portsc1), (r32(reg!(portsc1)) & !(portsc::PTS_MASK | portsc::PFSC)) | portsc::PTS_ULPI);
        w32(reg!(burstsize), BURSTSIZE_VALUE);
        w32(reg!(usbmode), USBMODE_DEVICE);
        for i in 0..2 * EPS {
            let q = qh(i);
            vw(addr_of_mut!((*q).cap), 0);
            vw(addr_of_mut!((*q).next), T);
            vw(addr_of_mut!((*q).token), 0);
        }
        let mps = CTRL_MPS.load(Relaxed) as u16;
        vw(addr_of_mut!((*qh(0)).cap), cap(mps, true));
        vw(addr_of_mut!((*qh(1)).cap), cap(mps, false));
        w32(reg!(eplistaddr), addr(qh(0)));
        w32(reg!(usbintr), USBINTR_VALUE);
        w32(reg!(usbcmd), (r32(reg!(usbcmd)) & !cmd::ITC_MASK) | cmd::RS);
    }
}

/// After a bus reset: disable EP1..3 and drop anything primed on them since (a task may have primed
/// before this ran, if the controller left the enables set), then wake their waiters so those transfers
/// end. EP0 is left alone (a setup may already be in flight).
unsafe fn after_reset() {
    let mut bits = 0;
    for n in 1..EPS {
        unsafe { w32(epcr_reg(n), 0) };
        bits |= bit(n, false) | bit(n, true);
    }
    unsafe { flush(bits) };
    for n in 1..EPS {
        for is_in in [false, true] {
            let q = qh(index(n, is_in));
            unsafe {
                vw(addr_of_mut!((*q).next), T);
                vw(addr_of_mut!((*q).token), 0);
            }
        }
    }
    wake_all();
}

unsafe fn flush(bits: u32) {
    unsafe {
        w32(reg!(epflush), bits);
        spin_until(|| r32(reg!(epflush)) & bits == 0);
    }
    COMPLETE.fetch_and(!bits, Relaxed);
}

/// Queue a dTD of `len` bytes on slot `i` (its bounce buffer) and prime it.
unsafe fn prime(i: usize, len: usize) {
    // the bounce-buffer copy (plain stores) must reach memory before the dTD and EPPRIME hand it to DMA
    compiler_fence(Release);
    let b = bit(i / 2, i % 2 == 1);
    COMPLETE.fetch_and(!b, Relaxed);
    let (t, q) = (td(i), qh(i));
    unsafe {
        vw(addr_of_mut!((*t).next), T);
        vw(addr_of_mut!((*t).token), token(len));
        for (k, p) in pages(addr(buf(i))).into_iter().enumerate() {
            vw(addr_of_mut!((*t).buf[k]), p);
        }
        vw(addr_of_mut!((*q).next), addr(t));
        vw(addr_of_mut!((*q).token), 0);
        w32(reg!(epprime), b);
    }
}

/// Wait for slot `i`'s dTD to retire and return the bytes it moved. `stale` ends the wait with `Disabled`.
async fn complete(i: usize, len: usize, stale: impl Fn() -> bool) -> Result<usize, EndpointError> {
    let b = bit(i / 2, i % 2 == 1);
    poll_fn(|cx| {
        EP_WAKERS[i].register(cx.waker());
        if COMPLETE.load(Relaxed) & b != 0 {
            Poll::Ready(Ok(()))
        } else if stale() {
            Poll::Ready(Err(EndpointError::Disabled))
        } else {
            Poll::Pending
        }
    })
    .await?;
    // and the caller's copy out of the bounce buffer must not be hoisted above the completion
    compiler_fence(Acquire);
    COMPLETE.fetch_and(!b, Relaxed);
    let tok = unsafe { vr(addr_of_mut!((*td(i)).token)) };
    if failed(tok) {
        return Err(EndpointError::BufferOverflow);
    }
    Ok(len - remaining(tok))
}

/// Flushes an endpoint whose transfer future is dropped before it completes (a caller's timeout), so the
/// next transfer doesn't re-prime a dTD the controller still owns. A packet landing in the instant before
/// the flush is lost with it.
struct InFlight(u32);

impl Drop for InFlight {
    fn drop(&mut self) {
        unsafe { flush(self.0) };
    }
}

/// A transfer on endpoint slot `i` started now is stale once a bus reset or a reconfiguration of the
/// endpoint happens, or its enable bit drops.
fn stale_check(i: usize, n: usize, is_in: bool) -> impl Fn() -> bool {
    let (reset, ep) = (RESET_SEQ.load(Relaxed), EP_SEQ[i].load(Relaxed));
    move || RESET_SEQ.load(Relaxed) != reset || EP_SEQ[i].load(Relaxed) != ep || !enabled(n, is_in)
}

/// Whether endpoint `n` is configured. False until the controller runs: before `Bus::enable` opens the clock gate the
/// controller's registers read all ones, which would show every endpoint as enabled.
fn enabled(n: usize, is_in: bool) -> bool {
    let e = if is_in { epcr::TXE } else { epcr::RXE };
    RUNNING.load(Relaxed) && unsafe { r32(epcr_reg(n)) & e != 0 }
}

/// Whether a host is attached (PORTSC1 CCS): it has seen the device on the bus. This says nothing about a program
/// having opened any interface yet.
pub fn attached() -> bool {
    RUNNING.load(Relaxed) && unsafe { r32(reg!(portsc1)) & portsc::CCS != 0 }
}

/// Whether the port is running at high speed (PORTSC1 PSPD = 10), for the USB screen's title bar.
pub fn high_speed() -> bool {
    RUNNING.load(Relaxed) && unsafe { (r32(reg!(portsc1)) >> portsc::PSPD_SHIFT) & 3 == portsc::PSPD_HIGH }
}

/// Detach and reset the controller, mask its interrupt and fail every waiting transfer, so whatever runs
/// next finds the controller at its reset values. Does nothing if the bus was never enabled.
pub fn shutdown() {
    if !RUNNING.swap(false, Relaxed) {
        return;
    }
    typelevel::USBOTG::disable();
    unsafe {
        w32(reg!(usbcmd), r32(reg!(usbcmd)) & !cmd::RS);
        w32(reg!(usbintr), 0);
        // let the detach reach the bus (the ULPI PHY drops the pull-up) before the reset, which also
        // switches PTS back to the serial interface
        spin_until(|| r32(reg!(portsc1)) & portsc::CCS == 0);
        w32(reg!(usbcmd), cmd::RST);
        spin_until(|| r32(reg!(usbcmd)) & cmd::RST == 0);
    }
    SETUP_READY.store(false, Relaxed);
    wake_all();
}

/// The 12 ULPI pads of the MCF54415–18 (each signal has only this pad, in its Alt 2 function). PJ0 and PK0..PK7 come
/// with PAR_FEC's group mode, which also switches PI0, PI1 and PJ1..PJ7 to RMII0 (see [`UlpiMode`]).
pub struct Ulpi<'d> {
    pub clkin: Peri<'d, PC5>,
    pub dir: Peri<'d, PD2>,
    pub nxt: Peri<'d, PE7>,
    pub stp: Peri<'d, PJ0>,
    pub data0: Peri<'d, PK4>,
    pub data1: Peri<'d, PK5>,
    pub data2: Peri<'d, PK1>,
    pub data3: Peri<'d, PK2>,
    pub data4: Peri<'d, PK7>,
    pub data5: Peri<'d, PK6>,
    pub data6: Peri<'d, PK3>,
    pub data7: Peri<'d, PK0>,
}

/// Settings.
#[non_exhaustive]
#[derive(Copy, Clone, Debug)]
pub struct Config {
    /// The PAR_FEC mode that gives the ULPI its nine MII0 pads.
    pub ulpi_mode: UlpiMode,
    /// Interrupt level of the USB OTG interrupt.
    pub priority: Priority,
}

impl Default for Config {
    /// RMII0 without MDC/MDIO plus ULPI, interrupt level 6 (as proven on hardware).
    fn default() -> Self {
        Self { ulpi_mode: UlpiMode::WithRmii0NoMdio, priority: Priority::P6 }
    }
}

/// The USB OTG interrupt handler.
pub struct InterruptHandler;

impl Handler<typelevel::USBOTG> for InterruptHandler {
    unsafe fn on_interrupt() {
        on_interrupt();
    }
}

pub struct Driver<'d> {
    /// Endpoint numbers taken, per direction (OUT, IN); EP0 is always taken.
    used: [u8; 2],
    pads: Pads,
    config: Config,
    _p: PhantomData<&'d ()>,
}

impl<'d> Driver<'d> {
    /// Resets the driver's state. If an earlier driver left the controller running, it is shut down first (see
    /// [`shutdown`]), so no DMA or interrupt still uses the descriptors this resets; otherwise nothing touches the
    /// controller until embassy-usb enables the bus.
    ///
    /// Dropping the driver, its `Bus` or the `UsbDevice` built on it does not stop the controller: call [`shutdown`].
    pub fn new(
        _usb: Peri<'d, USBOTG>,
        _irq: impl Binding<typelevel::USBOTG, InterruptHandler> + 'd,
        ulpi: Ulpi<'d>,
        config: Config,
    ) -> Self {
        let field = |f: Option<ParField>| f.expect("a ULPI pad with its own PAR field");
        let pads = Pads { dir: field(ulpi.dir.par()), nxt: field(ulpi.nxt.par()), clkin: field(ulpi.clkin.par()) };
        shutdown(); // a no-op unless a previous driver's bus is still enabled
        EVENTS.store(EV_POWER, Relaxed); // no VBUS sensing: report power once, at start
        COMPLETE.store(0, Relaxed);
        SETUP_READY.store(false, Relaxed);
        SUSPENDED.store(false, Relaxed);
        unsafe {
            QH.0.get().write_bytes(0, 1);
            TD.0.get().write_bytes(0, 1);
            SETUP.0.get().write_bytes(0, 1);
        }
        for i in 0..2 * EPS {
            MPS[i].store(0, Relaxed);
            TYPES[i].store(0, Relaxed);
        }
        Driver { used: [1, 1], pads, config, _p: PhantomData }
    }

    fn alloc(&mut self, dir: Direction, ep_type: EndpointType, ep_addr: Option<EndpointAddress>, max_packet_size: u16, interval_ms: u8) -> Result<EndpointInfo, EndpointAllocError> {
        if max_packet_size as usize > MAX_PACKET || ep_type == EndpointType::Isochronous {
            return Err(EndpointAllocError);
        }
        let d = matches!(dir, Direction::In) as usize;
        let n = match ep_addr {
            Some(a) if (1..EPS).contains(&a.index()) && self.used[d] & (1 << a.index()) == 0 => a.index(),
            Some(_) => return Err(EndpointAllocError),
            None => (1..EPS).find(|n| self.used[d] & (1 << n) == 0).ok_or(EndpointAllocError)?,
        };
        self.used[d] |= 1 << n;
        let i = index(n, d == 1);
        MPS[i].store(max_packet_size as u32, Relaxed);
        TYPES[i].store(ep_type as u8, Relaxed);
        Ok(EndpointInfo { addr: EndpointAddress::from_parts(n, dir), ep_type, max_packet_size, interval_ms })
    }
}

// Any shorter-lived use: embassy-usb's `Builder<'a>` borrows descriptor buffers that may live on a stack.
impl<'a, 'd: 'a> embassy_usb_driver::Driver<'a> for Driver<'d> {
    type EndpointOut = EndpointOut;
    type EndpointIn = EndpointIn;
    type ControlPipe = ControlPipe;
    type Bus = Bus<'d>;

    fn alloc_endpoint_out(&mut self, ep_type: EndpointType, ep_addr: Option<EndpointAddress>, max_packet_size: u16, interval_ms: u8) -> Result<EndpointOut, EndpointAllocError> {
        Ok(EndpointOut { info: self.alloc(Direction::Out, ep_type, ep_addr, max_packet_size, interval_ms)? })
    }

    fn alloc_endpoint_in(&mut self, ep_type: EndpointType, ep_addr: Option<EndpointAddress>, max_packet_size: u16, interval_ms: u8) -> Result<EndpointIn, EndpointAllocError> {
        Ok(EndpointIn { info: self.alloc(Direction::In, ep_type, ep_addr, max_packet_size, interval_ms)? })
    }

    fn start(self, control_max_packet_size: u16) -> (Bus<'d>, ControlPipe) {
        CTRL_MPS.store(control_max_packet_size as u32, Relaxed);
        (Bus { pads: self.pads, config: self.config, _p: PhantomData }, ControlPipe { mps: control_max_packet_size as usize, seq: 0 })
    }
}

pub struct Bus<'d> {
    pads: Pads,
    config: Config,
    _p: PhantomData<&'d ()>,
}

impl embassy_usb_driver::Bus for Bus<'_> {
    async fn enable(&mut self) {
        bring_up(self.pads, self.config.ulpi_mode);
        unsafe { init_controller() };
        RUNNING.store(true, Relaxed);
        typelevel::USBOTG::set_priority(self.config.priority);
        // SAFETY: the handler's state is reset (Driver::new) and the controller is running
        unsafe { typelevel::USBOTG::enable() };
    }

    async fn disable(&mut self) {
        shutdown();
    }

    async fn poll(&mut self) -> Event {
        poll_fn(|cx| {
            BUS_WAKER.register(cx.waker());
            let ev = EVENTS.load(Relaxed);
            for (b, e) in [(EV_RESET, Event::Reset), (EV_SUSPEND, Event::Suspend), (EV_RESUME, Event::Resume), (EV_POWER, Event::PowerDetected)] {
                if ev & b != 0 {
                    EVENTS.fetch_and(!b, Relaxed);
                    if b == EV_RESET {
                        unsafe { after_reset() };
                    }
                    return Poll::Ready(e);
                }
            }
            Poll::Pending
        })
        .await
    }

    fn endpoint_set_enabled(&mut self, a: EndpointAddress, en: bool) {
        let (n, is_in) = (a.index(), a.is_in());
        if n == 0 || n >= EPS {
            return;
        }
        let i = index(n, is_in);
        let reg = epcr_reg(n);
        EP_SEQ[i].fetch_add(1, Relaxed);
        unsafe {
            if en && enabled(n, is_in) {
                // already configured (SET_CONFIGURATION again): stop it before touching its dQH, which a
                // primed transfer may be using (RM §24.5.3.5.1)
                w32(reg, epcr_disable(r32(reg), is_in));
                flush(bit(n, is_in));
            }
            if en {
                let q = qh(i);
                let full_speed = (r32(reg!(portsc1)) >> portsc::PSPD_SHIFT) & 3 != portsc::PSPD_HIGH;
                let mps = MPS[i].load(Relaxed);
                let eff = packet_size(mps, full_speed, is_in, addr(buf(i)));
                EFF_MPS[i].store(eff, Relaxed);
                vw(addr_of_mut!((*q).cap), cap(eff as u16, false));
                vw(addr_of_mut!((*q).next), T);
                vw(addr_of_mut!((*q).token), 0);
                w32(reg, epcr_enable(r32(reg), TYPES[i].load(Relaxed), is_in));
            } else {
                w32(reg, epcr_disable(r32(reg), is_in));
                flush(bit(n, is_in));
            }
        }
        EP_WAKERS[i].wake();
    }

    fn endpoint_set_stalled(&mut self, a: EndpointAddress, stalled: bool) {
        let n = a.index();
        let reg = epcr_reg(n);
        unsafe {
            let v = r32(reg);
            let v = match (n, a.is_in(), stalled) {
                (0, _, true) => v | epcr::TXS | epcr::RXS,
                (0, _, false) => v & !(epcr::TXS | epcr::RXS),
                (_, true, true) => v | epcr::TXS,
                (_, true, false) => (v & !epcr::TXS) | epcr::TXR,
                (_, false, true) => v | epcr::RXS,
                (_, false, false) => (v & !epcr::RXS) | epcr::RXR,
            };
            w32(reg, v);
        }
    }

    fn endpoint_is_stalled(&mut self, a: EndpointAddress) -> bool {
        let s = if a.is_in() { epcr::TXS } else { epcr::RXS };
        unsafe { r32(reg!(epcr0) + 4 * a.index()) & s != 0 }
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

async fn wait_enabled(n: usize, is_in: bool) {
    poll_fn(|cx| {
        EP_WAKERS[index(n, is_in)].register(cx.waker());
        if enabled(n, is_in) { Poll::Ready(()) } else { Poll::Pending }
    })
    .await
}

pub struct EndpointOut {
    info: EndpointInfo,
}

pub struct EndpointIn {
    info: EndpointInfo,
}

impl embassy_usb_driver::Endpoint for EndpointOut {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }
    async fn wait_enabled(&mut self) {
        wait_enabled(self.info.addr.index(), false).await
    }
}

impl embassy_usb_driver::Endpoint for EndpointIn {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }
    async fn wait_enabled(&mut self) {
        wait_enabled(self.info.addr.index(), true).await
    }
}

impl embassy_usb_driver::EndpointOut for EndpointOut {
    /// One packet: an OUT dTD of max-packet size (OUT totals must be a multiple of it, RM Table 24-48).
    async fn read(&mut self, out: &mut [u8]) -> Result<usize, EndpointError> {
        let n = self.info.addr.index();
        if !enabled(n, false) {
            return Err(EndpointError::Disabled);
        }
        let i = index(n, false);
        let mps = EFF_MPS[i].load(Relaxed) as usize; // one packet: OUT totals must be a multiple of it
        let stale = stale_check(i, n, false);
        unsafe { prime(i, mps) };
        let guard = InFlight(bit(n, false));
        let got = complete(i, mps, stale).await;
        if got.is_ok() {
            core::mem::forget(guard);
        }
        let got = got?;
        if got > out.len() {
            return Err(EndpointError::BufferOverflow);
        }
        unsafe { core::ptr::copy_nonoverlapping(buf(i), out.as_mut_ptr(), got) };
        Ok(got)
    }
}

impl embassy_usb_driver::EndpointIn for EndpointIn {
    async fn write(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        let n = self.info.addr.index();
        if data.len() > self.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }
        if !enabled(n, true) {
            return Err(EndpointError::Disabled);
        }
        let i = index(n, true);
        let stale = stale_check(i, n, true);
        unsafe {
            core::ptr::copy_nonoverlapping(data.as_ptr(), buf(i), data.len());
            prime(i, data.len());
        }
        let guard = InFlight(bit(n, true));
        let r = complete(i, data.len(), stale).await;
        if r.is_ok() {
            core::mem::forget(guard);
        }
        r.map(|_| ())
    }
}

pub struct ControlPipe {
    mps: usize,
    /// SETUP_SEQ of the setup being handled.
    seq: u32,
}

impl ControlPipe {
    /// After priming a data/status stage, re-read EPSETUPSR (RM §24.5.3.4.4 "Data Phase"): if a newer
    /// setup has arrived, even one the ISR hasn't read yet, drop the stage before the host can take it.
    fn still_current(&self) -> Result<(), EndpointError> {
        if SETUP_SEQ.load(Relaxed) != self.seq || unsafe { r32(reg!(epsetupsr)) } & 1 != 0 {
            unsafe { flush(bit(0, false) | bit(0, true)) };
            return Err(EndpointError::Disabled);
        }
        Ok(())
    }

    async fn stage(&self, i: usize, len: usize) -> Result<usize, EndpointError> {
        let seq = self.seq;
        // embassy-usb never races a control stage, but dropping UsbDevice::run drops it mid-stage
        let guard = InFlight(bit(0, false) | bit(0, true));
        let r = complete(i, len, move || SETUP_SEQ.load(Relaxed) != seq).await;
        core::mem::forget(guard);
        r
    }
}

impl embassy_usb_driver::ControlPipe for ControlPipe {
    fn max_packet_size(&self) -> usize {
        self.mps
    }

    async fn setup(&mut self) -> [u8; 8] {
        poll_fn(|cx| {
            EP_WAKERS[0].register(cx.waker());
            if SETUP_READY.load(Relaxed) { Poll::Ready(()) } else { Poll::Pending }
        })
        .await;
        // drop data/status stages left from the previous transfer (RM §24.5.3.5.2 step 3)
        unsafe { flush(bit(0, false) | bit(0, true)) };
        critical_section::with(|_| {
            SETUP_READY.store(false, Relaxed);
            self.seq = SETUP_SEQ.load(Relaxed);
            unsafe { *SETUP.0.get() }
        })
    }

    async fn data_out(&mut self, out: &mut [u8], _first: bool, _last: bool) -> Result<usize, EndpointError> {
        unsafe { prime(0, self.mps) };
        self.still_current()?;
        let n = self.stage(0, self.mps).await?;
        if n > out.len() {
            return Err(EndpointError::BufferOverflow);
        }
        unsafe { core::ptr::copy_nonoverlapping(buf(0), out.as_mut_ptr(), n) };
        Ok(n)
    }

    async fn data_in(&mut self, data: &[u8], _first: bool, last: bool) -> Result<(), EndpointError> {
        if data.len() > self.mps {
            return Err(EndpointError::BufferOverflow);
        }
        unsafe {
            core::ptr::copy_nonoverlapping(data.as_ptr(), buf(1), data.len());
            if last {
                prime(0, 0); // the host's status-stage ZLP, queued with the data
            }
            prime(1, data.len()); // last, so the re-check follows it at once
        }
        self.still_current()?;
        self.stage(1, data.len()).await?;
        if last {
            self.stage(0, 0).await?;
        }
        Ok(())
    }

    async fn accept(&mut self) {
        unsafe { prime(1, 0) };
        if self.still_current().is_ok() {
            let _ = self.stage(1, 0).await;
        }
    }

    async fn reject(&mut self) {
        unsafe { w32(reg!(epcr0), r32(reg!(epcr0)) | epcr::TXS | epcr::RXS) };
    }

    async fn accept_set_address(&mut self, addr: u8) {
        unsafe { prime(1, 0) };
        if self.still_current().is_ok() && self.stage(1, 0).await.is_ok() {
            // no USBADRA on this part: the address takes effect only once the status stage is done
            unsafe { w32(reg!(deviceaddr), (addr as u32) << 25) };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_endpoint_is_disabled_until_the_controller_runs() {
        // not running: no register is read (there is none on the host), and nothing reads as enabled
        assert!(!RUNNING.load(Relaxed));
        assert!(!enabled(1, false) && !enabled(1, true));
    }

    #[test]
    fn config_defaults_to_mode_0x0a_at_level_6() {
        let c = Config::default();
        assert_eq!((c.ulpi_mode as u8, c.priority as u8), (0x0A, 6));
    }
}
