//! The USB OTG controller in device mode (RM ch. 24), a bus master, and the host-side primitives that drive it:
//! setup packets, IN tokens, OUT packets, bus resets, and host scripts that run on the machine's clock; and a
//! high-level host ([`UsbHost`]) that drives the machine itself between its transactions.

pub mod host;

pub use host::{HostError, UsbHost, endpoints};

use std::fmt;
use std::ops::Range;

use crate::bus::DmaLines;
use crate::device::{Device, DeviceCx};
use crate::mem::Size;
use crate::stop::Stop;

const ID: u32 = 0x000;
const HWGENERAL: u32 = 0x004;
const HWHOST: u32 = 0x008;
const HWDEVICE: u32 = 0x00C;
const HWTXBUF: u32 = 0x010;
const HWRXBUF: u32 = 0x014;
/// GPTIMER0LD, GPTIMER0CTL, GPTIMER1LD, GPTIMER1CTL (RM 24.3.2).
const GPTIMER0LD: u32 = 0x080;
const GPTIMER1CTL: u32 = 0x08C;
const CAPLENGTH: u32 = 0x100;
const HCSPARAMS: u32 = 0x104;
const HCCPARAMS: u32 = 0x108;
const DCIVERSION: u32 = 0x120;
const DCCPARAMS: u32 = 0x124;
const USBCMD: u32 = 0x140;
const USBSTS: u32 = 0x144;
const USBINTR: u32 = 0x148;
const DEVICEADDR: u32 = 0x154;
const EPLISTADDR: u32 = 0x158;
const BURSTSIZE: u32 = 0x160;
const CONFIGFLAG: u32 = 0x180;
const PORTSC1: u32 = 0x184;
const OTGSC: u32 = 0x1A4;
const USBMODE: u32 = 0x1A8;
const EPSETUPSR: u32 = 0x1AC;
const EPPRIME: u32 = 0x1B0;
const EPFLUSH: u32 = 0x1B4;
const EPSR: u32 = 0x1B8;
const EPCOMPLETE: u32 = 0x1BC;
const EPCR0: u32 = 0x1C0;
const EPCR3: u32 = 0x1CC;

/// The constant registers (RM Table 24-4), as 32-bit words: HCIVERSION 0x0100 and CAPLENGTH 0x40 share the word at
/// 0x100, DCIVERSION 0x0001 is the low half of the word at 0x120.
const CONSTANTS: [(u32, u32); 12] = [
    (ID, 0xE241_FA05),
    (HWGENERAL, 0x0000_07C5),
    (HWHOST, 0x1002_0001),
    (HWDEVICE, 0x0000_0009),
    (HWTXBUF, 0x8004_0604),
    (HWRXBUF, 0x0000_0404),
    (CAPLENGTH, 0x0100_0040),
    (HCSPARAMS, 0x0001_0011),
    (HCCPARAMS, 0x0000_0006),
    (DCIVERSION, 0x0000_0001),
    (DCCPARAMS, 0x0000_0184),
    (CONFIGFLAG, 0x0000_0001),
];

/// Reset values (RM Table 24-4).
const USBCMD_RESET: u32 = 0x0008_0000;
const USBSTS_RESET: u32 = 0x80;
/// Measured on an MCF54415 (RM Table 24-4 gives 0x0101 and Figure 24-26 0x0105).
const BURSTSIZE_RESET: u32 = 0x1010;
const PORTSC_RESET: u32 = 0xEC00_0004;
const OTGSC_RESET: u32 = 0x0000_1020;
const EPCR0_RESET: u32 = 0x0080_0080;

/// USBCMD (RM Table 24-19).
const RS: u32 = 1 << 0;
const RST: u32 = 1 << 1;
const SUTW: u32 = 1 << 13;

/// USBSTS (RM Table 24-20).
const UI: u32 = 1 << 0;
const UEI: u32 = 1 << 1;
const PCI: u32 = 1 << 2;
const SEI: u32 = 1 << 4;
const URI: u32 = 1 << 6;
/// The write-1-to-clear status bits, which USBINTR enables.
const STS_W1C: u32 = 0x0301_05FF;

/// GPTIMERnCTL (RM Table 24-12) and the timers' interrupt bits in USBSTS (TI0, TI1; the enables TIE0, TIE1 are the
/// same bits of USBINTR).
const GPT_RUN: u32 = 1 << 31;
const GPT_RST: u32 = 1 << 30;
const GPT_MODE: u32 = 1 << 24;
const GPT_COUNT: u32 = 0x00FF_FFFF;
const TI0: u32 = 1 << 24;

/// PORTSC1 (RM Table 24-33).
const CCS: u32 = 1 << 0;
const PR: u32 = 1 << 8;
const HSP: u32 = 1 << 9;
const PFSC: u32 = 1 << 24;
const PSPD: u32 = 3 << 26;
const PSPD_HIGH: u32 = 2;
/// PTS, PFSC, PHCD, PTC and FPR: the bits software writes.
const PORTSC_WRITABLE: u32 = 0xC000_0000 | PFSC | 1 << 23 | 0x000F_0000 | 1 << 6;

/// OTGSC (RM Table 24-34).
const BSV: u32 = 1 << 11;
const BSE: u32 = 1 << 12;
const BSVIS: u32 = 1 << 19;
const IDPU: u32 = 1 << 5;
/// The interrupt enables (30:24) and the control bits (5:0).
const OTGSC_WRITABLE: u32 = 0x7F00_003F;
/// The interrupt status bits, write 1 to clear.
const OTGSC_STATUS: u32 = 0x007F_0000;

/// EPCRn (RM Tables 24-41 and 24-42).
const RXS: u32 = 1 << 0;
const RXR: u32 = 1 << 6;
const RXE: u32 = 1 << 7;
const TXS: u32 = 1 << 16;
const TXR: u32 = 1 << 22;
const TXE: u32 = 1 << 23;
/// EPCR0's types: always control.
const EPCR0_TYPES: u32 = 0x000C_000C;

/// dQH capabilities and dTD token (RM 24.5.2).
const IOS: u32 = 1 << 15;
const ZLT: u32 = 1 << 29;
const IOC: u32 = 1 << 15;
const ACTIVE: u32 = 0x80;
const HALTED: u32 = 0x40;
const BUFFER_ERROR: u32 = 0x20;
const TOTAL_BYTES: u32 = 0x7FFF_0000;
/// The next-pointer terminate bit.
const T: u32 = 1;

/// The device TX buffer per endpoint, in bytes (RM 24.4.3, HWTXBUF).
const TX_BUFFER: u32 = 64;
/// The endpoints of the MCF5441x's controller (DCCPARAMS DEN).
const ENDPOINTS: usize = 4;

/// The board's wiring and timing of the USB OTG controller.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UsbConfig {
    /// The interrupt controller and source of the USB OTG interrupt (INTC2 source 17 on the chip, RM Table 17-17).
    pub controller: usize,
    pub source: u8,
    /// Ticks per microsecond, for host script delays and the detach delay.
    pub ticks_per_us: u64,
    /// The memory a high-speed IN packet of more than 64 bytes cannot stream from (see [`UsbOtg`]); `None` when
    /// every memory keeps up.
    pub slow_memory: Option<Range<u32>>,
}

/// One general-purpose timer (RM 24.3.2): the count at the tick `anchor`, counting down in microseconds.
#[derive(Debug, Clone, Default)]
struct GpTimer {
    ld: u32,
    run: bool,
    repeat: bool,
    /// The count at `anchor`.
    count: u32,
    anchor: u64,
    /// The tick of the next expiry while counting.
    deadline: Option<u64>,
    /// A one-shot expiry since the last RST: setting RUN again does not expire again.
    expired: bool,
}

impl GpTimer {
    /// The microseconds left at `now`.
    fn count_at(&self, now: u64, tpu: u64) -> u32 {
        match self.deadline {
            Some(_) => {
                let gone = now.saturating_sub(self.anchor) / tpu;
                u64::from(self.count).saturating_sub(gone) as u32
            }
            None => self.count,
        }
    }

    /// Count from `count` µs at `now`.
    fn start(&mut self, count: u32, now: u64, tpu: u64) {
        self.count = count;
        self.anchor = now;
        self.deadline = Some(now + (u64::from(count) + 1) * tpu);
    }

    /// Take the expiries due at `now`; whether there was one.
    fn expire(&mut self, now: u64, tpu: u64) -> bool {
        let Some(d) = self.deadline.filter(|&d| d <= now) else {
            return false;
        };
        if self.repeat {
            // a read in the same tick as the expiry (now == d) reloads with the anchor at d
            let period = (u64::from(self.ld) + 1) * tpu;
            self.anchor = d + (now - d) / period * period;
            self.count = self.ld;
            self.deadline = Some(self.anchor + period);
        } else {
            self.count = 0;
            self.deadline = None;
            self.expired = true;
        }
        true
    }

    fn ctl(&self, now: u64, tpu: u64) -> u32 {
        u32::from(self.run) << 31 | u32::from(self.repeat) << 24 | self.count_at(now, tpu)
    }
}

/// What the device answers a host packet with.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Token {
    /// An IN token's data packet (possibly empty).
    Data(Vec<u8>),
    /// A setup or OUT packet was taken.
    Ack,
    /// The endpoint is not ready; the host tries again later.
    Nak,
    /// The endpoint is stalled.
    Stall,
    /// An OUT packet that does not fit the dTD: the dTD retires with Halted and Data Buffer Error.
    Error,
    /// The device does not answer: it is not running (USBCMD\[RS\] clear) or has another address.
    NoAnswer,
}

/// What a host script does next.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Step {
    /// Run again after this many microseconds (at least one tick later).
    Wait(u64),
    /// Done, with a result: the run stops with [`Stop::Halt`] "usb host done".
    Done(i64),
    /// Failed, with the reason: the run stops with [`Stop::Halt`] "usb host failed" and the reason as its detail.
    Fail(String),
}

/// A USB host, as a state machine the controller runs on the machine's clock (see [`UsbOtg::run_script`]). Each
/// step gets the controller and its context (`cx.now` is the tick) and drives it with the host primitives.
pub trait HostScript {
    /// One step of the host.
    fn step(&mut self, usb: &mut UsbOtg, cx: &mut DeviceCx) -> Step;
}

impl<F: FnMut(&mut UsbOtg, &mut DeviceCx) -> Step> HostScript for F {
    fn step(&mut self, usb: &mut UsbOtg, cx: &mut DeviceCx) -> Step {
        self(usb, cx)
    }
}

/// How the host and the bus behave, for tests of a driver's corner cases.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UsbKnobs {
    /// The host and cable run at high speed (default `true`); else the bus reset ends at full speed.
    pub hs: bool,
    /// Every data-endpoint IN token is NAKed, as an IN that can never complete.
    pub mute_in: bool,
    /// A bus reset ends before the driver's clean-up: PORTSC1\[PR\] clears at once (RM 24.5.3.2.1 step 4).
    pub short_reset: bool,
    /// A bus reset leaves EPCR1-EPCR3's enables set (RM 24.5.3.2.1 does not say it clears them).
    pub reset_keeps_enables: bool,
    /// One-shot: this setup packet arrives just before the driver's next EP0 IN prime.
    pub setup_before_prime: Option<[u8; 8]>,
}

impl Default for UsbKnobs {
    fn default() -> UsbKnobs {
        UsbKnobs {
            hs: true,
            mute_in: false,
            short_reset: false,
            reset_keeps_enables: false,
            setup_before_prime: None,
        }
    }
}

/// What the controller saw, for tests. Kept across controller resets.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct UsbStats {
    /// USBCMD\[RST\] writes.
    pub resets: u64,
    /// USBCMD\[RST\] writes while still attached (PORTSC1\[CCS\]): before a detach reached the bus.
    pub rst_while_attached: u64,
    /// Data-endpoint IN packets that underflowed the TX buffer (see [`UsbOtg`]).
    pub underflows: u64,
    /// The sizes of the data-endpoint IN packets that went out.
    pub in_sizes: Vec<u32>,
    /// Ticks each EP0 prime made after an unread setup stayed primed, until EPFLUSH.
    pub stale_primes: Vec<u64>,
    /// Every accepted register write, (offset, value), in order.
    pub writes: Vec<(u32, u32)>,
}

/// The USB OTG controller (RM ch. 24) in device mode, mapped at 0xFC0B_0000 (a 0x1000 window: the USB host module
/// follows at 0xFC0B_4000) and added with [`Machine::add_bus_master`](crate::Machine::add_bus_master): its dQHs, dTDs
/// and buffers are in memory, reached through the [`Bus`](crate::Bus).
///
/// Registers (RM Table 24-4). Reads of any size return the bytes of the 32-bit word they fall in; writes must be
/// aligned 32-bit, anything else is refused with [`Stop::Unmodelled`]. Every accepted write is recorded
/// ([`UsbStats::writes`]) unless [`UsbOtg::record`] turned the log off.
/// - ID, HWGENERAL, HWHOST, HWDEVICE, HWTXBUF, HWRXBUF, HCIVERSION/CAPLENGTH, HCSPARAMS, HCCPARAMS, DCIVERSION,
///   DCCPARAMS, CONFIGFLAG: constant (RM Table 24-4).
/// - USBCMD (0x0008_0000): stored. RST resets the controller (every register to its reset value; OTGSC keeps its
///   VBUS bits, and the timers' GPTIMERnLD and GPTIMERnCTL keep theirs, measured on an MCF54415) and reads 0. Setting RS is refused unless USBMODE selects device mode with ES = 1 (big-endian
///   buffers): host mode and little-endian buffers are not modelled. Clearing RS while attached detaches 100 µs
///   later (PORTSC1\[CCS\] clears; guess: the PHY takes a while to drop the pull-up), unless RS is set again
///   before then. A setup clears SUTW.
/// - USBSTS (0x80): write 1 to clear. The model sets UI, UEI, PCI, SEI and URI.
/// - USBINTR (0), USBMODE (0), BURSTSIZE (0x0000_1010, measured on an MCF54415; RM Table 24-4 gives 0x0101):
///   stored.
/// - DEVICEADDR (0): USBADR (31:25) only; this part has no USBADRA (RM Table 24-25), so an address takes effect at
///   once. The bus reset clears it.
/// - EPLISTADDR (0): EPBASE (31:11).
/// - PORTSC1 (0xEC00_0004): PTS, PFSC, PHCD, PTC and FPR are written; CCS, PR, HSP and PSPD are the bus's.
/// - OTGSC (0x1020): the interrupt enables and control bits are written, the status bits are write 1 to clear;
///   BSV, BSE and BSVIS follow [`UsbOtg::vbus`].
/// - EPSETUPSR, EPCOMPLETE: write 1 to clear. EPSR: the primed endpoints. EPPRIME and EPFLUSH act and read 0.
/// - EPCR0-EPCR3 (0x0080_0080, 0...): stored; TXR and RXR read 0. EPCR0 is always control and enabled.
/// - GPTIMER0LD/GPTIMER1LD (0x80, 0x88): the 24-bit load value, minus 1 in microseconds. GPTIMER0CTL/GPTIMER1CTL
///   (0x84, 0x8C): RUN (31), RST (30, self-clearing, reads 0), MODE (24, 1 = repeat) and GPTCNT (23:0), which
///   reads the microseconds left. See the timers below.
/// - FRINDEX, ULPI_VIEWPORT and the rest of the window read 0 and ignore writes.
///
/// The interrupt request is asserted while USBSTS and USBINTR share a bit, or an OTGSC status bit is enabled.
///
/// The general-purpose timers count down in microseconds ([`UsbConfig::ticks_per_us`] ticks each) from GPTLD to 0
/// and expire one microsecond later, so a period is GPTLD + 1 µs (a load of 9999 is 10 ms). An expiry sets
/// USBSTS\[TI0\] or \[TI1\] (write 1 to clear), which interrupts while USBINTR\[TIE0\] or \[TIE1\] is set. In
/// repeat mode the count reloads from GPTLD and goes on; in one-shot mode it stays 0 and the timer stops (RUN
/// reads as written). RST reloads the count from GPTLD (measured on an MCF54415) and, with RUN, restarts the
/// period; setting RUN restarts the period from the count the timer holds; clearing RUN holds the count. A GPTLD
/// write does not load the count (measured on an MCF54415). RUN set from a count of 0 expires at once (measured on
/// an MCF54415: TI0 sets, the count reloads from GPTLD and counts down again in repeat mode; guess: exactly
/// when, taken as 1 µs later, and a one-shot that has not expired since its last RST just stops at 0). A one-shot
/// that has expired does not expire again when RUN is set again without RST (measured on an MCF54415: TI0 stays
/// clear, CTL reads 0x8000_0000; guess: also when RUN was cleared in between), until RST reloads it. The timers
/// are event driven ([`Device::next_event`]): nothing steps per tick. They are not part of the controller's reset
/// (measured on an MCF54415: GPTIMER0LD and GPTIMER0CTL keep their values across USBCMD\[RST\]; guess: a running
/// timer keeps counting). While the clock gate is closed ([`Device::gate`]) the count stops and resumes where it
/// stopped.
/// - The new GPTLD of a running timer applies at the next reload.
/// - guess: the expiry comes GPTLD + 1 µs after the load (the count reads 0 for the last microsecond).
///
/// Endpoints 0-3. dQH(ep, dir) is at EPLISTADDR + 64 × (2 × ep + dir) (dir 1 = IN), dTDs are 32 bytes (RM 24.5.2),
/// as big-endian words; the setup buffer at dQH(0, OUT) + 0x28 holds the 8 bytes little-endian within each word.
/// - Prime (EPPRIME): an endpoint whose dQH next pointer is valid and points at an Active dTD loads the dTD into the
///   overlay, makes it current and is primed (EPSR); anything else fails silently.
/// - Buffers: five 4 KB page pointers, the first with the offset; a transfer crosses pages.
/// - Retire: the token goes back to the dTD and the overlay with Active cleared and the status; EPCOMPLETE, UI on IOC,
///   UEI on an error. A cleanly retired dTD whose next dTD is valid and Active loads it and stays primed; otherwise
///   the endpoint is unprimed and the overlay's next pointer is the dTD's.
///
/// Packets move when the host asks ([`UsbOtg::setup`], [`UsbOtg::in_token`], [`UsbOtg::out_packet`]), and the
/// answer is immediate. The device answers only while running (USBCMD\[RS\]) and at its address. A high-speed IN
/// of more than 64 bytes from [`UsbConfig::slow_memory`] never completes: every IN token is NAKed and counted in
/// [`UsbStats::underflows`] (measured on an MCF54415 with the buffer in DDR; from SRAM the same IN completes). The TX
/// buffer is 64 bytes (RM 24.4.3, HWTXBUF), the rest streams from memory on the wire, and an underflow is a forced
/// bit-stuff error (RM Table 24-55) the controller retries without retiring the dTD.
///
/// Not modelled: host mode, isochronous endpoints, data toggles, SOF and FRINDEX, suspend and resume, the ATDTW
/// tripwire, the NAK registers, TXFILLTUNING and the ULPI viewport. These halt the run with
/// [`Stop::Unmodelled`] (refused when a register access reaches them): a data-endpoint IN with packets over 64 bytes at full speed (64 is bulk's limit), and a DMA access that
/// fails (the RM's system error for it is not modelled).
pub struct UsbOtg {
    config: UsbConfig,
    usbcmd: u32,
    usbsts: u32,
    usbintr: u32,
    deviceaddr: u32,
    eplist: u32,
    burstsize: u32,
    portsc: u32,
    otgsc: u32,
    usbmode: u32,
    setupsr: u32,
    epsr: u32,
    complete: u32,
    epcr: [u32; ENDPOINTS],
    timers: [GpTimer; 2],
    /// The tick the clock gate closed, while it is closed.
    gated_at: Option<u64>,
    /// A bus reset is in progress (between `bus_reset_start` and `bus_reset_end`).
    in_reset: bool,
    /// The driver flushed the endpoints during the bus reset.
    reset_acked: bool,
    /// The tick at which PORTSC1\[CCS\] clears after RS was cleared.
    detach_at: Option<u64>,
    /// The transfer position in each endpoint direction's current dTD.
    pos: [[u32; 2]; ENDPOINTS],
    /// The tick of an EP0 prime made after an unread setup.
    stale_since: Option<u64>,
    knobs: UsbKnobs,
    stats: UsbStats,
    record: bool,
    script: Option<Box<dyn HostScript>>,
    next: Option<u64>,
    result: Option<Result<i64, String>>,
}

impl fmt::Debug for UsbOtg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("UsbOtg")
            .field("usbcmd", &self.usbcmd)
            .field("usbsts", &self.usbsts)
            .field("portsc", &self.portsc)
            .field("epsr", &self.epsr)
            .field("script_next", &self.next)
            .finish_non_exhaustive()
    }
}

impl UsbOtg {
    /// A controller at reset (RM Table 24-4), with VBUS off.
    pub fn new(config: UsbConfig) -> UsbOtg {
        UsbOtg {
            config,
            usbcmd: USBCMD_RESET,
            usbsts: USBSTS_RESET,
            usbintr: 0,
            deviceaddr: 0,
            eplist: 0,
            burstsize: BURSTSIZE_RESET,
            portsc: PORTSC_RESET,
            otgsc: OTGSC_RESET,
            usbmode: 0,
            setupsr: 0,
            epsr: 0,
            complete: 0,
            epcr: [EPCR0_RESET, 0, 0, 0],
            timers: Default::default(),
            gated_at: None,
            in_reset: false,
            reset_acked: false,
            detach_at: None,
            pos: [[0; 2]; ENDPOINTS],
            stale_since: None,
            knobs: UsbKnobs::default(),
            stats: UsbStats::default(),
            record: true,
            script: None,
            next: None,
            result: None,
        }
    }

    /// The host and bus behaviour, to change.
    pub fn knobs(&mut self) -> &mut UsbKnobs {
        &mut self.knobs
    }

    /// What the controller saw.
    pub fn stats(&self) -> &UsbStats {
        &self.stats
    }

    /// Every accepted register write so far, emptying [`UsbStats::writes`].
    pub fn take_writes(&mut self) -> Vec<(u32, u32)> {
        std::mem::take(&mut self.stats.writes)
    }

    /// Whether register writes are logged in [`UsbStats::writes`] (default on). Off changes only that log.
    pub fn record(&mut self, on: bool) {
        self.record = on;
    }

    /// Whether the driver runs the controller (USBCMD\[RS\]): the device is on the bus.
    pub fn running(&self) -> bool {
        self.usbcmd & RS != 0
    }

    /// The device's address (DEVICEADDR\[USBADR\]).
    pub fn address(&self) -> u8 {
        (self.deviceaddr >> 25) as u8
    }

    /// Whether the port is at high speed (PORTSC1\[HSP\]): set by [`UsbOtg::bus_reset_end`] (see [`UsbKnobs::hs`]).
    pub fn high_speed(&self) -> bool {
        self.portsc & HSP != 0
    }

    /// Whether the driver flushed the endpoints (EPFLUSH) since the bus reset began.
    pub fn reset_acked(&self) -> bool {
        self.reset_acked
    }

    /// Run `script` from tick `at` on, from the controller's own events: each [`Step::Wait`] of `us` microseconds
    /// runs the next step `max(1, us × ticks_per_us)` ticks later. A finished script stops the run with
    /// [`Stop::Halt`] "usb host done" (or "usb host failed"), and its result is [`UsbOtg::script_result`]. Replaces
    /// any script running.
    pub fn run_script(&mut self, script: Box<dyn HostScript>, at: u64) {
        self.script = Some(script);
        self.next = Some(at);
        self.result = None;
    }

    /// The result of the last script that finished: [`Step::Done`]'s value or [`Step::Fail`]'s reason.
    pub fn script_result(&self) -> Option<&Result<i64, String>> {
        self.result.as_ref()
    }

    // the host's side ---------------------------------------------------------------------------------------------

    /// VBUS on or off: OTGSC\[BSV\] or \[BSE\], and BSVIS.
    pub fn vbus(&mut self, on: bool, cx: &mut DeviceCx) {
        self.settle(cx.now);
        self.otgsc = self.otgsc & !(BSV | BSE) | if on { BSV } else { BSE } | BSVIS;
        self.signal(cx);
    }

    /// A system bus error: USBSTS\[SEI\]. In device mode it only interrupts (RM Table 24-20).
    pub fn system_error(&mut self, cx: &mut DeviceCx) {
        self.settle(cx.now);
        self.usbsts |= SEI;
        self.signal(cx);
    }

    /// The host starts a bus reset: PORTSC1\[PR\] (cleared at once with [`UsbKnobs::short_reset`]), DEVICEADDR and
    /// EPSR cleared, EPCR1-EPCR3's enables cleared (unless [`UsbKnobs::reset_keeps_enables`]), USBSTS\[URI\]. The
    /// driver acknowledges it with EPFLUSH ([`UsbOtg::reset_acked`]).
    pub fn bus_reset_start(&mut self, cx: &mut DeviceCx) {
        self.settle(cx.now);
        self.portsc |= PR;
        if self.knobs.short_reset {
            self.portsc &= !PR;
        }
        self.deviceaddr = 0;
        self.epsr = 0;
        if !self.knobs.reset_keeps_enables {
            for e in &mut self.epcr[1..] {
                *e &= !(TXE | RXE);
            }
        }
        self.usbsts |= URI;
        self.in_reset = true;
        self.reset_acked = false;
        self.signal(cx);
    }

    /// The bus reset ends: PR clears, the port is connected (CCS) at high speed when [`UsbKnobs::hs`] and
    /// PORTSC1\[PFSC\] allow it, else at full speed; USBSTS\[PCI\].
    pub fn bus_reset_end(&mut self, cx: &mut DeviceCx) {
        self.settle(cx.now);
        let speed = if self.knobs.hs && self.portsc & PFSC == 0 {
            PSPD_HIGH
        } else {
            0
        };
        let hsp = if speed == PSPD_HIGH { HSP } else { 0 };
        self.portsc = self.portsc & !(PR | PSPD | HSP) | CCS | speed << 26 | hsp;
        self.usbsts |= PCI;
        self.in_reset = false;
        self.signal(cx);
    }

    /// A setup packet to EP0 at device address `addr`: [`Token::Ack`], or [`Token::NoAnswer`] (the packet is lost).
    /// It lands in dQH(0, OUT)'s setup buffer, clears USBCMD\[SUTW\] and EP0's stalls, unprimes EP0 both ways, sets
    /// EPSETUPSR bit 0 and, when the dQH has IOS, USBSTS\[UI\].
    pub fn setup(&mut self, data: [u8; 8], addr: u8, cx: &mut DeviceCx) -> Token {
        self.settle(cx.now);
        if !self.addressed(addr) {
            return Token::NoAnswer;
        }
        match self.deliver_setup(data, cx) {
            Ok(()) => Token::Ack,
            Err(why) => self.fail(why, Token::Error, cx),
        }
    }

    /// An IN token to endpoint `ep` (0-3) at device address `addr`: the next packet of the current dTD (up to the
    /// max packet size), [`Token::Stall`], [`Token::Nak`] (not primed, not enabled, or not able to send) or
    /// [`Token::NoAnswer`]. A packet shorter than the max packet size, or the last, retires the dTD.
    pub fn in_token(&mut self, ep: u8, addr: u8, cx: &mut DeviceCx) -> Token {
        let ep = usize::from(ep);
        assert!(ep < ENDPOINTS, "USB endpoint {ep} out of range");
        self.settle(cx.now);
        if !self.addressed(addr) {
            return Token::NoAnswer;
        }
        if self.epcr[ep] & TXS != 0 {
            return Token::Stall;
        }
        if (ep > 0 && self.epcr[ep] & TXE == 0) || self.epsr & bit(ep, 1) == 0 {
            return Token::Nak;
        }
        match self.send(ep, cx) {
            Ok(t) => t,
            Err(why) => self.fail(why, Token::Nak, cx),
        }
    }

    /// An OUT packet of `data` to endpoint `ep` (0-3) at device address `addr`: [`Token::Ack`], [`Token::Error`]
    /// (larger than the max packet size or what the dTD has left: the dTD retires with Halted and Data Buffer Error),
    /// [`Token::Stall`], [`Token::Nak`] or [`Token::NoAnswer`]. A packet shorter than the max packet size, or one
    /// that fills the dTD, retires it.
    pub fn out_packet(&mut self, ep: u8, data: &[u8], addr: u8, cx: &mut DeviceCx) -> Token {
        let ep = usize::from(ep);
        assert!(ep < ENDPOINTS, "USB endpoint {ep} out of range");
        self.settle(cx.now);
        if !self.addressed(addr) {
            return Token::NoAnswer;
        }
        if self.epcr[ep] & RXS != 0 {
            return Token::Stall;
        }
        if (ep > 0 && self.epcr[ep] & RXE == 0) || self.epsr & bit(ep, 0) == 0 {
            return Token::Nak;
        }
        match self.receive(ep, data, cx) {
            Ok(t) => t,
            Err(why) => self.fail(why, Token::Nak, cx),
        }
    }

    // internals ---------------------------------------------------------------------------------------------------

    /// Halt the run with `why` (a host action has no access in flight) and answer `answer`.
    fn fail(&mut self, why: String, answer: Token, cx: &mut DeviceCx) -> Token {
        cx.halt(Stop::Unmodelled, why);
        self.signal(cx);
        answer
    }

    /// Apply a detach that has come due.
    fn settle(&mut self, now: u64) {
        if let Some(t) = self.detach_at
            && now >= t
        {
            self.portsc &= !CCS;
            self.detach_at = None;
        }
    }

    fn addressed(&self, addr: u8) -> bool {
        self.running() && addr == self.address()
    }

    fn signal(&self, cx: &mut DeviceCx) {
        let on = self.usbsts & self.usbintr & STS_W1C != 0
            || self.otgsc & self.otgsc >> 8 & OTGSC_STATUS != 0;
        cx.irq.set(self.config.controller, self.config.source, on);
    }

    /// The controller reset (USBCMD\[RST\]).
    fn reset(&mut self) {
        let keep = UsbOtg {
            otgsc: self.otgsc & (BSV | BSE) | IDPU,
            knobs: self.knobs.clone(),
            stats: std::mem::take(&mut self.stats),
            script: self.script.take(),
            next: self.next,
            result: self.result.take(),
            // measured on an MCF54415: GPTIMERnLD and GPTIMERnCTL keep their values
            timers: std::mem::take(&mut self.timers),
            ..UsbOtg::new(self.config.clone())
        };
        *self = keep;
    }

    fn qh(&self, ep: usize, dir: usize) -> u32 {
        self.eplist + 64 * (2 * ep + dir) as u32
    }

    fn rd(&self, a: u32, cx: &mut DeviceCx) -> Result<u32, String> {
        let mut bus = cx.bus().expect("the USB OTG controller is a bus master");
        bus.read(a, Size::Long)
            .map_err(|f| format!("USB: bus error reading {a:#010x} ({f:?})"))
    }

    fn wr(&self, a: u32, v: u32, cx: &mut DeviceCx) -> Result<(), String> {
        let mut bus = cx.bus().expect("the USB OTG controller is a bus master");
        bus.write(a, Size::Long, v)
            .map_err(|f| format!("USB: bus error writing {a:#010x} ({f:?})"))
    }

    /// The address of byte `p` of the transfer of dQH `q`, through its page pointers.
    fn buffer(&self, q: u32, p: u32, cx: &mut DeviceCx) -> Result<u32, String> {
        let page0 = self.rd(q + 0x10, cx)?;
        let lin = (page0 & 0xFFF) + p;
        let k = lin >> 12;
        let base = match k {
            0 => page0 & !0xFFF,
            1..=4 => self.rd(q + 0x10 + 4 * k, cx)? & !0xFFF,
            _ => {
                return Err(format!(
                    "USB: transfer at byte {p} runs past the dTD's five buffer pages"
                ));
            }
        };
        Ok(base + (lin & 0xFFF))
    }

    /// Read `n` bytes of the transfer of dQH `q` at position `p`, across pages.
    fn dma_read(&self, q: u32, p: u32, n: u32, cx: &mut DeviceCx) -> Result<Vec<u8>, String> {
        let mut out = vec![0; n as usize];
        let mut i = 0;
        while i < n {
            let a = self.buffer(q, p + i, cx)?;
            let k = (n - i).min(0x1000 - (a & 0xFFF));
            let mut bus = cx.bus().expect("the USB OTG controller is a bus master");
            bus.read_bytes(a, &mut out[i as usize..(i + k) as usize])
                .map_err(|f| format!("USB: bus error reading {a:#010x} ({f:?})"))?;
            i += k;
        }
        Ok(out)
    }

    /// Write `data` into the transfer of dQH `q` at position `p`, across pages.
    fn dma_write(&self, q: u32, p: u32, data: &[u8], cx: &mut DeviceCx) -> Result<(), String> {
        let n = data.len() as u32;
        let mut i = 0;
        while i < n {
            let a = self.buffer(q, p + i, cx)?;
            let k = (n - i).min(0x1000 - (a & 0xFFF));
            let mut bus = cx.bus().expect("the USB OTG controller is a bus master");
            bus.write_bytes(a, &data[i as usize..(i + k) as usize])
                .map_err(|f| format!("USB: bus error writing {a:#010x} ({f:?})"))?;
            i += k;
        }
        Ok(())
    }

    /// Whether `next` is a valid pointer to an Active dTD.
    fn active(&self, next: u32, cx: &mut DeviceCx) -> Result<bool, String> {
        Ok(next & T == 0 && self.rd((next & !0x1F) + 4, cx)? & ACTIVE != 0)
    }

    /// Copy dTD `dtd` into the overlay of dQH `q` and make it current.
    fn load(
        &mut self,
        q: u32,
        dtd: u32,
        ep: usize,
        dir: usize,
        cx: &mut DeviceCx,
    ) -> Result<(), String> {
        for i in 0..7 {
            let v = self.rd(dtd + 4 * i, cx)?;
            self.wr(q + 8 + 4 * i, v, cx)?;
        }
        self.wr(q + 4, dtd, cx)?;
        self.pos[ep][dir] = 0;
        Ok(())
    }

    fn prime(&mut self, v: u32, cx: &mut DeviceCx) -> Result<(), String> {
        for ep in 0..ENDPOINTS {
            for dir in 0..2 {
                if v & bit(ep, dir) == 0 {
                    continue;
                }
                let q = self.qh(ep, dir);
                let next = self.rd(q + 8, cx)?;
                if self.active(next, cx)? {
                    self.load(q, next & !0x1F, ep, dir, cx)?;
                    self.epsr |= bit(ep, dir);
                }
            }
        }
        Ok(())
    }

    /// Retire the current dTD of an endpoint direction with `status`.
    fn retire(
        &mut self,
        ep: usize,
        dir: usize,
        status: u32,
        cx: &mut DeviceCx,
    ) -> Result<(), String> {
        let q = self.qh(ep, dir);
        let cur = self.rd(q + 4, cx)? & !0x1F;
        let token = self.rd(q + 0xC, cx)? & !0xFF | status;
        self.wr(cur + 4, token, cx)?;
        self.wr(q + 0xC, token, cx)?;
        self.complete |= bit(ep, dir);
        if token & IOC != 0 {
            self.usbsts |= UI;
        }
        if status != 0 {
            self.usbsts |= UEI;
        }
        let next = self.rd(cur, cx)?;
        if status == 0 && self.active(next, cx)? {
            self.load(q, next & !0x1F, ep, dir, cx)
        } else {
            self.epsr &= !bit(ep, dir);
            self.wr(q + 8, next, cx)
        }
    }

    fn deliver_setup(&mut self, data: [u8; 8], cx: &mut DeviceCx) -> Result<(), String> {
        let q = self.qh(0, 0);
        let [a, b, c, d, e, f, g, h] = data;
        self.wr(q + 0x28, u32::from_le_bytes([a, b, c, d]), cx)?;
        self.wr(q + 0x2C, u32::from_le_bytes([e, f, g, h]), cx)?;
        self.usbcmd &= !SUTW;
        self.epcr[0] &= !(TXS | RXS);
        // a setup drops EP0's primed stages
        self.epsr &= !(bit(0, 0) | bit(0, 1));
        self.setupsr |= 1;
        if self.rd(q, cx)? & IOS != 0 {
            self.usbsts |= UI;
        }
        self.signal(cx);
        Ok(())
    }

    fn send(&mut self, ep: usize, cx: &mut DeviceCx) -> Result<Token, String> {
        let q = self.qh(ep, 1);
        let (cap, token) = (self.rd(q, cx)?, self.rd(q + 0xC, cx)?);
        let (mps, total) = (cap >> 16 & 0x7FF, token >> 16 & 0x7FFF);
        let high_speed = self.portsc >> 26 & 3 == PSPD_HIGH;
        if ep > 0 && mps > 64 && !high_speed {
            return Err(format!(
                "USB: EP{ep} IN with {mps}-byte packets at full speed (64 at most for bulk)"
            ));
        }
        let (n, p) = (total.min(mps), self.pos[ep][1]);
        if ep > 0 && self.knobs.mute_in {
            return Ok(Token::Nak);
        }
        if ep > 0 && n > TX_BUFFER {
            let at = self.buffer(q, p, cx)?;
            if self
                .config
                .slow_memory
                .as_ref()
                .is_some_and(|r| r.contains(&at))
            {
                // measured on an MCF54415: a high-speed IN of more than 64 bytes from DDR never completes (the TX
                // buffer underflows into a forced bit-stuff error, retried for ever); from SRAM it does
                self.stats.underflows += 1;
                return Ok(Token::Nak);
            }
        }
        let data = self.dma_read(q, p, n, cx)?;
        if ep > 0 {
            self.stats.in_sizes.push(n);
        }
        self.pos[ep][1] = p + n;
        let total = total - n;
        self.wr(q + 0xC, token & !TOTAL_BYTES | total << 16, cx)?;
        // ZLT clear: a transfer ending on a full packet is closed by a zero-length packet, sent on the next IN token
        let zlp_due = cap & ZLT == 0 && total == 0 && n == mps && n > 0;
        if (total == 0 && !zlp_due) || n < mps {
            self.retire(ep, 1, 0, cx)?;
        }
        self.signal(cx);
        Ok(Token::Data(data))
    }

    fn receive(&mut self, ep: usize, data: &[u8], cx: &mut DeviceCx) -> Result<Token, String> {
        let q = self.qh(ep, 0);
        let (cap, token) = (self.rd(q, cx)?, self.rd(q + 0xC, cx)?);
        let (mps, total) = (cap >> 16 & 0x7FF, token >> 16 & 0x7FFF);
        let n = data.len() as u32;
        if n > mps || n > total {
            self.retire(ep, 0, HALTED | BUFFER_ERROR, cx)?;
            self.signal(cx);
            return Ok(Token::Error);
        }
        let p = self.pos[ep][0];
        self.dma_write(q, p, data, cx)?;
        self.pos[ep][0] = p + n;
        let total = total - n;
        self.wr(q + 0xC, token & !TOTAL_BYTES | total << 16, cx)?;
        if n < mps || total == 0 {
            self.retire(ep, 0, 0, cx)?;
        }
        self.signal(cx);
        Ok(Token::Ack)
    }

    fn read32(&self, off: u32, now: u64) -> u32 {
        if let Some(&(_, v)) = CONSTANTS.iter().find(|&&(o, _)| o == off) {
            return v;
        }
        match off {
            GPTIMER0LD..=GPTIMER1CTL => {
                let (n, t) = (((off - GPTIMER0LD) / 8) as usize, self.tpu());
                if off & 4 == 0 {
                    self.timers[n].ld
                } else {
                    self.timers[n].ctl(now, t)
                }
            }
            USBCMD => self.usbcmd,
            USBSTS => self.usbsts,
            USBINTR => self.usbintr,
            DEVICEADDR => self.deviceaddr,
            EPLISTADDR => self.eplist,
            BURSTSIZE => self.burstsize,
            PORTSC1 => self.portsc,
            OTGSC => self.otgsc,
            USBMODE => self.usbmode,
            EPSETUPSR => self.setupsr,
            EPSR => self.epsr,
            EPCOMPLETE => self.complete,
            EPCR0..=EPCR3 => self.epcr[((off - EPCR0) / 4) as usize],
            _ => 0,
        }
    }

    fn write32(&mut self, off: u32, v: u32, cx: &mut DeviceCx) -> Result<(), String> {
        match off {
            GPTIMER0LD..=GPTIMER1CTL => self.write_gptimer(off, v, cx.now),
            USBCMD => return self.write_usbcmd(v, cx.now),
            USBSTS => self.usbsts &= !(v & STS_W1C),
            USBINTR => self.usbintr = v,
            DEVICEADDR => self.deviceaddr = v & 0xFE00_0000,
            EPLISTADDR => self.eplist = v & 0xFFFF_F800,
            BURSTSIZE => self.burstsize = v,
            PORTSC1 => self.portsc = self.portsc & !PORTSC_WRITABLE | v & PORTSC_WRITABLE,
            OTGSC => {
                self.otgsc = self.otgsc & !OTGSC_WRITABLE | v & OTGSC_WRITABLE;
                self.otgsc &= !(v & OTGSC_STATUS);
            }
            USBMODE => self.usbmode = v,
            EPSETUPSR => self.setupsr &= !v,
            EPPRIME => {
                if v & bit(0, 1) != 0
                    && let Some(data) = self.knobs.setup_before_prime.take()
                    && self.running()
                {
                    self.deliver_setup(data, cx)?;
                }
                self.prime(v, cx)?;
                if self.setupsr & 1 != 0
                    && self.epsr & (bit(0, 0) | bit(0, 1)) != 0
                    && self.stale_since.is_none()
                {
                    // primed for a control transfer the host has replaced
                    self.stale_since = Some(cx.now);
                }
            }
            EPFLUSH => {
                self.epsr &= !v;
                if let Some(t) = self.stale_since
                    && self.epsr & (bit(0, 0) | bit(0, 1)) == 0
                {
                    self.stats.stale_primes.push(cx.now - t);
                    self.stale_since = None;
                }
                if self.in_reset {
                    self.reset_acked = true;
                }
            }
            EPCOMPLETE => self.complete &= !v,
            EPCR0..=EPCR3 => {
                let n = ((off - EPCR0) / 4) as usize;
                // the data toggle resets self-clear (toggles are not modelled)
                let v = v & !(TXR | RXR);
                self.epcr[n] = if n == 0 {
                    v & !EPCR0_TYPES | TXE | RXE
                } else {
                    v
                };
            }
            _ => {}
        }
        Ok(())
    }

    /// Ticks per microsecond, at least 1.
    fn tpu(&self) -> u64 {
        self.config.ticks_per_us.max(1)
    }

    fn write_gptimer(&mut self, off: u32, v: u32, now: u64) {
        let (n, tpu) = (((off - GPTIMER0LD) / 8) as usize, self.tpu());
        let t = &mut self.timers[n];
        let was = t.count_at(now, tpu);
        if off & 4 == 0 {
            t.ld = v & GPT_COUNT;
            // measured on an MCF54415: an LD write does not load the count, stopped or not
            return;
        }
        let run = v & GPT_RUN != 0;
        let rising = run && !t.run;
        t.repeat = v & GPT_MODE != 0;
        if v & GPT_RST != 0 {
            t.count = t.ld;
            t.deadline = None;
            t.expired = false;
        }
        t.run = run;
        if !run {
            if v & GPT_RST == 0 {
                t.count = was; // clearing RUN holds the count
            }
            t.deadline = None;
        } else if v & GPT_RST != 0 || rising {
            // RUN rising counts from the count the timer holds; from 0 it expires at the next microsecond
            // boundary (measured on an MCF54415: it expires at once, reloads LD and goes on; guess: exactly when,
            // taken as 1 µs)
            let from = if v & GPT_RST != 0 { t.count } else { was };
            // measured on an MCF54415: an expired one-shot set running again (no RST) does not expire again
            if !(from == 0 && !t.repeat && t.expired) {
                t.start(from, now, tpu);
            }
        }
    }

    fn write_usbcmd(&mut self, v: u32, now: u64) -> Result<(), String> {
        if v & RST != 0 {
            self.stats.resets += 1;
            if self.portsc & CCS != 0 {
                self.stats.rst_while_attached += 1;
            }
            self.reset();
            return Ok(());
        }
        if v & RS != 0 && !self.running() && (self.usbmode & 3 != 2 || self.usbmode & 4 == 0) {
            return Err(format!(
                "USB: run (USBCMD RS) with USBMODE {:#x} (device mode with ES = 1 only)",
                self.usbmode
            ));
        }
        if self.running() && v & RS == 0 && self.portsc & CCS != 0 {
            // guess: the PHY drops the pull-up 100 µs after RS clears
            self.detach_at = Some(now + 100 * self.config.ticks_per_us);
        }
        if v & RS != 0 {
            // guess: setting RS again before the detach keeps the pull-up
            self.detach_at = None;
        }
        self.usbcmd = v;
        Ok(())
    }
}

/// The bit of an endpoint direction in EPPRIME, EPFLUSH, EPSR and EPCOMPLETE.
fn bit(ep: usize, dir: usize) -> u32 {
    1 << (ep + 16 * dir)
}

impl Device for UsbOtg {
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32 {
        let (n, lane) = (size.bytes(), offset & 3);
        if lane + n > 4 {
            cx.refuse(
                Stop::Unmodelled,
                format!("USB: {n}-byte read at +{offset:#x} across a register"),
            );
            return 0;
        }
        self.settle(cx.now);
        self.read32(offset & !3, cx.now) >> (8 * (4 - n - lane))
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        if size != Size::Long || !offset.is_multiple_of(4) {
            return cx.refuse(
                Stop::Unmodelled,
                format!(
                    "USB: {}-byte write at +{offset:#x} (aligned 32-bit writes only)",
                    size.bytes()
                ),
            );
        }
        self.settle(cx.now);
        match self.write32(offset, value, cx) {
            Ok(()) => {
                if self.record {
                    self.stats.writes.push((offset, value));
                }
            }
            Err(why) => cx.refuse(Stop::Unmodelled, why),
        }
        self.signal(cx);
    }

    fn next_event(&self) -> Option<u64> {
        let timers = self.timers.iter().filter_map(|t| t.deadline);
        timers.chain(self.next).min()
    }

    fn advance(&mut self, now: u64, cx: &mut DeviceCx) {
        let tpu = self.tpu();
        let mut fired = false;
        for (i, t) in self.timers.iter_mut().enumerate() {
            if t.expire(now, tpu) {
                self.usbsts |= TI0 << i;
                fired = true;
            }
        }
        if fired {
            self.signal(cx);
        }
        if self.next.is_none_or(|t| now < t) {
            return;
        }
        let Some(mut script) = self.script.take() else {
            self.next = None;
            return;
        };
        self.settle(now);
        let step = script.step(self, cx);
        if self.script.is_some() {
            return; // the step started another script
        }
        match step {
            Step::Wait(us) => {
                let ticks = us.saturating_mul(self.config.ticks_per_us).max(1);
                self.next = Some(now.saturating_add(ticks));
                self.script = Some(script);
            }
            Step::Done(v) => {
                self.next = None;
                self.result = Some(Ok(v));
                cx.halt(Stop::Halt("usb host done"), format!("result {v}"));
            }
            Step::Fail(why) => {
                self.next = None;
                self.result = Some(Err(why.clone()));
                cx.halt(Stop::Halt("usb host failed"), why);
            }
        }
    }

    fn poll_counted(&self) -> bool {
        false
    }

    // guess: a gated controller's timers stop counting and resume where they stopped (not measured).
    fn gate(&mut self, closed: bool, now: u64) {
        if closed {
            self.gated_at.get_or_insert(now);
        } else if let Some(at) = self.gated_at.take() {
            let d = now.saturating_sub(at);
            for t in &mut self.timers {
                t.anchor += d;
                t.deadline = t.deadline.map(|x| x + d);
            }
        }
    }

    /// It serves no DMA requests: its bus is for its own descriptors and buffers.
    fn serves(&self, _dma: &DmaLines) -> bool {
        false
    }

    /// The USB interrupt while a host script runs or a timer with its enable counts: nothing else raises it by
    /// itself.
    fn wake_sources(&self, out: &mut Vec<(usize, u8)>) -> bool {
        let timer = self
            .timers
            .iter()
            .enumerate()
            .any(|(i, t)| t.deadline.is_some() && self.usbintr & TI0 << i != 0);
        if self.script.is_some() || timer {
            out.push((self.config.controller, self.config.source));
        }
        true
    }
}
