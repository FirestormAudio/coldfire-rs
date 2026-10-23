//! eDMA through the HAL's channels and TCDs, memory to memory on channels 30 and 31 (two of the channels RM Table
//! 19-6 leaves to software), each started by CSR[START]: copies at 8, 16 and 32 bits, a source ring (SMOD), SLAST
//! and DLAST, the major-loop interrupt, and a major-loop link from channel 30 to 31.
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use coldfire_rt::entry;
use embassy_mcf5441x::bind_interrupts;
use embassy_mcf5441x::dma::{attr, csr, Channel, Size, Tcd};
use embassy_mcf5441x::interrupt::typelevel::{EDMA_INTR_INT30, Handler, Interrupt};
use embassy_mcf5441x::interrupt::Priority;
use embassy_mcf5441x::pac;
use mcf5441x_hal_tests::{check, check_eq, pass, say};

const REF_HZ: u32 = 24_000_000;
/// TCD CSR: link channel MAJORLINKCH when the major loop completes (MAJORELINK, RM 19.3.26).
const MAJORELINK: u16 = 1 << 5;
const fn majorlinkch(ch: u16) -> u16 {
    ch << 8
}

/// A buffer aligned for the 16-byte source ring: SMOD wraps the low address bits (RM 19.3.17).
#[repr(C, align(16))]
struct Buf([u8; 256]);

static mut SRC: Buf = Buf([0; 256]);
static mut DST: Buf = Buf([0; 256]);
static mut DST2: Buf = Buf([0; 256]);
static MAJOR_DONE: AtomicBool = AtomicBool::new(false);

/// Channel 30's major-loop interrupt: clear it (CINT) and say so.
struct MajorDone;

impl Handler<EDMA_INTR_INT30> for MajorDone {
    unsafe fn on_interrupt() {
        pac::EDMA.cint().write(|w| w.set_cint(30));
        MAJOR_DONE.store(true, Ordering::Release);
    }
}

bind_interrupts!(struct Irqs { EDMA_INTR_INT30 => MajorDone; });

fn src() -> *mut u8 {
    &raw mut SRC as *mut u8
}

fn dst() -> *mut u8 {
    &raw mut DST as *mut u8
}

fn dst2() -> *mut u8 {
    &raw mut DST2 as *mut u8
}

fn clear(p: *mut u8) {
    for i in 0..256 {
        // SAFETY: the buffers are 256 bytes and no channel runs now
        unsafe { p.add(i).write_volatile(0) };
    }
}

fn byte(p: *mut u8, i: usize) -> u8 {
    // SAFETY: as in `clear`; the channel has finished
    unsafe { p.add(i).read_volatile() }
}

/// One major iteration of `nbytes` from `s` to `d`, `size` each way, with `extra` CSR bits.
fn tcd(s: *mut u8, d: *mut u8, size: Size, step: i16, nbytes: u32, extra: u16) -> Tcd {
    Tcd {
        saddr: s as u32,
        soff: step,
        attr: attr(size, 0, size, 0),
        nbytes,
        slast: 0,
        daddr: d as u32,
        doff: step,
        citer: 1,
        dlast_sga: 0,
        biter: 1,
        csr: extra,
    }
}

/// Start `ch` with `t` and wait for DONE.
fn run(ch: &mut Channel, t: &Tcd) {
    ch.clear_done();
    // SAFETY: the buffers are statics
    unsafe { ch.configure(&Tcd { csr: t.csr | csr::START, ..*t }) };
    while ch.csr() & csr::DONE == 0 {}
}

#[entry]
fn main() -> ! {
    let p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(REF_HZ));
    let _ = Irqs;
    let mut ch30 = Channel::new(p.DMA_CH30);
    let mut ch31 = Channel::new(p.DMA_CH31);
    for i in 0..256 {
        // SAFETY: no channel runs yet
        unsafe { src().add(i).write_volatile((i * 3 + 1) as u8) };
    }

    for (size, step) in [(Size::Bits8, 1), (Size::Bits16, 2), (Size::Bits32, 4)] {
        say!("copy {:?}", size);
        clear(dst());
        run(&mut ch30, &tcd(src(), dst(), size, step, 256, 0));
        for i in 0..256 {
            check_eq!(byte(dst(), i), byte(src(), i));
        }
    }

    say!("source ring");
    clear(dst());
    let mut t = tcd(src(), dst(), Size::Bits32, 4, 64, 0);
    t.attr = attr(Size::Bits32, 4, Size::Bits32, 0); // a 16-byte source ring
    run(&mut ch30, &t);
    for i in 0..64 {
        check_eq!(byte(dst(), i), byte(src(), i % 16));
    }

    say!("slast and dlast");
    let mut t = tcd(src(), dst(), Size::Bits32, 4, 16, 0);
    t.slast = -16;
    t.dlast_sga = 64;
    run(&mut ch30, &t);
    check_eq!(pac::EDMA.tcd(30).saddr().read().0, src() as u32);
    check_eq!(ch30.daddr(), dst() as u32 + 16 + 64);

    say!("major-loop interrupt");
    EDMA_INTR_INT30::set_priority(Priority::P3);
    // SAFETY: the handler above is ready; the channel's request is cleared in it
    unsafe {
        EDMA_INTR_INT30::enable();
        coldfire::interrupt::enable();
    }
    run(&mut ch30, &tcd(src(), dst(), Size::Bits32, 4, 32, csr::INT_MAJOR));
    while !MAJOR_DONE.load(Ordering::Acquire) {}
    check!(MAJOR_DONE.load(Ordering::Acquire));
    EDMA_INTR_INT30::disable();

    say!("major-loop link");
    clear(dst());
    clear(dst2());
    ch31.clear_done();
    // SAFETY: the buffers are statics; channel 31 is started by the link
    unsafe { ch31.configure(&tcd(src(), dst2(), Size::Bits8, 1, 128, 0)) };
    run(&mut ch30, &tcd(src(), dst(), Size::Bits8, 1, 128, MAJORELINK | majorlinkch(31)));
    while ch31.csr() & csr::DONE == 0 {}
    for i in 0..128 {
        check_eq!(byte(dst(), i), byte(src(), i));
        check_eq!(byte(dst2(), i), byte(src(), i));
    }
    pass()
}
