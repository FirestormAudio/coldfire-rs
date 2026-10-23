//! The interrupt controllers INTC0-2 and the global IACK registers (RM ch. 17), driven through their registers.

mod common;

use common::{STACK, prepare, set_vector, until};
use mcf5441x_emu::soc::{Edma, Intc};
use mcf5441x_emu::{DeviceId, Fault, InterruptController, Irq, Limit, Machine, Size, Stop};

const INTC0: u32 = 0xFC04_8000;
const INTC1: u32 = 0xFC04_C000;
const INTC2: u32 = 0xFC05_0000;
/// The global IACK registers (RM Table 17-1).
const GIACK: u32 = 0xFC05_4000;
/// A counter in RAM the snippets' handlers increment.
const COUNT: u32 = 0x4010_2000;

const IPRH: u32 = 0x00;
const IPRL: u32 = 0x04;
const IMRH: u32 = 0x08;
const IMRL: u32 = 0x0C;
const INTFRCH: u32 = 0x10;
const INTFRCL: u32 = 0x14;
const ICONFIG: u32 = 0x1A;
const SIMR: u32 = 0x1C;
const CIMR: u32 = 0x1D;
const CLMASK: u32 = 0x1E;
const SLMASK: u32 = 0x1F;
const ICR0: u32 = 0x40;
const SWIACK: u32 = 0xE0;
const EMASK: u32 = 0x20;

/// The IACK register for level `n` (1-7).
fn iack(n: u32) -> u32 {
    SWIACK + 4 * n
}

fn base(index: usize) -> u32 {
    [INTC0, INTC1, INTC2][index]
}

/// Controller `index` of a chip, on its own (its siblings and the global IACK block are dropped).
fn intc(index: usize) -> Intc {
    let ([i0, i1, i2], _) = Intc::chip();
    [i0, i1, i2].into_iter().nth(index).unwrap()
}

/// A machine running nothing yet, with controller `index` (alone) at its base.
fn one(index: usize) -> (Machine, DeviceId) {
    let (mut m, _) = prepare("nop", &[]);
    let id = m.add_interrupt_controller(base(index), 0x4000, Box::new(intc(index)));
    (m, id)
}

/// The chip's three controllers, added in order, and the global IACK block.
fn add_chip(m: &mut Machine) -> [DeviceId; 3] {
    let (intcs, global) = Intc::chip();
    let ids = intcs.map(|c| {
        let b = base(c.index());
        m.add_interrupt_controller(b, 0x4000, Box::new(c))
    });
    m.add_device(GIACK, 0x4000, Box::new(global));
    ids
}

fn r(m: &mut Machine, addr: u32, size: Size) -> u32 {
    m.access(addr, size, None)
        .unwrap_or_else(|f| panic!("read {addr:#x}: {f:?} {:?}", m.take_refusal()))
}

fn w(m: &mut Machine, addr: u32, size: Size, value: u32) {
    m.access(addr, size, Some(value))
        .unwrap_or_else(|f| panic!("write {addr:#x}: {f:?} {:?}", m.take_refusal()));
}

/// The access at `addr` is refused as unmodelled; returns the detail.
fn refused(m: &mut Machine, addr: u32, size: Size, write: Option<u32>) -> String {
    assert_eq!(
        m.access(addr, size, write),
        Err(Fault::Device(Stop::Unmodelled)),
        "{addr:#x} {size:?}"
    );
    m.take_refusal().expect("refusal").detail
}

/// What controller `id` reports to the core for the machine's lines (level 7 allowed).
fn pending(m: &Machine, id: DeviceId) -> Option<Irq> {
    m.device::<Intc>(id).pending(m.mem.irq_lines(), true)
}

fn irq(level: u8, vector: u8) -> Option<Irq> {
    Some(Irq { level, vector })
}

#[test]
fn force_bits_read_back_and_pend() {
    let (mut m, id) = one(0);
    w(&mut m, INTC0 + ICR0 + 33, Size::Byte, 3);
    w(&mut m, INTC0 + CIMR, Size::Byte, 33);
    assert_eq!(pending(&m, id), None);
    // INTFRCH bit 1 is source 33.
    w(&mut m, INTC0 + INTFRCH, Size::Long, 1 << 1);
    assert_eq!(r(&mut m, INTC0 + INTFRCH, Size::Long), 2);
    assert_eq!(r(&mut m, INTC0 + INTFRCL, Size::Long), 0);
    assert_eq!(pending(&m, id), irq(3, 64 + 33));
    w(&mut m, INTC0 + INTFRCH, Size::Long, 0);
    assert_eq!(pending(&m, id), None);
}

#[test]
fn masked_force_bit_is_still_pending() {
    // RM 17.2.3: a forced request is not affected by IMR. Measured on an MCF54415: INTC1 source 62 at level 1,
    // forced with its IMR bit set, reads 0xBE (vector 190) from L1IACK1 and GL1IACK, as when unmasked.
    let (mut m, id) = one(0);
    w(&mut m, INTC0 + ICR0 + 5, Size::Byte, 3);
    w(&mut m, INTC0 + INTFRCL, Size::Long, 1 << 5);
    assert_eq!(r(&mut m, INTC0 + INTFRCL, Size::Long), 1 << 5);
    assert_eq!(r(&mut m, INTC0 + IMRL, Size::Long), 0xFFFF_FFFF);
    assert_eq!(pending(&m, id), irq(3, 64 + 5));
    assert_eq!(r(&mut m, INTC0 + iack(3), Size::Byte), 64 + 5);
    assert_eq!(r(&mut m, INTC0 + SWIACK, Size::Byte), 64 + 5);
    // The measured case, through the chip and the global IACK block; IPR still does not show it.
    let (mut m, _) = prepare("nop", &[]);
    let ids = add_chip(&mut m);
    w(&mut m, INTC1 + ICR0 + 62, Size::Byte, 1);
    w(&mut m, INTC1 + INTFRCH, Size::Long, 1 << 30);
    assert_eq!(pending(&m, ids[1]), irq(1, 0xBE));
    assert_eq!(r(&mut m, INTC1 + iack(1), Size::Byte), 0xBE);
    assert_eq!(r(&mut m, GIACK + iack(1), Size::Byte), 0xBE);
    assert_eq!(r(&mut m, INTC1 + IPRH, Size::Long), 0);
    // A forced level-7 source raises the level-7 line, masked or not.
    w(&mut m, INTC1 + ICR0 + 62, Size::Byte, 7);
    assert!(m.device::<Intc>(ids[1]).level7(m.mem.irq_lines()));
    // An asserted (not forced) source stays masked by IMR.
    w(&mut m, INTC1 + INTFRCH, Size::Long, 0);
    m.mem.irq_lines_mut().set(1, 62, true);
    assert_eq!(pending(&m, ids[1]), None);
    assert_eq!(r(&mut m, INTC1 + iack(7), Size::Byte), 0x18);
}

#[test]
fn ipr_shows_peripheral_requests_but_not_forced_sources() {
    // Measured on an MCF54415: with INTFRCH1 bit 30 set, source 62 unmasked and the core halted, IPRH1 read 0.
    // A peripheral's request shows, masked or not (RM 17.2.1).
    let (mut m, _) = one(1);
    m.mem.irq_lines_mut().set(1, 40, true);
    w(&mut m, INTC1 + INTFRCH, Size::Long, 1 << 30);
    w(&mut m, INTC1 + ICR0 + 62, Size::Byte, 3);
    w(&mut m, INTC1 + CIMR, Size::Byte, 62);
    m.mem.irq_lines_mut().set(1, 3, true);
    assert_eq!(
        (
            r(&mut m, INTC1 + IPRH, Size::Long),
            r(&mut m, INTC1 + IPRL, Size::Long)
        ),
        (1 << 8, 1 << 3)
    );
    m.mem.irq_lines_mut().set(1, 40, false);
    assert_eq!(r(&mut m, INTC1 + IPRH, Size::Long), 0);
    // IPR is read-only: a write is ignored (RM 17.2.1).
    w(&mut m, INTC1 + IPRL, Size::Long, !0);
    assert_eq!(r(&mut m, INTC1 + IPRL, Size::Long), 1 << 3);
}

/// `body` after setting INTC0 source 5 (vector 69) to level 3 and unmasking it; the handler clears the force
/// bits and increments [`COUNT`].
fn forced(body: &str) -> (Machine, u32) {
    let src = format!(
        "moveq #3,%d0\nmove.b %d0,0xFC048045\nmoveq #5,%d0\nmove.b %d0,0xFC04801D\n{body}\n\
         done: bra.s done\n\
         isr: clr.l 0xFC048014\nmove.l {COUNT:#x},%d1\naddq.l #1,%d1\nmove.l %d1,{COUNT:#x}\nrte"
    );
    let (mut m, l) = prepare(&src, &["done", "isr"]);
    set_vector(&mut m, 69, l[1]);
    m.add_interrupt_controller(INTC0, 0x4000, Box::new(intc(0)));
    (m, l[0])
}

#[test]
fn forced_source_is_taken_once() {
    let (mut m, done) = forced("move.w #0x2000,%sr\nmoveq #32,%d0\nmove.l %d0,0xFC048014\nnop");
    let r = m.run(until(done));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.mem.read(COUNT, Size::Long), Ok(1));
    assert_eq!(m.cpu.a[7], STACK);
}

#[test]
fn forced_source_waits_for_ipl_then_wakes_stop() {
    // Forced while IPL 7 masks it; STOP wakes on it: forced bits are controller state, seen with any lines.
    let body = format!(
        "moveq #32,%d0\nmove.l %d0,0xFC048014\nmove.l {COUNT:#x},%d2\nmove.l %d2,{:#x}\nstop #0x2000",
        COUNT + 4
    );
    let (mut m, done) = forced(&body);
    let r = m.run(until(done));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.mem.read(COUNT, Size::Long), Ok(1));
    assert_eq!(
        m.mem.read(COUNT + 4, Size::Long),
        Ok(0),
        "not taken at IPL 7"
    );
}

#[test]
fn intc_reset_values_follow_the_rm() {
    // RM Table 17-2.
    let (mut m, _) = one(0);
    for (off, v) in [
        (IPRH, 0),
        (IPRL, 0),
        (IMRH, 0xFFFF_FFFF),
        (IMRL, 0xFFFF_FFFF),
        (INTFRCH, 0),
        (INTFRCL, 0),
    ] {
        assert_eq!(r(&mut m, INTC0 + off, Size::Long), v, "{off:#x}");
    }
    assert_eq!(r(&mut m, INTC0 + ICONFIG, Size::Word), 0);
    // SIMR and CIMR read as zero (RM 17.2.5, RM 17.2.6); CLMASK and SLMASK reset to 0x0F (RM 17.2.7, RM 17.2.8).
    for (off, v) in [(SIMR, 0), (CIMR, 0), (CLMASK, 0x0F), (SLMASK, 0x0F)] {
        assert_eq!(r(&mut m, INTC0 + off, Size::Byte), v, "{off:#x}");
    }
    for n in 0..64 {
        assert_eq!(r(&mut m, INTC0 + ICR0 + n, Size::Byte), 0);
    }
    // SWIACK resets to 0, LnIACK to 0x18 (RM 17.2.10).
    assert_eq!(r(&mut m, INTC0 + SWIACK, Size::Byte), 0);
    for n in 1..=7 {
        assert_eq!(r(&mut m, INTC0 + iack(n), Size::Byte), 0x18);
    }
}

#[test]
fn intc_register_basics() {
    let (mut m, id) = one(2);
    assert_eq!(r(&mut m, INTC2 + IMRH, Size::Long), 0xFFFF_FFFF);
    assert_eq!(r(&mut m, INTC2 + IMRL, Size::Long), 0xFFFF_FFFF);
    w(&mut m, INTC2 + CIMR, Size::Byte, 13);
    assert_eq!(r(&mut m, INTC2 + IMRL, Size::Long), !(1 << 13));
    w(&mut m, INTC2 + ICR0 + 13, Size::Byte, 6);
    m.mem.irq_lines_mut().set(2, 13, true);
    assert_eq!(pending(&m, id), irq(6, 64 + 128 + 13));
    w(&mut m, INTC2 + SIMR, Size::Byte, 13);
    assert_eq!(pending(&m, id), None);
    // CALL (0x40) clears every mask bit, SALL sets them all.
    w(&mut m, INTC2 + CIMR, Size::Byte, 64);
    assert_eq!(r(&mut m, INTC2 + IMRH, Size::Long), 0);
    assert_eq!(r(&mut m, INTC2 + IMRL, Size::Long), 0);
    w(&mut m, INTC2 + SIMR, Size::Byte, 0x40 | 5);
    assert_eq!(r(&mut m, INTC2 + IMRH, Size::Long), 0xFFFF_FFFF);
    assert_eq!(r(&mut m, INTC2 + IMRL, Size::Long), 0xFFFF_FFFF);
    // IMR is read/write as two longwords.
    w(&mut m, INTC2 + IMRH, Size::Long, 0x1234_5678);
    w(&mut m, INTC2 + IMRL, Size::Long, 0x9ABC_DEF0);
    assert_eq!(r(&mut m, INTC2 + IMRH, Size::Long), 0x1234_5678);
    assert_eq!(r(&mut m, INTC2 + IMRL, Size::Long), 0x9ABC_DEF0);
    // Within a level the higher-numbered source wins (RM 17.3.1); the highest level wins over that.
    w(&mut m, INTC2 + CIMR, Size::Byte, 64);
    w(&mut m, INTC2 + ICR0 + 40, Size::Byte, 6);
    m.mem.irq_lines_mut().set(2, 40, true);
    assert_eq!(pending(&m, id), irq(6, 192 + 40));
    w(&mut m, INTC2 + ICR0 + 2, Size::Byte, 7);
    m.mem.irq_lines_mut().set(2, 2, true);
    assert_eq!(pending(&m, id), irq(7, 192 + 2));
    assert!(m.device::<Intc>(id).level7(m.mem.irq_lines()));
    // Level 7 is left to the core's edge unless allowed.
    assert_eq!(
        m.device::<Intc>(id).pending(m.mem.irq_lines(), false),
        irq(6, 192 + 40)
    );
    // ICR 0 disables a source (RM Table 17-14).
    w(&mut m, INTC2 + ICR0 + 2, Size::Byte, 0);
    w(&mut m, INTC2 + ICR0 + 40, Size::Byte, 0);
    assert_eq!(pending(&m, id), irq(6, 192 + 13));
}

#[test]
fn icr_bytes_pack_big_endian() {
    let (mut m, _) = one(0);
    for (k, v) in [1, 2, 3, 4].into_iter().enumerate() {
        w(&mut m, INTC0 + ICR0 + 8 + k as u32, Size::Byte, v);
    }
    assert_eq!(r(&mut m, INTC0 + ICR0 + 8, Size::Long), 0x0102_0304);
    assert_eq!(r(&mut m, INTC0 + ICR0 + 0xA, Size::Word), 0x0304);
    w(&mut m, INTC0 + ICR0 + 0xC, Size::Long, 0x0506_0700);
    let icr: Vec<u32> = (12..16)
        .map(|k| r(&mut m, INTC0 + ICR0 + k, Size::Byte))
        .collect();
    assert_eq!(icr, [5, 6, 7, 0]);
    // Only the level (bits 2:0) is kept (RM Table 17-14).
    w(&mut m, INTC0 + ICR0 + 63, Size::Byte, 0xFF);
    assert_eq!(r(&mut m, INTC0 + ICR0 + 60, Size::Long), 7);
}

#[test]
fn irq_snippet_acknowledges_with_simr() {
    // Vector 70 is INTC0 source 6; the handler counts and masks the source again with SIMR.
    let src = format!(
        "sr_set: move.w #0x2000,%sr\nclr.l %d2\n\
         spin: addq.l #1,%d2\ncmp.l #200,%d2\nbne.s spin\n\
         done: nop\n\
         handler: move.l {COUNT:#x},%d1\naddq.l #1,%d1\nmove.l %d1,{COUNT:#x}\n\
         moveq #6,%d1\nmove.b %d1,0xFC04801C\nrte"
    );
    let (mut m, l) = prepare(&src, &["done", "handler"]);
    set_vector(&mut m, 70, l[1]);
    m.add_interrupt_controller(INTC0, 0x4000, Box::new(intc(0)));
    w(&mut m, INTC0 + ICR0 + 6, Size::Byte, 3);
    w(&mut m, INTC0 + CIMR, Size::Byte, 6);
    m.mem.irq_lines_mut().set(0, 6, true);
    let r0 = m.run(until(l[0]));
    assert_eq!(r0.kind, Stop::UntilPc, "{r0:?}");
    assert_eq!(m.mem.read(COUNT, Size::Long), Ok(1), "taken once");
    assert_eq!(m.cpu.d[2], 200, "the interrupted loop completes");
    assert_eq!(m.cpu.a[7], STACK);
    assert_eq!(r(&mut m, INTC0 + IMRL, Size::Long), 0xFFFF_FFFF);
    // Without ICONFIG[EMASK] the acknowledge leaves the level masks alone (RM Table 17-9).
    assert_eq!(r(&mut m, INTC0 + CLMASK, Size::Byte), 0x0F);
    assert_eq!(r(&mut m, INTC0 + SLMASK, Size::Byte), 0x0F);
}

#[test]
fn level_7_through_intc0_is_edge_triggered() {
    // Level 7 is taken at IPL 7, once per rise of the request (RM 17.1.1); the handler leaves it asserted.
    let src = "move.w #0x2700,%sr\nloop: addq.l #1,%d3\ncmp.l #200,%d3\nbne.s loop\n\
               done: nop\nisr: addq.l #1,%d4\nrte";
    let (mut m, l) = prepare(src, &["loop", "done", "isr"]);
    set_vector(&mut m, 69, l[2]);
    m.add_interrupt_controller(INTC0, 0x4000, Box::new(intc(0)));
    w(&mut m, INTC0 + ICR0 + 5, Size::Byte, 7);
    w(&mut m, INTC0 + CIMR, Size::Byte, 5);
    assert_eq!(m.run(until(l[0])).kind, Stop::UntilPc);
    m.mem.irq_lines_mut().set(0, 5, true);
    let r0 = m.run(until(l[1]));
    assert_eq!(r0.kind, Stop::UntilPc, "{r0:?}");
    assert_eq!(m.cpu.d[4], 1);
    // Lines changed between runs are sampled when the next run starts: a run with the line low, then one with it
    // high again, is a new edge.
    let again = |m: &mut Machine| {
        m.cpu.d[3] = 0;
        m.cpu.pc = l[0];
        let r0 = m.run(until(l[1]));
        assert_eq!(r0.kind, Stop::UntilPc, "{r0:?}");
    };
    again(&mut m);
    assert_eq!(m.cpu.d[4], 1, "held: no new edge");
    m.mem.irq_lines_mut().set(0, 5, false);
    again(&mut m);
    assert_eq!(m.cpu.d[4], 1, "the line fell: nothing taken");
    m.mem.irq_lines_mut().set(0, 5, true);
    again(&mut m);
    assert_eq!(m.cpu.d[4], 2);
    // A forced level-7 source raises the line too.
    m.mem.irq_lines_mut().set(0, 5, false);
    again(&mut m);
    w(&mut m, INTC0 + INTFRCL, Size::Long, 1 << 5);
    again(&mut m);
    assert_eq!(m.cpu.d[4], 3);
}

#[test]
fn emask_masks_levels_up_to_clmask_after_acknowledge() {
    // RM 17.2.4, RM 17.2.7, RM 17.3.1.3: with ICONFIG[EMASK] the acknowledge saves CLMASK in SLMASK and loads
    // CLMASK with the level taken; requests at levels up to CLMASK are masked in all three controllers (one copy
    // of CLMASK, in INTC0's space). Each handler logs at A4.
    //
    // INTC0 source 6 and INTC1 source 2 are asserted at level 3: INTC0's is taken (vector 70). Its handler lowers
    // SR to 0, where INTC1's level-3 request stays masked by CLMASK = 3; it forces INTC2 source 1 at level 4, which
    // is taken (vector 193) at once and restores CLMASK from SLMASK; then it masks its source and restores CLMASK
    // to 0x0F, and INTC1's request is taken (vector 130).
    let src = "move.w #0x2000,%sr\nnop\nend: nop\n\
               h70: moveq #70,%d0\nmove.l %d0,(%a4)+\n\
               move.b 0xFC04801E,%d5\nmove.b 0xFC04801F,%d6\n\
               move.w #0x2000,%sr\nmoveq #1,%d0\nmove.l %d0,(%a4)+\n\
               moveq #2,%d0\nmove.l %d0,0xFC050014\n\
               moveq #2,%d0\nmove.l %d0,(%a4)+\n\
               moveq #6,%d1\nmove.b %d1,0xFC04801C\nmove.b %d6,0xFC04801E\n\
               moveq #3,%d0\nmove.l %d0,(%a4)+\nrte\n\
               h193: move.l #193,(%a4)+\n\
               move.b 0xFC04801E,%d3\nmove.b 0xFC04801F,%d2\n\
               clr.l 0xFC050014\nmove.b %d2,0xFC04801E\nrte\n\
               h130: move.l #130,(%a4)+\n\
               moveq #2,%d1\nmove.b %d1,0xFC04C01C\n\
               move.b 0xFC04801F,%d1\nmove.b %d1,0xFC04801E\nrte";
    let (mut m, l) = prepare(src, &["end", "h70", "h193", "h130"]);
    set_vector(&mut m, 70, l[1]);
    set_vector(&mut m, 193, l[2]);
    set_vector(&mut m, 130, l[3]);
    let ids = add_chip(&mut m);
    for (index, source, level) in [(0, 6, 3), (1, 2, 3), (2, 1, 4)] {
        w(&mut m, base(index) + ICR0 + source, Size::Byte, level);
        w(&mut m, base(index) + CIMR, Size::Byte, source);
    }
    m.mem.irq_lines_mut().set(0, 6, true);
    m.mem.irq_lines_mut().set(1, 2, true);

    // CLMASK masks only with EMASK set (RM 17.3.1.2: level masking is performed if the feature is enabled).
    w(&mut m, INTC0 + CLMASK, Size::Byte, 7);
    assert_eq!(pending(&m, ids[0]), irq(3, 70));
    w(&mut m, INTC0 + ICONFIG, Size::Word, EMASK);
    assert_eq!(r(&mut m, INTC0 + ICONFIG, Size::Word), EMASK);
    assert_eq!((pending(&m, ids[0]), pending(&m, ids[1])), (None, None));
    assert!(!m.device::<Intc>(ids[0]).level7(m.mem.irq_lines()));
    // CLMASK = 2 lets level 3 through, in every controller; 0x0F masks nothing.
    w(&mut m, INTC0 + CLMASK, Size::Byte, 2);
    assert_eq!(pending(&m, ids[1]), irq(3, 130));
    w(&mut m, INTC0 + CLMASK, Size::Byte, 0x0F);

    const LOG: u32 = 0x4010_3000;
    m.cpu.a[4] = LOG;
    let r0 = m.run(until(l[0]));
    assert_eq!(r0.kind, Stop::UntilPc, "{r0:?}");
    let log: Vec<u32> = (0..6)
        .map(|k| m.mem.read(LOG + 4 * k, Size::Long).unwrap())
        .collect();
    assert_eq!(log, [70, 1, 193, 2, 130, 3]);
    assert_eq!(m.cpu.a[4], LOG + 24, "nothing else was taken");
    // In h70: CLMASK = 3, SLMASK = 0x0F; in h193: CLMASK = 4, SLMASK = 3.
    assert_eq!((m.cpu.d[5] & 0xFF, m.cpu.d[6] & 0xFF), (3, 0x0F));
    assert_eq!((m.cpu.d[3] & 0xFF, m.cpu.d[2] & 0xFF), (4, 3));
    assert_eq!(r(&mut m, INTC0 + CLMASK, Size::Byte), 0x0F);
    assert_eq!(r(&mut m, INTC0 + SLMASK, Size::Byte), 0x0F);
}

#[test]
fn level_mask_registers_are_only_in_intc0() {
    // RM 17.2.4, RM 17.2.7, RM 17.2.8: one copy, accessed through INTC0's space; INTC1 and INTC2 do not list
    // them (RM Table 17-2).
    let (mut m, _) = prepare("nop", &[]);
    add_chip(&mut m);
    for b in [INTC1, INTC2] {
        assert!(refused(&mut m, b + ICONFIG, Size::Word, None).contains("ICONFIG"));
        assert!(refused(&mut m, b + CLMASK, Size::Byte, Some(3)).contains("CLMASK"));
        assert!(refused(&mut m, b + SLMASK, Size::Byte, None).contains("SLMASK"));
    }
    // CLMASK values 8-14 are reserved (RM Table 17-12): refused, the mask is unchanged.
    let d = refused(&mut m, INTC0 + CLMASK, Size::Byte, Some(9));
    assert!(d.contains("CLMASK") && d.contains("reserved"), "{d}");
    assert_eq!(r(&mut m, INTC0 + CLMASK, Size::Byte), 0x0F);
    // Bits 7:4 are reserved (RM Table 17-12, RM Table 17-13): a value with any of them set is refused too.
    let d = refused(&mut m, INTC0 + SLMASK, Size::Byte, Some(0xF5));
    assert!(d.contains("SLMASK") && d.contains("reserved"), "{d}");
    let d = refused(&mut m, INTC0 + CLMASK, Size::Byte, Some(0x13));
    assert!(d.contains("CLMASK") && d.contains("reserved"), "{d}");
    assert_eq!(r(&mut m, INTC0 + SLMASK, Size::Byte), 0x0F);
    w(&mut m, INTC0 + SLMASK, Size::Byte, 5);
    // ICONFIG keeps only ELVLPRI and EMASK (RM Table 17-9).
    w(&mut m, INTC0 + ICONFIG, Size::Word, 0xFFFF);
    assert_eq!(r(&mut m, INTC0 + ICONFIG, Size::Word), 0xFE20);
    // CLMASK and SLMASK pack with SIMR and CIMR in a longword read.
    w(&mut m, INTC0 + CLMASK, Size::Byte, 6);
    assert_eq!(r(&mut m, INTC0 + SIMR, Size::Long), 0x0000_0605);
}

#[test]
fn wide_writes_at_simr_ignore_cimr_measured() {
    // measured on an MCF54415: a word write at SIMR (0x1C) applies SIMR only, the CIMR byte ignored; a longword
    // write there applies SIMR, CLMASK and SLMASK, CIMR again ignored; byte writes act normally; a longword read
    // at 0x1C reads 0x0000_0302 with CLMASK 3 and SLMASK 2 (SIMR and CIMR read 0)
    let (mut m, _) = one(0);
    w(&mut m, INTC0 + IMRL, Size::Long, 1 << 3);
    w(&mut m, INTC0 + SIMR, Size::Word, 4 << 8 | 3);
    assert_eq!(r(&mut m, INTC0 + IMRL, Size::Long), 1 << 4 | 1 << 3);
    w(&mut m, INTC0 + SIMR, Size::Long, 0x0504_0302);
    assert_eq!(
        r(&mut m, INTC0 + IMRL, Size::Long),
        1 << 5 | 1 << 4 | 1 << 3
    );
    assert_eq!(r(&mut m, INTC0 + SIMR, Size::Long), 0x0000_0302);
    assert_eq!(r(&mut m, INTC0 + CLMASK, Size::Byte), 3);
    assert_eq!(r(&mut m, INTC0 + SLMASK, Size::Byte), 2);
    w(&mut m, INTC0 + CIMR, Size::Byte, 4);
    assert_eq!(r(&mut m, INTC0 + IMRL, Size::Long), 1 << 5 | 1 << 3);
}

#[test]
fn iack_registers_return_the_pending_vector_or_0x18() {
    // RM 17.2.10: SWIACK gives the vector of the highest unmasked request (0 when none), LnIACK the highest at
    // level n (0x18 when none); the global ones do the same over all controllers, INTC0 first (RM 17.3.2).
    let (mut m, _) = prepare("nop", &[]);
    add_chip(&mut m);
    assert_eq!(r(&mut m, GIACK + SWIACK, Size::Byte), 0);
    for n in 1..=7 {
        assert_eq!(r(&mut m, GIACK + iack(n), Size::Byte), 0x18);
    }
    for (index, source, level) in [(0, 6, 3), (0, 9, 3), (1, 4, 5), (2, 20, 3), (2, 30, 6)] {
        w(&mut m, base(index) + ICR0 + source, Size::Byte, level);
        if index != 2 || source != 30 {
            w(&mut m, base(index) + CIMR, Size::Byte, source);
            m.mem.irq_lines_mut().set(index, source as u8, true);
        }
    }
    // INTC2 source 30 is at level 6 but masked: it does not count.
    m.mem.irq_lines_mut().set(2, 30, true);
    assert_eq!(r(&mut m, INTC0 + SWIACK, Size::Byte), 64 + 9);
    assert_eq!(r(&mut m, INTC0 + iack(3), Size::Byte), 64 + 9);
    assert_eq!(r(&mut m, INTC0 + iack(5), Size::Byte), 0x18);
    assert_eq!(r(&mut m, INTC1 + SWIACK, Size::Byte), 128 + 4);
    assert_eq!(r(&mut m, INTC1 + iack(5), Size::Byte), 128 + 4);
    assert_eq!(r(&mut m, INTC2 + iack(6), Size::Byte), 0x18);
    assert_eq!(r(&mut m, GIACK + SWIACK, Size::Byte), 128 + 4);
    assert_eq!(r(&mut m, GIACK + iack(3), Size::Byte), 64 + 9);
    assert_eq!(r(&mut m, GIACK + iack(5), Size::Byte), 128 + 4);
    assert_eq!(r(&mut m, GIACK + iack(6), Size::Byte), 0x18);
    // A forced source counts.
    w(&mut m, INTC0 + ICR0 + 50, Size::Byte, 3);
    w(&mut m, INTC0 + CIMR, Size::Byte, 50);
    w(&mut m, INTC0 + INTFRCH, Size::Long, 1 << 18);
    assert_eq!(r(&mut m, INTC0 + iack(3), Size::Byte), 64 + 50);
    w(&mut m, INTC0 + INTFRCH, Size::Long, 0);
    // Without EMASK the reads leave the level masks alone.
    assert_eq!(r(&mut m, INTC0 + CLMASK, Size::Byte), 0x0F);

    // measured on an MCF54415: with EMASK every read of SWIACK or LnIACK saves CLMASK in SLMASK and loads CLMASK,
    // SWIACK with the level of the request it returns (0 when none), LnIACK with n even when it reads spurious
    // (0x18); an IACK sees only the levels CLMASK lets through (L3IACK with CLMASK 3 read 0x18)
    w(&mut m, INTC0 + ICONFIG, Size::Word, EMASK);
    let masks = |m: &mut Machine| {
        (
            r(m, INTC0 + CLMASK, Size::Byte),
            r(m, INTC0 + SLMASK, Size::Byte),
        )
    };
    assert_eq!(r(&mut m, INTC0 + SWIACK, Size::Byte), 64 + 9);
    assert_eq!(masks(&mut m), (3, 0x0F));
    // Level 3 is now masked everywhere: INTC0 has nothing above it.
    assert_eq!(r(&mut m, INTC0 + SWIACK, Size::Byte), 0);
    assert_eq!(masks(&mut m), (0, 3));
    assert_eq!(r(&mut m, GIACK + SWIACK, Size::Byte), 128 + 4);
    assert_eq!(masks(&mut m), (5, 0));
    // A spurious level-n IACK loads n all the same.
    assert_eq!(r(&mut m, INTC0 + iack(3), Size::Byte), 0x18);
    assert_eq!(masks(&mut m), (3, 5));
    assert_eq!(r(&mut m, INTC2 + iack(3), Size::Byte), 0x18);
    assert_eq!(masks(&mut m), (3, 3));
    w(&mut m, INTC0 + CLMASK, Size::Byte, 2);
    assert_eq!(r(&mut m, INTC0 + iack(3), Size::Byte), 64 + 9);
    assert_eq!(masks(&mut m), (3, 2));
    // guess: the global IACK registers load the masks as INTC0's do (measured on INTC0's).
    assert_eq!(r(&mut m, GIACK + iack(5), Size::Byte), 128 + 4);
    assert_eq!(masks(&mut m), (5, 3));
    assert_eq!(r(&mut m, INTC0 + iack(5), Size::Byte), 0x18);
    assert_eq!(masks(&mut m), (5, 5));
    // Without EMASK no read loads them.
    w(&mut m, INTC0 + ICONFIG, Size::Word, 0);
    w(&mut m, INTC0 + CLMASK, Size::Byte, 0x0F);
    assert_eq!(r(&mut m, INTC0 + SWIACK, Size::Byte), 64 + 9);
    assert_eq!(r(&mut m, INTC0 + iack(3), Size::Byte), 64 + 9);
    assert_eq!(r(&mut m, INTC0 + iack(4), Size::Byte), 0x18);
    assert_eq!(masks(&mut m), (0x0F, 5));

    // A write to an IACK register is an error termination (RM Table 17-18): refused.
    assert!(refused(&mut m, INTC1 + iack(2), Size::Byte, Some(0)).contains("L2IACK"));
    assert!(refused(&mut m, GIACK + SWIACK, Size::Byte, Some(0)).contains("SWIACK"));
}

#[test]
fn narrow_access_to_imr_refuses() {
    // IPR, IMR and INTFRC are 32-bit registers (RM Table 17-2): a byte or word access is refused, naming the
    // register, and changes nothing.
    let (mut m, _) = one(0);
    assert!(refused(&mut m, INTC0 + IMRL + 3, Size::Byte, None).contains("IMRL"));
    assert!(refused(&mut m, INTC0 + IMRH, Size::Word, Some(0)).contains("IMRH"));
    assert!(refused(&mut m, INTC0 + IPRH + 2, Size::Word, None).contains("IPRH"));
    assert!(refused(&mut m, INTC0 + INTFRCL, Size::Byte, Some(1)).contains("INTFRCL"));
    assert_eq!(r(&mut m, INTC0 + IMRH, Size::Long), 0xFFFF_FFFF);
    assert_eq!(r(&mut m, INTC0 + INTFRCL, Size::Long), 0);
    // ICONFIG is 16-bit.
    assert!(refused(&mut m, INTC0 + ICONFIG + 1, Size::Byte, Some(1)).contains("ICONFIG"));
    // Offsets the RM map leaves out are refused too.
    for off in [0x18, 0x20, 0x3C, 0x80, 0xE1, 0x100] {
        let d = refused(&mut m, INTC0 + off, Size::Byte, None);
        assert!(d.starts_with("INTC0"), "{d}");
    }

    // From a program: the instruction is refused and PC stays on it.
    let (mut m, l) = prepare("move.b 0xFC04800F,%d0\nend: nop", &["end"]);
    m.add_interrupt_controller(INTC0, 0x4000, Box::new(intc(0)));
    let r0 = m.run(until(l[0]));
    assert_eq!((r0.kind, r0.pc), (Stop::Unmodelled, common::BASE), "{r0:?}");
    assert!(r0.detail.contains("IMRL"), "{r0:?}");
}

#[test]
fn equal_levels_across_controllers_go_to_intc0() {
    // RM 17.3.2: at the same level INTC0 is serviced first, then INTC1, then INTC2. Each handler records the
    // order in D5/D6/D2 and masks its source with SIMR.
    let src = "move.w #0x2000,%sr\nnop\nend: nop\n\
               h0: move.l %d7,%d5\naddq.l #1,%d7\nmoveq #6,%d1\nmove.b %d1,0xFC04801C\nrte\n\
               h1: move.l %d7,%d6\naddq.l #1,%d7\nmoveq #6,%d1\nmove.b %d1,0xFC04C01C\nrte\n\
               h2: move.l %d7,%d2\naddq.l #1,%d7\nmoveq #6,%d1\nmove.b %d1,0xFC05001C\nrte";
    let (mut m, l) = prepare(src, &["end", "h0", "h1", "h2"]);
    for index in 0..3 {
        set_vector(&mut m, 64 + 64 * index as u32 + 6, l[1 + index]);
    }
    add_chip(&mut m);
    // Set up through the registers, INTC2 first.
    for index in (0..3).rev() {
        w(&mut m, base(index) + ICR0 + 6, Size::Byte, 3);
        w(&mut m, base(index) + CIMR, Size::Byte, 6);
        m.mem.irq_lines_mut().set(index, 6, true);
    }
    let r0 = m.run(until(l[0]));
    assert_eq!(r0.kind, Stop::UntilPc, "{r0:?}");
    assert_eq!(
        (m.cpu.d[5], m.cpu.d[6], m.cpu.d[2], m.cpu.d[7]),
        (0, 1, 2, 3)
    );
}

// The controllers' change counter (`InterruptController::generation`): every change to their shared state moves it
// on, so the machine never reuses an answer the change made stale. Each test below fails when one of those moves
// is left out.

/// Where the eDMA is mapped, and its SSRT register.
const EDMA: u32 = 0xFC04_4000;
const SSRT: u32 = EDMA + 0x1E;
/// A scratch byte the eDMA writes.
const SINK: u32 = 0x4010_2100;

/// `body`, then `end`, on the chip's controllers and the eDMA: INTC0 source 6 (vector 70) at level 3, unmasked and
/// asserted, with ICONFIG\[EMASK\] set and CLMASK at `clmask`. The handler counts in D4, keeps the stacked PC in
/// D6 and masks source 6. Channel 0's TCD reads one byte at `dma_from` into [`SINK`] at each SSRT 0.
fn held(body: &str, clmask: u32, dma_from: u32) -> (Machine, Vec<u32>) {
    let src = format!(
        "{body}\nend: nop\n\
         h70: addq.l #1,%d4\nmove.l 4(%sp),%d6\nmoveq #6,%d1\nmove.b %d1,0xFC04801C\nrte"
    );
    let (mut m, l) = prepare(&src, &["end", "h70", "next"]);
    set_vector(&mut m, 70, l[1]);
    add_chip(&mut m);
    m.add_bus_master(EDMA, 0x2000, Box::new(Edma::new()));
    for (f, size, v) in [
        (0x00, Size::Long, dma_from),
        (0x04, Size::Word, 0),
        (0x06, Size::Word, 0),
        (0x08, Size::Long, 1),
        (0x10, Size::Long, SINK),
        (0x14, Size::Word, 1),
        (0x16, Size::Word, 0),
        (0x1C, Size::Word, 1),
        (0x1E, Size::Word, 0),
    ] {
        w(&mut m, EDMA + 0x1000 + f, size, v);
    }
    w(&mut m, INTC0 + ICR0 + 6, Size::Byte, 3);
    w(&mut m, INTC0 + CIMR, Size::Byte, 6);
    w(&mut m, INTC0 + ICONFIG, Size::Word, EMASK);
    w(&mut m, INTC0 + CLMASK, Size::Byte, clmask);
    m.mem.irq_lines_mut().set(0, 6, true);
    (m, l)
}

/// [`held`] with the level-3 request held off by SR, then `iack` (an instruction acknowledging level 3 with
/// EMASK: CLMASK becomes 3), then SR lowered to 0: the request stays masked. Returns the handler's count.
fn acknowledged_then_lowered(iack: &str, dma_from: u32) -> u32 {
    let body = format!("move.w #0x2700,%sr\nnop\n{iack}\nmove.w #0x2000,%sr\nnop\nnext: nop");
    let (mut m, l) = held(&body, 0x0F, dma_from);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(
        r_byte(&mut m, INTC0 + CLMASK),
        3,
        "the acknowledge loaded CLMASK"
    );
    m.cpu.d[4]
}

fn r_byte(m: &mut Machine, addr: u32) -> u32 {
    r(m, addr, Size::Byte)
}

#[test]
fn a_request_a_global_iack_read_masks_stays_masked_when_sr_is_lowered() {
    let gl3 = GIACK + iack(3);
    assert_eq!(
        acknowledged_then_lowered(&format!("move.b {gl3:#x},%d0"), 0),
        0
    );
}

#[test]
fn a_request_an_edma_read_of_the_global_iack_masks_stays_masked_when_sr_is_lowered() {
    let ssrt = format!("moveq #0,%d0\nmove.b %d0,{SSRT:#x}");
    assert_eq!(acknowledged_then_lowered(&ssrt, GIACK + iack(3)), 0);
}

#[test]
fn a_request_an_intc_iack_read_masks_stays_masked_when_sr_is_lowered() {
    let l3 = INTC0 + iack(3);
    assert_eq!(
        acknowledged_then_lowered(&format!("move.b {l3:#x},%d0"), 0),
        0
    );
}

#[test]
fn with_emask_a_forced_source_whose_handler_only_returns_is_taken_once() {
    // The core's acknowledge loads CLMASK with the level taken, which masks the source the handler leaves forced.
    let src = "move.w #0x2000,%sr\nnop\nnop\nnop\nend: nop\nh69: addq.l #1,%d4\nrte";
    let (mut m, l) = prepare(src, &["end", "h69"]);
    set_vector(&mut m, 69, l[1]);
    add_chip(&mut m);
    w(&mut m, INTC0 + ICR0 + 5, Size::Byte, 3);
    w(&mut m, INTC0 + CIMR, Size::Byte, 5);
    w(&mut m, INTC0 + ICONFIG, Size::Word, EMASK);
    w(&mut m, INTC0 + INTFRCL, Size::Long, 1 << 5);
    // Bounded: a source taken again and again would never reach `end`.
    let r = m.run(Limit {
        max_instructions: Some(1000),
        ..until(l[0])
    });
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[4], 1);
    assert_eq!(r_byte(&mut m, INTC0 + CLMASK), 3);
}

#[test]
fn clearing_emask_lets_a_masked_request_in_at_the_next_instruction() {
    let body = "move.w #0x2000,%sr\nnop\nclr.l %d0\nmove.w %d0,0xFC04801A\nnext: nop";
    let (mut m, l) = held(body, 3, 0);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(
        (m.cpu.d[4], m.cpu.d[6]),
        (1, l[2]),
        "taken right after the write to ICONFIG"
    );
}
