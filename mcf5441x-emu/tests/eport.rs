//! The edge port (RM ch. 18) on the built chip: pins driven by the host, interrupts on INTC0 sources 1-7.

mod common;

use common::prepare;
use mcf5441x_emu::soc::{Chip, ChipOptions, Eport, Mcf5441x};
use mcf5441x_emu::{Fault, Machine, Size, Stop};

/// Edge port 0 (RM Table 18-2).
const EPORT: u32 = 0xFC09_0000;
const EPPAR: u32 = EPORT;
const EPIER: u32 = EPORT + 3;
const EPFR: u32 = EPORT + 6;

/// EPPAR modes (RM Table 18-3).
const LEVEL: u32 = 0;
const RISING: u32 = 1;
const BOTH: u32 = 3;

fn chip() -> (Machine, Chip) {
    let (mut m, _) = prepare("nop", &[]);
    let chip = Mcf5441x::build(&mut m, ChipOptions::default());
    (m, chip)
}

fn r(m: &mut Machine, addr: u32, size: Size) -> u32 {
    m.access(addr, size, None)
        .unwrap_or_else(|f| panic!("read {addr:#x}: {f:?} {:?}", m.take_refusal()))
}

fn w(m: &mut Machine, addr: u32, size: Size, value: u32) {
    m.access(addr, size, Some(value))
        .unwrap_or_else(|f| panic!("write {addr:#x}: {f:?} {:?}", m.take_refusal()));
}

fn pin(m: &mut Machine, chip: &Chip, n: u8, high: bool) {
    m.with_device(chip.eport, |e: &mut Eport, cx| e.set_pin(n, high, cx));
}

/// INTC0 source `n` (RM Table 17-15: EPF n).
fn line(m: &Machine, n: u8) -> bool {
    m.mem.irq_lines().asserted(0) >> n & 1 != 0
}

#[test]
fn eport_reset_values() {
    // RM Table 18-2: EPPAR, EPIER and EPFR reset 0; measured on an MCF54415: every byte of +0..+7 reads 0.
    // guess: every pin starts high.
    let (mut m, chip) = chip();
    assert_eq!(r(&mut m, EPPAR, Size::Word), 0);
    assert_eq!(r(&mut m, EPIER, Size::Byte), 0);
    assert_eq!(r(&mut m, EPFR, Size::Byte), 0);
    for off in 0..8 {
        assert_eq!(r(&mut m, EPORT + off, Size::Byte), 0, "+{off}");
    }
    assert_eq!(m.device::<Eport>(chip.eport).pins(), 0xFF);
}

#[test]
fn eport_both_edges_raise_irq2() {
    // IRQ2 on both edges (EPPAR 0x30) with EPIE2: each edge sets EPF2 and asserts INTC0 source 2 at once
    let (mut m, chip) = chip();
    w(&mut m, EPPAR, Size::Word, BOTH << 4);
    w(&mut m, EPIER, Size::Byte, 1 << 2);
    assert!(!line(&m, 2));
    pin(&mut m, &chip, 2, false);
    assert!(line(&m, 2), "falling edge");
    assert_eq!(r(&mut m, EPFR, Size::Byte), 1 << 2);
    w(&mut m, EPFR, Size::Byte, 1 << 2);
    assert!(!line(&m, 2));
    pin(&mut m, &chip, 2, true);
    assert!(line(&m, 2), "rising edge");
    w(&mut m, EPFR, Size::Byte, 1 << 2);
    // the same level again is no edge
    pin(&mut m, &chip, 2, true);
    assert!(!line(&m, 2));
    // other pins leave it alone; a rising-only pin ignores a falling edge
    w(&mut m, EPPAR, Size::Word, BOTH << 4 | RISING << 6);
    pin(&mut m, &chip, 3, false);
    assert_eq!(r(&mut m, EPFR, Size::Byte), 0);
    pin(&mut m, &chip, 3, true);
    assert_eq!(r(&mut m, EPFR, Size::Byte), 1 << 3);
    assert!(!line(&m, 3), "EPIE3 clear: flagged but not requested");
    w(&mut m, EPIER, Size::Byte, 1 << 2 | 1 << 3);
    assert!(line(&m, 3));
    // clearing an enable bit drops the request (RM Table 18-4)
    w(&mut m, EPIER, Size::Byte, 1 << 2);
    assert!(!line(&m, 3));
}

#[test]
fn eport_level_mode_follows_the_pin() {
    // level-sensitive pins are active low and not latched (RM Table 18-3); EPFR stays clear (RM Table 18-5)
    let (mut m, chip) = chip();
    w(&mut m, EPPAR, Size::Word, LEVEL);
    w(&mut m, EPIER, Size::Byte, 1 << 7);
    assert!(!line(&m, 7));
    pin(&mut m, &chip, 7, false);
    assert!(line(&m, 7));
    pin(&mut m, &chip, 7, true);
    assert!(!line(&m, 7));
    assert_eq!(r(&mut m, EPFR, Size::Byte), 0);
    // a low pin requests as soon as it is enabled
    pin(&mut m, &chip, 5, false);
    assert!(!line(&m, 5));
    w(&mut m, EPIER, Size::Byte, 1 << 5);
    assert!(line(&m, 5));
}

#[test]
fn eport_flag_clears_on_write_one() {
    let (mut m, chip) = chip();
    w(&mut m, EPPAR, Size::Word, BOTH << 2 | BOTH << 8);
    pin(&mut m, &chip, 1, false);
    pin(&mut m, &chip, 4, false);
    assert_eq!(r(&mut m, EPFR, Size::Byte), 0x12);
    // writing 0 has no effect
    w(&mut m, EPFR, Size::Byte, 0);
    assert_eq!(r(&mut m, EPFR, Size::Byte), 0x12);
    w(&mut m, EPFR, Size::Byte, 0x02);
    assert_eq!(r(&mut m, EPFR, Size::Byte), 0x10);
    // a word write at +6 covers EPFR
    w(&mut m, EPFR, Size::Word, 0x1000);
    assert_eq!(r(&mut m, EPFR, Size::Byte), 0);
}

#[test]
fn eport_has_no_ddr_dr_or_pdr_measured() {
    // measured on an MCF54415: writes of 0x02 to +2 and +4 read back 0 (no EPDDR, no EPDR), +5 reads 0 (no
    // EPPDR), and EPPAR (word, 0x0030) and EPIER (0x04) read back as written. guess: writes to +5 and +7 are
    // ignored too.
    let (mut m, chip) = chip();
    w(&mut m, EPORT + 2, Size::Byte, 0x02);
    w(&mut m, EPORT + 4, Size::Byte, 0x02);
    w(&mut m, EPORT + 5, Size::Byte, 0x02);
    w(&mut m, EPORT + 7, Size::Byte, 0x02);
    w(&mut m, EPPAR, Size::Word, 0x0030);
    w(&mut m, EPIER, Size::Byte, 0x04);
    assert_eq!(r(&mut m, EPORT + 2, Size::Byte), 0);
    assert_eq!(r(&mut m, EPORT + 4, Size::Byte), 0);
    assert_eq!(r(&mut m, EPORT + 5, Size::Byte), 0);
    assert_eq!(r(&mut m, EPORT + 7, Size::Byte), 0);
    assert_eq!(r(&mut m, EPPAR, Size::Word), 0x0030);
    assert_eq!(r(&mut m, EPIER, Size::Byte), 0x04);
    assert_eq!(r(&mut m, EPORT + 2, Size::Word), 0x0004);
    // a low pin shows nowhere in the registers: the pins are inputs the program cannot read
    pin(&mut m, &chip, 6, false);
    assert_eq!(r(&mut m, EPORT + 4, Size::Word), 0);
    assert!(!m.device::<Eport>(chip.eport).pin(6));
    // longword accesses are a bus error (RM 18.4): the run stops
    assert_eq!(
        m.access(EPPAR, Size::Long, None),
        Err(Fault::Device(Stop::Unmodelled))
    );
    let d = m.take_refusal().expect("a refusal").detail;
    assert!(d.contains("longword"), "{d}");
}
