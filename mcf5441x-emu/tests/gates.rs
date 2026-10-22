//! The clock gates (RM 9.2.2-9.2.4): PPMSR0/1, PPMCR0/1, PPMHR0/1 and PPMLR0/1 in the SCM, and what a gated module
//! does. A gated module reading all ones and the reset values are measured on an MCF54415; a gated module dropping
//! writes and holding its events are guesses.

use mcf5441x_emu::soc::{Chip, ChipOptions, Gates, Mcf5441x, Pit, StubBlock};
use mcf5441x_emu::{Machine, Size};

const DDR: u32 = 0x4000_0000;

const SCM: u32 = 0xFC04_0000;
const PPMSR0: u32 = SCM + 0x2C;
const PPMCR0: u32 = SCM + 0x2D;
const PPMSR1: u32 = SCM + 0x2E;
const PPMCR1: u32 = SCM + 0x2F;
const PPMHR0: u32 = SCM + 0x30;
const PPMLR0: u32 = SCM + 0x34;
const PPMHR1: u32 = SCM + 0x38;
const PPMLR1: u32 = SCM + 0x3C;
/// SAMCD, CAMCD.
const ALL: u32 = 0x40;

/// SSI1 (RM Table 1-3), CD50 of PPMHR0 (RM Table 9-5).
const SSI1: u32 = 0xFC0C_8000;
const SSI1_CD: u32 = 50;
const SSI_CR: u32 = SSI1 + 0x10;
const SSI_ISR: u32 = SSI1 + 0x14;
const SSI_IER: u32 = SSI1 + 0x18;
const SSI_FCSR: u32 = SSI1 + 0x2C;
/// DMA timer 2 (RM Table 1-3), CD30 of PPMLR0 (RM Table 9-6): not modelled.
const DMA_TIMER_2: u32 = 0xFC07_8000;
/// PIT0 (RM Table 1-3), CD32 of PPMHR0 (RM Table 9-5).
const PIT0: u32 = 0xFC08_0000;
const PIT0_CD: u32 = 32;
const PCSR: u32 = PIT0;
const PMR: u32 = PIT0 + 2;
const PCNTR: u32 = PIT0 + 4;
const EN: u32 = 0x01;
const RLD: u32 = 0x02;

/// The gates measured after a soft reset: PPMHR0, PPMLR0, PPMHR1, PPMLR1.
const RESET: [u32; 4] = [0x7F9E_FBFF, 0xFFDE_F300, 0xFFFF_FFCF, 0x3F00_C0F4];

fn chip(gates: Gates) -> (Machine, Chip) {
    let mut m = Machine::new();
    m.mem.map_ram(DDR, 0x0100_0000);
    let chip = Mcf5441x::build(&mut m, ChipOptions::default().with_gates(gates));
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

fn registers(m: &mut Machine) -> [u32; 4] {
    [PPMHR0, PPMLR0, PPMHR1, PPMLR1].map(|a| r(m, a, Size::Long))
}

#[test]
fn a_gated_module_reads_all_ones() {
    // measured on an MCF54415: SSI1 (and the eSDHC, the DDR controller) after reset, and DMA timer 2 under a boot
    // chain, read all ones at every size
    let (mut m, _) = chip(Gates::Reset);
    for addr in [SSI_CR, SSI_FCSR, SSI1 + 0x3FFC] {
        assert_eq!(r(&mut m, addr, Size::Long), 0xFFFF_FFFF, "{addr:#x}");
        assert_eq!(r(&mut m, addr + 2, Size::Word), 0xFFFF, "{addr:#x}");
        assert_eq!(r(&mut m, addr + 3, Size::Byte), 0xFF, "{addr:#x}");
    }
    // a module nobody models reads all ones too, instead of stopping the run
    assert_eq!(r(&mut m, DMA_TIMER_2, Size::Long), 0xFFFF_FFFF);
    // a misaligned access is split, and every piece reads ones
    assert_eq!(r(&mut m, SSI_CR + 1, Size::Long), 0xFFFF_FFFF);
}

#[test]
fn opening_the_gate_through_ppmcr0_gives_reset_values() {
    // measured on an MCF54415: with CD50 cleared through PPMCR0, SSI1 reads RM ch. 35's reset values
    let (mut m, chip) = chip(Gates::Reset);
    w(&mut m, PPMCR0, Size::Byte, SSI1_CD);
    assert_eq!(chip.gates(&m)[0], RESET[0] & !(1 << (SSI1_CD - 32)));
    assert_eq!(r(&mut m, SSI_CR, Size::Long), 0);
    assert_eq!(r(&mut m, SSI_ISR, Size::Long), 0x3003);
    assert_eq!(r(&mut m, SSI_IER, Size::Long), 0x3003);
    assert_eq!(r(&mut m, SSI_FCSR, Size::Long), 0x0081_0081);
    // the SCM stub still logs the write
    let scm = chip.stubs.iter().find(|(n, _)| *n == "SCM").unwrap().1;
    let log = m.device::<StubBlock>(scm).writes().to_vec();
    assert_eq!(log, [(0, 0x2D, Size::Byte, SSI1_CD)]);
}

#[test]
fn writes_to_a_gated_module_are_dropped_guess() {
    // guess: a write to a gated module is dropped (not measured)
    let (mut m, _) = chip(Gates::Reset);
    w(&mut m, SSI_IER, Size::Long, 0);
    w(&mut m, SSI_FCSR, Size::Long, 0x0044_0044);
    // a write the model would refuse never reaches it
    w(&mut m, SSI_CR, Size::Long, 0xFFFF_FFFF);
    w(&mut m, DMA_TIMER_2, Size::Long, 1);
    w(&mut m, PPMCR0, Size::Byte, SSI1_CD);
    assert_eq!(r(&mut m, SSI_CR, Size::Long), 0);
    assert_eq!(r(&mut m, SSI_IER, Size::Long), 0x3003);
    assert_eq!(r(&mut m, SSI_FCSR, Size::Long), 0x0081_0081);
}

#[test]
fn a_gated_pit_does_not_count_guess() {
    // guess: a gated module's events are held, and it continues where it stopped when the gate opens (not
    // measured)
    let (mut m, chip) = chip(Gates::Reset);
    // PIT0 is gated after reset
    assert_eq!(r(&mut m, PCNTR, Size::Word), 0xFFFF);
    w(&mut m, PPMCR0, Size::Byte, PIT0_CD);
    w(&mut m, PMR, Size::Word, 99);
    w(&mut m, PCSR, Size::Word, EN | RLD); // a period of 100 ticks from tick 0
    assert_eq!(m.idle_until(250), None);
    assert_eq!(m.device::<Pit>(chip.pit[0]).expiries(), 2);
    assert_eq!(r(&mut m, PCNTR, Size::Word), 49);

    w(&mut m, PPMSR0, Size::Byte, PIT0_CD);
    assert_eq!(r(&mut m, PCNTR, Size::Word), 0xFFFF);
    assert_eq!(m.idle_until(10_000), None);
    assert_eq!(
        m.device::<Pit>(chip.pit[0]).expiries(),
        2,
        "no expiry while gated"
    );

    w(&mut m, PPMCR0, Size::Byte, PIT0_CD);
    assert_eq!(
        r(&mut m, PCNTR, Size::Word),
        49,
        "the count went on from where it stopped"
    );
    assert_eq!(m.idle_until(10_049), None);
    assert_eq!(m.device::<Pit>(chip.pit[0]).expiries(), 2);
    assert_eq!(m.idle_until(10_050), None);
    assert_eq!(m.device::<Pit>(chip.pit[0]).expiries(), 3);
}

#[test]
fn a_gated_module_reads_all_ones_to_a_bus_master() {
    // the eDMA (CD17) copies SSI1's FCSR into RAM while SSI1 is gated
    let (mut m, _) = chip(Gates::Open);
    w(&mut m, PPMSR0, Size::Byte, SSI1_CD);
    let tcd = 0xFC04_4000 + 0x1000 + 32;
    for (off, size, v) in [
        (0x00, Size::Long, SSI_FCSR), // SADDR
        (0x04, Size::Word, 0x0202),   // ATTR: 32-bit
        (0x06, Size::Word, 0),        // SOFF
        (0x08, Size::Long, 4),        // NBYTES
        (0x0C, Size::Long, 0),        // SLAST
        (0x10, Size::Long, DDR),      // DADDR
        (0x14, Size::Word, 1),        // CITER
        (0x16, Size::Word, 4),        // DOFF
        (0x18, Size::Long, 0),        // DLAST
        (0x1C, Size::Word, 1),        // BITER
        (0x1E, Size::Word, 1),        // CSR: START
    ] {
        w(&mut m, tcd + off, size, v);
    }
    assert_eq!(m.mem.peek(DDR, 4).unwrap(), [0xFF; 4]);
}

#[test]
fn ppmsr_and_ppmcr_set_and_clear_one_or_all() {
    let (mut m, chip) = chip(Gates::Open);
    w(&mut m, PPMSR0, Size::Byte, SSI1_CD);
    w(&mut m, PPMSR0, Size::Byte, 2);
    assert_eq!(chip.gates(&m), [1 << 18, 1 << 2, 0, 0]);
    w(&mut m, PPMSR1, Size::Byte, ALL);
    assert_eq!(chip.gates(&m), [1 << 18, 1 << 2, !0, !0]);
    w(&mut m, PPMCR1, Size::Byte, 36);
    w(&mut m, PPMCR1, Size::Byte, 63);
    assert_eq!(chip.gates(&m), [1 << 18, 1 << 2, !(1 << 4 | 1 << 31), !0]);
    w(&mut m, PPMCR0, Size::Byte, ALL);
    assert_eq!(chip.gates(&m), [0, 0, !(1 << 4 | 1 << 31), !0]);
    assert_eq!(registers(&mut m), chip.gates(&m));
    // the set and clear registers read 0
    for a in [PPMSR0, PPMCR0, PPMSR1, PPMCR1] {
        assert_eq!(r(&mut m, a, Size::Byte), 0, "{a:#x}");
    }
    // PPM{H,L}Rn are read/write, at every size
    w(&mut m, PPMLR1, Size::Long, 0x1234_5678);
    w(&mut m, PPMHR0 + 2, Size::Word, 0xABCD);
    w(&mut m, PPMLR0 + 3, Size::Byte, 0x5C);
    assert_eq!(
        chip.gates(&m),
        [0xABCD, 0x5C, !(1 << 4 | 1 << 31), 0x1234_5678]
    );
    assert_eq!(r(&mut m, PPMLR1 + 1, Size::Byte), 0x34);
    // a longword write at +0x2C is a PPMSR0, PPMCR0, PPMSR1 and PPMCR1 write, in that order
    w(&mut m, PPMSR0, Size::Long, 0x0102_4040);
    assert_eq!(chip.gates(&m), [0xABCD, 0x5A, 0, 0]);
}

#[test]
fn reset_gates_are_the_measured_values() {
    // measured on an MCF54415 after a soft reset; RM Table 9-1 gives PPMHR0 0x7E1EFBFF (CD56 and CD55 clear)
    let (mut m, chip) = chip(Gates::Reset);
    assert_eq!(chip.gates(&m), RESET);
    assert_eq!(registers(&mut m), RESET);
    let (mut m, chip) = self::chip(Gates::Set {
        hr0: 1,
        lr0: 2,
        hr1: 3,
        lr1: 4,
    });
    assert_eq!(chip.gates(&m), [1, 2, 3, 4]);
    assert_eq!(registers(&mut m), [1, 2, 3, 4]);
}

#[test]
fn open_is_the_default() {
    let mut m = Machine::new();
    m.mem.map_ram(DDR, 0x0100_0000);
    let chip = Mcf5441x::build(&mut m, ChipOptions::default());
    assert_eq!(chip.gates(&m), [0; 4]);
    assert_eq!(registers(&mut m), [0; 4]);
    assert_eq!(r(&mut m, SSI_FCSR, Size::Long), 0x0081_0081);
    assert_eq!(r(&mut m, PCNTR, Size::Word), 0xFFFF);
}

#[test]
fn a_gated_module_keeps_its_lines_guess() {
    // guess: the interrupt and DMA request lines of a gated module stay as it left them (not measured)
    let (mut m, chip) = chip(Gates::Open);
    w(&mut m, PMR, Size::Word, 9);
    w(&mut m, PCSR, Size::Word, EN | RLD | 0x08); // PIE
    assert_eq!(m.idle_until(10), None);
    let pif = 1 << 13; // INTC2 source 13
    assert_eq!(m.mem.irq_lines().asserted(2) & pif, pif);
    w(&mut m, PPMSR0, Size::Byte, PIT0_CD);
    assert_eq!(m.mem.irq_lines().asserted(2) & pif, pif);
    // writing PIF to clear it is dropped: the line stays up
    w(&mut m, PCSR, Size::Word, EN | RLD | 0x08 | 0x04);
    assert_eq!(m.mem.irq_lines().asserted(2) & pif, pif);
    assert_eq!(m.device::<Pit>(chip.pit[0]).expiries(), 1);
    w(&mut m, PPMCR0, Size::Byte, PIT0_CD);
    w(&mut m, PCSR, Size::Word, EN | RLD | 0x08 | 0x04);
    assert_eq!(m.mem.irq_lines().asserted(2) & pif, 0);
}
