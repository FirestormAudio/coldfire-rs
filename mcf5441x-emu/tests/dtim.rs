//! The DMA timers (RM ch. 39) on the built chip, driven through their registers and by a snippet.

mod common;

use common::{prepare, set_vector, until};
use mcf5441x_emu::soc::{Chip, ChipOptions, Dtim, Gates, Mcf5441x};
use mcf5441x_emu::{Device, Fault, Machine, Size, Stop};

/// DMA timer n (RM Table 39-1).
const DTIM: [u32; 4] = [0xFC07_0000, 0xFC07_4000, 0xFC07_8000, 0xFC07_C000];
const DTMR: u32 = 0x0;
const DTXMR: u32 = 0x2;
const DTER: u32 = 0x3;
const DTRR: u32 = 0x4;
const DTCR: u32 = 0x8;
const DTCN: u32 = 0xC;

/// DTMR fields (RM Table 39-2).
const RST: u32 = 0x01;
const BUS: u32 = 0x02;
const BUS_16: u32 = 0x04;
const DTIN: u32 = 0x06;
const FRR: u32 = 0x08;
const ORRI: u32 = 0x10;
/// DTER REF (RM Table 39-4).
const REF: u32 = 0x02;

/// INTC0 (RM Table 1-3); DMA timer n is its source 32 + n (RM Table 17-15).
const INTC0: u32 = 0xFC04_8000;
const SOURCE: u8 = 32;

/// A counter the snippet's handler increments.
const COUNT: u32 = 0x4010_2000;

fn chip(options: ChipOptions) -> (Machine, Chip) {
    let (mut m, _) = prepare("nop", &[]);
    let chip = Mcf5441x::build(&mut m, options);
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

fn idle(m: &mut Machine, tick: u64) {
    assert_eq!(m.idle_until(tick), None);
    assert_eq!(m.now(), tick);
}

fn dtcn(m: &mut Machine, n: usize) -> u32 {
    r(m, DTIM[n] + DTCN, Size::Long)
}

fn line(m: &Machine, n: usize) -> bool {
    m.mem.irq_lines().asserted(0) >> (SOURCE as usize + n) & 1 != 0
}

fn refusal(m: &mut Machine, addr: u32, size: Size, value: u32) -> String {
    assert_eq!(
        m.access(addr, size, Some(value)),
        Err(Fault::Device(Stop::Unmodelled)),
        "{addr:#x} <- {value:#x}"
    );
    m.take_refusal().expect("a refusal").detail
}

#[test]
fn dtim_reset_values_follow_the_rm() {
    // RM Table 39-1: DTMR 0, DTXMR 0, DTER 0, DTRR 0xFFFF_FFFF, DTCR 0, DTCN 0; a timer with RST clear holds DTCN.
    let (mut m, _) = chip(ChipOptions::default());
    for base in DTIM {
        assert_eq!(r(&mut m, base + DTMR, Size::Long), 0);
        assert_eq!(r(&mut m, base + DTRR, Size::Long), 0xFFFF_FFFF);
        assert_eq!(r(&mut m, base + DTCR, Size::Long), 0);
        assert_eq!(r(&mut m, base + DTCN, Size::Long), 0);
    }
    idle(&mut m, 1000);
    assert_eq!(dtcn(&mut m, 0), 0);
}

#[test]
fn dtim_counts_the_bus_clock() {
    // DTMR0 = 3 (RST, CLK = bus clock / 1): DTCN counts one per tick.
    let (mut m, _) = chip(ChipOptions::default());
    idle(&mut m, 100);
    w(&mut m, DTIM[0] + DTMR, Size::Word, RST | BUS);
    assert_eq!(dtcn(&mut m, 0), 0);
    idle(&mut m, 100 + 12_345);
    assert_eq!(dtcn(&mut m, 0), 12_345);
    // reading does not disturb the count (RM 39.2.6), and a write clears it
    assert_eq!(dtcn(&mut m, 0), 12_345);
    w(&mut m, DTIM[0] + DTCN + 3, Size::Byte, 0x55);
    assert_eq!(dtcn(&mut m, 0), 0);
    idle(&mut m, 100 + 12_345 + 7);
    assert_eq!(dtcn(&mut m, 0), 7);
    // CLK = 0 stops the count and holds it; RST from 1 to 0 clears it (guess: DTCN and DTER cleared)
    w(&mut m, DTIM[0] + DTMR, Size::Word, RST);
    idle(&mut m, 100 + 12_345 + 50);
    assert_eq!(dtcn(&mut m, 0), 7);
    w(&mut m, DTIM[0] + DTMR, Size::Word, 0);
    assert_eq!(dtcn(&mut m, 0), 0);
}

#[test]
fn dtim_prescaler_and_divide_by_16() {
    let (mut m, _) = chip(ChipOptions::default());
    // PS = 4: the bus clock divided by 5
    w(&mut m, DTIM[1] + DTMR, Size::Word, 4 << 8 | RST | BUS);
    idle(&mut m, 5 * 100 + 4);
    assert_eq!(r(&mut m, DTIM[1] + DTCN, Size::Long), 100);
    // the bus clock / 16 with PS = 0, started at tick 0
    let (mut m, _) = chip(ChipOptions::default());
    w(&mut m, DTIM[2] + DTMR, Size::Word, RST | BUS_16);
    idle(&mut m, 16 * 30 + 15);
    assert_eq!(dtcn(&mut m, 2), 30);
    // both: / 16 / 3
    let (mut m, _) = chip(ChipOptions::default());
    w(&mut m, DTIM[3] + DTMR, Size::Word, 2 << 8 | RST | BUS_16);
    idle(&mut m, 48 * 10);
    assert_eq!(dtcn(&mut m, 3), 10);
    // MODE16: each increment adds 65 537 (RM Table 39-3)
    let (mut m, _) = chip(ChipOptions::default());
    w(&mut m, DTIM[0] + DTXMR, Size::Byte, 0x01);
    w(&mut m, DTIM[0] + DTMR, Size::Word, RST | BUS);
    idle(&mut m, 3);
    assert_eq!(dtcn(&mut m, 0), 3 * 65_537);
}

#[test]
fn dtim_reference_interrupt_restarts_with_frr() {
    // DTRR3 = 275000 and DTMR3 = 0x1D (bus clock / 16, FRR, ORRI, RST): REF and the interrupt every
    // (275000 + 1) x 16 bus clocks (RM Eqn. 39-1: the reference matches after DTRR + 1 increments), and the count
    // starts again from 0.
    let period = (275_000 + 1) * 16;
    let (mut m, chip) = chip(ChipOptions::default());
    w(&mut m, DTIM[3] + DTRR, Size::Long, 275_000);
    w(&mut m, DTIM[3] + DTMR, Size::Word, 0x1D);
    assert_eq!(0x1D, ORRI | FRR | BUS_16 | RST);
    assert_eq!(m.device::<Dtim>(chip.dtim[3]).next_event(), Some(period));
    idle(&mut m, period - 1);
    assert_eq!(dtcn(&mut m, 3), 275_000);
    assert_eq!(r(&mut m, DTIM[3] + DTER, Size::Byte), 0);
    assert!(!line(&m, 3));
    idle(&mut m, period);
    assert_eq!(dtcn(&mut m, 3), 0, "FRR: a new count starts");
    assert_eq!(r(&mut m, DTIM[3] + DTER, Size::Byte), REF);
    assert!(line(&m, 3), "INTC0 source 35");
    // write 1 to clear (RM 39.2.3) drops the line
    w(&mut m, DTIM[3] + DTER, Size::Byte, REF);
    assert!(!line(&m, 3));
    idle(&mut m, 3 * period);
    assert_eq!(m.device::<Dtim>(chip.dtim[3]).references(), 3);
    assert_eq!(dtcn(&mut m, 3), 0);

    // without FRR the count goes on past the reference
    let (mut m, _) = chip_default();
    w(&mut m, DTIM[0] + DTRR, Size::Long, 9);
    w(&mut m, DTIM[0] + DTMR, Size::Word, RST | BUS);
    idle(&mut m, 10);
    assert_eq!(r(&mut m, DTIM[0] + DTER, Size::Byte), REF);
    assert!(!line(&m, 0), "ORRI clear: no interrupt");
    idle(&mut m, 25);
    assert_eq!(dtcn(&mut m, 0), 25);
}

fn chip_default() -> (Machine, Chip) {
    chip(ChipOptions::default())
}

#[test]
fn dtim_interrupt_reaches_the_core() {
    // The handler counts and clears REF; INTC0 source 35 at level 3 is vector 64 + 35.
    let src = "
        lea 0x40102000, %a1
        lea 0xFC07C003, %a2
        clr.l (%a1)
        move.w #0x2000, %sr
    spin:
        move.l (%a1), %d0
        cmp.l #3, %d0
        bne.s spin
        bra.s done
    handler:
        addq.l #1, (%a1)
        move.b #2, (%a2)
        rte
    done:
        nop
    ";
    let (mut m, l) = prepare(src, &["handler", "done"]);
    Mcf5441x::build(&mut m, ChipOptions::default());
    set_vector(&mut m, 64 + 35, l[0]);
    w(&mut m, INTC0 + 0x40 + 35, Size::Byte, 3); // ICR35: level 3
    w(&mut m, INTC0 + 0x1D, Size::Byte, 35); // CIMR: unmask 35
    w(&mut m, DTIM[3] + DTRR, Size::Long, 99);
    w(&mut m, DTIM[3] + DTMR, Size::Word, FRR | ORRI | RST | BUS);
    let stop = m.run(until(l[1]));
    assert_eq!(stop.kind, Stop::UntilPc, "{stop:?}");
    assert_eq!(m.mem.read(COUNT, Size::Long).unwrap(), 3);
    assert!(m.now() >= 300 && m.now() < 400, "{}", m.now());
}

#[test]
fn dtim_dtin_counts_edges_when_given() {
    // DTMR2 = 7 (RST, CLK = DTIN2): one count per falling edge, every 40 ticks from tick 0
    // (guess: the edges are on a grid from tick 0; a board's DTIN rate is the board's)
    let (mut m, _) = chip(ChipOptions::default().with_dtin(2, Some(40)));
    idle(&mut m, 10);
    w(&mut m, DTIM[2] + DTMR, Size::Word, RST | DTIN);
    idle(&mut m, 39);
    assert_eq!(dtcn(&mut m, 2), 0);
    idle(&mut m, 40);
    assert_eq!(dtcn(&mut m, 2), 1);
    idle(&mut m, 40 * 50 + 39);
    assert_eq!(dtcn(&mut m, 2), 50);
}

#[test]
fn dtim_dtin_never_counts_without_dtin_guess() {
    // guess: with no DTIN given the pin never toggles, so CLK = DTIN never counts and no reference event comes
    let (mut m, chip) = chip(ChipOptions::default());
    w(&mut m, DTIM[2] + DTRR, Size::Long, 0);
    w(&mut m, DTIM[2] + DTMR, Size::Word, ORRI | RST | DTIN);
    assert_eq!(m.device::<Dtim>(chip.dtim[2]).next_event(), None);
    idle(&mut m, 1_000_000);
    assert_eq!(dtcn(&mut m, 2), 0);
    assert_eq!(r(&mut m, DTIM[2] + DTER, Size::Byte), 0);
}

#[test]
fn dtim_capture_mode_stops_unmodelled() {
    let (mut m, _) = chip(ChipOptions::default());
    for ce in [0x40, 0x80, 0xC0] {
        let d = refusal(&mut m, DTIM[1] + DTMR, Size::Word, ce | RST | BUS);
        assert_eq!(d, "DMA timer 1: capture mode (DTMR CE) not modelled");
    }
    let d = refusal(&mut m, DTIM[1] + DTMR, Size::Word, 0x20 | RST | BUS);
    assert_eq!(d, "DMA timer 1: toggle output (DTMR OM) not modelled");
    // a refused write changes nothing
    assert_eq!(r(&mut m, DTIM[1] + DTMR, Size::Word), 0);
    // and a program that sets CE stops on that instruction
    let (mut m, l) = prepare(
        "lea 0xFC074000, %a0\nmove.w #0x0043, (%a0)\nend:\nnop",
        &["end"],
    );
    Mcf5441x::build(&mut m, ChipOptions::default());
    let stop = m.run(until(l[0]));
    assert_eq!(stop.kind, Stop::Unmodelled);
    assert_eq!(stop.pc, common::BASE + 6, "on the move");
}

#[test]
fn dtim_dmaen_stops_unmodelled() {
    let (mut m, _) = chip(ChipOptions::default());
    let d = refusal(&mut m, DTIM[0] + DTXMR, Size::Byte, 0x80);
    assert_eq!(d, "DMA timer 0: DMA request (DTXMR DMAEN) not modelled");
    // through a long write at +0 as well, and nothing of it is taken
    let d = refusal(&mut m, DTIM[0] + DTMR, Size::Long, 0x0003_8000);
    assert_eq!(d, "DMA timer 0: DMA request (DTXMR DMAEN) not modelled");
    assert_eq!(r(&mut m, DTIM[0] + DTMR, Size::Long), 0);
    let d = refusal(&mut m, DTIM[0] + DTXMR, Size::Byte, 0x20);
    assert_eq!(
        d,
        "DMA timer 0: IEEE 1588 timebase (DTXMR EN1588) not modelled"
    );
    // HALTED and MODE16 are taken; the reserved bits are not
    w(&mut m, DTIM[0] + DTXMR, Size::Byte, 0x4F);
    assert_eq!(r(&mut m, DTIM[0] + DTXMR, Size::Byte), 0x41);
}

#[test]
fn gated_dtim_reads_all_ones() {
    // measured on an MCF54415 for a gated module: all ones at every size. guess: the gated timer stops counting
    // and continues where it stopped when its gate (CD28-CD31, RM Table 9-6) opens.
    let (mut m, _) = chip(ChipOptions::default().with_gates(Gates::Open));
    w(&mut m, DTIM[0] + DTMR, Size::Word, RST | BUS);
    idle(&mut m, 100);
    w(&mut m, 0xFC04_002C, Size::Byte, 28); // PPMSR0: close CD28
    assert_eq!(dtcn(&mut m, 0), 0xFFFF_FFFF);
    assert_eq!(r(&mut m, DTIM[0] + DTMR, Size::Word), 0xFFFF);
    assert_eq!(r(&mut m, DTIM[0] + DTER, Size::Byte), 0xFF);
    idle(&mut m, 1000);
    w(&mut m, 0xFC04_002D, Size::Byte, 28); // PPMCR0: open CD28
    assert_eq!(dtcn(&mut m, 0), 100);
    idle(&mut m, 1010);
    assert_eq!(dtcn(&mut m, 0), 110);
    // and under the reset gates every DMA timer is gated
    let (mut m, _) = chip(ChipOptions::default().with_gates(Gates::Reset));
    for base in DTIM {
        assert_eq!(r(&mut m, base + DTCN, Size::Long), 0xFFFF_FFFF);
    }
}

#[test]
fn dtim_clock_change_while_running_stops_the_count_guess() {
    // RM Table 39-2: "Avoid setting CLK when RST is already set. Doing so causes CLK to zero (stop counting)".
    // guess: a write that changes CLK while RST stays set leaves CLK 0; one that keeps CLK keeps counting.
    let (mut m, _) = chip(ChipOptions::default());
    w(&mut m, DTIM[0] + DTMR, Size::Word, RST | BUS);
    idle(&mut m, 10);
    w(&mut m, DTIM[0] + DTMR, Size::Word, ORRI | RST | BUS);
    idle(&mut m, 20);
    assert_eq!(dtcn(&mut m, 0), 20);
    w(&mut m, DTIM[0] + DTMR, Size::Word, RST | BUS_16);
    assert_eq!(r(&mut m, DTIM[0] + DTMR, Size::Word), RST);
    idle(&mut m, 100);
    assert_eq!(dtcn(&mut m, 0), 20);
}
