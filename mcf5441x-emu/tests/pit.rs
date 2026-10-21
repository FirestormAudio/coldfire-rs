//! The programmable interrupt timers (RM ch. 38), driven through their registers and by snippets.

mod common;

use common::{STACK, prepare, set_vector, until};
use mcf5441x_emu::soc::{Intc, Pit, Uart, UartConfig};
use mcf5441x_emu::{DeviceId, Limit, Machine, Size, Stop};

const PIT0: u32 = 0xFC08_0000;
const UART9: u32 = 0xEC07_4000;
const INTC2: u32 = 0xFC05_0000;
/// The counter the fixtures' handlers increment.
const COUNT: u32 = 0x4010_2000;
/// INTC2 source of PIT0 (vector 205).
const SOURCE: u8 = 13;

const PCSR: u32 = PIT0;
const PMR: u32 = PIT0 + 2;
const PCNTR: u32 = PIT0 + 4;

const EN: u32 = 0x01;
const RLD: u32 = 0x02;
const PIF: u32 = 0x04;
const PIE: u32 = 0x08;
const OVW: u32 = 0x10;

/// Assemble `src`, with INTC2, PIT0 (controller 2, source 13) and UART9 (transmitter enabled, no interrupt
/// controller behind it) added; returns the machine, the PIT, the UART and the addresses of `labels`.
fn board_uart(src: &str, labels: &[&str]) -> (Machine, DeviceId, DeviceId, Vec<u32>) {
    let (mut m, l) = prepare(src, labels);
    let ([_, _, intc2], _) = Intc::chip();
    m.add_interrupt_controller(INTC2, 0x4000, Box::new(intc2));
    let id = m.add_device(PIT0, 0x4000, Box::new(Pit::new(2, SOURCE)));
    let uart = m.add_device(
        UART9,
        0x4000,
        Box::new(Uart::new(UartConfig {
            controller: 1,
            source: 53,
            rx_dma: None,
            ticks_per_byte: 0,
        })),
    );
    w(&mut m, UART9 + 8, Size::Byte, 0x04); // UCR: transmitter enable
    (m, id, uart, l)
}

/// [`board_uart`] without the UART's id.
fn board(src: &str, labels: &[&str]) -> (Machine, DeviceId, Vec<u32>) {
    let (m, id, _, l) = board_uart(src, labels);
    (m, id, l)
}

/// A board running nothing, for tests that drive the registers.
fn regs() -> (Machine, DeviceId) {
    let (m, id, _) = board("nop", &[]);
    (m, id)
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

fn line(m: &Machine) -> bool {
    m.mem.irq_lines().asserted(2) & (1 << SOURCE) != 0
}

fn next(m: &Machine, id: DeviceId) -> Option<u64> {
    use mcf5441x_emu::Device;
    m.device::<Pit>(id).next_event()
}

#[test]
fn pit_reset_values_follow_the_rm() {
    // RM 38.2: PCSR 0x0000, PMR 0xFFFF, PCNTR 0xFFFF.
    let (mut m, id) = regs();
    assert_eq!(r(&mut m, PCSR, Size::Word), 0);
    assert_eq!(r(&mut m, PMR, Size::Word), 0xFFFF);
    assert_eq!(r(&mut m, PCNTR, Size::Word), 0xFFFF);
    assert_eq!(next(&m, id), None);
    idle(&mut m, 1000);
    assert_eq!(
        r(&mut m, PCNTR, Size::Word),
        0xFFFF,
        "a disabled counter is held"
    );
}

#[test]
fn byte_accesses_hit_their_lanes() {
    let (mut m, _) = regs();
    w(&mut m, PCSR, Size::Byte, 0x02); // PCSR high byte: PRE = 2
    w(&mut m, PCSR + 1, Size::Byte, 0x18); // low byte: PIE | OVW
    assert_eq!(r(&mut m, PCSR, Size::Word), 0x0218);
    assert_eq!(r(&mut m, PCSR, Size::Byte), 0x02);
    assert_eq!(r(&mut m, PCSR + 1, Size::Byte), 0x18);
    w(&mut m, PMR + 1, Size::Byte, 0x34);
    assert_eq!(r(&mut m, PMR, Size::Word), 0xFF34);
}

#[test]
fn byte_writes_keep_the_other_lane_and_clear_pif() {
    let (mut m, _) = regs();
    w(&mut m, PMR, Size::Word, 9);
    w(&mut m, PCSR, Size::Word, RLD | EN);
    idle(&mut m, 10);
    assert_ne!(r(&mut m, PCSR, Size::Word) & PIF, 0);
    // A high-byte write leaves PIF and EN alone.
    w(&mut m, PCSR, Size::Byte, 0x01);
    assert_eq!(r(&mut m, PCSR, Size::Word), 0x0100 | PIF | RLD | EN);
    // A low-byte write with PIF set clears PIF and keeps the high byte.
    w(&mut m, PCSR + 1, Size::Byte, PIF | RLD | EN);
    assert_eq!(r(&mut m, PCSR, Size::Word), 0x0100 | RLD | EN);
    // A PMR high-byte write keeps the low byte and clears PIF.
    idle(&mut m, 10 + 20);
    assert_ne!(r(&mut m, PCSR, Size::Word) & PIF, 0);
    w(&mut m, PMR, Size::Byte, 0x01);
    assert_eq!(r(&mut m, PMR, Size::Word), 0x0109);
    assert_eq!(r(&mut m, PCSR, Size::Word) & PIF, 0);
}

#[test]
fn pre_change_while_running_keeps_the_count_continuous() {
    // PMR 99, PRE 0: PCNTR is 49 at tick 50. Writing PRE = 2 keeps the count and slows it to one per 4 ticks.
    // guess: the prescaler's partial count is dropped (RM 38.2.1 only says the prescaler counter resets).
    let (mut m, id) = regs();
    w(&mut m, PMR, Size::Word, 99);
    w(&mut m, PCSR, Size::Word, RLD | EN);
    idle(&mut m, 50);
    assert_eq!(r(&mut m, PCNTR, Size::Word), 49);
    w(&mut m, PCSR, Size::Word, 0x0200 | RLD | EN);
    assert_eq!(r(&mut m, PCNTR, Size::Word), 49);
    assert_eq!(next(&m, id), Some(50 + 50 * 4));
    idle(&mut m, 50 + 8);
    assert_eq!(r(&mut m, PCNTR, Size::Word), 47);
    // Disabling captures the count at the new rate.
    w(&mut m, PCSR, Size::Word, 0x0200 | RLD);
    assert_eq!(r(&mut m, PCNTR, Size::Word), 47);
    // The expiry lands when the count runs out, then the period is PMR at the new rate.
    w(&mut m, PCSR, Size::Word, 0x0200 | RLD | EN);
    assert_eq!(next(&m, id), Some(58 + 100 * 4));
    // Raising the rate mid-run, too.
    idle(&mut m, 58 + 40);
    assert_eq!(r(&mut m, PCNTR, Size::Word), 89);
    w(&mut m, PCSR, Size::Word, RLD | EN);
    assert_eq!(next(&m, id), Some(58 + 40 + 90));
    idle(&mut m, 58 + 40 + 90);
    assert_eq!(m.device::<Pit>(id).expiries(), 1);
}

/// Let the UART's transmitter take the character still in its holding register.
fn finish_sending(m: &mut Machine) {
    let t = m.now() + 10;
    assert_eq!(m.idle_until(t), None);
}

#[test]
fn pit_stop_wakes_and_fast_forwards() {
    let (mut m, _, uart, l) = board_uart(include_str!("fixtures/pitstop.s"), &["done", "pit_isr"]);
    set_vector(&mut m, 205, l[1]);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.mem.read(COUNT, Size::Long), Ok(5));
    // Five 132000-tick periods of emulated time, which passed in STOP, not in executed code.
    assert!((5 * 132_000..6 * 132_000).contains(&m.now()), "{}", m.now());
    assert!(m.instructions() < m.now() / 100, "{}", m.instructions());
    assert_eq!(m.cpu.a[7], STACK);
    // The last character starts out on the line a few ticks after its write, past `done`.
    finish_sending(&mut m);
    assert_eq!(m.device::<Uart>(uart).text(), "ok");
}

#[test]
fn uart_bytes_carry_cycle_stamps() {
    // The bytes the program writes after the fifth period are stamped with sorted ticks, at least 5 * 132000.
    let (mut m, _, uart, l) = board_uart(include_str!("fixtures/pitstop.s"), &["done", "pit_isr"]);
    set_vector(&mut m, 205, l[1]);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    finish_sending(&mut m);
    let stamps: Vec<u64> = m
        .device::<Uart>(uart)
        .sent()
        .iter()
        .map(|&(t, _)| t)
        .collect();
    assert_eq!(stamps.len(), 2);
    assert!(stamps.is_sorted(), "{stamps:?}");
    assert!(stamps[0] >= 5 * 132_000, "{stamps:?}");
}

#[test]
fn pif_clear_keeps_period() {
    // Expiries stay on the original grid, and clearing PIF does not move the next one.
    let (mut m, id) = regs();
    w(&mut m, PMR, Size::Word, 32999);
    w(&mut m, PCSR, Size::Word, 0x021F);
    idle(&mut m, 132_000 * 3 + 10);
    assert_eq!(m.device::<Pit>(id).expiries(), 3);
    assert_eq!(next(&m, id), Some(132_000 * 4));
    w(&mut m, PCSR, Size::Word, 0x021F); // PIF set in the write: write-1-to-clear
    assert_eq!(r(&mut m, PCSR, Size::Word) & PIF, 0);
    assert_eq!(next(&m, id), Some(132_000 * 4));
}

#[test]
fn interrupt_pending_at_stop_is_taken_once() {
    // No poll stop: the wait loop reads PCSR tens of thousands of times (PIT reads are not poll-counted).
    let (mut m, _, l) = board(include_str!("fixtures/stoppending.s"), &["done", "pit_isr"]);
    set_vector(&mut m, 205, l[1]);
    let r = m.run(Limit {
        until_pc: Some(l[0]),
        max_instructions: Some(200_000),
    });
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.a[7], STACK, "no leaked exception frame");
    assert_eq!(
        m.mem.read(COUNT, Size::Long),
        Ok(2),
        "once before the stop, once to wake it"
    );
}

/// Set up PIT0 as the fixtures do, then `body`.
fn setup(body: &str) -> String {
    format!(
        "moveq #6,%d0\nmove.b %d0,0xFC05004D\nmoveq #13,%d0\nmove.b %d0,0xFC05001D\n\
         move.w #32999,%d0\nmove.w %d0,0xFC080002\n{body}\nend: nop"
    )
}

#[test]
fn stop_with_only_masked_timer_halts() {
    // Running with PIE set, but at IPL 7 a level 6 request cannot be taken: halted, not an endless fast-forward.
    let (mut m, _, l) = board(
        &setup("move.w #0x021F,%d0\nmove.w %d0,0xFC080000\nstop #0x2700"),
        &["end"],
    );
    let r = m.run(Limit {
        until_pc: Some(l[0]),
        max_instructions: Some(10_000),
    });
    assert_eq!(r.kind, Stop::HaltedForever, "{r:?}");
}

#[test]
fn stop_with_a_running_timer_and_pie_clear_halts() {
    // The timer keeps scheduling, its source is unmasked at a level the STOP allows, but PIE is clear: nothing it
    // does can wake the core. The time limit only guards the test.
    let (mut m, _, l) = board(
        &setup("move.w #0x0013,%d0\nmove.w %d0,0xFC080000\nstop #0x2000"),
        &["end"],
    );
    m.set_time_limit(Some(10_000_000));
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::HaltedForever, "{r:?}");
    assert!(m.now() < 1000, "{}", m.now());
}

#[test]
fn clearing_pie_with_pif_set_deasserts() {
    let (mut m, _) = regs();
    w(&mut m, PMR, Size::Word, 99);
    w(&mut m, PCSR, Size::Word, 0x001B); // OVW PIE RLD EN, PRE = 0
    idle(&mut m, 100);
    assert!(line(&m));
    w(&mut m, PCSR, Size::Word, 0x0013); // PIE off, PIF left pending (written as 0)
    assert!(!line(&m));
    assert_ne!(r(&mut m, PCSR, Size::Word) & PIF, 0);
    w(&mut m, PCSR, Size::Word, 0x001B); // PIE back on with PIF still set
    assert!(line(&m));
}

#[test]
fn pmr_write_without_ovw_takes_effect_at_the_next_reload() {
    let (mut m, id) = regs();
    w(&mut m, PMR, Size::Word, 99); // period 100
    w(&mut m, PCSR, Size::Word, RLD | EN); // no OVW
    idle(&mut m, 30);
    w(&mut m, PMR, Size::Word, 49);
    assert_eq!(
        r(&mut m, PCNTR, Size::Word),
        69,
        "still counting down the old period"
    );
    assert_eq!(next(&m, id), Some(100));
    idle(&mut m, 100);
    assert_eq!(next(&m, id), Some(150), "the new period from the reload on");
    idle(&mut m, 110);
    assert_eq!(r(&mut m, PCNTR, Size::Word), 39);
    // RM 38.2.1: the PMR write also clears PIF (set at the reload at 100).
    assert_ne!(r(&mut m, PCSR, Size::Word) & PIF, 0);
    w(&mut m, PMR, Size::Word, 49);
    assert_eq!(r(&mut m, PCSR, Size::Word) & PIF, 0);
}

#[test]
fn pmr_write_clears_pif() {
    // RM 38.2.1 and RM 38.3.4: "Clear PIF by writing a 1 to it or by writing to the PMR", and the request with it.
    let (mut m, _) = regs();
    w(&mut m, PMR, Size::Word, 9);
    w(&mut m, PCSR, Size::Word, RLD | PIE | EN);
    idle(&mut m, 10);
    assert_ne!(r(&mut m, PCSR, Size::Word) & PIF, 0);
    assert!(line(&m));
    w(&mut m, PMR, Size::Word, 20);
    assert_eq!(r(&mut m, PCSR, Size::Word) & PIF, 0);
    assert!(!line(&m));
    // A write of 0 to PIF alone does nothing (writing 0 has no effect).
    idle(&mut m, 20);
    assert!(line(&m));
    w(&mut m, PCSR, Size::Word, RLD | PIE | EN);
    assert!(line(&m));
}

#[test]
fn free_running_mode_rolls_over_without_reload() {
    // RM 38.3.2: with RLD clear the counter rolls over from 0x0000 to 0xFFFF; it does not load PMR.
    let (mut m, id) = regs();
    w(&mut m, PMR, Size::Word, 9);
    w(&mut m, PCSR, Size::Word, EN | OVW);
    assert_eq!(next(&m, id), Some(10));
    idle(&mut m, 12);
    assert_eq!(m.device::<Pit>(id).expiries(), 1);
    assert_ne!(r(&mut m, PCSR, Size::Word) & PIF, 0);
    assert_eq!(r(&mut m, PCNTR, Size::Word), 0xFFFD);
    assert_eq!(next(&m, id), Some(10 + 0x1_0000));
    idle(&mut m, 10 + 0x1_0000);
    assert_eq!(m.device::<Pit>(id).expiries(), 2);
    // With RLD set the same timer reloads PMR instead.
    let (mut m, id) = regs();
    w(&mut m, PMR, Size::Word, 9);
    w(&mut m, PCSR, Size::Word, EN | RLD);
    idle(&mut m, 12);
    assert_eq!(r(&mut m, PCNTR, Size::Word), 7);
    assert_eq!(next(&m, id), Some(20));
}

#[test]
fn pcsr_keeps_dbg_and_doze() {
    let (mut m, _) = regs();
    w(&mut m, PMR, Size::Word, 0x1234);
    w(&mut m, PCSR, Size::Word, 0x0260); // PRE 2, DOZE, DBG
    assert_eq!(r(&mut m, PCSR, Size::Word), 0x0260);
}

#[test]
fn longword_accesses_read_all_ones_and_write_nothing() {
    // RM ch. 38 allows only byte and word accesses. Measured on an MCF54415 (PIT1, PCSR 0, PMR 0x83): a longword
    // read at +0 or +4 returns 0xFFFFFFFF with no exception, and a longword write at +0 changes nothing.
    let (mut m, id) = regs();
    w(&mut m, PMR, Size::Word, 0x83);
    for off in [0, 4] {
        assert_eq!(r(&mut m, PIT0 + off, Size::Long), u32::MAX);
    }
    w(&mut m, PCSR, Size::Long, 0x0003_0031);
    assert_eq!(r(&mut m, PCSR, Size::Word), 0);
    assert_eq!(r(&mut m, PMR, Size::Word), 0x83);
    assert_eq!(next(&m, id), None);
}
