//! The UARTs (RM ch. 41): UART9 on INTC1 source 53 with its receive DMA on eDMA channel 36 and its transmit DMA
//! on channel 37, driven through its registers, by the host's line input and by snippets.

mod common;

use mcf5441x_emu::soc::{Edma, Intc, Uart, UartConfig};
use mcf5441x_emu::{Device, DeviceId, Fault, Machine, Size, Stop};

// UART9: the same UART as the rest of this file; its transmit request is eDMA channel 37 (RM Table 19-6).
const UART9: u32 = 0xEC07_4000;
const INTC1: u32 = 0xFC04_C000;
const EDMA: u32 = 0xFC04_4000;
const RAM: u32 = 0x4000_0000;
/// The receive ring channel 36 writes (1 KiB aligned, for DMOD 10).
const RING: u32 = 0x4010_0400;

const UMR: u32 = UART9;
const USR: u32 = UART9 + 0x04;
const UCR: u32 = UART9 + 0x08;
const URB: u32 = UART9 + 0x0C;
const UISR: u32 = UART9 + 0x14;
const UIP: u32 = UART9 + 0x34;

const RXRDY: u32 = 0x01;
const TXRDY: u32 = 0x04;
const TXEMP: u32 = 0x08;
const OE: u32 = 0x10;
const FE: u32 = 0x40;

/// INTC1 source of UART9 (vector 181).
const SOURCE: u8 = 53;
/// UART9's receive DMA channel; its interrupt is INTC1 source 28.
const CH: u8 = 36;
/// UART9's transmit DMA channel (RM Table 19-6).
const TX_CH: u8 = 37;
/// The transmit ring channel 37 reads: 4 KiB, aligned for SMOD 12.
const TX_RING: u32 = 0x4040_0000;

/// RAM, INTC1, the eDMA and UART9 with 100 ticks per byte (its receive DMA on channel 36 and its transmit DMA on
/// channel 37 when `dma`).
fn board(dma: bool) -> (Machine, DeviceId) {
    let mut m = Machine::new();
    m.mem.map_ram(RAM, 0x0100_0000);
    add(m, dma)
}

/// Add INTC1, the eDMA and UART9 to `m`.
fn add(mut m: Machine, dma: bool) -> (Machine, DeviceId) {
    let ([_, intc1, _], _) = Intc::chip();
    m.add_interrupt_controller(INTC1, 0x4000, Box::new(intc1));
    m.add_bus_master(EDMA, 0x2000, Box::new(Edma::new()));
    let id = m.add_device(
        UART9,
        0x4000,
        Box::new(Uart::new(UartConfig {
            controller: 1,
            source: SOURCE,
            rx_dma: dma.then_some(CH),
            tx_dma: dma.then_some(TX_CH),
            ticks_per_byte: 100,
        })),
    );
    (m, id)
}

fn r(m: &mut Machine, addr: u32) -> u32 {
    m.access(addr, Size::Byte, None)
        .unwrap_or_else(|f| panic!("read {addr:#x}: {f:?} {:?}", m.take_refusal()))
}

fn w(m: &mut Machine, addr: u32, value: u32) {
    m.access(addr, Size::Byte, Some(value))
        .unwrap_or_else(|f| panic!("write {addr:#x}: {f:?} {:?}", m.take_refusal()));
}

fn w_sized(m: &mut Machine, addr: u32, size: Size, value: u32) {
    m.access(addr, size, Some(value))
        .unwrap_or_else(|f| panic!("write {addr:#x}: {f:?} {:?}", m.take_refusal()));
}

fn idle(m: &mut Machine, tick: u64) {
    assert_eq!(m.idle_until(tick), None);
    assert_eq!(m.now(), tick);
}

fn receive(m: &mut Machine, id: DeviceId, bytes: &[(u64, Option<u8>)]) {
    m.device_mut::<Uart>(id).receive(bytes);
}

fn uart(m: &Machine, id: DeviceId) -> &Uart {
    m.device::<Uart>(id)
}

fn source(m: &Machine, controller: usize, source: u8) -> bool {
    m.mem.irq_lines().asserted(controller) & 1 << source != 0
}

/// Channel 36 moving one byte per request from URB into [`RING`], INTMAJOR on each (SERQ not written).
fn rx_tcd(m: &mut Machine) {
    let t = EDMA + 0x1000 + 32 * u32::from(CH);
    for (off, size, v) in [
        (0x00, Size::Long, URB),     // SADDR
        (0x04, Size::Word, 10 << 3), // ATTR: DMOD 10, 8-bit
        (0x08, Size::Long, 1),       // NBYTES
        (0x10, Size::Long, RING),    // DADDR
        (0x16, Size::Word, 1),       // DOFF
        (0x14, Size::Word, 1),       // CITER
        (0x1C, Size::Word, 1),       // BITER
        (0x1E, Size::Word, 0x2),     // CSR: INTMAJOR
    ] {
        w_sized(m, t + off, size, v);
    }
}

fn serq(m: &mut Machine) {
    w(m, EDMA + 0x18, u32::from(CH));
}

fn daddr(m: &mut Machine) -> u32 {
    let a = EDMA + 0x1000 + 32 * u32::from(CH) + 0x10;
    m.access(a, Size::Long, None).unwrap()
}

#[test]
fn uart_reset_values_follow_the_rm() {
    // RM Table 41-3: USR, UISR, UMR1 and UMR2 0x00, URB 0xFF, UIP 0xFF; the receiver and the transmitter start
    // disabled (RM 41.3.5: they are enabled by UCR commands).
    let (mut m, id) = board(false);
    assert_eq!(r(&mut m, USR), 0);
    assert_eq!(r(&mut m, UISR), 0);
    assert_eq!(r(&mut m, URB), 0xFF);
    assert_eq!(r(&mut m, UIP), 0xFF);
    assert_eq!(r(&mut m, UMR), 0, "UMR1");
    assert_eq!(r(&mut m, UMR), 0, "UMR2");
    receive(&mut m, id, &[(0, Some(0x55))]);
    idle(&mut m, 100);
    assert_eq!(r(&mut m, USR) & RXRDY, 0, "the receiver is off");
    assert_eq!(uart(&m, id).lost(), 1);
    w(&mut m, URB, 0x41);
    assert!(uart(&m, id).sent().is_empty(), "the transmitter is off");
    assert_eq!(uart(&m, id).dropped(), 1);
    assert_eq!(m.dma_lines().asserted(), 0);
}

#[test]
fn mode_registers_follow_their_pointer() {
    // RM 41.3.1, RM 41.3.2: UMR1 is accessed first, then the pointer stays on UMR2 until RESET MODE REGISTER
    // POINTER (UCR MISC 1).
    let (mut m, _) = board(false);
    w(&mut m, UMR, 0x13);
    w(&mut m, UMR, 0x07);
    assert_eq!(r(&mut m, UMR), 0x07);
    w(&mut m, UCR, 0x10);
    assert_eq!(r(&mut m, UMR), 0x13);
    assert_eq!(r(&mut m, UMR), 0x07);
    assert_eq!(r(&mut m, UMR), 0x07);
}

#[test]
fn rx_byte_lands_after_its_frame_and_is_read_once() {
    let (mut m, id) = board(false);
    w(&mut m, UCR, 0x01); // receiver enable
    receive(&mut m, id, &[(1000, Some(0x90))]);
    assert_eq!(uart(&m, id).next_event(), Some(1100));
    idle(&mut m, 1099);
    assert_eq!(r(&mut m, USR) & RXRDY, 0);
    idle(&mut m, 1100);
    assert_ne!(r(&mut m, USR) & RXRDY, 0);
    assert_eq!(r(&mut m, URB), 0x90);
    assert_eq!(r(&mut m, USR) & RXRDY, 0);
    // guess: an empty URB reads the last character again (RM Table 41-3 gives only its reset value, 0xFF)
    assert_eq!(r(&mut m, URB), 0x90);
}

#[test]
fn second_byte_before_read_sets_oe_and_ucr_0x40_clears_it() {
    let (mut m, id) = board(false);
    w(&mut m, UCR, 0x01);
    receive(&mut m, id, &[(0, Some(1)), (100, Some(2))]);
    idle(&mut m, 200);
    assert_ne!(r(&mut m, USR) & OE, 0);
    assert_eq!(uart(&m, id).lost(), 1, "byte 2 lost");
    assert_eq!(r(&mut m, URB), 1);
    assert_ne!(r(&mut m, USR) & OE, 0, "OE stays until RESET ERROR STATUS");
    w(&mut m, UCR, 0x40); // reset error status
    assert_eq!(r(&mut m, USR) & OE, 0);
}

#[test]
fn bytes_before_receiver_enable_are_dropped() {
    let (mut m, id) = board(false);
    receive(&mut m, id, &[(0, Some(0xF8))]);
    idle(&mut m, 100);
    assert_eq!(r(&mut m, USR) & RXRDY, 0);
    assert_eq!(uart(&m, id).lost(), 1);
    w(&mut m, UCR, 0x01);
    w(&mut m, UCR, 0x20); // reset receiver disables it again
    receive(&mut m, id, &[(200, Some(0xF8))]);
    idle(&mut m, 300);
    assert_eq!(uart(&m, id).lost(), 2);
}

#[test]
fn enable_and_disable_come_before_misc() {
    // RC and TC are processed before MISC: 0x21 (enable, reset receiver) ends disabled, 0x34 likewise.
    let (mut m, id) = board(false);
    w(&mut m, UCR, 0x21);
    receive(&mut m, id, &[(0, Some(1))]);
    idle(&mut m, 100);
    assert_eq!(uart(&m, id).lost(), 1);
    w(&mut m, UCR, 0x34);
    assert_eq!(r(&mut m, USR) & (TXRDY | TXEMP), 0);
}

#[test]
fn rx_through_edma_before_and_after_serq() {
    // Without SERQ the byte waits in URB; with it, the request (RXRDY on DMA line 36) has the eDMA take it.
    let (mut m, id) = board(true);
    w(&mut m, UCR, 0x01);
    rx_tcd(&mut m);
    receive(&mut m, id, &[(0, Some(0x11))]);
    idle(&mut m, 100);
    assert_ne!(r(&mut m, USR) & RXRDY, 0, "still in URB");
    assert_eq!(
        m.dma_lines().asserted(),
        1 << CH,
        "the request follows RXRDY"
    );
    assert_eq!(r(&mut m, URB), 0x11);
    assert_eq!(m.dma_lines().asserted(), 0);
    serq(&mut m);
    let mut out = Vec::new();
    assert!(uart(&m, id).wake_sources(&mut out));
    assert_eq!(out, [], "nothing pending");
    receive(&mut m, id, &[(200, Some(0x22)), (300, Some(0x33))]);
    assert!(uart(&m, id).wake_sources(&mut out));
    assert_eq!(out, [], "the eDMA lists its own route; UIMR is 0");
    idle(&mut m, 400);
    assert_eq!(m.mem.peek(RING, 2).unwrap(), [0x22, 0x33]);
    assert_eq!(daddr(&mut m), RING + 2);
    assert_eq!(r(&mut m, USR) & (RXRDY | OE), 0, "no RXRDY, no OE");
    assert_eq!(uart(&m, id).lost(), 0);
    assert!(source(&m, 1, 28), "channel 36 -> INTC1 source 28");
}

#[test]
fn txrdy_interrupt_follows_uimr_and_byte_timing() {
    let (mut m, id) = board(false);
    w(&mut m, UCR, 0x04); // transmitter enable
    w(&mut m, UISR, 0x01); // UIMR: TXRDY
    assert!(source(&m, 1, SOURCE), "idle: TXRDY now");
    w(&mut m, URB, 0x41); // UTB
    assert!(!source(&m, 1, SOURCE));
    assert_eq!(
        uart(&m, id).next_event(),
        Some(3),
        "TXRDY returns when the character moves to the shift register"
    );
    idle(&mut m, 2);
    assert!(!source(&m, 1, SOURCE));
    idle(&mut m, 3);
    assert!(source(&m, 1, SOURCE));
    let mut out = Vec::new();
    assert!(uart(&m, id).wake_sources(&mut out));
    assert_eq!(out, []);
    w(&mut m, URB, 0x42);
    assert!(uart(&m, id).wake_sources(&mut out));
    assert_eq!(
        out,
        [(1, SOURCE)],
        "a TXRDY edge to come with UIMR TXRDY set"
    );
    assert!(!source(&m, 1, SOURCE), "the second waits for the first");
    assert_eq!(uart(&m, id).next_event(), Some(103));
    idle(&mut m, 103);
    assert!(source(&m, 1, SOURCE));
    w(&mut m, UISR, 0x00);
    assert!(!source(&m, 1, SOURCE));
    assert_eq!(uart(&m, id).text(), "AB");
    assert_eq!(uart(&m, id).sent(), [(3, 0x41), (103, 0x42)]);
}

#[test]
fn rxrdy_interrupt_and_uisr() {
    let (mut m, id) = board(false);
    w(&mut m, UCR, 0x05);
    w(&mut m, UISR, 0x02); // UIMR: RXRDY
    assert_eq!(r(&mut m, UISR), 0x01, "UISR shows TXRDY unmasked");
    assert!(!source(&m, 1, SOURCE));
    receive(&mut m, id, &[(0, Some(7))]);
    let mut out = Vec::new();
    assert!(uart(&m, id).wake_sources(&mut out));
    assert_eq!(out, [(1, SOURCE)], "a pending byte with UIMR RXRDY set");
    idle(&mut m, 100);
    assert_eq!(r(&mut m, UISR), 0x03);
    assert!(source(&m, 1, SOURCE));
    assert_eq!(r(&mut m, URB), 7);
    assert!(!source(&m, 1, SOURCE));
}

#[test]
fn byte_held_in_urb_is_taken_when_serq_is_written() {
    // The request is a level (RXRDY), not an edge at arrival: enabling the channel with a byte already held
    // serves it, and reception carries on afterwards.
    let (mut m, id) = board(true);
    w(&mut m, UCR, 0x01);
    rx_tcd(&mut m);
    receive(&mut m, id, &[(0, Some(0x11))]);
    idle(&mut m, 100);
    serq(&mut m);
    receive(&mut m, id, &[(200, Some(0x22)), (300, Some(0x33))]);
    idle(&mut m, 400);
    assert_eq!(m.mem.peek(RING, 3).unwrap(), [0x11, 0x22, 0x33]);
    assert_eq!(uart(&m, id).lost(), 0);
    assert_eq!(r(&mut m, USR) & (RXRDY | OE), 0);
}

#[test]
fn scheduled_framing_error_receives_a_garbage_byte_with_fe() {
    let (mut m, id) = board(false);
    w(&mut m, UCR, 0x01);
    receive(&mut m, id, &[(0, None)]);
    idle(&mut m, 100);
    assert_eq!(r(&mut m, USR) & (FE | RXRDY), FE | RXRDY);
    assert_eq!(uart(&m, id).lost(), 0);
    assert_eq!(r(&mut m, URB), 0xFF);
}

#[test]
fn framing_error_belongs_to_its_character() {
    // RM 41.4.2.3: in character mode (UMR1[ERR] = 0) FE is the status of the character at the top of the FIFO
    // and is popped with it; in block mode it holds until RESET ERROR STATUS.
    let (mut m, id) = board(false);
    w(&mut m, UCR, 0x01);
    receive(&mut m, id, &[(0, None), (200, Some(1))]);
    idle(&mut m, 100);
    assert_eq!(r(&mut m, URB), 0xFF);
    assert_eq!(
        r(&mut m, USR) & FE,
        0,
        "character mode: popped with its character"
    );
    idle(&mut m, 300);
    assert_eq!(r(&mut m, USR) & (FE | RXRDY), RXRDY);
    assert_eq!(r(&mut m, URB), 1);
    // Block mode: the OR of every character's status since RESET ERROR STATUS.
    w(&mut m, UMR, 0x20); // UMR1: ERR
    receive(&mut m, id, &[(400, None), (600, Some(2))]);
    idle(&mut m, 500);
    assert_eq!(r(&mut m, URB), 0xFF);
    idle(&mut m, 700);
    assert_eq!(r(&mut m, URB), 2);
    assert_ne!(r(&mut m, USR) & FE, 0, "block mode: kept");
    w(&mut m, UCR, 0x40);
    assert_eq!(r(&mut m, USR) & FE, 0);
    // RESET ERROR STATUS clears the FE of the character held, too (RM Table 41-8).
    w(&mut m, UMR, 0); // pointer on UMR2 now: UMR2 = 0
    w(&mut m, UCR, 0x10);
    w(&mut m, UMR, 0); // UMR1: character mode
    receive(&mut m, id, &[(800, None)]);
    idle(&mut m, 900);
    assert_ne!(r(&mut m, USR) & FE, 0);
    w(&mut m, UCR, 0x40);
    assert_eq!(r(&mut m, USR) & (FE | RXRDY), RXRDY);
}

#[test]
fn framing_errors_of_lost_characters_are_lost() {
    // RM 41.3.5 RECEIVER DISABLE: a character arriving with the receiver off is lost, so is its FE; RM 41.3.3
    // OE: an overrunning character's framing error is lost with it.
    let (mut m, id) = board(false);
    receive(&mut m, id, &[(0, None)]);
    idle(&mut m, 100);
    assert_eq!(r(&mut m, USR) & FE, 0);
    assert_eq!(uart(&m, id).lost(), 1);
    w(&mut m, UCR, 0x01);
    receive(&mut m, id, &[(100, Some(3)), (200, None)]);
    idle(&mut m, 300);
    assert_eq!(r(&mut m, USR) & (FE | OE | RXRDY), OE | RXRDY);
    assert_eq!(r(&mut m, URB), 3);
    assert_eq!(uart(&m, id).lost(), 2);
}

#[test]
fn transmitter_is_off_until_enabled_and_after_reset() {
    let (mut m, id) = board(false);
    assert_eq!(r(&mut m, USR) & (TXRDY | TXEMP), 0, "off at reset");
    w(&mut m, URB, 0x41); // a byte written while off goes nowhere
    assert!(uart(&m, id).sent().is_empty());
    w(&mut m, UCR, 0x04); // transmitter enable
    assert_eq!(r(&mut m, USR) & (TXRDY | TXEMP), TXRDY | TXEMP);
    w(&mut m, UCR, 0x30); // reset transmitter: off again
    assert_eq!(r(&mut m, USR) & (TXRDY | TXEMP), 0);
    w(&mut m, UCR, 0x04);
    w(&mut m, UCR, 0x08); // transmitter disable
    assert_eq!(r(&mut m, USR) & (TXRDY | TXEMP), 0);
}

#[test]
fn transmitter_has_a_holding_and_a_shift_register() {
    // Measured on an MCF54415 (UART9 at 31 250 baud): idle USR 0x0C. Back-to-back USR reads after a UTB write
    // from idle read 0x00 until the character moves to the shift register, then 0x04 (TXRDY back, TXEMP clear);
    // TXEMP sets one character later. TXRDY was first seen on the 3rd read in one run and the 10th in another.
    // guess: the shortest measured hand-over, the 3rd instruction after the write. A second byte written while
    // the first shifts keeps USR at 0x00 until the first has gone.
    let src = "move.l #0xEC074000,%a1\nmove.l #0x40300000,%a2\nmove.b #0x04,8(%a1)\nmoveq #0x41,%d0\n\
               move.b 4(%a1),(%a2)+\nmove.b %d0,12(%a1)\n\
               .rept 6\nmove.b 4(%a1),(%a2)+\n.endr\nend: nop";
    let (mut m, id, l) = snippet(src, &["end"], false);
    let r0 = m.run(common::until(l[0]));
    assert_eq!(r0.kind, Stop::UntilPc, "{r0:?}");
    assert_eq!(
        m.mem.peek(0x4030_0000, 7).unwrap(),
        [0x0C, 0x00, 0x00, 0x04, 0x04, 0x04, 0x04]
    );
    assert_eq!(uart(&m, id).text(), "A");

    // The same from the host, with the timing of TXEMP and of a second byte.
    let (mut m, id) = board(false);
    w(&mut m, UCR, 0x04);
    w(&mut m, URB, 0x41);
    assert_eq!(r(&mut m, USR), 0x00, "holding register loaded");
    idle(&mut m, 2);
    assert_eq!(r(&mut m, USR), 0x00);
    idle(&mut m, 3);
    assert_eq!(
        r(&mut m, USR),
        0x04,
        "in the shift register: TXRDY back, TXEMP clear"
    );
    idle(&mut m, 102);
    assert_eq!(r(&mut m, USR), 0x04);
    idle(&mut m, 103);
    assert_eq!(
        r(&mut m, USR),
        0x0C,
        "TXEMP one character after it began shifting"
    );
    // The second waits in the holding register until the first has gone.
    idle(&mut m, 200);
    w(&mut m, URB, 0x42);
    idle(&mut m, 203);
    w(&mut m, URB, 0x43);
    assert_eq!(r(&mut m, USR), 0x00);
    idle(&mut m, 302);
    assert_eq!(r(&mut m, USR), 0x00, "still holding while the first shifts");
    // RM 41.3.7: the holding register takes a character only when TXRDY is set.
    w(&mut m, URB, 0x44);
    assert_eq!(uart(&m, id).dropped(), 1);
    idle(&mut m, 303);
    assert_eq!(r(&mut m, USR), 0x04);
    idle(&mut m, 403);
    assert_eq!(r(&mut m, USR), 0x0C);
    // Each character is stamped with the tick it began shifting out.
    assert_eq!(uart(&m, id).sent(), [(3, 0x41), (203, 0x42), (303, 0x43)]);
}

#[test]
fn tx_idle_at_is_when_both_registers_are_empty() {
    let tx_idle_at = |m: &Machine, id| uart(m, id).tx_idle_at();
    let (mut m, id) = board(false);
    // idle from reset: at or before the clock
    assert!(tx_idle_at(&m, id) <= m.now());
    w(&mut m, UCR, 0x04);
    idle(&mut m, 10);
    assert!(tx_idle_at(&m, id) <= m.now());
    // one character in the holding register: it moves at 13 and ends a character later
    w(&mut m, URB, 0x41);
    assert_eq!(tx_idle_at(&m, id), 113);
    // in the shift register, a second in the holding register behind it
    idle(&mut m, 20);
    w(&mut m, URB, 0x42);
    assert_eq!(tx_idle_at(&m, id), 213);
    // the host drains it without running instructions: both characters go out, then TXEMP
    let end = tx_idle_at(&m, id);
    idle(&mut m, end);
    assert_eq!(uart(&m, id).sent(), [(13, 0x41), (113, 0x42)]);
    assert_eq!(r(&mut m, USR), TXRDY | TXEMP);
    assert_eq!(
        tx_idle_at(&m, id),
        m.now(),
        "idle again, after the last character"
    );
}

#[test]
fn back_to_back_writes_drop_the_second() {
    // RM 41.3.7: the second write finds TXRDY clear (the first is still in the holding register).
    let (mut m, id, l) = snippet(
        "move.l #0xEC074000,%a1\nmove.b #0x04,8(%a1)\nmove.b #0x41,12(%a1)\nmove.b #0x42,12(%a1)\nend: nop",
        &["end"],
        false,
    );
    let r0 = m.run(common::until(l[0]));
    assert_eq!(r0.kind, Stop::UntilPc, "{r0:?}");
    let t = m.now() + 1000;
    idle(&mut m, t);
    assert_eq!(uart(&m, id).text(), "A");
    assert_eq!(uart(&m, id).dropped(), 1);
}

#[test]
fn disabling_lets_the_shifting_character_finish() {
    // RM Table 41-8 TRANSMITTER DISABLE: the character being sent completes; TXRDY and TXEMP clear at once.
    // guess: a character still in the holding register is not sent (RM Table 41-6 TXRDY: characters loaded with
    // the transmitter disabled are not sent).
    let (mut m, id) = board(false);
    w(&mut m, UCR, 0x04);
    w(&mut m, URB, 0x41);
    idle(&mut m, 3);
    w(&mut m, URB, 0x42);
    w(&mut m, UCR, 0x08);
    assert_eq!(r(&mut m, USR) & (TXRDY | TXEMP), 0);
    idle(&mut m, 500);
    assert_eq!(uart(&m, id).sent(), [(3, 0x41)]);
    assert_eq!(uart(&m, id).dropped(), 1);
    w(&mut m, UCR, 0x04);
    assert_eq!(r(&mut m, USR), 0x0C);
}

#[test]
fn unmodelled_modes_are_refused() {
    for (writes, what) in [
        (&[0x40][..], "RxIRQ"),
        (&[0x18][..], "multidrop"),
        (&[0x00, 0x40][..], "automatic echo"),
        (&[0x00, 0x80][..], "local loopback"),
        (&[0x00, 0xC0][..], "remote loopback"),
    ] {
        let (mut m, _) = board(false);
        let (last, first) = writes.split_last().unwrap();
        for &v in first {
            w(&mut m, UMR, v);
        }
        assert_eq!(
            m.access(UMR, Size::Byte, Some(*last)),
            Err(Fault::Device(Stop::Unmodelled)),
            "{what}"
        );
        let d = m.take_refusal().expect("a refusal").detail;
        assert!(d.contains("UART") && d.contains(what), "{what}: {d}");
        // the refused value is not stored
        w(&mut m, UCR, 0x10);
        assert_eq!(r(&mut m, UMR), 0, "{what}: UMR1");
        assert_eq!(r(&mut m, UMR), 0, "{what}: UMR2");
    }
}

#[test]
fn on_byte_sees_each_byte_with_its_tick() {
    use std::cell::RefCell;
    use std::rc::Rc;
    let (mut m, id) = board(false);
    let seen = Rc::new(RefCell::new(Vec::new()));
    let s = seen.clone();
    m.device_mut::<Uart>(id)
        .on_byte(Box::new(move |t, b| s.borrow_mut().push((t, b))));
    w(&mut m, UCR, 0x04);
    w(&mut m, URB, 0x31);
    idle(&mut m, 250);
    w(&mut m, URB, 0x32);
    idle(&mut m, 253);
    assert_eq!(*seen.borrow(), [(3, 0x31), (253, 0x32)]);
    assert_eq!(uart(&m, id).text(), "12");
}

/// A machine running `src` with the board's devices; returns it, the UART and the labels.
fn snippet(src: &str, labels: &[&str], dma: bool) -> (Machine, DeviceId, Vec<u32>) {
    let (m, l) = common::prepare(src, labels);
    let (m, id) = add(m, dma);
    (m, id, l)
}

/// INTC1 source `s` at level 3, unmasked.
fn unmask(m: &mut Machine, s: u8) {
    w(m, INTC1 + 0x40 + u32::from(s), 3);
    let imr = if s < 32 { INTC1 + 0x0C } else { INTC1 + 0x08 };
    w_sized(m, imr, Size::Long, !(1 << (s % 32)));
}

#[test]
fn stop_wakes_on_a_received_byte() {
    let src = "stop #0x2000\nend: nop\nhandler: move.b 0xEC07400C,%d0\nrte";
    let (mut m, id, l) = snippet(src, &["end", "handler"], false);
    unmask(&mut m, SOURCE);
    common::set_vector(&mut m, 128 + u32::from(SOURCE), l[1]);
    w(&mut m, UCR, 0x01);
    w(&mut m, UISR, 0x02);
    receive(&mut m, id, &[(1000, Some(0x5A))]);
    m.set_time_limit(Some(100_000));
    let r = m.run(common::until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[0] & 0xFF, 0x5A);
    assert!(m.now() >= 1100);
}

#[test]
fn stop_with_a_byte_that_cannot_interrupt_halts_at_once() {
    // UIMR is 0 and there is no DMA: the byte's arrival cannot wake the core, so the STOP halts without
    // fast-forwarding to it.
    let (mut m, id, l) = snippet("stop #0x2000\nend: nop", &["end"], false);
    unmask(&mut m, SOURCE);
    w(&mut m, UCR, 0x01);
    receive(&mut m, id, &[(1000, Some(0x5A))]);
    let r = m.run(common::until(l[0]));
    assert_eq!(r.kind, Stop::HaltedForever, "{r:?}");
    assert!(m.now() < 1000, "{}", m.now());
}

#[test]
fn stop_wakes_when_a_byte_finishes_sending() {
    // Two characters, the second written once TXRDY is back: it holds TXRDY clear until the first has gone.
    let src = "move.l #0xEC074000,%a0\nmove.b #0x04,8(%a0)\nmove.b #0x41,12(%a0)\n\
               1: move.b 4(%a0),%d1\nbtst #2,%d1\nbeq.s 1b\nmove.b #0x42,12(%a0)\nmove.b #0x01,20(%a0)\n\
               stop #0x2000\nend: nop\nhandler: clr.b 20(%a0)\nrte";
    let (mut m, id, l) = snippet(src, &["end", "handler"], false);
    unmask(&mut m, SOURCE);
    common::set_vector(&mut m, 128 + u32::from(SOURCE), l[1]);
    m.set_time_limit(Some(100_000));
    let r = m.run(common::until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert!(m.now() >= 100, "{}", m.now());
    assert_eq!(uart(&m, id).text(), "AB");
}

#[test]
fn stop_wakes_on_the_dma_interrupt_of_a_received_byte() {
    let src = "stop #0x2000\nend: nop\nhandler: move.l #0xFC04401C,%a0\nmove.b #36,(%a0)\nrte";
    let (mut m, id, l) = snippet(src, &["end", "handler"], true);
    unmask(&mut m, 28);
    common::set_vector(&mut m, 128 + 28, l[1]);
    w(&mut m, UCR, 0x01);
    rx_tcd(&mut m);
    serq(&mut m);
    receive(&mut m, id, &[(1000, Some(0x66))]);
    m.set_time_limit(Some(100_000));
    let r = m.run(common::until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.mem.peek(RING, 1).unwrap(), [0x66]);
}

#[test]
fn take_sent_drains_and_record_off_logs_nothing() {
    use std::cell::RefCell;
    use std::rc::Rc;
    let (mut m, id) = board(false);
    let seen = Rc::new(RefCell::new(Vec::new()));
    let s = seen.clone();
    m.device_mut::<Uart>(id)
        .on_byte(Box::new(move |t, b| s.borrow_mut().push((t, b))));
    w(&mut m, UCR, 0x04);
    w(&mut m, URB, 0x31);
    idle(&mut m, 250);
    assert_eq!(m.device_mut::<Uart>(id).take_sent(), [(3, 0x31)]);
    assert!(uart(&m, id).sent().is_empty());
    m.device_mut::<Uart>(id).record(false);
    w(&mut m, URB, 0x32);
    idle(&mut m, 500);
    assert!(uart(&m, id).sent().is_empty(), "not logged");
    assert_eq!(seen.borrow().len(), 2, "but the hook saw it");
}

/// The TCD word or longword at `off` of channel `ch`.
fn tcd(m: &mut Machine, ch: u8, off: u32, size: Size) -> u32 {
    let a = EDMA + 0x1000 + 32 * u32::from(ch) + off;
    m.access(a, size, None).unwrap()
}

/// Channel 37 in the shape of a transmit ring: SADDR in a 4 KiB SMOD ring at [`TX_RING`] (8-bit, SOFF 1),
/// DADDR UTB (DOFF 0), NBYTES 1, CITER = BITER = `count`, CSR `csr`. SERQ not written.
fn tx_tcd(m: &mut Machine, start: u32, count: u32, csr: u32) {
    let t = EDMA + 0x1000 + 32 * u32::from(TX_CH);
    for (off, size, v) in [
        (0x00, Size::Long, TX_RING + start), // SADDR
        (0x04, Size::Word, 12 << 11),        // ATTR: SMOD 12, 8-bit both sides
        (0x06, Size::Word, 1),               // SOFF
        (0x08, Size::Long, 1),               // NBYTES
        (0x10, Size::Long, URB),             // DADDR: UTB
        (0x16, Size::Word, 0),               // DOFF
        (0x14, Size::Word, count),           // CITER
        (0x1C, Size::Word, count),           // BITER
        (0x1E, Size::Word, csr),             // CSR
    ] {
        w_sized(m, t + off, size, v);
    }
}

#[test]
fn a_gated_uart_keeps_its_request_and_the_backstop_names_the_gate_guess() {
    // guess: a clock-gated UART keeps its DMA request line as it left it and drops the UTB writes that would lower
    // it (not measured), so a served channel never sees the request drop; the stop's detail names the gate
    let (mut m, id) = board(true);
    m.set_clock_gate(id, Some(0));
    tx_tcd(&mut m, 0, 4, 0); // no D_REQ: the major loop restarts for ever
    w(&mut m, UCR, 0x04); // transmitter enable: TXRDY requests
    m.set_clock_gates(1);
    assert_ne!(
        m.dma_lines().asserted() & 1 << TX_CH,
        0,
        "the line stays up"
    );
    let refused = m.access(EDMA + 0x18, Size::Byte, Some(u32::from(TX_CH))); // SERQ
    assert!(refused.is_err(), "{refused:?}");
    let r = m.take_refusal().unwrap();
    assert_eq!(r.kind, Stop::Unmodelled);
    assert!(r.detail.contains("clock gate is closed"), "{}", r.detail);
}

#[test]
fn uart_tx_dma_moves_a_ring_into_utb_one_byte_per_txrdy() {
    // A transmit ring as a UART's TCD drives it: 8-bit reads from a 4 KiB SMOD ring, one byte per minor loop into
    // UTB, D_REQ. The transmit request is UISR[TXRDY] (RM 41.5.1.2), so each byte goes in when the holding
    // register frees: the first at once, each later one when the one before moves to the shift register. Every
    // byte is sent, once, in ring order across the wrap, one per character time.
    const COUNT: u32 = 4200;
    const START: u32 = 0xF00;
    let (mut m, id) = board(true);
    let ring: Vec<u8> = (0..4096u32).map(|i| (i * 7 + i / 256) as u8).collect();
    m.mem.load(TX_RING, &ring).unwrap();
    tx_tcd(&mut m, START, COUNT, 0x08); // CSR: D_REQ
    w(&mut m, UCR, 0x04); // transmitter enable
    assert_ne!(m.dma_lines().asserted() & 1 << TX_CH, 0, "TXRDY requests");
    w(&mut m, EDMA + 0x18, u32::from(TX_CH)); // SERQ
    let end = 3 + 100 * u64::from(COUNT);
    idle(&mut m, end);
    assert_eq!(uart(&m, id).tx_idle_at(), end);
    let sent = uart(&m, id).sent();
    assert_eq!(sent.len(), COUNT as usize);
    for (i, &(t, b)) in sent.iter().enumerate() {
        assert_eq!(b, ring[(START as usize + i) % 4096], "byte {i}");
        assert_eq!(
            t,
            3 + 100 * i as u64,
            "byte {i} leaves one character time after the one before"
        );
    }
    assert_eq!(
        uart(&m, id).dropped(),
        0,
        "none written over a full holding register"
    );
    assert_eq!(tcd(&mut m, TX_CH, 0x1E, Size::Word) & 0x80, 0x80, "DONE");
    assert_eq!(
        tcd(&mut m, TX_CH, 0x14, Size::Word),
        COUNT,
        "CITER reloaded"
    );
    assert_eq!(
        tcd(&mut m, TX_CH, 0x00, Size::Long),
        TX_RING + (START + COUNT) % 4096,
        "SADDR wrapped in the ring"
    );
    assert_ne!(
        m.dma_lines().asserted() & 1 << TX_CH,
        0,
        "TXRDY still requests; D_REQ cleared ERQ"
    );
}

#[test]
fn uart_tx_request_drops_inside_the_utb_write() {
    // RM 41.5.1.2, RM Table 41-15: the transmit request is UISR[TXRDY]. UIMR does not mask it (RM 41.3.10: UISR is
    // true status), the transmitter must be on (RM Table 41-6), and the UTB write clears it (RM 41.3.7) before the
    // write returns. guess: the chip's request-to-acknowledge timing is not measured; the request falls inside
    // the write so that a served channel puts one character into UTB per TXRDY.
    let (mut m, id) = board(true);
    let line = |m: &Machine| m.dma_lines().asserted() & 1 << TX_CH != 0;
    assert!(!line(&m), "transmitter off");
    w(&mut m, UCR, 0x04);
    assert_eq!(r(&mut m, UISR), 0x01);
    assert!(line(&m), "TXRDY with UIMR 0");
    w(&mut m, URB, 0x41);
    assert!(!line(&m), "the write lowered it");
    idle(&mut m, 2);
    assert!(!line(&m));
    idle(&mut m, 3);
    assert!(line(&m), "back with TXRDY, the hand-over");
    w(&mut m, UCR, 0x08); // transmitter disable
    assert!(!line(&m));

    // The eDMA serving it: one minor loop per TXRDY, never a second write into the full holding register.
    let (mut m, id2) = board(true);
    m.mem.load(TX_RING, b"abcdefghij").unwrap();
    tx_tcd(&mut m, 0, 10, 0);
    w(&mut m, UCR, 0x04);
    w(&mut m, EDMA + 0x18, u32::from(TX_CH)); // SERQ
    assert_eq!(
        tcd(&mut m, TX_CH, 0x14, Size::Word),
        9,
        "one minor loop at tick 0"
    );
    assert!(!line(&m));
    idle(&mut m, 3);
    assert_eq!(
        tcd(&mut m, TX_CH, 0x14, Size::Word),
        8,
        "the next at the hand-over"
    );
    idle(&mut m, 102);
    assert_eq!(
        tcd(&mut m, TX_CH, 0x14, Size::Word),
        8,
        "waits for the shift register"
    );
    idle(&mut m, 103);
    assert_eq!(tcd(&mut m, TX_CH, 0x14, Size::Word), 7);
    assert_eq!(uart(&m, id2).dropped(), 0);
    assert_eq!(uart(&m, id2).text(), "ab");
    assert_eq!(uart(&m, id).dropped(), 0);
}
