//! Serial peers: what sits at the far end of a UART's lines (UART9 on INTC1 source 53), hearing each character
//! sent, replying into the receiver and halting the run.

mod common;

use mcf5441x_emu::soc::{Edma, Intc, PeerCx, SerialPeer, Uart, UartConfig};
use mcf5441x_emu::{Device, DeviceId, Machine, Size, Stop};

const UART9: u32 = 0xEC07_4000;
const INTC1: u32 = 0xFC04_C000;
const EDMA: u32 = 0xFC04_4000;
const RAM: u32 = 0x4010_0000;

const USR: u32 = UART9 + 0x04;
const UCR: u32 = UART9 + 0x08;
const URB: u32 = UART9 + 0x0C;
const UISR: u32 = UART9 + 0x14;

const RXRDY: u32 = 0x01;

/// INTC1 source of UART9 (vector 181).
const SOURCE: u8 = 53;

/// Add INTC1, the eDMA and UART9 (100 ticks per byte, no receive DMA) to `m`.
fn add(mut m: Machine) -> (Machine, DeviceId) {
    let ([_, intc1, _], _) = Intc::chip();
    m.add_interrupt_controller(INTC1, 0x4000, Box::new(intc1));
    m.add_bus_master(EDMA, 0x2000, Box::new(Edma::new()));
    let id = m.add_device(
        UART9,
        0x4000,
        Box::new(Uart::new(UartConfig {
            controller: 1,
            source: SOURCE,
            rx_dma: None,
            tx_dma: None,
            ticks_per_byte: 100,
        })),
    );
    (m, id)
}

fn board() -> (Machine, DeviceId) {
    let mut m = Machine::new();
    m.mem.map_ram(RAM, 0x0100_0000);
    add(m)
}

/// A machine running `src` with the board's devices; returns it, the UART and the labels.
fn snippet(src: &str, labels: &[&str]) -> (Machine, DeviceId, Vec<u32>) {
    let (m, l) = common::prepare(src, labels);
    let (m, id) = add(m);
    (m, id, l)
}

fn r(m: &mut Machine, addr: u32) -> u32 {
    m.access(addr, Size::Byte, None)
        .unwrap_or_else(|f| panic!("read {addr:#x}: {f:?} {:?}", m.take_refusal()))
}

fn w(m: &mut Machine, addr: u32, value: u32) {
    m.access(addr, Size::Byte, Some(value))
        .unwrap_or_else(|f| panic!("write {addr:#x}: {f:?} {:?}", m.take_refusal()));
}

fn idle(m: &mut Machine, tick: u64) {
    assert_eq!(m.idle_until(tick), None);
    assert_eq!(m.now(), tick);
}

/// INTC1 source `s` at level 3, unmasked.
fn unmask(m: &mut Machine, s: u8) {
    w(m, INTC1 + 0x40 + u32::from(s), 3);
    let imr = if s < 32 { INTC1 + 0x0C } else { INTC1 + 0x08 };
    m.access(imr, Size::Long, Some(!(1 << (s % 32)))).unwrap();
}

/// Records every character it hears with the tick it heard it.
#[derive(Default)]
struct Recorder {
    heard: Vec<(u64, u8)>,
}

impl SerialPeer for Recorder {
    fn transmitted(&mut self, byte: u8, cx: &mut PeerCx) {
        self.heard.push((cx.now, byte));
    }
}

/// Sends each character back, starting as it hears it.
struct Echo;

impl SerialPeer for Echo {
    fn transmitted(&mut self, byte: u8, cx: &mut PeerCx) {
        cx.send(cx.now, Some(byte));
    }
}

#[test]
fn peer_hears_each_transmitted_character_at_its_shift_start() {
    let (mut m, id) = board();
    m.device_mut::<Uart>(id)
        .attach(Box::new(Recorder::default()));
    w(&mut m, UCR, 0x04);
    w(&mut m, URB, 0x31);
    idle(&mut m, 250);
    w(&mut m, URB, 0x32);
    idle(&mut m, 253);
    let u = m.device::<Uart>(id);
    assert_eq!(u.peer::<Recorder>().heard, [(3, 0x31), (253, 0x32)]);
    assert_eq!(u.sent(), [(3, 0x31), (253, 0x32)]);
}

#[test]
fn peer_replies_arrive_one_character_time_after_their_start() {
    let (mut m, id) = board();
    m.device_mut::<Uart>(id).attach(Box::new(Echo));
    w(&mut m, UCR, 0x05); // receiver and transmitter on
    w(&mut m, URB, 0x41);
    idle(&mut m, 3); // shifting out: the echo starts now
    assert_eq!(m.device::<Uart>(id).next_event(), Some(103));
    idle(&mut m, 102);
    assert_eq!(r(&mut m, USR) & RXRDY, 0);
    idle(&mut m, 103);
    assert_eq!(r(&mut m, USR) & RXRDY, RXRDY);
    assert_eq!(r(&mut m, URB), 0x41);
}

/// Sends `byte` with its start bit at `at`.
struct Later {
    at: Option<u64>,
    byte: u8,
}

impl SerialPeer for Later {
    fn transmitted(&mut self, _byte: u8, _cx: &mut PeerCx) {}

    fn next_event(&self) -> Option<u64> {
        self.at
    }

    fn advance(&mut self, cx: &mut PeerCx) {
        if let Some(at) = self.at.take() {
            cx.send(at, Some(self.byte));
        }
    }
}

#[test]
fn peer_events_wake_a_stopped_core() {
    let src = "stop #0x2000\nend: nop\nhandler: move.b 0xEC07400C,%d0\nrte";
    let (mut m, id, l) = snippet(src, &["end", "handler"]);
    unmask(&mut m, SOURCE);
    common::set_vector(&mut m, 128 + u32::from(SOURCE), l[1]);
    w(&mut m, UCR, 0x01);
    w(&mut m, UISR, 0x02);
    m.device_mut::<Uart>(id).attach(Box::new(Later {
        at: Some(1000),
        byte: 0x5A,
    }));
    m.set_time_limit(Some(100_000));
    let r = m.run(common::until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[0] & 0xFF, 0x5A);
    assert!(m.now() >= 1100, "{}", m.now());

    // With RXRDY masked the peer's character cannot wake the core: the STOP halts at once.
    let (mut m, id, l) = snippet("stop #0x2000\nend: nop", &["end"]);
    unmask(&mut m, SOURCE);
    w(&mut m, UCR, 0x01);
    m.device_mut::<Uart>(id).attach(Box::new(Later {
        at: Some(1000),
        byte: 0x5A,
    }));
    let r = m.run(common::until(l[0]));
    assert_eq!(r.kind, Stop::HaltedForever, "{r:?}");
    assert!(m.now() < 1000, "{}", m.now());

    // A character still in the holding register will reach an echoing peer, whose reply wakes the core.
    let src = "move.l #0xEC074000,%a0\nmove.b #0x05,8(%a0)\nmove.b #0x02,20(%a0)\nmove.b #0x41,12(%a0)\n\
               stop #0x2000\nend: nop\nhandler: move.b 12(%a0),%d0\nrte";
    let (mut m, id, l) = snippet(src, &["end", "handler"]);
    unmask(&mut m, SOURCE);
    common::set_vector(&mut m, 128 + u32::from(SOURCE), l[1]);
    m.device_mut::<Uart>(id).attach(Box::new(Echo));
    m.set_time_limit(Some(100_000));
    let r = m.run(common::until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[0] & 0xFF, 0x41);
}

/// Collects a line and ends the run with it.
#[derive(Default)]
struct Console {
    line: Vec<u8>,
}

impl SerialPeer for Console {
    fn transmitted(&mut self, byte: u8, cx: &mut PeerCx) {
        if byte == b'\n' {
            let line = String::from_utf8_lossy(&self.line).into_owned();
            self.line.clear();
            cx.halt(Stop::Halt("console line"), line);
        } else {
            self.line.push(byte);
        }
    }
}

#[test]
fn peer_can_halt_the_run_after_a_line() {
    // Send "ok\n" then more, polling TXRDY before each character; the console stops the run once the newline
    // starts shifting out.
    let src = "move.l #0xEC074000,%a0\nlea text,%a1\nmove.b #0x04,8(%a0)\n\
               next: move.b (%a1)+,%d0\nbeq.s end\n\
               1: move.b 4(%a0),%d1\nbtst #2,%d1\nbeq.s 1b\nmove.b %d0,12(%a0)\nbra.s next\n\
               end: nop\n\
               text: .asciz \"ok\\nmore\"";
    let (mut m, id, l) = snippet(src, &["end"]);
    m.device_mut::<Uart>(id)
        .attach(Box::new(Console::default()));
    m.set_time_limit(Some(100_000));
    let r = m.run(common::until(l[0]));
    assert_eq!(r.kind, Stop::Halt("console line"), "{r:?}");
    assert_eq!(r.detail, "ok");
    let u = m.device::<Uart>(id);
    assert_eq!(u.text(), "ok\n");
    // At the instruction boundary where the newline began shifting out.
    assert_eq!(m.now(), u.sent()[2].0);
    assert_eq!(r.pc, m.cpu.pc);
    // A later run continues from there.
    let r = m.run(common::until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    let t = m.now() + 1000;
    idle(&mut m, t);
    assert_eq!(m.device::<Uart>(id).text(), "ok\nmore");
}

#[test]
fn attach_replaces_and_peer_downcasts() {
    let (mut m, id) = board();
    let u = m.device_mut::<Uart>(id);
    u.attach(Box::new(Echo));
    u.attach(Box::new(Recorder::default()));
    u.peer_mut::<Recorder>().heard.push((0, 0));
    w(&mut m, UCR, 0x05);
    w(&mut m, URB, 0x41);
    idle(&mut m, 500);
    let u = m.device::<Uart>(id);
    assert_eq!(u.peer::<Recorder>().heard, [(0, 0), (3, 0x41)]);
    assert_eq!(u.lost(), 0);
    assert_eq!(r(&mut m, USR) & RXRDY, 0, "the echo was replaced");
    let u = m.device::<Uart>(id);
    let wrong = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        u.peer::<Echo>();
    }));
    assert!(wrong.is_err(), "a peer of another type");
    let none = std::panic::catch_unwind(|| {
        Uart::new(UartConfig {
            controller: 1,
            source: SOURCE,
            rx_dma: None,
            tx_dma: None,
            ticks_per_byte: 100,
        })
        .peer::<Recorder>();
    });
    assert!(none.is_err(), "no peer");
}
