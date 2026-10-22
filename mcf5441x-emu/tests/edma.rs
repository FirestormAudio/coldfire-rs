//! The eDMA (RM ch. 19), driven through its registers, with peripheral requests from a test device that drives
//! DMA request lines.

mod common;

use std::collections::VecDeque;

use mcf5441x_emu::soc::{Edma, Intc, Unmodelled};
use mcf5441x_emu::{Device, DeviceCx, DeviceId, Fault, Limit, Machine, Size, Stop, StopReason};

const EDMA: u32 = 0xFC04_4000;
const RAM: u32 = 0x4000_0000;
/// Where [`pending_halt`] runs its one instruction.
const CODE: u32 = 0x4010_F000;

const ES: u32 = EDMA + 0x04;
const ERRH: u32 = EDMA + 0x28;
const ERRL: u32 = EDMA + 0x2C;
const SERQ: u32 = EDMA + 0x18;
const CERQ: u32 = EDMA + 0x19;
const SEEI: u32 = EDMA + 0x1A;
const CINT: u32 = EDMA + 0x1C;
const CERR: u32 = EDMA + 0x1D;
const SSRT: u32 = EDMA + 0x1E;
const CDNE: u32 = EDMA + 0x1F;

/// TCD fields: offset in the 32-byte TCD and size (RM 19.4.17).
type Field = (u32, Size);
const SADDR: Field = (0x00, Size::Long);
const ATTR: Field = (0x04, Size::Word);
const SOFF: Field = (0x06, Size::Word);
const NBYTES: Field = (0x08, Size::Long);
const SLAST: Field = (0x0C, Size::Long);
const DADDR: Field = (0x10, Size::Long);
const CITER: Field = (0x14, Size::Word);
const DOFF: Field = (0x16, Size::Word);
const DLAST: Field = (0x18, Size::Long);
const BITER: Field = (0x1C, Size::Word);
const CSR: Field = (0x1E, Size::Word);

const START: u32 = 0x01;
const INTMAJOR: u32 = 0x02;
const INTHALF: u32 = 0x04;
const DREQ: u32 = 0x08;
const ESG: u32 = 0x10;
const MAJORELINK: u32 = 0x20;
const DONE: u32 = 0x80;

/// Peripherals that request DMA. A port has a 2 KiB register window at +0 (big-endian, any size), a receive FIFO
/// whose +0x800 pops one entry (0 when empty) and a transmit register at +0x804 that records what is written. Its
/// DMA line is asserted while the FIFO holds anything, while it still wants `budget` accesses to its window or
/// transmit register, or while `stuck`.
const PORT: u32 = 0xFC0E_0000;
const PORT2: u32 = 0xFC0E_1000;
const RX: u32 = 0x800;
const TX: u32 = 0x804;

struct Port {
    line: u8,
    mem: Vec<u8>,
    fifo: VecDeque<u32>,
    out: Vec<u32>,
    budget: u32,
    stuck: bool,
    /// A tick at which the FIFO receives [`ARRIVAL`].
    event: Option<u64>,
}

const ARRIVAL: u32 = 0x55;

impl Port {
    fn new(line: u8) -> Port {
        Port {
            line,
            mem: vec![0; 0x800],
            fifo: VecDeque::new(),
            out: Vec::new(),
            budget: 0,
            stuck: false,
            event: None,
        }
    }

    fn drive(&self, cx: &mut DeviceCx) {
        let on = self.stuck || self.budget > 0 || !self.fifo.is_empty();
        cx.dma.set(self.line, on);
    }

    fn spend(&mut self) {
        self.budget = self.budget.saturating_sub(1);
    }
}

impl Device for Port {
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32 {
        let v = if offset < RX {
            self.spend();
            let o = offset as usize;
            self.mem[o..o + size.bytes() as usize]
                .iter()
                .fold(0, |v, &b| v << 8 | u32::from(b))
        } else if offset == RX {
            self.fifo.pop_front().unwrap_or(0)
        } else {
            0
        };
        self.drive(cx);
        v
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        if offset < RX {
            self.spend();
            let n = size.bytes() as usize;
            for i in 0..n {
                self.mem[offset as usize + i] = (value >> (8 * (n - 1 - i))) as u8;
            }
        } else if offset == TX {
            self.spend();
            self.out.push(value);
        }
        self.drive(cx);
    }

    fn next_event(&self) -> Option<u64> {
        self.event
    }

    fn advance(&mut self, _: u64, cx: &mut DeviceCx) {
        self.event = None;
        self.fifo.push_back(ARRIVAL);
        self.drive(cx);
    }

    /// Its events raise no interrupt of its own: only through the DMA it requests.
    fn wake_sources(&self, _: &mut Vec<(usize, u8)>) -> bool {
        true
    }
}

/// RAM at `0x40000000` (16 MiB), the eDMA at its base and a port at [`PORT`] driving `line`.
fn board(line: u8) -> (Machine, DeviceId, DeviceId) {
    let mut m = Machine::new();
    m.mem.map_ram(RAM, 0x0100_0000);
    let edma = m.add_bus_master(EDMA, 0x2000, Box::new(Edma::new()));
    let port = m.add_device(PORT, 0x1000, Box::new(Port::new(line)));
    (m, edma, port)
}

fn r(m: &mut Machine, addr: u32, size: Size) -> u32 {
    m.access(addr, size, None)
        .unwrap_or_else(|f| panic!("read {addr:#x}: {f:?} {:?}", m.take_refusal()))
}

fn w(m: &mut Machine, addr: u32, size: Size, value: u32) {
    m.access(addr, size, Some(value))
        .unwrap_or_else(|f| panic!("write {addr:#x}: {f:?} {:?}", m.take_refusal()));
}

/// The access is refused as unmodelled; returns the detail.
fn refused(m: &mut Machine, addr: u32, size: Size, write: Option<u32>) -> String {
    assert_eq!(
        m.access(addr, size, write),
        Err(Fault::Device(Stop::Unmodelled)),
        "{addr:#x} {size:?} {write:?}"
    );
    let r = m.take_refusal().expect("a refusal");
    assert_eq!(r.kind, Stop::Unmodelled);
    r.detail
}

fn tcd_addr(ch: u32, f: Field) -> u32 {
    EDMA + 0x1000 + 32 * ch + f.0
}

fn tcd(m: &mut Machine, ch: u32, fields: &[(Field, u32)]) {
    for &(f, v) in fields {
        w(m, tcd_addr(ch, f), f.1, v);
    }
}

fn get(m: &mut Machine, ch: u32, f: Field) -> u32 {
    r(m, tcd_addr(ch, f), f.1)
}

fn with_port(m: &mut Machine, port: DeviceId, f: impl FnOnce(&mut Port)) {
    m.with_device::<Port, _>(port, |p, cx| {
        f(p);
        p.drive(cx);
    });
}

/// The port's receive FIFO gets `v`: its request rises.
fn push(m: &mut Machine, port: DeviceId, v: u32) {
    with_port(m, port, |p| p.fifo.push_back(v));
}

/// The port requests until it has seen `n` more accesses.
fn grant(m: &mut Machine, port: DeviceId, n: u32) {
    with_port(m, port, |p| p.budget = n);
}

fn source(m: &Machine, controller: usize, source: u8) -> bool {
    m.mem.irq_lines().asserted(controller) & 1 << source != 0
}

/// The halt left pending by host-side calls: run one instruction, which the halt stops before.
fn pending_halt(m: &mut Machine) -> StopReason {
    m.mem.load(CODE, &[0x4E, 0x71]).unwrap();
    m.cpu.pc = CODE;
    m.run(Limit {
        max_instructions: Some(1),
        ..Limit::default()
    })
}

fn no_halt(m: &mut Machine) {
    let r = pending_halt(m);
    assert_eq!(r.kind, Stop::InstructionLimit, "{r:?}");
}

/// Channel 36 in the shape of a UART receive channel: one byte per request from the port's receive FIFO into a
/// 256-byte ring (DMOD 8), INTMAJOR on each; then SERQ.
fn rx_channel(m: &mut Machine, ring: u32) {
    tcd(
        m,
        36,
        &[
            (SADDR, PORT + RX),
            (SOFF, 0),
            (ATTR, 8 << 3),
            (NBYTES, 1),
            (SLAST, 0),
            (DADDR, ring),
            (DOFF, 1),
            (CITER, 1),
            (BITER, 1),
            (DLAST, 0),
            (CSR, INTMAJOR),
        ],
    );
    w(m, SERQ, Size::Byte, 36);
}

#[test]
fn one_byte_per_request_with_dmod_wrap_and_interrupt() {
    let (mut m, _, port) = board(36);
    let ring = 0x4010_1000;
    rx_channel(&mut m, ring);
    for i in 0..300u32 {
        push(&mut m, port, i & 0xFF);
    }
    assert_eq!(
        get(&mut m, 36, DADDR),
        ring + 300 % 256,
        "DADDR wrapped inside the ring"
    );
    // writes 256..259 (bytes 0..3) wrapped to the start
    assert_eq!(m.mem.peek(ring, 4).unwrap(), [0, 1, 2, 3]);
    assert_eq!(m.mem.peek(ring + 255, 1).unwrap(), [255]);
    assert_eq!(
        m.mem.peek(ring + 256, 1).unwrap(),
        [0],
        "never written past the ring"
    );
    assert_eq!(get(&mut m, 36, CITER), 1, "CITER reloaded from BITER");
    assert_ne!(get(&mut m, 36, CSR) & DONE, 0);
    assert!(source(&m, 1, 28), "channel 36 -> INTC1 source 28");
    w(&mut m, CINT, Size::Byte, 36);
    assert!(!source(&m, 1, 28));
    no_halt(&mut m);
}

#[test]
fn disabled_channel_does_nothing_and_dreq_disables() {
    let (mut m, _, port) = board(36);
    let ring = 0x4010_1000;
    rx_channel(&mut m, ring);
    w(&mut m, CERQ, Size::Byte, 36);
    push(&mut m, port, 0x41);
    assert_eq!(get(&mut m, 36, DADDR), ring, "disabled: nothing moved");
    assert_eq!(
        m.device::<Port>(port).fifo.len(),
        1,
        "the request stays asserted"
    );
    w(&mut m, SERQ, Size::Byte, 36);
    assert_eq!(
        get(&mut m, 36, DADDR),
        ring + 1,
        "enabled: the pending request is served"
    );
    tcd(&mut m, 36, &[(CSR, INTMAJOR | DREQ)]);
    push(&mut m, port, 0x42);
    assert_eq!(get(&mut m, 36, DADDR), ring + 2);
    push(&mut m, port, 0x43);
    assert_eq!(
        get(&mut m, 36, DADDR),
        ring + 2,
        "DREQ cleared the request enable"
    );
    assert_eq!(m.device::<Port>(port).fifo.len(), 1);
    assert_eq!(m.mem.peek(ring, 2).unwrap(), [0x41, 0x42]);
}

#[test]
fn major_loop_applies_slast_and_dlast() {
    let (mut m, _, port) = board(20);
    let (src, dst) = (PORT + 0x100, 0x4010_0200);
    with_port(&mut m, port, |p| {
        p.mem[0x100..0x104].copy_from_slice(b"abcd")
    });
    tcd(
        &mut m,
        20,
        &[
            (SADDR, src),
            (SOFF, 1),
            (ATTR, 0),
            (NBYTES, 2),
            (SLAST, -4i32 as u32),
            (DADDR, dst),
            (DOFF, 1),
            (CITER, 2),
            (BITER, 2),
            (DLAST, -4i32 as u32),
            (CSR, 0),
        ],
    );
    w(&mut m, SERQ, Size::Byte, 20);
    grant(&mut m, port, 4); // two requests of two reads
    assert_eq!(m.mem.peek(dst, 4).unwrap(), b"abcd");
    assert_eq!(get(&mut m, 20, SADDR), src, "SADDR back to the start");
    assert_eq!(get(&mut m, 20, DADDR), dst);
    assert_eq!(get(&mut m, 20, CITER), 2);
    assert_eq!(m.dma_lines().asserted(), 0);
}

#[test]
fn unmodelled_features_halt() {
    let cases: [(&[(Field, u32)], &str); 5] = [
        (&[(ATTR, 0x0404)], "transfer size"), // NBYTES 1 with 16-byte transfers
        (&[(CITER, 0x8001)], "channel link"), // E_LINK in CITER, not in BITER
        (&[(CITER, 0)], "CITER 0"),
        (&[(NBYTES, 0)], "NBYTES 0"),
        // RM 19.4.2's SOE/DOE are not modelled
        (&[(ATTR, 0x0202), (NBYTES, 4), (DOFF, 2)], "offset"),
    ];
    for (fields, what) in cases {
        let (mut m, _, port) = board(36);
        rx_channel(&mut m, 0x4010_1000);
        tcd(&mut m, 36, fields);
        push(&mut m, port, 1);
        let r = pending_halt(&mut m);
        assert_eq!(r.kind, Stop::Unmodelled, "{what}: {r:?}");
        assert!(
            r.detail.contains("eDMA") && r.detail.contains(what),
            "{what}: {r:?}"
        );
        assert_eq!(get(&mut m, 36, DADDR), 0x4010_1000, "{what}: nothing moved");
    }
}

#[test]
fn unmodelled_registers_halt() {
    let (mut m, _, _) = board(36);
    let d = refused(&mut m, EDMA + 0x0E, Size::Word, None); // ERQ: not modelled on purpose
    assert!(d.contains("eDMA"), "{d}");
    let d = refused(&mut m, SERQ, Size::Byte, Some(0x40)); // SERQ "all channels"
    assert!(d.contains("eDMA"), "{d}");
    // the error interrupt (INTC0 source 24) is not modelled: enabling it is refused, disabling it is not
    let d = refused(&mut m, SEEI, Size::Byte, Some(3));
    assert!(d.contains("error interrupt"), "{d}");
    let d = refused(&mut m, EDMA + 0x14, Size::Long, Some(1 << 3));
    assert!(d.contains("error interrupt"), "{d}");
    w(&mut m, EDMA + 0x14, Size::Long, 0);
    assert_eq!(r(&mut m, EDMA + 0x10, Size::Long), 0);
    w(&mut m, EDMA + 0x1B, Size::Byte, 3); // CEEI
    // NOP (bit 7) makes the command do nothing (RM 19.4.5)
    w(&mut m, SEEI, Size::Byte, 0x83);
    // the control register at other sizes, non-byte commands and the rest of the window
    refused(&mut m, EDMA, Size::Word, None);
    refused(&mut m, SERQ, Size::Long, Some(0x0101_0101));
    refused(&mut m, EDMA + 0x100, Size::Byte, None);
    refused(&mut m, EDMA + 0x1800, Size::Long, None);
    no_halt(&mut m);
}

#[test]
fn csr_esg_and_major_link_are_stored_unless_done() {
    for bit in [ESG, MAJORELINK] {
        let (mut m, _, _) = board(36);
        w(&mut m, tcd_addr(36, CSR), Size::Word, INTMAJOR | bit);
        assert_eq!(get(&mut m, 36, CSR), INTMAJOR | bit, "{bit:#x}: stored");
    }
    // with DONE set, a write forces E_SG and MAJOR_E_LINK to zero (RM 19.4.17, Table 19-31)
    let (mut m, _, port) = board(36);
    rx_channel(&mut m, 0x4010_1000);
    push(&mut m, port, 7);
    assert_ne!(get(&mut m, 36, CSR) & DONE, 0);
    w(
        &mut m,
        tcd_addr(36, CSR),
        Size::Word,
        DONE | INTMAJOR | ESG | MAJORELINK,
    );
    assert_eq!(get(&mut m, 36, CSR), DONE | INTMAJOR);
    no_halt(&mut m);
}

#[test]
fn minor_loops_of_32bit_words_with_half_and_major_interrupts() {
    // a transmit-shaped channel 54: 8 frames of 2 words, SLAST/DLAST wrap, INTHALF | INTMAJOR
    let (mut m, edma, port) = board(54);
    let (ring, dst) = (PORT + 0x200, 0x4010_3000);
    let data: Vec<u8> = (0..16u32)
        .flat_map(|i| (0x0100_0000 * i + 0x00AB_CD00).to_be_bytes())
        .collect();
    with_port(&mut m, port, |p| p.mem[0x200..0x240].copy_from_slice(&data));
    tcd(
        &mut m,
        54,
        &[
            (SADDR, ring),
            (SOFF, 4),
            (ATTR, 0x0202),
            (NBYTES, 8),
            (SLAST, -64i32 as u32),
            (DADDR, dst),
            (DOFF, 4),
            (CITER, 8),
            (BITER, 8),
            (DLAST, -64i32 as u32),
            (CSR, INTHALF | INTMAJOR),
        ],
    );
    let mut wake = Vec::new();
    assert!(m.device::<Edma>(edma).wake_sources(&mut wake));
    assert_eq!(wake, [], "the request is not enabled yet");
    w(&mut m, SERQ, Size::Byte, 54);
    assert!(m.device::<Edma>(edma).wake_sources(&mut wake));
    assert_eq!(wake, [(1, 46)], "channel 54 -> INTC1 source 46");
    grant(&mut m, port, 8); // four requests of two words
    assert!(source(&m, 1, 46), "half: CITER 8 -> 4");
    assert_eq!(get(&mut m, 54, CITER), 4);
    assert_eq!(get(&mut m, 54, SADDR), ring + 32);
    w(&mut m, CINT, Size::Byte, 54);
    assert!(!source(&m, 1, 46));
    grant(&mut m, port, 8);
    assert!(source(&m, 1, 46), "major");
    assert_eq!(get(&mut m, 54, SADDR), ring);
    assert_eq!(get(&mut m, 54, DADDR), dst);
    assert_eq!(get(&mut m, 54, CITER), 8);
    assert_eq!(m.mem.peek(dst, 64).unwrap(), data);
}

#[test]
fn mixed_transfer_sizes_move() {
    // reads fill a buffer of the larger size, which the writes empty (RM Example 19-1: 8-bit in, 32-bit out)
    let src = 0x4010_0000;
    let data: Vec<u8> = (0x10..0x30).collect();
    for (attr, soff, doff) in [
        (0x0002, 1, 4),
        (0x0200, 4, 1),
        (0x0101, 2, 2),
        (0x0001, 1, 2),
        (0x0402, 16, 4),
        (0x0204, 4, 16),
    ] {
        let (mut m, _, _) = board(54);
        let dst = 0x4010_0100;
        m.mem.load(src, &data).unwrap();
        tcd(
            &mut m,
            54,
            &[
                (SADDR, src),
                (SOFF, soff),
                (ATTR, attr),
                (NBYTES, 16),
                (DADDR, dst),
                (DOFF, doff),
                (CITER, 2),
                (BITER, 2),
                (CSR, START),
            ],
        );
        assert_eq!(
            m.mem.peek(dst, 32).unwrap()[..16],
            data[..16],
            "{attr:#06x}"
        );
        assert_eq!(
            m.mem.peek(dst + 16, 16).unwrap(),
            [0; 16],
            "{attr:#06x}: one minor loop"
        );
        assert_eq!(get(&mut m, 54, SADDR), src + 16, "{attr:#06x}");
        assert_eq!(get(&mut m, 54, DADDR), dst + 16, "{attr:#06x}");
        assert_eq!(get(&mut m, 54, CITER), 1, "{attr:#06x}");
        no_halt(&mut m);
    }
}

/// Channel 1 with transfer sizes `attr`, one minor loop of `nbytes` from 0x4030_0000 to 0x4030_0100, started by
/// software.
fn start_sized(m: &mut Machine, attr: u32, nbytes: u32) {
    m.mem
        .load(0x4030_0000, &(1..=32).collect::<Vec<u8>>())
        .unwrap();
    m.mem.load(0x4030_0100, &[0; 32]).unwrap();
    tcd(
        m,
        1,
        &[
            (SADDR, 0x4030_0000),
            (ATTR, attr),
            (SOFF, nbytes),
            (NBYTES, nbytes),
            (SLAST, 0),
            (DADDR, 0x4030_0100),
            (CITER, 1),
            (DOFF, nbytes),
            (DLAST, 0),
            (BITER, 1),
            (CSR, 0),
        ],
    );
    w(m, tcd_addr(1, CSR), Size::Word, START);
}

/// The configuration error of a reserved transfer size on channel 1: nothing moved, START cleared, DONE not set,
/// ES with VLD, ERRCHN 1 and SAE, SOE, DAE and DOE; then CERR.
fn assert_size_error(m: &mut Machine, what: &str) {
    assert_eq!(r(m, ES, Size::Long), 0x8000_01F0, "{what}");
    assert_eq!(r(m, ERRL, Size::Long), 1 << 1, "{what}");
    assert_eq!(get(m, 1, CSR), 0, "{what}");
    assert_eq!(get(m, 1, SADDR), 0x4030_0000, "{what}");
    assert_eq!(get(m, 1, DADDR), 0x4030_0100, "{what}");
    assert_eq!(get(m, 1, CITER), 1, "{what}");
    assert_eq!(m.mem.peek(0x4030_0100, 32).unwrap(), [0; 32], "{what}");
    w(m, CERR, Size::Byte, 1);
    assert_eq!(r(m, ES, Size::Long), 0x0000_01F0, "{what}");
}

#[test]
fn reserved_size_101_is_a_configuration_error_measured() {
    // measured on an MCF54415: ATTR 0x0505 (101, reserved in RM Table 19-21) with NBYTES 32 is a configuration
    // error, not a 32-byte transfer: nothing moves, ES 0x800001F0, ERRL bit 1, CSR 0, SADDR/DADDR unchanged
    let (mut m, _, _) = board(1);
    start_sized(&mut m, 0x0505, 32);
    assert_size_error(&mut m, "101");
    no_halt(&mut m);
}

#[test]
fn reserved_size_011_is_a_configuration_error_measured() {
    // measured on an MCF54415: ATTR 0x0303 (011) gives exactly what 101 gives
    let (mut m, _, _) = board(1);
    start_sized(&mut m, 0x0303, 32);
    assert_size_error(&mut m, "011");
    no_halt(&mut m);
}

#[test]
fn reserved_sizes_110_111_and_one_sided_are_configuration_errors() {
    // guess: 110 and 111, and a reserved size on one side only, give what 011 and 101 give (not measured)
    for attr in [0x0606, 0x0707, 0x0602, 0x0005] {
        let (mut m, _, _) = board(1);
        start_sized(&mut m, attr, 32);
        assert_size_error(&mut m, &format!("{attr:#06x}"));
        no_halt(&mut m);
    }
    // served at a request: the error ends the serving, the request stays asserted, nothing halts
    let (mut m, _, port) = board(1);
    start_sized(&mut m, 0x0505, 32);
    w(&mut m, CERR, Size::Byte, 1);
    w(&mut m, SERQ, Size::Byte, 1);
    grant(&mut m, port, 1);
    assert_eq!(r(&mut m, ES, Size::Long), 0x8000_01F0);
    assert_eq!(m.dma_lines().asserted(), 1 << 1);
    grant(&mut m, port, 0);
    no_halt(&mut m);
}

#[test]
fn serq_keeps_servicing_a_level_request_until_it_drops() {
    let (mut m, _, port) = board(20);
    for v in [1, 2, 3] {
        push(&mut m, port, v);
    }
    tcd(
        &mut m,
        20,
        &[
            (SADDR, PORT + RX),
            (SOFF, 0),
            (ATTR, 0),
            (NBYTES, 1),
            (DADDR, 0x4010_0200),
            (DOFF, 1),
            (CITER, 10),
            (BITER, 10),
        ],
    );
    assert_eq!(m.dma_lines().asserted(), 1 << 20);
    w(&mut m, SERQ, Size::Byte, 20);
    assert!(m.device::<Port>(port).fifo.is_empty());
    assert_eq!(get(&mut m, 20, CITER), 7, "three minor loops");
    assert_eq!(m.mem.peek(0x4010_0200, 3).unwrap(), [1, 2, 3]);
    assert_eq!(m.dma_lines().asserted(), 0);
}

#[test]
fn done_is_cleared_when_the_channel_runs_again() {
    let (mut m, _, port) = board(36);
    rx_channel(&mut m, 0x4010_1000);
    push(&mut m, port, 1);
    assert_ne!(get(&mut m, 36, CSR) & DONE, 0);
    w(&mut m, CINT, Size::Byte, 36);
    assert!(!source(&m, 1, 28));
    push(&mut m, port, 2); // a new major loop starts (and completes: CITER 1)
    assert_ne!(get(&mut m, 36, CSR) & DONE, 0);
    tcd(&mut m, 36, &[(CITER, 2), (BITER, 2)]);
    push(&mut m, port, 3); // starts, doesn't complete
    assert_eq!(get(&mut m, 36, CSR) & DONE, 0);
    assert!(
        source(&m, 1, 28),
        "the second completion's interrupt is still uncleared"
    );
    // CDNE clears DONE
    push(&mut m, port, 4);
    assert_ne!(get(&mut m, 36, CSR) & DONE, 0);
    w(&mut m, CDNE, Size::Byte, 36);
    assert_eq!(get(&mut m, 36, CSR), INTMAJOR);
}

#[test]
fn a_request_that_never_drops_halts_at_serq() {
    let (mut m, _, port) = board(20);
    with_port(&mut m, port, |p| p.stuck = true);
    let ch20 = [
        (SADDR, 0x4010_0100),
        (SOFF, 0),
        (ATTR, 0),
        (NBYTES, 1),
        (DADDR, 0x4010_0200),
        (DOFF, 0),
        (CITER, 1),
        (BITER, 1),
    ];
    tcd(&mut m, 20, &ch20);
    let d = refused(&mut m, SERQ, Size::Byte, Some(20));
    assert!(
        d.contains("eDMA channel 20") && d.contains("never drops"),
        "{d}"
    );

    // raised with the request already enabled, it halts when served
    let (mut m, _, port) = board(20);
    tcd(&mut m, 20, &ch20);
    w(&mut m, SERQ, Size::Byte, 20);
    with_port(&mut m, port, |p| p.stuck = true);
    let r = pending_halt(&mut m);
    assert_eq!(r.kind, Stop::Unmodelled, "{r:?}");
    assert!(r.detail.contains("never drops"), "{r:?}");
}

#[test]
fn channels_56_to_63_share_intc2_source_0() {
    let (mut m, _, p58) = board(58);
    let p59 = m.add_device(PORT2, 0x1000, Box::new(Port::new(59)));
    let dst = 0x4010_1000;
    for (ch, port, base, d) in [(58, p58, PORT, dst), (59, p59, PORT2, dst + 0x100)] {
        with_port(&mut m, port, |p| {
            p.mem[..16].copy_from_slice(&(0..16).collect::<Vec<u8>>())
        });
        tcd(
            &mut m,
            ch,
            &[
                (SADDR, base),
                (SOFF, 4),
                (ATTR, 0x0202),
                (NBYTES, 16),
                (SLAST, 0),
                (DADDR, d),
                (DOFF, 4),
                (CITER, 1),
                (BITER, 1),
                (DLAST, 0),
                (CSR, INTMAJOR),
            ],
        );
        w(&mut m, SERQ, Size::Byte, ch);
        grant(&mut m, port, 4);
    }
    let expect: Vec<u8> = (0..16).collect();
    assert_eq!(m.mem.peek(dst + 0x100, 16).unwrap(), expect);
    assert!(source(&m, 2, 0));
    w(&mut m, CINT, Size::Byte, 58); // 59 still pending
    assert!(source(&m, 2, 0));
    w(&mut m, CINT, Size::Byte, 59);
    assert!(!source(&m, 2, 0));
    no_halt(&mut m);
}

#[test]
fn a_misaligned_address_is_a_configuration_error_and_moves_nothing() {
    // RM 19.4.2: SAE/DAE when an address isn't a multiple of its transfer size. ES values measured on an MCF54415
    let (mut m, _, port) = board(59);
    let src = 0x4010_8000;
    m.mem.load(src, &[0x11, 0x22, 0x33, 0x44]).unwrap();
    tcd(
        &mut m,
        59,
        &[
            (SADDR, src),
            (SOFF, 0),
            (ATTR, 2 << 8 | 2),
            (NBYTES, 4),
            (SLAST, 0),
            (DADDR, 0x4010_1001),
            (DOFF, 4),
            (CITER, 1),
            (BITER, 1),
            (DLAST, 0),
            (CSR, DREQ),
        ],
    );
    w(&mut m, SERQ, Size::Byte, 59);
    grant(&mut m, port, 1);
    assert_eq!(m.mem.peek(0x4010_1000, 8).unwrap(), [0; 8]);
    assert_eq!(
        r(&mut m, ES, Size::Long),
        0x8000_0000 | 59 << 8 | 0x20,
        "VLD, ERRCHN, DAE"
    );
    assert_eq!(r(&mut m, ERRH, Size::Long), 1 << (59 - 32));
    assert_eq!(r(&mut m, ERRL, Size::Long), 0);
    assert_eq!(get(&mut m, 59, CSR), DREQ, "not DONE");
    assert_eq!(get(&mut m, 59, CITER), 1, "CITER unchanged");
    assert_eq!(get(&mut m, 59, DADDR), 0x4010_1001);
    assert_eq!(
        m.dma_lines().asserted(),
        1 << 59,
        "the request is still asserted"
    );
    grant(&mut m, port, 0);
    w(&mut m, CERR, Size::Byte, 59);
    assert_eq!(r(&mut m, ERRH, Size::Long), 0);
    // CERR clears VLD; ES keeps ERRCHN and DAE (measured on an MCF54415)
    assert_eq!(r(&mut m, ES, Size::Long), 0x0000_3B20);
    tcd(&mut m, 59, &[(SADDR, src + 1), (DADDR, 0x4010_2000)]);
    grant(&mut m, port, 1);
    assert_eq!(
        r(&mut m, ES, Size::Long),
        0x8000_0000 | 59 << 8 | 0x80,
        "SAE"
    );
    grant(&mut m, port, 0);
    w(&mut m, CERR, Size::Byte, 59);
    assert_eq!(r(&mut m, ES, Size::Long), 0x0000_3B80);
    assert_eq!(m.mem.peek(0x4010_2000, 4).unwrap(), [0; 4]);
    no_halt(&mut m);
}

#[test]
fn edma_moves_between_a_device_register_and_ram() {
    let (mut m, _, port) = board(20);
    let ram = 0x4010_4000;
    // the port's receive register into RAM, 8-bit then 32-bit
    for (attr, v, n) in [(0x0000, 0x5A, 1), (0x0202, 0x1234_5678, 4)] {
        tcd(
            &mut m,
            20,
            &[
                (SADDR, PORT + RX),
                (SOFF, 0),
                (ATTR, attr),
                (NBYTES, n),
                (DADDR, ram),
                (DOFF, n),
                (CITER, 1),
                (BITER, 1),
                (CSR, 0),
            ],
        );
        w(&mut m, SERQ, Size::Byte, 20);
        push(&mut m, port, v);
        let got = m.mem.peek(ram, n).unwrap();
        assert_eq!(got, &v.to_be_bytes()[4 - n as usize..], "{attr:#x}");
    }
    // RAM into the port's transmit register, 8-bit then 32-bit
    m.mem
        .load(
            ram + 0x100,
            &[
                0xA1, 0xA2, 0, 0, 0xCA, 0xFE, 0xF0, 0x0D, 0xDE, 0xAD, 0xBE, 0xEF,
            ],
        )
        .unwrap();
    for (attr, s, n, expect) in [
        (0x0000, ram + 0x100, 1, vec![0xA1, 0xA2]),
        (0x0202, ram + 0x104, 4, vec![0xCAFE_F00D, 0xDEAD_BEEF]),
    ] {
        with_port(&mut m, port, |p| p.out.clear());
        tcd(
            &mut m,
            20,
            &[
                (SADDR, s),
                (SOFF, n),
                (ATTR, attr),
                (NBYTES, n),
                (DADDR, PORT + TX),
                (DOFF, 0),
                (CITER, 2),
                (BITER, 2),
                (SLAST, 0),
                (DLAST, 0),
                (CSR, 0),
            ],
        );
        w(&mut m, SERQ, Size::Byte, 20);
        grant(&mut m, port, 2);
        assert_eq!(m.device::<Port>(port).out, expect, "{attr:#x}");
    }
    no_halt(&mut m);
}

#[test]
fn channels_0_to_15_interrupt_on_intc0() {
    let (mut m, _, port) = board(3);
    let p15 = m.add_device(PORT2, 0x1000, Box::new(Port::new(15)));
    for (ch, port, base, source_no) in [(3, port, PORT, 11), (15, p15, PORT2, 23)] {
        tcd(
            &mut m,
            ch,
            &[
                (SADDR, base + RX),
                (SOFF, 0),
                (ATTR, 0),
                (NBYTES, 1),
                (DADDR, 0x4010_5000 + ch),
                (DOFF, 0),
                (CITER, 1),
                (BITER, 1),
                (CSR, INTMAJOR),
            ],
        );
        w(&mut m, SERQ, Size::Byte, ch);
        push(&mut m, port, ch);
        assert!(
            source(&m, 0, source_no),
            "channel {ch} -> INTC0 source {source_no}"
        );
        assert_eq!(m.mem.peek(0x4010_5000 + ch, 1).unwrap(), [ch as u8]);
        w(&mut m, CINT, Size::Byte, ch);
        assert!(!source(&m, 0, source_no));
    }
    assert_eq!(m.mem.irq_lines().asserted(0), 0);
    no_halt(&mut m);
}

/// Channel 1 in the shape of the measurement: 32-bit, one 4-byte minor loop, CITER = BITER = 1.
fn start_channel(m: &mut Machine, saddr: u32, daddr: u32) {
    m.mem.load(0x4030_0000, &[0x11, 0x22, 0x33, 0x44]).unwrap();
    m.mem.load(0x4030_0100, &[0; 8]).unwrap();
    tcd(
        m,
        1,
        &[
            (SADDR, saddr),
            (ATTR, 0x0202),
            (SOFF, 4),
            (NBYTES, 4),
            (SLAST, 0),
            (DADDR, daddr),
            (CITER, 1),
            (DOFF, 4),
            (DLAST, 0),
            (BITER, 1),
            (CSR, 0),
        ],
    );
    w(m, tcd_addr(1, CSR), Size::Word, START);
}

#[test]
fn software_start_runs_one_minor_loop() {
    // the aligned control case, measured on an MCF54415: the data moves and DONE is set. START clears as the
    // channel begins (RM Table 19-31)
    let (mut m, _, _) = board(1);
    start_channel(&mut m, 0x4030_0000, 0x4030_0100);
    assert_eq!(
        m.mem.peek(0x4030_0100, 8).unwrap(),
        [0x11, 0x22, 0x33, 0x44, 0, 0, 0, 0]
    );
    assert_eq!(get(&mut m, 1, CSR), DONE);
    assert_eq!(get(&mut m, 1, CITER), 1);
    assert_eq!(get(&mut m, 1, SADDR), 0x4030_0004);
    assert_eq!(get(&mut m, 1, DADDR), 0x4030_0104);
    assert_eq!(r(&mut m, ES, Size::Long), 0);
    // one minor loop per START (RM 19.6.4.1), with the request not enabled
    tcd(
        &mut m,
        1,
        &[
            (SADDR, 0x4030_0000),
            (DADDR, 0x4030_0200),
            (CITER, 2),
            (BITER, 2),
            (CSR, INTHALF | INTMAJOR),
        ],
    );
    w(&mut m, SSRT, Size::Byte, 1);
    assert_eq!(
        get(&mut m, 1, CSR),
        INTHALF | INTMAJOR,
        "START clears; DONE cleared by the activation"
    );
    assert_eq!(get(&mut m, 1, CITER), 1);
    assert!(source(&m, 0, 9), "half: channel 1 -> INTC0 source 9");
    w(&mut m, CINT, Size::Byte, 1);
    w(&mut m, tcd_addr(1, CSR), Size::Word, INTMAJOR | START);
    assert_eq!(get(&mut m, 1, CSR), INTMAJOR | DONE);
    assert_eq!(get(&mut m, 1, CITER), 2);
    assert!(source(&m, 0, 9), "major");
    assert_eq!(
        m.mem.peek(0x4030_0200, 8).unwrap(),
        [0x11, 0x22, 0x33, 0x44, 0, 0, 0, 0]
    );
    no_halt(&mut m);
}

#[test]
fn software_start_with_a_misaligned_address_is_a_configuration_error() {
    // measured on an MCF54415: nothing moves, START clears, DONE not set, CITER unchanged; CERR clears ES.VLD
    let (mut m, _, _) = board(1);
    start_channel(&mut m, 0x4030_0000, 0x4030_0101);
    assert_eq!(r(&mut m, ES, Size::Long), 0x8000_0120);
    assert_eq!(r(&mut m, ERRL, Size::Long), 1 << 1);
    assert_eq!(get(&mut m, 1, CSR), 0);
    assert_eq!(get(&mut m, 1, CITER), 1);
    assert_eq!(get(&mut m, 1, SADDR), 0x4030_0000);
    assert_eq!(get(&mut m, 1, DADDR), 0x4030_0101);
    assert_eq!(m.mem.peek(0x4030_0100, 8).unwrap(), [0; 8]);
    w(&mut m, CERR, Size::Byte, 1);
    assert_eq!(r(&mut m, ES, Size::Long), 0x0000_0120);
    assert_eq!(r(&mut m, ERRL, Size::Long), 0);
    start_channel(&mut m, 0x4030_0001, 0x4030_0100);
    assert_eq!(r(&mut m, ES, Size::Long), 0x8000_0180);
    assert_eq!(get(&mut m, 1, CSR), 0);
    assert_eq!(m.mem.peek(0x4030_0100, 8).unwrap(), [0; 8]);
    w(&mut m, CERR, Size::Byte, 1);
    // aligned control: moves, DONE set, ES.VLD stays clear
    start_channel(&mut m, 0x4030_0000, 0x4030_0100);
    assert_eq!(r(&mut m, ES, Size::Long) & 0x8000_0000, 0);
    assert_eq!(get(&mut m, 1, CSR), DONE);
    no_halt(&mut m);
}

#[test]
fn a_bus_error_halts() {
    // RM 19.4.2's bus errors (SBE/DBE) are not modelled
    let (mut m, _, _) = board(1);
    tcd(
        &mut m,
        1,
        &[
            (SADDR, 0x4030_0000),
            (ATTR, 0),
            (SOFF, 1),
            (NBYTES, 1),
            (DADDR, 0x2000_0000),
            (CITER, 1),
            (BITER, 1),
        ],
    );
    let d = refused(&mut m, tcd_addr(1, CSR), Size::Word, Some(START));
    assert!(
        d.contains("eDMA channel 1") && d.contains("bus error"),
        "{d}"
    );
}

#[test]
fn a_peripheral_holding_more_than_64_entries_is_served_whole() {
    // 4096 minor loops of four longs from the receive register, behind one request
    let (mut m, _, port) = board(59);
    let dst = 0x4020_0000;
    tcd(
        &mut m,
        59,
        &[
            (SADDR, PORT + RX),
            (SOFF, 0),
            (ATTR, 0x0202),
            (NBYTES, 16),
            (SLAST, 0),
            (DADDR, dst),
            (DOFF, 4),
            (CITER, 4096),
            (BITER, 4096),
            (DLAST, 0),
            (CSR, INTMAJOR),
        ],
    );
    w(&mut m, SERQ, Size::Byte, 59);
    with_port(&mut m, port, |p| p.fifo.extend(0..4096 * 4));
    no_halt(&mut m);
    assert!(m.device::<Port>(port).fifo.is_empty());
    assert_ne!(get(&mut m, 59, CSR) & DONE, 0);
    let got = m.mem.peek(dst, 4096 * 16).unwrap();
    let expect: Vec<u8> = (0..4096 * 4u32).flat_map(u32::to_be_bytes).collect();
    assert_eq!(got, expect);
}

#[test]
fn a_request_served_without_end_halts() {
    // every minor loop reads the port, which never lowers its request: a backstop of 1 << 20 minor loops
    let (mut m, _, port) = board(20);
    tcd(
        &mut m,
        20,
        &[
            (SADDR, PORT),
            (SOFF, 0),
            (ATTR, 0),
            (NBYTES, 1),
            (DADDR, 0x4010_0200),
            (DOFF, 0),
            (CITER, 1),
            (BITER, 1),
        ],
    );
    w(&mut m, SERQ, Size::Byte, 20);
    with_port(&mut m, port, |p| p.stuck = true);
    let r = pending_halt(&mut m);
    assert_eq!(r.kind, Stop::Unmodelled, "{r:?}");
    assert!(
        r.detail.contains("eDMA channel 20") && r.detail.contains("1048576 minor loops"),
        "{r:?}"
    );
}

#[test]
fn a_core_in_stop_wakes_on_the_interrupt_of_a_dma_it_waits_for() {
    // The port's arrival raises no interrupt of its own; the eDMA channel it requests raises INTC1 source 28 at
    // the end of its major loop. The STOP probe must count the eDMA's route.
    const INTC1: u32 = 0xFC04_C000;
    let src = "stop #0x2000\nend: nop\nhandler: move.l #0xFC04401C,%a0\nmove.b #36,(%a0)\nrte";
    let (mut m, l) = common::prepare(src, &["end", "handler"]);
    let ([_, intc1, _], _) = Intc::chip();
    m.add_interrupt_controller(INTC1, 0x4000, Box::new(intc1));
    m.add_bus_master(EDMA, 0x2000, Box::new(Edma::new()));
    let port = m.add_device(PORT, 0x1000, Box::new(Port::new(36)));
    w(&mut m, INTC1 + 0x40 + 28, Size::Byte, 3); // ICR28: level 3
    w(&mut m, INTC1 + 0x0C, Size::Long, !(1 << 28)); // IMRL: source 28 unmasked
    common::set_vector(&mut m, 64 + 64 + 28, l[1]);
    rx_channel(&mut m, 0x4030_0000);
    with_port(&mut m, port, |p| p.event = Some(1000));
    m.set_time_limit(Some(100_000));
    let r = m.run(common::until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.mem.peek(0x4030_0000, 1).unwrap(), [ARRIVAL as u8]);
    assert!(m.now() >= 1000);
}

#[test]
fn a_permissive_edma_takes_serq_on_any_channel() {
    // Edma::new() has every request line as driven: SERQ on a channel nobody drives just waits
    let (mut m, _, _) = board(1);
    for ch in [3, 39, 58] {
        w(&mut m, SERQ, Size::Byte, ch);
    }
    no_halt(&mut m);
}

#[test]
fn serq_outside_the_driven_lines_is_refused_but_software_start_runs() {
    let mut m = Machine::new();
    m.mem.map_ram(RAM, 0x0100_0000);
    m.add_bus_master(EDMA, 0x2000, Box::new(Edma::with_driven(1 << 2)));
    w(&mut m, SERQ, Size::Byte, 2);
    assert_eq!(
        refused(&mut m, SERQ, Size::Byte, Some(1)),
        "eDMA: channel 1: its peripheral request is not modelled"
    );
    // a software start needs no peripheral request: START works on any channel
    start_channel(&mut m, 0x4030_0000, 0x4030_0100);
    assert_eq!(
        m.mem.peek(0x4030_0100, 4).unwrap(),
        [0x11, 0x22, 0x33, 0x44]
    );
    assert_eq!(get(&mut m, 1, CSR), DONE);
    no_halt(&mut m);
}

#[test]
fn a_host_access_refused_by_a_bus_error_leaves_no_halt_pending() {
    // the served transfer's write reaches a block that refuses it: the SERQ fails with the eDMA's detail, and
    // nothing is left for the next run
    const BLOCK: u32 = 0xFC0E_8000;
    let (mut m, _, port) = board(5);
    m.add_device(BLOCK, 0x1000, Box::new(Unmodelled::new("a block")));
    tcd(
        &mut m,
        5,
        &[
            (SADDR, PORT + RX),
            (ATTR, 0x0202),
            (SOFF, 0),
            (NBYTES, 4),
            (DADDR, BLOCK),
            (DOFF, 0),
            (CITER, 1),
            (BITER, 1),
        ],
    );
    push(&mut m, port, 7);
    let d = refused(&mut m, SERQ, Size::Byte, Some(5));
    assert!(
        d.contains("eDMA channel 5") && d.contains("bus error writing"),
        "{d}"
    );
    w(&mut m, CERQ, Size::Byte, 5);
    no_halt(&mut m);
}

/// A 32-byte TCD as it sits in memory for scatter/gather (RM 19.4.17, big-endian).
fn tcd_image(fields: &[(Field, u32)]) -> [u8; 32] {
    let mut t = [0; 32];
    for &(f, v) in fields {
        let (o, n) = (f.0 as usize, f.1.bytes() as usize);
        t[o..o + n].copy_from_slice(&v.to_be_bytes()[4 - n..]);
    }
    t
}

/// The fields of a 4-byte copy from `src` to `dst`: one 32-bit transfer, CITER = BITER = 1.
fn copy4(src: u32, dst: u32) -> Vec<(Field, u32)> {
    vec![
        (SADDR, src),
        (ATTR, 0x0202),
        (SOFF, 4),
        (NBYTES, 4),
        (SLAST, 0),
        (DADDR, dst),
        (DOFF, 4),
        (CITER, 1),
        (BITER, 1),
    ]
}

#[test]
fn cr_is_stored_and_reads_back() {
    let (mut m, _, _) = board(1);
    assert_eq!(
        r(&mut m, EDMA, Size::Long),
        0x0000_E400,
        "reset: GRP3PRI 3, GRP2PRI 2, GRP1PRI 1, GRP0PRI 0 (RM Figure 19-3)"
    );
    let cr = r(&mut m, EDMA, Size::Long) | 6; // EDBG | ERCA
    w(&mut m, EDMA, Size::Long, cr);
    assert_eq!(r(&mut m, EDMA, Size::Long), 0x0000_E406);
    w(&mut m, EDMA, Size::Long, 0x0000_E40E); // and ERGA
    assert_eq!(r(&mut m, EDMA, Size::Long), 0x0000_E40E);
    // reserved bits (31-18, 0) are dropped (RM 19.4)
    w(&mut m, EDMA, Size::Long, 0x8004_E407);
    assert_eq!(r(&mut m, EDMA, Size::Long), 0x0000_E406);
    // guess: EDBG, ERCA and ERGA change nothing, since transfers take no time
    start_channel(&mut m, 0x4030_0000, 0x4030_0100);
    assert_eq!(
        m.mem.peek(0x4030_0100, 4).unwrap(),
        [0x11, 0x22, 0x33, 0x44]
    );
    assert_eq!(get(&mut m, 1, CSR), DONE);
    no_halt(&mut m);
}

#[test]
fn cr_halt_and_clm_stop_unmodelled() {
    let (mut m, _, _) = board(1);
    for (bit, name) in [
        (0x20, "HALT"),
        (0x40, "CLM"),
        (0x80, "EMLM"),
        (0x10, "HOE"),
        (1 << 16, "ECX"),
        (1 << 17, "CX"),
    ] {
        let d = refused(&mut m, EDMA, Size::Long, Some(0xE400 | bit));
        assert!(d.contains("eDMA CR") && d.contains(name), "{name}: {d}");
        assert_eq!(r(&mut m, EDMA, Size::Long), 0xE400, "{name}: not stored");
    }
    // group priorities other than the reset ones
    let d = refused(&mut m, EDMA, Size::Long, Some(0x1B00));
    assert!(d.contains("GRPnPRI"), "{d}");
    assert_eq!(r(&mut m, EDMA, Size::Long), 0xE400);
    no_halt(&mut m);
}

#[test]
fn scatter_gather_loads_the_next_tcd_and_runs_it() {
    let (mut m, _, _) = board(7);
    let src = 0x4041_0000;
    let (sg1, sg2) = (0x4040_0000, 0x4040_0020);
    let (d1, d2, d3) = (0x4042_0000, 0x4042_0100, 0x4042_0200);
    m.mem.load(src, &(1..=12).collect::<Vec<u8>>()).unwrap();
    // B runs as it is loaded (START | E_SG) and loads C, which waits (no START)
    let mut b = copy4(src + 4, d2);
    b.extend([(DLAST, sg2), (CSR, START | ESG)]);
    let mut c = copy4(src + 8, d3);
    c.extend([(DLAST, 0), (CSR, INTMAJOR)]);
    m.mem.load(sg1, &tcd_image(&b)).unwrap();
    m.mem.load(sg2, &tcd_image(&c)).unwrap();
    let mut a = copy4(src, d1);
    a.extend([(DLAST, sg1), (CSR, START | ESG | INTMAJOR)]);
    tcd(&mut m, 7, &a);
    assert_eq!(m.mem.peek(d1, 4).unwrap(), [1, 2, 3, 4]);
    assert_eq!(m.mem.peek(d2, 4).unwrap(), [5, 6, 7, 8]);
    assert_eq!(
        m.mem.peek(d3, 4).unwrap(),
        [0; 4],
        "C is loaded, not started"
    );
    assert!(
        source(&m, 0, 15),
        "A's INT_MAJOR: channel 7 -> INTC0 source 15"
    );
    // the channel's TCD is C as it sits in memory, with DONE set onto its CSR (measured on an MCF54415)
    for &(f, v) in &c {
        let v = if f == CSR { v | DONE } else { v };
        assert_eq!(get(&mut m, 7, f), v, "{f:?}");
    }
    w(&mut m, CINT, Size::Byte, 7);
    w(&mut m, SSRT, Size::Byte, 7);
    assert_eq!(m.mem.peek(d3, 4).unwrap(), [9, 10, 11, 12]);
    assert_eq!(get(&mut m, 7, CSR), INTMAJOR | DONE);
    assert_eq!(get(&mut m, 7, DADDR), d3 + 4, "DLAST 0 added: no E_SG");
    assert!(source(&m, 0, 15));
    assert_eq!(r(&mut m, ES, Size::Long), 0);
    no_halt(&mut m);
}

#[test]
fn misaligned_dlast_sga_moves_the_data_sets_done_and_adds_it_to_daddr_measured() {
    // measured on an MCF54415: channel 1, one 4-byte minor loop, CSR E_SG | START, DLAST_SGA an aligned TCD + 4.
    // The completion happens, then SGE: data moved, DONE set, E_SG kept, CITER reloaded, SADDR advanced (SLAST 0),
    // DLAST_SGA added to DADDR as DLAST, no TCD loaded; ES VLD | ERRCHN | SGE
    let (mut m, _, _) = board(1);
    let (src, dst, tcd_at) = (0x4041_0000, 0x4042_0000, 0x4040_0000);
    let sga = tcd_at + 4;
    m.mem.load(src, &[1, 2, 3, 4]).unwrap();
    m.mem
        .load(tcd_at, &tcd_image(&copy4(src, 0x4043_0000)))
        .unwrap();
    let mut a = copy4(src, dst);
    a.extend([(DLAST, sga), (CSR, START | ESG)]);
    tcd(&mut m, 1, &a);
    assert_eq!(m.mem.peek(dst, 4).unwrap(), [1, 2, 3, 4], "the data moved");
    assert_eq!(r(&mut m, ES, Size::Long), 0x8000_0104, "VLD, ERRCHN 1, SGE");
    assert_eq!(r(&mut m, ERRL, Size::Long), 1 << 1);
    assert_eq!(get(&mut m, 1, CSR), 0x0090, "DONE, E_SG kept, START clear");
    assert_eq!(get(&mut m, 1, CITER), 1, "reloaded from BITER");
    assert_eq!(get(&mut m, 1, SADDR), src + 4);
    assert_eq!(
        get(&mut m, 1, DADDR),
        (dst + 4).wrapping_add(sga),
        "DLAST_SGA added as DLAST"
    );
    assert_eq!(get(&mut m, 1, DLAST), sga, "no TCD loaded");
    assert_eq!(m.mem.peek(0x4043_0000, 4).unwrap(), [0; 4]);
    w(&mut m, CERR, Size::Byte, 1);
    assert_eq!(r(&mut m, ES, Size::Long), 0x0000_0104);
    // guess: INT_MAJOR still raises the interrupt, which comes before scatter/gather in RM's order
    w(&mut m, CDNE, Size::Byte, 1); // with DONE set, E_SG would be written as 0
    let mut a = copy4(src, dst);
    a.extend([(DLAST, sga), (CSR, START | ESG | INTMAJOR)]);
    tcd(&mut m, 1, &a);
    assert_eq!(r(&mut m, ES, Size::Long), 0x8000_0104);
    assert!(source(&m, 0, 9), "channel 1 -> INTC0 source 9");
    no_halt(&mut m);
}

#[test]
fn major_link_start_is_overwritten_by_the_scatter_gather_load_measured() {
    // measured on an MCF54415: E_SG and MAJOR_E_LINK to the channel itself. RM's order: interrupts, the major
    // link, then the load, which overwrites the link's START: the loaded TCD is in place and does not run
    let (mut m, _, _) = board(1);
    let (src, sga) = (0x4041_0000, 0x4040_0000);
    let (d1, d2) = (0x4042_0000, 0x4042_0100);
    m.mem.load(src, &[1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
    let mut loaded = copy4(src + 4, d2);
    loaded.extend([(DLAST, 0), (CSR, 0)]);
    m.mem.load(sga, &tcd_image(&loaded)).unwrap();
    let mut a = copy4(src, d1);
    a.extend([(DLAST, sga), (CSR, START | ESG | MAJORELINK | 1 << 8)]);
    tcd(&mut m, 1, &a);
    assert_eq!(m.mem.peek(d1, 4).unwrap(), [1, 2, 3, 4]);
    assert_eq!(
        m.mem.peek(d2, 4).unwrap(),
        [0; 4],
        "the loaded TCD did not run"
    );
    assert_eq!(get(&mut m, 1, SADDR), src + 4);
    assert_eq!(get(&mut m, 1, DADDR), d2);
    assert_eq!(get(&mut m, 1, DLAST), 0);
    assert_eq!(get(&mut m, 1, CSR), 0x0080, "DONE onto the loaded CSR");
    assert_eq!(r(&mut m, ES, Size::Long), 0);
    no_halt(&mut m);
}

#[test]
fn scatter_gather_load_sets_done_measured() {
    // measured on an MCF54415: after the load CSR reads 0x0080 although the loaded CSR is 0
    scatter_gather_load_sets_done(0);
}

#[test]
fn scatter_gather_load_of_int_major_sets_done_guess() {
    // guess: DONE is set onto a loaded CSR that has INT_MAJOR as it is onto one that is 0 (only the 0 case is
    // measured)
    scatter_gather_load_sets_done(INTMAJOR);
}

fn scatter_gather_load_sets_done(loaded_csr: u32) {
    let (mut m, _, _) = board(1);
    let (src, sga) = (0x4041_0000, 0x4040_0000);
    m.mem.load(src, &[1, 2, 3, 4]).unwrap();
    let mut loaded = copy4(src, 0x4042_0100);
    loaded.extend([(DLAST, 0), (CSR, loaded_csr)]);
    m.mem.load(sga, &tcd_image(&loaded)).unwrap();
    let mut a = copy4(src, 0x4042_0000);
    a.extend([(DLAST, sga), (CSR, START | ESG)]);
    tcd(&mut m, 1, &a);
    assert_eq!(get(&mut m, 1, CSR), loaded_csr | DONE, "{loaded_csr:#x}");
    assert_eq!(get(&mut m, 1, DADDR), 0x4042_0100, "{loaded_csr:#x}");
    no_halt(&mut m);
}

#[test]
fn queued_linked_starts_run_highest_channel_first() {
    // guess: the queued starts run in the order the reset channel priorities give (RM 19.4.16), highest channel
    // first. Channel 10's load (with START) and its major link to channel 3 queue both; both write the same byte,
    // so the last to run wins
    let (mut m, _, _) = board(1);
    let (src, sga, d) = (0x4041_0000, 0x4040_0000, 0x4042_0000);
    m.mem
        .load(src, &[0, 0, 0, 0, 0xAA, 0, 0, 0, 0xBB, 0, 0, 0])
        .unwrap();
    let mut loaded = copy4(src + 4, d);
    loaded.extend([(DLAST, 0), (CSR, START)]);
    m.mem.load(sga, &tcd_image(&loaded)).unwrap();
    tcd(&mut m, 3, &copy4(src + 8, d));
    let mut a = copy4(src, 0x4042_0100);
    a.extend([(DLAST, sga), (CSR, START | ESG | MAJORELINK | 3 << 8)]);
    tcd(&mut m, 10, &a);
    assert_eq!(get(&mut m, 10, CSR), DONE, "the loaded TCD ran");
    assert_eq!(get(&mut m, 3, CSR), DONE, "channel 3 ran");
    assert_eq!(
        m.mem.peek(d, 1).unwrap(),
        [0xBB],
        "channel 10 first, then 3"
    );
    no_halt(&mut m);
}

#[test]
fn biter_1_with_int_half_raises_no_half_interrupt() {
    // RM Table 19-31: the half-way interrupt is disabled for BITER below 2
    let (mut m, _, _) = board(1);
    m.mem.load(0x4041_0000, &[1, 2, 3, 4]).unwrap();
    let mut a = copy4(0x4041_0000, 0x4042_0000);
    a.push((CSR, START | INTHALF));
    tcd(&mut m, 1, &a);
    assert_eq!(get(&mut m, 1, CSR), INTHALF | DONE);
    assert!(!source(&m, 0, 9));
    no_halt(&mut m);
}

#[test]
fn a_linked_channel_with_a_configuration_error_recovers_after_cerr() {
    let (mut m, _, _) = board(1);
    let src = 0x4041_0000;
    m.mem.load(src, &(1..=16).collect::<Vec<u8>>()).unwrap();
    let iter = 0x8000 | 4 << 9 | 3;
    tcd(
        &mut m,
        10,
        &[
            (SADDR, src),
            (ATTR, 0),
            (SOFF, 1),
            (NBYTES, 1),
            (DADDR, 0x4042_0000),
            (DOFF, 1),
            (CITER, iter),
            (BITER, iter),
            (CSR, 0),
        ],
    );
    tcd(&mut m, 4, &copy4(src + 9, 0x4042_0100)); // misaligned source
    w(&mut m, SSRT, Size::Byte, 10);
    assert_eq!(m.mem.peek(0x4042_0000, 1).unwrap(), [1], "channel 10 moved");
    assert_eq!(
        r(&mut m, ES, Size::Long),
        0x8000_0000 | 4 << 8 | 0x80,
        "SAE"
    );
    assert_eq!(get(&mut m, 4, CSR), 0, "START cleared, not DONE");
    assert_eq!(m.mem.peek(0x4042_0100, 4).unwrap(), [0; 4]);
    w(&mut m, CERR, Size::Byte, 4);
    tcd(&mut m, 4, &[(SADDR, src + 8)]);
    w(&mut m, SSRT, Size::Byte, 10); // a new start through the link
    assert_eq!(m.mem.peek(0x4042_0100, 4).unwrap(), [9, 10, 11, 12]);
    assert_eq!(get(&mut m, 4, CSR), DONE);
    assert_eq!(r(&mut m, ES, Size::Long) & 0x8000_0000, 0);
    no_halt(&mut m);
}

#[test]
fn mutual_minor_links_end_with_the_major_loop() {
    // channels 8 and 9 link to each other after every minor loop but the last: 8, 9, 8, 9, 8, then the chain ends
    let (mut m, _, _) = board(1);
    let src = 0x4041_0000;
    m.mem.load(src, &(1..=16).collect::<Vec<u8>>()).unwrap();
    for (ch, other, at) in [(8, 9u32, 0), (9, 8, 8)] {
        let iter = 0x8000 | other << 9 | 3;
        tcd(
            &mut m,
            ch,
            &[
                (SADDR, src + at),
                (ATTR, 0),
                (SOFF, 1),
                (NBYTES, 1),
                (DADDR, 0x4042_0000 + 0x100 * ch),
                (DOFF, 1),
                (CITER, iter),
                (BITER, iter),
                (CSR, 0),
            ],
        );
    }
    w(&mut m, SSRT, Size::Byte, 8);
    assert_eq!(m.mem.peek(0x4042_0800, 4).unwrap(), [1, 2, 3, 0]);
    assert_eq!(m.mem.peek(0x4042_0900, 3).unwrap(), [9, 10, 0]);
    assert_eq!(get(&mut m, 8, CSR), DONE);
    assert_eq!(get(&mut m, 8, CITER), 0x8000 | 9 << 9 | 3, "reloaded");
    assert_eq!(get(&mut m, 9, CITER), 0x8000 | 8 << 9 | 1);
    assert_eq!(get(&mut m, 9, CSR), 0);
    no_halt(&mut m);
}

#[test]
fn an_endless_link_chain_from_a_request_halts() {
    // the 1 << 20 backstop on the service path: a served channel links to one that links to itself forever
    let (mut m, _, port) = board(20);
    m.mem.load(0x4041_0000, &[1]).unwrap();
    tcd(
        &mut m,
        20,
        &[
            (SADDR, PORT + 0x100),
            (ATTR, 0),
            (SOFF, 0),
            (NBYTES, 1),
            (DADDR, 0x4042_0000),
            (DOFF, 0),
            (CITER, 1),
            (BITER, 1),
            (CSR, MAJORELINK | 5 << 8),
        ],
    );
    tcd(
        &mut m,
        5,
        &[
            (SADDR, 0x4041_0000),
            (ATTR, 0),
            (SOFF, 0),
            (NBYTES, 1),
            (DADDR, 0x4042_0100),
            (DOFF, 0),
            (CITER, 1),
            (BITER, 1),
            (CSR, MAJORELINK | 5 << 8),
        ],
    );
    w(&mut m, SERQ, Size::Byte, 20);
    grant(&mut m, port, 1);
    let r = pending_halt(&mut m);
    assert_eq!(r.kind, Stop::Unmodelled, "{r:?}");
    assert!(
        r.detail.contains("eDMA channel 5") && r.detail.contains("1048576 minor loops"),
        "{r:?}"
    );
    assert_eq!(get(&mut m, 5, CSR) & START, 0, "START cleared");
}

#[test]
fn a_refused_chain_clears_start_on_the_queued_channels() {
    // channel 10's load (with START) and its major link to channel 3 queue both; the loaded TCD cannot be run
    // (NBYTES 0): the start is refused, and neither channel is left with START
    let (mut m, _, _) = board(1);
    let (src, sga) = (0x4041_0000, 0x4040_0000);
    m.mem.load(src, &[1, 2, 3, 4]).unwrap();
    let mut loaded = copy4(src, 0x4042_0200);
    loaded.extend([(NBYTES, 0), (DLAST, 0), (CSR, START)]);
    m.mem.load(sga, &tcd_image(&loaded)).unwrap();
    tcd(&mut m, 3, &copy4(src, 0x4042_0300));
    let mut a = copy4(src, 0x4042_0100);
    a.push((DLAST, sga));
    tcd(&mut m, 10, &a);
    let d = refused(
        &mut m,
        tcd_addr(10, CSR),
        Size::Word,
        Some(START | ESG | MAJORELINK | 3 << 8),
    );
    assert!(
        d.contains("eDMA channel 10") && d.contains("NBYTES 0"),
        "{d}"
    );
    assert_eq!(
        m.mem.peek(0x4042_0100, 4).unwrap(),
        [1, 2, 3, 4],
        "the first ran"
    );
    assert_eq!(get(&mut m, 10, CSR), DONE, "the loaded CSR, START cleared");
    assert_eq!(get(&mut m, 3, CSR), 0, "channel 3: START cleared, not run");
    assert_eq!(m.mem.peek(0x4042_0300, 4).unwrap(), [0; 4]);
}

#[test]
fn minor_loop_link_starts_the_linked_channel() {
    // RM 19.6.6: a link after every minor loop but the last
    let (mut m, _, _) = board(1);
    let src = 0x4041_0000;
    m.mem.load(src, &(1..=16).collect::<Vec<u8>>()).unwrap();
    let iter = 0x8000 | 4 << 9 | 3; // E_LINK, LINKCH 4, count 3
    tcd(
        &mut m,
        10,
        &[
            (SADDR, src),
            (ATTR, 0),
            (SOFF, 1),
            (NBYTES, 1),
            (DADDR, 0x4042_0000),
            (DOFF, 1),
            (CITER, iter),
            (BITER, iter),
            (CSR, 0),
        ],
    );
    tcd(
        &mut m,
        4,
        &[
            (SADDR, src + 8),
            (ATTR, 0),
            (SOFF, 1),
            (NBYTES, 1),
            (DADDR, 0x4042_0100),
            (DOFF, 1),
            (CITER, 10),
            (BITER, 10),
            (CSR, 0),
        ],
    );
    w(&mut m, SSRT, Size::Byte, 10);
    assert_eq!(get(&mut m, 10, CITER), 0x8000 | 4 << 9 | 2, "LINKCH kept");
    assert_eq!(get(&mut m, 4, CITER), 9, "the linked channel ran");
    assert_eq!(get(&mut m, 4, CSR), 0, "its START cleared");
    w(&mut m, SSRT, Size::Byte, 10);
    assert_eq!(get(&mut m, 4, CITER), 8);
    w(&mut m, SSRT, Size::Byte, 10); // the last minor loop: no link
    assert_eq!(get(&mut m, 10, CITER), iter, "reloaded from BITER");
    assert_eq!(get(&mut m, 10, CSR), DONE);
    assert_eq!(get(&mut m, 4, CITER), 8);
    assert_eq!(m.mem.peek(0x4042_0000, 4).unwrap(), [1, 2, 3, 0]);
    assert_eq!(m.mem.peek(0x4042_0100, 3).unwrap(), [9, 10, 0]);
    no_halt(&mut m);
}

#[test]
fn major_loop_link_starts_the_linked_channel() {
    // a peripheral-paced channel: its link runs within the same service
    let (mut m, _, port) = board(20);
    with_port(&mut m, port, |p| p.mem[0x100..0x102].copy_from_slice(b"ab"));
    let src = 0x4041_0000;
    m.mem.load(src, b"xyz").unwrap();
    tcd(
        &mut m,
        20,
        &[
            (SADDR, PORT + 0x100),
            (ATTR, 0),
            (SOFF, 1),
            (NBYTES, 1),
            (SLAST, -2i32 as u32),
            (DADDR, 0x4042_0000),
            (DOFF, 1),
            (CITER, 2),
            (BITER, 2),
            (DLAST, -2i32 as u32),
            (CSR, MAJORELINK | 4 << 8),
        ],
    );
    tcd(
        &mut m,
        4,
        &[
            (SADDR, src),
            (ATTR, 0),
            (SOFF, 1),
            (NBYTES, 1),
            (DADDR, 0x4042_0100),
            (DOFF, 1),
            (CITER, 5),
            (BITER, 5),
            (CSR, 0),
        ],
    );
    w(&mut m, SERQ, Size::Byte, 20);
    grant(&mut m, port, 1);
    assert_eq!(get(&mut m, 20, CITER), 1);
    assert_eq!(
        get(&mut m, 4, CITER),
        5,
        "no link before the major loop completes"
    );
    grant(&mut m, port, 1);
    assert_eq!(get(&mut m, 20, CSR), MAJORELINK | 4 << 8 | DONE);
    assert_eq!(get(&mut m, 4, CITER), 4, "linked at the major loop's end");
    assert_eq!(m.mem.peek(0x4042_0000, 2).unwrap(), b"ab");
    assert_eq!(m.mem.peek(0x4042_0100, 2).unwrap(), [b'x', 0]);
    no_halt(&mut m);
}

#[test]
fn self_linked_channel_runs_all_minor_loops() {
    // The shape of a software channel 30: 16-byte reads inside a 4 MiB SMOD ring, 32-bit writes, CITER = BITER =
    // 0xBC11 (E_LINK to itself, count 17), DLAST_SGA the next TCD in RAM and CSR START | E_SG, written by the
    // program. The source starts 256 bytes before the ring's end, so the reads wrap to its start. Each TCD of the
    // chain runs its 17 minor loops within the CSR write; the last one waits.
    const TCD30: u32 = EDMA + 0x1000 + 30 * 32;
    let ring = 0x4040_0000; // 0-modulo 4 MiB
    let src = ring + 0x3F_FF00;
    let dst = 0x4090_0000;
    let (sg1, sg2) = (0x40A0_0000, 0x40A0_0020);
    let snippet = format!(
        "lea {TCD30:#x},%a0
        move.l #{src:#x},%d0
        move.l %d0,(%a0)
        move.l #0xB4020010,%d0
        move.l %d0,(4,%a0)
        moveq #0x10,%d0
        move.l %d0,(8,%a0)
        clr.l (12,%a0)
        move.l #{dst:#x},%d0
        move.l %d0,(16,%a0)
        move.l #0xBC110004,%d0
        move.l %d0,(20,%a0)
        move.l #{sg1:#x},%d0
        move.l %d0,(24,%a0)
        move.w #0xBC11,%d0
        move.w %d0,(28,%a0)
        moveq #0x11,%d0
        move.w %d0,(30,%a0)
        end: nop"
    );
    let (mut m, l) = common::prepare(&snippet, &["end"]);
    m.add_bus_master(EDMA, 0x2000, Box::new(Edma::new()));
    let data: Vec<u8> = (0..2 * 17 * 16u32).map(|i| (i * 7 + 3) as u8).collect();
    m.mem.load(src, &data[..256]).unwrap();
    m.mem.load(ring, &data[256..]).unwrap();
    m.mem.load(ring + 0x40_0000, &[0xEE; 64]).unwrap(); // past the ring: never read
    let shape = |saddr: u32, daddr: u32, sga: u32| {
        tcd_image(&[
            (SADDR, saddr),
            (ATTR, 0xB402),
            (SOFF, 0x10),
            (NBYTES, 0x10),
            (SLAST, 0),
            (DADDR, daddr),
            (CITER, 0xBC11),
            (DOFF, 4),
            (DLAST, sga),
            (BITER, 0xBC11),
            (CSR, START | ESG),
        ])
    };
    m.mem
        .load(sg1, &shape(ring + 16, dst + 17 * 16, sg2))
        .unwrap();
    let last = copy4(0x4041_0000, 0x4042_0000);
    m.mem.load(sg2, &tcd_image(&last)).unwrap();
    let stop = m.run(common::until(l[0]));
    assert_eq!(stop.kind, Stop::UntilPc, "{stop:?}");
    assert_eq!(
        m.mem.peek(dst, data.len() as u32).unwrap(),
        data,
        "2 x 17 minor loops"
    );
    assert_eq!(
        m.mem.peek(dst + data.len() as u32, 16).unwrap(),
        [0; 16],
        "no more"
    );
    for (f, v) in &last {
        assert_eq!(get(&mut m, 30, *f), *v, "{f:?}: the last TCD, loaded");
    }
    assert_eq!(
        get(&mut m, 30, CSR),
        DONE,
        "DONE set onto the loaded CSR (measured on an MCF54415)"
    );
    assert_eq!(r(&mut m, ES, Size::Long), 0);
    no_halt(&mut m);
}

#[test]
fn an_endless_link_chain_is_refused() {
    // a channel whose major loop links to itself starts again forever: a backstop of 1 << 20 minor loops
    let (mut m, _, _) = board(1);
    m.mem.load(0x4041_0000, &[1]).unwrap();
    tcd(
        &mut m,
        5,
        &[
            (SADDR, 0x4041_0000),
            (ATTR, 0),
            (SOFF, 0),
            (NBYTES, 1),
            (DADDR, 0x4042_0000),
            (DOFF, 0),
            (CITER, 1),
            (BITER, 1),
        ],
    );
    let d = refused(
        &mut m,
        tcd_addr(5, CSR),
        Size::Word,
        Some(MAJORELINK | 5 << 8 | START),
    );
    assert!(
        d.contains("eDMA channel 5") && d.contains("1048576 minor loops"),
        "{d}"
    );
}

#[test]
fn sixteen_byte_transfers_with_smod_ring() {
    // measured on an MCF54415: ATTR 0x0404 (100, 16-byte), NBYTES 16 moves 16 bytes, sets DONE, SADDR/DADDR +16
    let (mut m, _, _) = board(1);
    start_sized(&mut m, 0x0404, 16);
    let src: Vec<u8> = (1..=16).collect();
    assert_eq!(m.mem.peek(0x4030_0100, 16).unwrap(), src);
    assert_eq!(get(&mut m, 1, CSR), DONE);
    assert_eq!(get(&mut m, 1, SADDR), 0x4030_0010);
    assert_eq!(get(&mut m, 1, DADDR), 0x4030_0110);
    assert_eq!(r(&mut m, ES, Size::Long), 0);
    // the same size with an SMOD ring and two transfers per minor loop
    let (mut m, _, port) = board(1);
    let ring = 0x4010_0000; // a 64-byte ring: SMOD 6
    let dst = 0x4020_0000;
    let data: Vec<u8> = (0..64).collect();
    m.mem.load(ring, &data).unwrap();
    tcd(
        &mut m,
        12,
        &[
            (SADDR, ring + 32),
            (ATTR, 6 << 11 | 4 << 8 | 4),
            (SOFF, 16),
            (NBYTES, 32),
            (SLAST, 0),
            (DADDR, dst),
            (DOFF, 16),
            (CITER, 3),
            (BITER, 3),
            (DLAST, 0),
            (CSR, START),
        ],
    );
    w(&mut m, SSRT, Size::Byte, 12);
    w(&mut m, SSRT, Size::Byte, 12);
    let expect = [&data[32..], &data[..32], &data[32..]].concat();
    assert_eq!(
        m.mem.peek(dst, 96).unwrap(),
        expect,
        "SADDR wrapped in the ring"
    );
    assert_eq!(get(&mut m, 12, SADDR), ring, "wrapped to the ring's start");
    assert_eq!(get(&mut m, 12, DADDR), dst + 96);
    assert_eq!(get(&mut m, 12, CSR), DONE);
    // a 16-byte transfer to a device is a burst of four longword accesses
    with_port(&mut m, port, |p| p.budget = 100);
    tcd(
        &mut m,
        12,
        &[
            (SADDR, ring),
            (ATTR, 4 << 8 | 4),
            (DADDR, PORT + 0x100),
            (NBYTES, 16),
            (CITER, 1),
            (BITER, 1),
            (CSR, START),
        ],
    );
    assert_eq!(m.device::<Port>(port).mem[0x100..0x110], data[..16]);
    assert_eq!(m.device::<Port>(port).budget, 96, "four accesses");
    with_port(&mut m, port, |p| p.budget = 0);
    // addresses must be 0-modulo 16 (RM 19.4.2), the errors as for the smaller sizes
    tcd(&mut m, 12, &[(SADDR, ring + 8), (DADDR, dst), (CSR, START)]);
    assert_eq!(
        r(&mut m, ES, Size::Long),
        0x8000_0000 | 12 << 8 | 0x80,
        "SAE"
    );
    w(&mut m, CERR, Size::Byte, 12);
    tcd(&mut m, 12, &[(SADDR, ring), (DADDR, dst + 4), (CSR, START)]);
    assert_eq!(
        r(&mut m, ES, Size::Long),
        0x8000_0000 | 12 << 8 | 0x20,
        "DAE"
    );
    w(&mut m, CERR, Size::Byte, 12);
    no_halt(&mut m);
}
