//! The eDMA (RM ch. 19), driven through its registers, with peripheral requests from a test device that drives
//! DMA request lines.

use std::collections::VecDeque;

use mcf5441x_emu::soc::Edma;
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
}

impl Port {
    fn new(line: u8) -> Port {
        Port {
            line,
            mem: vec![0; 0x800],
            fifo: VecDeque::new(),
            out: Vec::new(),
            budget: 0,
            stuck: false,
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
    let cases: [(&[(Field, u32)], &str); 4] = [
        (&[(ATTR, 0x0101)], "transfer size"),
        (&[(CITER, 0x8001)], "channel link"),
        (&[(CITER, 0)], "CITER 0"),
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
    // ESG is refused when written (the TCD keeps its old CSR)
    let (mut m, _, _) = board(36);
    rx_channel(&mut m, 0x4010_1000);
    let d = refused(&mut m, tcd_addr(36, CSR), Size::Word, Some(ESG));
    assert!(d.contains("CSR"), "{d}");
    assert_eq!(get(&mut m, 36, CSR), INTMAJOR);
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
    // the control register, non-byte commands and the rest of the window
    refused(&mut m, EDMA, Size::Long, None);
    refused(&mut m, SERQ, Size::Long, Some(0x0101_0101));
    refused(&mut m, EDMA + 0x100, Size::Byte, None);
    refused(&mut m, EDMA + 0x1800, Size::Long, None);
    no_halt(&mut m);
}

#[test]
fn unmodelled_csr_bits_halt_when_written() {
    // START (bit 0) is modelled: see software_start_runs_one_minor_loop
    for bit in [ESG, MAJORELINK] {
        let (mut m, _, _) = board(36);
        let d = refused(&mut m, tcd_addr(36, CSR), Size::Word, Some(INTMAJOR | bit));
        assert!(d.contains("eDMA") && d.contains("CSR"), "{bit:#x}: {d}");
        assert_eq!(get(&mut m, 36, CSR), 0, "{bit:#x}: not stored");
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
fn mixed_transfer_sizes_halt() {
    for attr in [0x0200, 0x0002, 0x0101] {
        // 32 -> 8, 8 -> 32, 16-bit
        let (mut m, _, port) = board(54);
        tcd(
            &mut m,
            54,
            &[
                (SADDR, 0x4010_0000),
                (SOFF, 4),
                (ATTR, attr),
                (NBYTES, 8),
                (DADDR, 0x4010_0100),
                (DOFF, 4),
                (CITER, 1),
                (BITER, 1),
            ],
        );
        w(&mut m, SERQ, Size::Byte, 54);
        grant(&mut m, port, 1);
        let r = pending_halt(&mut m);
        assert_eq!(r.kind, Stop::Unmodelled, "{attr:#x}: {r:?}");
        assert!(r.detail.contains("transfer size"), "{attr:#x}: {r:?}");
    }
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
    // the aligned control case, measured on an MCF54415: the data moves and DONE is set; START clears
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
