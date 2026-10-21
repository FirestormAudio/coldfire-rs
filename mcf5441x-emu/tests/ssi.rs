//! The SSI (RM ch. 35): SSI1 with its transmit request on eDMA channel 54 and its receive request on channel 52,
//! a frame every 2750 ticks, driven through its registers and the eDMA.

use mcf5441x_emu::soc::{Edma, Intc, Ssi, SsiConfig};
use mcf5441x_emu::{Device, DeviceId, Fault, Machine, Size, Stop};

const SSI: u32 = 0xFC0C_8000;
const INTC1: u32 = 0xFC04_C000;
const EDMA: u32 = 0xFC04_4000;
const RAM: u32 = 0x4000_0000;

const TX0: u32 = 0x00;
const TX1: u32 = 0x04;
const RX0: u32 = 0x08;
const CR: u32 = 0x10;
const ISR: u32 = 0x14;
const IER: u32 = 0x18;
const TCR: u32 = 0x1C;
const RCR: u32 = 0x20;
const CCR: u32 = 0x24;
const FCSR: u32 = 0x2C;
const ACR: u32 = 0x38;
const TMASK: u32 = 0x48;
const ACCDIS: u32 = 0x58;

const TDMAE: u32 = 0x0010_0000;
const RDMAE: u32 = 0x0040_0000;
const TFRC: u32 = 0x0080_0000;
const TUE0: u32 = 0x100;
/// SYN | I2S master | TE | SSI_EN.
const RUN: u32 = 0x33;
const FRAME: u64 = 2750;

const TX_CH: u8 = 54;
const RX_CH: u8 = 52;

const SERQ: u32 = EDMA + 0x18;
const CINT: u32 = EDMA + 0x1C;

/// What SSI1 sends of a FIFO word: bits 23:0, a signed 24-bit sample (the bits above are dropped).
fn s24(w: u32) -> i32 {
    ((w << 8) as i32) >> 8
}

struct Board {
    m: Machine,
    ssi: DeviceId,
    edma: DeviceId,
}

/// RAM, INTC1, the eDMA and SSI1 (TX0 on channel 54, RX0 on 52, a frame every 2750 ticks).
fn board() -> Board {
    let mut m = Machine::new();
    m.mem.map_ram(RAM, 0x0100_0000);
    let ([_, intc1, _], _) = Intc::chip();
    m.add_interrupt_controller(INTC1, 0x4000, Box::new(intc1));
    let edma = m.add_bus_master(EDMA, 0x2000, Box::new(Edma::new()));
    let ssi = m.add_device(
        SSI,
        0x4000,
        Box::new(Ssi::new(SsiConfig {
            tx_dma: TX_CH,
            rx_dma: RX_CH,
            ticks_per_frame: FRAME,
        })),
    );
    Board { m, ssi, edma }
}

impl Board {
    fn r(&mut self, off: u32) -> u32 {
        let m = &mut self.m;
        m.access(SSI + off, Size::Long, None)
            .unwrap_or_else(|f| panic!("read +{off:#x}: {f:?} {:?}", m.take_refusal()))
    }

    fn w(&mut self, off: u32, value: u32) {
        let m = &mut self.m;
        m.access(SSI + off, Size::Long, Some(value))
            .unwrap_or_else(|f| panic!("write +{off:#x}: {f:?} {:?}", m.take_refusal()));
    }

    /// A write the SSI refuses: returns the detail.
    fn refused(&mut self, off: u32, size: Size, value: Option<u32>) -> String {
        assert_eq!(
            self.m.access(SSI + off, size, value),
            Err(Fault::Device(Stop::Unmodelled)),
            "+{off:#x} {size:?} {value:x?}"
        );
        self.m.take_refusal().expect("a refusal").detail
    }

    fn host(&mut self, addr: u32, size: Size, value: u32) {
        self.m.access(addr, size, Some(value)).unwrap();
    }

    fn at(&mut self, tick: u64) {
        assert_eq!(self.m.idle_until(tick), None);
    }

    fn ssi(&self) -> &Ssi {
        self.m.device::<Ssi>(self.ssi)
    }

    fn capture(&self) -> Vec<(u64, Vec<i32>)> {
        self.ssi().capture().to_vec()
    }

    /// The transmit sequence a driver writes, TX only, then `prefill` zero words.
    fn configure(&mut self, prefill: usize, ier: u32) {
        self.w(CR, 0x30);
        self.w(TCR, 0xE8);
        self.w(CCR, 0x16101);
        self.w(FCSR, 0x22);
        self.w(IER, ier);
        self.w(CR, 0x31);
        for _ in 0..prefill {
            self.w(TX0, 0);
        }
    }

    /// Write channel `ch`'s TCD: (offset, size, value).
    fn tcd(&mut self, ch: u8, fields: &[(u32, Size, u32)]) {
        let t = EDMA + 0x1000 + 32 * u32::from(ch);
        for &(off, size, v) in fields {
            self.host(t + off, size, v);
        }
    }

    fn tcd_read(&mut self, ch: u8, off: u32, size: Size) -> u32 {
        let t = EDMA + 0x1000 + 32 * u32::from(ch);
        self.m.access(t + off, size, None).unwrap()
    }
}

#[test]
fn only_bits_23_to_0_of_a_word_are_sent() {
    // a left-aligned word (sample << 8) loses its top byte: measured on an MCF54415, a 16-bit sample << 16 played
    // as its low byte
    assert_eq!(
        (
            s24(0x0012_3456),
            s24(0x00FF_FFFF),
            s24(0x7F00_0000),
            s24(0x1234 << 16)
        ),
        (0x123456, -1, 0, 0x340000)
    );
    // and the model sends them so (24-bit words, CCR WL 11)
    let mut b = board();
    b.configure(0, 0);
    for w in [0x0012_3456, 0x00FF_FFFF, 0x7F00_0000, 0x1234 << 16] {
        b.w(TX0, w);
    }
    b.w(CR, RUN);
    b.at(2 * FRAME);
    assert_eq!(
        b.capture(),
        [(FRAME, vec![0x123456, -1]), (2 * FRAME, vec![0, 0x340000])]
    );
}

#[test]
fn frames_pop_two_words_every_2750_cycles() {
    let mut b = board();
    b.configure(0, 0);
    // bits 23:0, sign-extended as a driver stores them
    for w in [0x0000_0001, 0xFFFF_FFFF, 0x007F_FFFF, 0xFF80_0000] {
        b.w(TX0, w);
    }
    b.w(CR, RUN);
    assert_eq!(b.ssi().next_event(), Some(FRAME));
    b.at(FRAME - 1);
    assert_eq!(b.capture(), []);
    b.at(2 * FRAME);
    assert_eq!(
        b.capture(),
        [
            (FRAME, vec![1, -1]),
            (2 * FRAME, vec![0x7F_FFFF, -0x80_0000])
        ]
    );
    assert_eq!(b.ssi().counters().frames, 2);
}

#[test]
fn empty_fifo_underruns_with_silence_and_tue() {
    // guess: the frame sends silence and empties the FIFO, where RM Table 35-8 re-transmits the previous data on
    // an underrun. Not measured.
    let mut b = board();
    b.configure(1, 0);
    b.w(CR, RUN);
    b.at(FRAME);
    assert_eq!(b.capture(), [(FRAME, vec![0, 0])]);
    assert_eq!(b.ssi().counters().underruns, 1);
    assert_ne!(b.r(ISR) & TUE0, 0);
    assert_eq!(b.r(FCSR) >> 8 & 0xF, 0, "the odd word went with the frame");
    b.w(ISR, TUE0);
    assert_eq!(b.r(ISR) & TUE0, 0);
}

#[test]
fn fcsr_count_and_tfe_follow_the_watermark() {
    let mut b = board();
    b.configure(14, 0);
    assert_eq!(b.r(FCSR), 0x22 | 14 << 8);
    assert_eq!(b.r(ISR) & 1, 0, "1 free < watermark 2");
    b.w(CR, RUN);
    b.at(FRAME);
    assert_ne!(b.r(ISR) & 1, 0, "3 free");
}

#[test]
fn write_to_a_full_fifo_is_dropped() {
    let mut b = board();
    b.configure(15, 0);
    b.w(TX0, 5);
    assert_eq!(b.ssi().counters().overruns, 1);
    assert_eq!(b.r(FCSR) >> 8 & 0xF, 15);
}

#[test]
fn unmodelled_configuration_halts_when_te_is_set() {
    // what the model doesn't cover: TX watermark 0, the second FIFO, I2S slave (clocks from outside), CPU interrupts
    for (reg, val, what) in [
        (FCSR, 0x20, "FCSR"),
        (TCR, 0xE8 | 0x100, "TCR"),
        (CR, 0x10 | 0b10 << 5, "CR"),
        (IER, TDMAE | 1 << 19, "IER"),
    ] {
        let mut b = board();
        b.configure(14, TDMAE);
        let cr = if reg == CR { val } else { 0x30 } | 0x03;
        if reg != CR {
            b.w(reg, val);
        }
        let d = b.refused(CR, Size::Long, Some(cr));
        assert!(
            d.contains("SSI") && d.contains("configuration") && d.contains(what),
            "{reg:#x} {val:#x}: {d}"
        );
        // the refused write is not stored: the SSI is not running
        assert_eq!(b.r(CR), 0x31, "{reg:#x}");
        assert_eq!(b.ssi().next_event(), None, "{reg:#x}");
    }
}

#[test]
fn tx_chain_plays_the_ring_and_interrupts_at_each_half() {
    // channel 54 moving a 64-frame ring into TX0, one frame (8 bytes) per request: SADDR is ring + 256 at the half
    // interrupt and ring at the major
    let mut b = board();
    let ring = RAM + 0x10_2000;
    let words: Vec<u32> = (0..128u32)
        .map(|i| ((i * 0x10101) & 0xFF_FFFF) << 8)
        .collect();
    let bytes: Vec<u8> = words.iter().flat_map(|w| w.to_be_bytes()).collect();
    b.m.mem.load(ring, &bytes).unwrap();
    b.tcd(
        TX_CH,
        &[
            (0x00, Size::Long, ring),           // SADDR
            (0x06, Size::Word, 4),              // SOFF
            (0x04, Size::Word, 0x0202),         // ATTR: 32-bit
            (0x08, Size::Long, 8),              // NBYTES
            (0x0C, Size::Long, -512i32 as u32), // SLAST
            (0x10, Size::Long, SSI + TX0),      // DADDR
            (0x16, Size::Word, 0),              // DOFF
            (0x14, Size::Word, 64),             // CITER
            (0x18, Size::Long, 0),              // DLAST
            (0x1C, Size::Word, 64),             // BITER
            (0x1E, Size::Word, 0x6),            // CSR: INTMAJOR | INTHALF
        ],
    );
    b.configure(14, TDMAE);
    b.host(SERQ, Size::Byte, u32::from(TX_CH));
    assert_eq!(
        b.tcd_read(TX_CH, 0x14, Size::Word),
        64,
        "1 free < watermark 2: no request yet"
    );
    b.w(CR, RUN);
    let mut halves = Vec::new();
    for k in 1..72u64 {
        b.at(FRAME * k);
        if b.m.mem.irq_lines().asserted(1) & 1 << 46 != 0 {
            halves.push((k, b.tcd_read(TX_CH, 0x00, Size::Long) - ring));
            b.host(CINT, Size::Byte, u32::from(TX_CH));
        }
    }
    let played: Vec<Vec<i32>> = b.capture().into_iter().map(|(_, w)| w).collect();
    assert_eq!(played[..7], vec![vec![0, 0]; 7], "the 14 pre-filled words");
    let expect: Vec<Vec<i32>> = (0..64)
        .map(|i| vec![s24(words[2 * i]), s24(words[2 * i + 1])])
        .collect();
    assert_eq!(played[7..71], expect[..]);
    assert_eq!(halves, [(32, 256), (64, 0)]);
    let c = b.ssi().counters();
    assert_eq!((c.underruns, c.overruns), (0, 0));
    // the SSI raises no interrupt of its own; the eDMA lists the route of channel 54 (INTC1 source 46)
    let mut out = Vec::new();
    assert!(b.ssi().wake_sources(&mut out));
    assert_eq!(out, []);
    assert!(b.m.device::<Edma>(b.edma).wake_sources(&mut out));
    assert_eq!(out, [(1, 46)]);
}

#[test]
fn configuration_written_while_running_is_checked() {
    let mut b = board();
    b.configure(14, TDMAE);
    b.w(CR, RUN);
    let d = b.refused(TCR, Size::Long, Some(0xE8 | 0x100)); // TFEN1: the second FIFO
    assert!(d.contains("SSI") && d.contains("TCR"), "{d}");
    assert_eq!(b.r(TCR), 0xE8, "the refused write is not stored");
}

#[test]
fn hal_config_is_accepted() {
    // I2S master in network mode, 16-bit words, 4 slots, FIFO 0 both ways, DMA both ways
    let mut b = board();
    b.w(CR, 0x10 | 0x08); // SYN | NET
    b.w(TCR, 0xE8);
    b.w(RCR, 0x480);
    b.w(CCR, 7 << 13 | 3 << 8 | 1);
    b.w(FCSR, 0x22);
    b.w(IER, TDMAE | RDMAE);
    b.w(CR, 0x10 | 0x08 | 0x07);
    assert_eq!(b.ssi().next_event(), Some(FRAME));
    b.at(FRAME);
    assert_eq!(b.capture(), [(FRAME, vec![0; 4])]);
}

/// Channel 52: RX0 into a ring at RAM + 0x101000, one 8-byte frame per request, wrapping after 64.
fn rx_tcd(b: &mut Board) {
    b.tcd(
        RX_CH,
        &[
            (0x00, Size::Long, SSI + RX0),           // SADDR
            (0x04, Size::Word, 0x0202),              // ATTR: 32-bit
            (0x06, Size::Word, 0),                   // SOFF
            (0x08, Size::Long, 8),                   // NBYTES
            (0x0C, Size::Long, 0),                   // SLAST
            (0x10, Size::Long, RAM + 0x10_1000),     // DADDR
            (0x14, Size::Word, 64),                  // CITER
            (0x16, Size::Word, 4),                   // DOFF
            (0x18, Size::Long, (-8i32 * 64) as u32), // DLAST
            (0x1C, Size::Word, 64),                  // BITER
            (0x1E, Size::Word, 0),                   // CSR
        ],
    );
    b.host(SERQ, Size::Byte, u32::from(RX_CH));
}

fn rx_run(b: &mut Board) {
    b.w(CR, 0x30);
    b.w(TCR, 0xE8);
    b.w(RCR, 0x480);
    b.w(CCR, 0x16101);
    b.w(FCSR, 0x22);
    b.w(IER, RDMAE);
    b.w(CR, 0x30 | 0x01 | 0x02 | 0x04);
}

#[test]
fn rx_frames_reach_the_dma_ring() {
    let mut b = board();
    b.m.device_mut::<Ssi>(b.ssi)
        .set_source(Box::new(|n| vec![n as i32, -(n as i32)]));
    rx_tcd(&mut b);
    rx_run(&mut b);
    for t in 1..=40 {
        b.at(t * FRAME);
    }
    let ring = b.m.mem.peek(RAM + 0x10_1000, 8 * 40).unwrap();
    let words: Vec<i32> = ring
        .chunks(4)
        .map(|c| i32::from_be_bytes(c.try_into().unwrap()))
        .collect();
    let got: Vec<(i32, i32)> = words.chunks(2).map(|p| (p[0], p[1])).collect();
    let expect: Vec<(i32, i32)> = (0..38).map(|n| (n, -n)).collect();
    assert_eq!(
        got[..38],
        expect[..],
        "in order (the last frames may sit in the FIFO)"
    );
    assert_eq!(b.ssi().counters().rx_overruns, 0);
}

#[test]
fn rx_overrun_counts() {
    let mut b = board();
    b.m.device_mut::<Ssi>(b.ssi)
        .set_source(Box::new(|_| vec![1, 2]));
    rx_run(&mut b); // no DMA channel: the RX FIFO fills
    for t in 1..20 {
        b.at(t * FRAME);
    }
    assert!(b.ssi().counters().rx_overruns > 0);
    assert_eq!(b.r(FCSR) >> 12 & 0xF, 14);
}

#[test]
fn underruns_before_the_dma_runs_are_not_dma_underruns() {
    // clocks started ahead of the stream (a codec's reset) underrun by design; only a running DMA failing to keep
    // up counts
    let mut b = board();
    b.configure(0, TDMAE);
    b.w(CR, RUN);
    b.at(FRAME);
    let c = b.ssi().counters();
    assert_eq!((c.underruns, c.dma_underruns), (1, 0));
}

#[test]
fn dma_underruns_count_while_the_channel_is_enabled() {
    // channel 54 enabled but moving one word per request with TFWM0 15 (a request only with the FIFO empty): each
    // frame finds one word for two slots
    let mut b = board();
    let src = RAM + 0x10_3000;
    b.tcd(
        TX_CH,
        &[
            (0x00, Size::Long, src),
            (0x04, Size::Word, 0x0202),
            (0x08, Size::Long, 4),
            (0x10, Size::Long, SSI + TX0),
            (0x14, Size::Word, 100),
            (0x1C, Size::Word, 100),
        ],
    );
    b.configure(0, TDMAE);
    b.w(FCSR, 0x2F);
    b.host(SERQ, Size::Byte, u32::from(TX_CH));
    assert_eq!(b.r(FCSR) >> 8 & 0xF, 1, "one word per request");
    b.w(CR, RUN);
    b.at(2 * FRAME);
    let c = b.ssi().counters();
    assert_eq!((c.underruns, c.dma_underruns), (2, 2));
}

#[test]
fn words_written_into_a_starved_transmitter_count_as_unaligned() {
    // the FIFO has no slot tags: after an underrun, the next word goes out in whichever slot comes next (RM 35.4.1)
    let mut b = board();
    b.configure(0, 0);
    b.w(CR, RUN);
    b.at(FRAME);
    b.w(TX0, 0);
    b.w(TX0, 0);
    assert_eq!(b.ssi().counters().unaligned_refills, 1);
}

#[test]
fn clearing_te_sets_tfrc_at_the_frame_end_and_restarts_aligned() {
    // TE cleared: the transmitter stops at the end of the frame (TFRC); set again, it starts at a frame boundary
    let mut b = board();
    b.configure(0, 0);
    b.w(CR, RUN);
    b.at(FRAME); // starved
    b.w(CR, 0x31); // TE off, SSI_EN on: the clocks keep running
    assert_eq!(b.r(ISR) & TFRC, 0);
    b.at(2 * FRAME);
    assert_ne!(b.r(ISR) & TFRC, 0);
    for _ in 0..14 {
        b.w(TX0, 0);
    }
    b.w(CR, RUN);
    assert_eq!(b.ssi().counters().unaligned_refills, 0);
    assert_eq!(b.ssi().next_event(), Some(2 * FRAME + FRAME));
}

#[test]
fn reset_values_follow_the_rm() {
    // RM Table 35-4, measured equal on an MCF54415: CR 0, ISR 0x3003 (TFE0, TFE1, TDE0, TDE1), IER 0x3003,
    // TCR 0x200, RCR 0x200, CCR 0x0004_0000, FCSR 0x0081_0081; ACR..ACCDIS 0
    let mut b = board();
    for (off, v) in [
        (CR, 0),
        (ISR, 0x3003),
        (IER, 0x3003),
        (TCR, 0x200),
        (RCR, 0x200),
        (CCR, 0x0004_0000),
        (FCSR, 0x0081_0081),
        (RX0, 0),
    ] {
        assert_eq!(b.r(off), v, "+{off:#x}");
    }
    for off in (ACR..=ACCDIS).step_by(4) {
        assert_eq!(b.r(off), 0, "+{off:#x}");
    }
    assert_eq!(b.m.dma_lines().asserted(), 0, "TDMAE and RDMAE are clear");
    // the reset IER enables interrupts the model doesn't have: TE is refused until IER is written
    b.w(CR, 0x30);
    b.w(TCR, 0xE8);
    b.w(CCR, 0x16101);
    let d = b.refused(CR, Size::Long, Some(RUN));
    assert!(d.contains("IER 0x3003"), "{d}");
    b.w(IER, TDMAE);
    b.w(CR, RUN);
    assert_eq!(b.ssi().next_event(), Some(FRAME));
    assert_eq!(
        b.m.dma_lines().asserted(),
        1 << TX_CH,
        "FCSR TFWM0 1, all free"
    );
}

#[test]
fn clearing_ssi_en_empties_the_fifos() {
    // RM 35.3.7: disabling the SSI resets its status bits and clears the Tx and Rx FIFOs
    let mut b = board();
    b.m.device_mut::<Ssi>(b.ssi)
        .set_source(Box::new(|_| vec![7, 8]));
    rx_run(&mut b);
    b.at(FRAME);
    b.w(TX0, 1);
    assert_eq!(b.r(FCSR) >> 8 & 0xFF, 0x21);
    assert_ne!(b.r(ISR) & TUE0, 0);
    b.w(CR, 0x30);
    assert_eq!(b.r(FCSR) >> 8 & 0xFF, 0);
    assert_eq!(b.r(ISR) & (TUE0 | TFRC), 0);
}

#[test]
fn unmodelled_registers_are_refused() {
    let mut b = board();
    for (off, value, what) in [
        (TX1, None, "TX1"),
        (TX1, Some(0), "TX1"),
        (0x0C, None, "RX1"),
        (RX0, Some(0), "RX0"),
        (ACR, Some(1), "AC97"),
        (TMASK, Some(1), "time slot"),
        (0x28, None, "+0x28"),
        (0x5C, Some(0), "+0x5c"),
    ] {
        let d = b.refused(off, Size::Long, value);
        assert!(d.contains("SSI") && d.contains(what), "+{off:#x}: {d}");
    }
    let d = b.refused(FCSR, Size::Byte, None);
    assert!(d.contains("SSI") && d.contains("Byte"), "{d}");
    b.w(ACR, 0);
    b.w(TMASK, 0);
}

#[test]
fn take_capture_drains_and_record_off_logs_nothing() {
    let mut b = board();
    b.configure(0, 0);
    for w in [1, 2, 3, 4] {
        b.w(TX0, w);
    }
    b.w(CR, RUN);
    b.at(FRAME);
    let taken = b.m.device_mut::<Ssi>(b.ssi).take_capture();
    assert_eq!(taken, [(FRAME, vec![1, 2])]);
    assert_eq!(b.capture(), []);
    b.m.device_mut::<Ssi>(b.ssi).record(false);
    b.at(2 * FRAME);
    assert_eq!(b.capture(), [], "not captured");
    assert_eq!(b.ssi().counters().frames, 2, "but the frame went out");
}
