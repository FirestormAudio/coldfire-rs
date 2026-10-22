//! The DSPI (RM ch. 40): DSPI0 with an SPI NOR flash on PCS1, driven through its registers the way a boot loader's
//! read routine drives it (up to 4 frames in flight, pop then push, CONT on all frames but the last, EOQ on it).

mod common;

use mcf5441x_emu::parts::SpiNor;
use mcf5441x_emu::soc::{Dspi, Edma, Frame};
use mcf5441x_emu::{DeviceId, Fault, Machine, Size, Stop};

const DSPI: u32 = 0xFC05_C000;
const RAM: u32 = 0x4000_0000;

const MCR: u32 = 0x00;
const TCR: u32 = 0x08;
const CTAR0: u32 = 0x0C;
const CTAR1: u32 = 0x10;
const SR: u32 = 0x2C;
const RSER: u32 = 0x30;
const PUSHR: u32 = 0x34;
const POPR: u32 = 0x38;
const TXFR0: u32 = 0x3C;
const RXFR0: u32 = 0x7C;

const CONT: u32 = 1 << 31;
const EOQ: u32 = 1 << 27;
const CTCNT: u32 = 1 << 26;
const PCS0: u32 = 1 << 16;
const PCS1: u32 = 1 << 17;
const PCS2: u32 = 1 << 18;

const TCF: u32 = 1 << 31;
const TXRXS: u32 = 1 << 30;
const EOQF: u32 = 1 << 28;
const TFFF: u32 = 1 << 25;
const RFOF: u32 = 1 << 19;
const RFDF: u32 = 1 << 17;

/// FMSZ 7: 8-bit frames.
const CTAR_8: u32 = 0x3801_0000;
/// FMSZ 15: 16-bit frames.
const CTAR_16: u32 = 0x7801_0000;
/// Master, PCSIS0 and PCSIS1 set (both selects idle high), clear both FIFOs, running.
const MCR_RUN: u32 = 0x8003_0C00;

const SIZE: u32 = 16 << 20;
const JEDEC_64K: [u8; 6] = [0x01, 0x20, 0x18, 0x4D, 0x01, 0x80];
const JEDEC_256K: [u8; 6] = [0x01, 0x20, 0x18, 0x4D, 0x00, 0x80];

struct Board {
    m: Machine,
    dspi: DeviceId,
}

/// RAM and DSPI0 with a NOR on PCS1, at reset.
fn bare(contents: &[u8], jedec: [u8; 6]) -> Board {
    let mut m = Machine::new();
    m.mem.map_ram(RAM, 0x0100_0000);
    let mut dspi = Dspi::new();
    dspi.attach(1, Box::new(SpiNor::new(SIZE, jedec, contents)));
    let dspi = m.add_device(DSPI, 0x4000, Box::new(dspi));
    Board { m, dspi }
}

/// As a boot loader leaves it: 8-bit frames through CTAR0, master, running.
fn board(contents: &[u8], jedec: [u8; 6]) -> Board {
    let mut b = bare(contents, jedec);
    b.w(CTAR0, CTAR_8);
    b.w(MCR, MCR_RUN);
    b
}

/// Deterministic bytes.
fn noise(n: usize) -> Vec<u8> {
    let mut x = 0x1234_5678u32;
    (0..n)
        .map(|_| {
            x = x.wrapping_mul(1_103_515_245).wrapping_add(12345);
            (x >> 16) as u8
        })
        .collect()
}

impl Board {
    fn r(&mut self, off: u32) -> u32 {
        let m = &mut self.m;
        m.access(DSPI + off, Size::Long, None)
            .unwrap_or_else(|f| panic!("read +{off:#x}: {f:?} {:?}", m.take_refusal()))
    }

    fn w(&mut self, off: u32, value: u32) {
        let m = &mut self.m;
        m.access(DSPI + off, Size::Long, Some(value))
            .unwrap_or_else(|f| panic!("write +{off:#x}: {f:?} {:?}", m.take_refusal()));
    }

    /// An access the DSPI refuses: returns the detail.
    fn refused(&mut self, off: u32, size: Size, value: Option<u32>) -> String {
        assert_eq!(
            self.m.access(DSPI + off, size, value),
            Err(Fault::Device(Stop::Unmodelled)),
            "+{off:#x} {size:?} {value:x?}"
        );
        self.m.take_refusal().expect("a refusal").detail
    }

    fn dspi(&self) -> &Dspi {
        self.m.device::<Dspi>(self.dspi)
    }

    fn nor(&self) -> &SpiNor {
        self.dspi().device::<SpiNor>(1)
    }

    fn ops(&self) -> Vec<u8> {
        self.nor().log().iter().map(|&(_, op, _)| op).collect()
    }

    /// The read routine's pattern: clear the FIFOs and EOQF, keep up to 4 frames in flight, pop then push, PCS1,
    /// CTCNT on the first frame, CONT on all but the last, EOQ on the last. Returns the `n` bytes after `header`.
    fn transfer(&mut self, header: &[u8], n: usize) -> Vec<u8> {
        let mcr = self.r(MCR);
        self.w(MCR, mcr | 0xC00);
        self.w(SR, EOQF);
        let total = header.len() + n;
        let frame = |i: usize| {
            PCS1 | u32::from(header.get(i).copied().unwrap_or(0))
                | if i == 0 { CTCNT } else { 0 }
                | if i == total - 1 { EOQ } else { CONT }
        };
        let mut pushed = 0;
        while pushed < total.min(4) {
            self.w(PUSHR, frame(pushed));
            pushed += 1;
        }
        let mut got = Vec::new();
        for i in 0..total {
            assert!(self.r(SR) >> 4 & 0xF > 0, "RXCTR before pop {i}");
            let b = self.r(POPR) as u8;
            if i >= header.len() {
                got.push(b);
            }
            if pushed < total {
                self.w(PUSHR, frame(pushed));
                pushed += 1;
            }
        }
        assert_ne!(self.r(SR) & EOQF, 0);
        self.w(SR, EOQF);
        assert_eq!(self.r(SR) & EOQF, 0);
        got
    }
}

#[test]
fn jedec_id() {
    let mut b = board(&[], JEDEC_64K);
    assert_eq!(b.transfer(&[0x9F], 6), JEDEC_64K);
    assert_eq!(b.ops(), [0x9F]);
}

#[test]
fn frames_of_16_bits_through_ctar1_shift_two_bytes_msb_first() {
    // PUSHR[CTAS] = 1 picks CTAR1; with FMSZ = 15 one frame moves two bytes, the first in the high byte
    let data: Vec<u8> = (0..16).collect();
    let mut b = board(&data, JEDEC_64K);
    b.w(CTAR1, CTAR_16);
    for (i, v) in [0x03, 0x00, 0x00, 0x04].into_iter().enumerate() {
        b.w(PUSHR, PCS1 | CONT | v | if i == 0 { CTCNT } else { 0 });
    }
    b.w(PUSHR, PCS1 | CONT | 1 << 28);
    b.w(PUSHR, PCS1 | EOQ);
    let got: Vec<u32> = (0..6).map(|_| b.r(POPR)).collect();
    assert_eq!(got[4..], [0x0405, 0x06]);
}

#[test]
fn read_returns_memory_at_the_address() {
    let data = noise(0x1000);
    let mut b = board(&data, JEDEC_64K);
    assert_eq!(
        b.transfer(&[0x03, 0x00, 0x08, 0x10], 300),
        data[0x810..0x810 + 300]
    );
    let last = *b.nor().log().last().unwrap();
    assert_eq!((last.1, last.2), (0x03, Some(0x810)));
    assert_eq!(b.nor().writes(), []);
}

#[test]
fn unmodelled_opcode_halts() {
    let mut b = board(&[], JEDEC_64K);
    let d = b.refused(PUSHR, Size::Long, Some(PCS1 | CTCNT | EOQ | 0x66)); // reset enable
    assert!(d.contains("SPI NOR") && d.contains("opcode 0x66"), "{d}");
}

#[test]
fn rx_fifo_overflow_drops_and_flags() {
    // a page program that never pops RX and paces on TXCTR: the FIFOs are 16 deep, and with MCR[ROOE] = 0 the
    // overflowing frame's data is dropped and RFOF set (RM 40.4.2.5.1)
    let data: Vec<u8> = (0..32).collect();
    let mut b = board(&data, JEDEC_64K);
    for v in [0x03, 0, 0, 0].into_iter().chain([0; 13]) {
        b.w(PUSHR, PCS1 | CONT | v);
    }
    let sr = b.r(SR);
    assert_ne!(sr & RFOF, 0);
    assert_ne!(sr & RFDF, 0);
    // guess: RXCTR is 4 bits wide, and reads 15 with 16 entries held
    assert_eq!(sr >> 4 & 0xF, 15);
    let got: Vec<u8> = (0..16).map(|_| b.r(POPR) as u8).collect();
    assert_eq!(got[..4], [0xFF; 4]);
    assert_eq!(got[4..], (0..12).collect::<Vec<u8>>());
    assert_eq!(b.r(SR) & RFDF, 0);
    b.w(SR, RFOF);
    assert_eq!(b.r(SR) & RFOF, 0);
}

#[test]
fn other_chip_select_with_nothing_attached_reads_ones() {
    let mut b = board(&[], JEDEC_64K);
    b.w(PUSHR, PCS0 | 0x9F);
    assert_eq!(b.r(POPR), 0xFF);
    b.w(CTAR1, CTAR_16);
    b.w(PUSHR, PCS2 | 1 << 28 | 0x9F00);
    assert_eq!(b.r(POPR), 0xFFFF);
    assert!(b.nor().log().is_empty());
}

#[test]
fn program_and_erase_are_logged_and_honour_wren() {
    let mut b = board(&[], JEDEC_64K);
    b.transfer(&[0x02, 0x04, 0x00, 0x00, 0x12], 0); // PP without WREN: ignored
    assert_eq!(b.nor().memory()[0x40000], 0xFF);
    assert_eq!(b.nor().writes(), []);
    b.transfer(&[0x06], 0);
    b.transfer(&[0x02, 0x04, 0x00, 0x00, 0x12, 0x34], 0);
    assert_eq!(b.nor().memory()[0x40000..0x40002], [0x12, 0x34]);
    b.transfer(&[0x06], 0);
    b.transfer(&[0xD8, 0x04, 0x00, 0x10], 0); // 64 KiB sector erase
    assert_eq!(b.nor().memory()[0x40000], 0xFF);
    let writes: Vec<(u8, u32)> = b.nor().writes().iter().map(|&(_, op, a)| (op, a)).collect();
    assert_eq!(writes, [(0x02, 0x40000), (0xD8, 0x40010)]);
}

#[test]
fn status_register_reads_idle() {
    let mut b = board(&[], JEDEC_64K);
    assert_eq!(b.transfer(&[0x05], 1), [0x00]);
    b.transfer(&[0x06], 0);
    assert_eq!(b.transfer(&[0x05], 1), [0x02]); // WEL
}

#[test]
fn push_while_halted_halts() {
    let mut b = bare(&[], JEDEC_64K); // MCR at reset: HALT
    let d = b.refused(PUSHR, Size::Long, Some(PCS1 | EOQ | 0x9F));
    assert!(d.contains("DSPI") && d.contains("halted"), "{d}");
    assert!(b.dspi().frames().is_empty());
}

#[test]
fn narrow_register_read_halts() {
    // a compiler may turn `r32(SR) & EOQF` into `btst #4,(SR)`, a byte read: the model refuses it rather than
    // answer with the wrong byte (the chip itself would answer)
    let mut b = board(&[], JEDEC_64K);
    let d = b.refused(SR, Size::Byte, None);
    assert!(d.contains("DSPI") && d.contains("1-byte"), "{d}");
    let d = b.refused(PUSHR + 2, Size::Word, Some(0x9F));
    assert!(d.contains("2-byte"), "{d}");
}

#[test]
fn sector_erase_size_follows_the_id() {
    // S25FL-S ID byte 4: 0x00 = uniform 256 KiB sectors, 0x01 = 64 KiB
    for (jedec, size) in [(JEDEC_256K, 0x40000usize), (JEDEC_64K, 0x10000)] {
        let mut b = board(&vec![0; 0x10_0000], jedec);
        b.transfer(&[0x06], 0);
        b.transfer(&[0xD8, 0x04, 0x00, 0x10], 0);
        let m = b.nor().memory();
        assert!(
            m[0x40000..0x40000 + size].iter().all(|&x| x == 0xFF),
            "{size:#x}"
        );
        assert_eq!((m[0x40000 + size], m[0x3FFFF]), (0, 0), "{size:#x}");
    }
}

#[test]
fn a_frame_with_no_chip_select_reaches_no_device() {
    // a device selected by a GPIO: no PCS bit; nothing is modelled there, so SIN reads all ones
    let mut b = board(&[], JEDEC_64K);
    b.w(PUSHR, 0x9F | EOQ);
    assert_eq!(b.r(POPR), 0xFF);
    assert!(b.nor().log().is_empty());
}

#[test]
fn a_pcs_idling_low_selects_its_device_without_a_pcs_bit() {
    // PCSIS1 = 0: PCS1's inactive level is low, the active-low part's selected level, so even a frame naming no
    // chip select reaches it
    let mut b = board(&[], JEDEC_64K);
    b.w(MCR, 0x8000_0C00);
    b.w(PUSHR, 0x9F | EOQ);
    assert_eq!(b.ops(), [0x9F]);
}

#[test]
fn frames_are_recorded_with_pcs_and_cont() {
    let mut b = board(&[], JEDEC_64K);
    b.w(CTAR1, CTAR_16);
    b.w(PUSHR, PCS1 | CONT | CTCNT | 0x9F);
    assert_eq!(b.m.idle_until(7), None);
    b.w(PUSHR, PCS1 | CONT | 1 << 28 | 0xABCD);
    assert_eq!(b.m.idle_until(9), None);
    b.w(PUSHR, PCS0 | EOQ | 0x1234); // 8 bits: TXDATA beyond the frame is not sent
    b.w(PUSHR, 0x55); // no chip select, queued behind EOQF
    assert_eq!(b.m.idle_until(12), None);
    b.w(SR, EOQF);
    assert_eq!(
        b.dspi().frames(),
        [
            (0, 0x9F, 0x02, true),
            (7, 0xABCD, 0x02, true),
            (9, 0x34, 0x01, false),
            (12, 0x55, 0x00, false)
        ]
    );
}

#[test]
fn dspi_reset_values_follow_the_rm() {
    // RM Table 40-3: MCR 0x0000_4001, TCR 0, CTAR0-CTAR7 0x7800_0000, SR 0, RSER 0, PUSHR 0, POPR 0
    let mut b = bare(&[], JEDEC_64K);
    assert_eq!(b.r(MCR), 0x0000_4001);
    assert_eq!(b.r(TCR), 0);
    for n in 0..8 {
        assert_eq!(b.r(CTAR0 + 4 * n), 0x7800_0000, "CTAR{n}");
    }
    // stopped (HALT, so TXRXS clear) and disabled (MDIS: TFFF clear until MDIS clears, RM Table 40-7)
    assert_eq!(b.r(SR), 0);
    assert_eq!(b.r(RSER), 0);
    assert_eq!(b.r(PUSHR), 0);
    b.w(MCR, MCR_RUN);
    assert_eq!(
        b.r(MCR),
        0x8003_0000,
        "CLR_TXF and CLR_RXF read as zero (RM Table 40-4)"
    );
    assert_eq!(b.r(SR), TXRXS | TFFF);
}

#[test]
fn frames_pushed_after_an_end_of_queue_wait_for_eoqf_to_clear() {
    // RM 40.4.1: EOQF stops the DSPI after the current frame; it runs again once EOQF is cleared (and HALT is
    // clear). The frames pushed meanwhile wait in the TX FIFO (TXCTR).
    let mut b = board(&[], JEDEC_64K);
    b.w(PUSHR, PCS1 | EOQ | 0x06); // WREN, ends the queue
    assert_eq!(b.r(SR) & (TXRXS | EOQF | TCF), EOQF | TCF);
    b.w(PUSHR, PCS1 | CONT | 0x05);
    b.w(PUSHR, PCS1 | EOQ);
    let sr = b.r(SR);
    assert_eq!((sr >> 12 & 0xF, sr >> 4 & 0xF), (2, 1), "TXCTR, RXCTR");
    assert_eq!(
        b.r(TXFR0 + 4),
        PCS1 | CONT | 0x05,
        "TXFR1 holds the first waiting entry"
    );
    assert_eq!(b.ops(), [0x06]);
    b.w(SR, EOQF);
    assert_eq!(b.ops(), [0x06, 0x05]);
    let sr = b.r(SR);
    assert_eq!((sr >> 12 & 0xF, sr >> 4 & 0xF), (0, 3));
    assert_ne!(sr & EOQF, 0, "the second queue ended too");
    let got: Vec<u32> = (0..3).map(|_| b.r(POPR)).collect();
    assert_eq!(got, [0xFF, 0xFF, 0x02], "WEL");
}

#[test]
fn tcr_counts_frames_and_ctcnt_clears_it() {
    // RM 40.3.2: SPI_TCNT (bits 31:16) counts frames; CTCNT clears it before its frame; a write presets it
    let mut b = board(&[], JEDEC_64K);
    b.w(TCR, 0xFFFF_0000);
    b.w(PUSHR, 0x00);
    assert_eq!(b.r(TCR), 0, "wraps past 65535");
    b.w(PUSHR, 0x00);
    b.w(PUSHR, 0x00);
    assert_eq!(b.r(TCR), 2 << 16);
    b.w(PUSHR, CTCNT);
    assert_eq!(b.r(TCR), 1 << 16);
}

#[test]
fn fifo_pointers_and_entries_follow_the_rm() {
    // RM 40.4.2.4, RM 40.4.2.5: TXNXTPTR moves with every frame shifted out, POPNXTPTR with every pop, and the
    // TXFRn and RXFRn registers show the FIFO entries
    let mut b = board(&noise(8), JEDEC_64K);
    for v in [0x03, 0, 0, 0, 0] {
        b.w(PUSHR, PCS1 | CONT | v);
    }
    b.r(POPR);
    let sr = b.r(SR);
    assert_eq!(
        (sr >> 8 & 0xF, sr >> 4 & 0xF, sr & 0xF),
        (5, 4, 1),
        "TXNXTPTR, RXCTR, POPNXTPTR"
    );
    assert_eq!(b.r(TXFR0 + 4 * 4), PCS1 | CONT);
    assert_eq!(b.r(RXFR0 + 4 * 4), u32::from(noise(8)[0]));
    b.w(MCR, MCR_RUN); // CLR_RXF empties the RX FIFO
    assert_eq!(b.r(SR) >> 4 & 0xF, 0);
}

#[test]
fn unmodelled_uses_are_refused() {
    // the tripwires: frame sizes other than 8 and 16, two attached parts selected at once, an empty POPR, RSER
    // interrupt requests and DMA requests with no request line, writes outside the writable registers
    let mut b = board(&[], JEDEC_64K);
    b.w(CTAR1, 0x5801_0000); // FMSZ 11: 12-bit frames
    let d = b.refused(PUSHR, Size::Long, Some(PCS1 | 1 << 28));
    assert!(d.contains("12-bit"), "{d}");
    let d = b.refused(POPR, Size::Long, None);
    assert!(d.contains("POPR") && d.contains("empty"), "{d}");
    for v in [0x0002_0000, 0x0200_0000, 1 << 31, 1 << 28, 1 << 27, 1 << 19] {
        let d = b.refused(RSER, Size::Long, Some(v));
        assert!(d.contains("RSER") && d.contains("interrupt"), "{d}");
    }
    let d = b.refused(RSER, Size::Long, Some(0x0000_0001));
    assert!(d.contains("RSER") && d.contains("reserved"), "{d}");
    // Dspi::new has no DMA request lines
    let d = b.refused(RSER, Size::Long, Some(0x0300_0000));
    assert!(d.contains("RSER") && d.contains("no request line"), "{d}");
    assert_eq!(b.r(RSER), 0);
    b.w(RSER, 0x0101_0000); // DIRS alone requests nothing: stored
    assert_eq!(b.r(RSER), 0x0101_0000);
    let d = b.refused(RXFR0, Size::Long, Some(0));
    assert!(d.contains("+0x7c"), "{d}");
    assert!(b.dspi().frames().is_empty());

    let mut m = Machine::new();
    let mut dspi = Dspi::new();
    dspi.attach(1, Box::new(SpiNor::new(SIZE, JEDEC_64K, &[])));
    dspi.attach(2, Box::new(SpiNor::new(SIZE, JEDEC_256K, &[])));
    let id = m.add_device(DSPI, 0x4000, Box::new(dspi));
    m.access(DSPI + CTAR0, Size::Long, Some(CTAR_8)).unwrap();
    m.access(DSPI + MCR, Size::Long, Some(0x8007_0C00)).unwrap();
    assert_eq!(
        m.access(DSPI + PUSHR, Size::Long, Some(PCS1 | PCS2 | 0x9F)),
        Err(Fault::Device(Stop::Unmodelled))
    );
    let d = m.take_refusal().unwrap().detail;
    assert!(d.contains("PCS1") && d.contains("PCS2"), "{d}");
    // PCS2 idling low selects its part along with the PCS1 the frame asserts
    m.access(DSPI + MCR, Size::Long, Some(0x8003_0C00)).unwrap();
    assert_eq!(
        m.access(DSPI + PUSHR, Size::Long, Some(PCS1 | 0x9F)),
        Err(Fault::Device(Stop::Unmodelled))
    );
    m.take_refusal().unwrap();
    // one at a time, each part answers with its own ID
    m.access(DSPI + MCR, Size::Long, Some(0x8007_0C00)).unwrap();
    for pcs in [PCS1, PCS2] {
        m.access(DSPI + PUSHR, Size::Long, Some(pcs | CONT | 0x9F))
            .unwrap();
        for _ in 0..4 {
            m.access(DSPI + PUSHR, Size::Long, Some(pcs | CONT))
                .unwrap();
        }
        m.access(DSPI + PUSHR, Size::Long, Some(pcs)).unwrap();
    }
    let got: Vec<u8> = (0..12)
        .map(|_| m.access(DSPI + POPR, Size::Long, None).unwrap() as u8)
        .collect();
    assert_eq!(got[1..6], JEDEC_64K[..5]);
    assert_eq!(got[7..12], JEDEC_256K[..5]);
    let dspi = m.device::<Dspi>(id);
    assert_eq!(dspi.device::<SpiNor>(2).log()[0].1, 0x9F);
}

#[test]
#[should_panic(expected = "SpiNor")]
fn device_of_the_wrong_type_panics() {
    struct Other;
    impl mcf5441x_emu::soc::dspi::SpiDevice for Other {
        fn select(&mut self, _now: u64) {}
        fn transfer(&mut self, _byte: u8) -> u8 {
            0
        }
        fn deselect(&mut self, _now: u64) {}
    }
    let mut dspi = Dspi::new();
    dspi.attach(3, Box::new(Other));
    let _ = dspi.device::<SpiNor>(3);
}

#[test]
fn a_push_to_a_full_tx_fifo_is_refused() {
    // stopped behind EOQF (RM 40.4.1), the TX FIFO fills; the chip ignores a push to a full FIFO (RM 40.4.2.4.1),
    // which would lose the frame silently: the model refuses it, naming why the DSPI is stopped
    let mut b = board(&[], JEDEC_64K);
    b.w(PUSHR, PCS1 | EOQ | 0x06);
    for _ in 0..16 {
        b.w(PUSHR, PCS1 | CONT | 0x05);
    }
    assert_eq!(
        b.r(SR) & (TFFF | 0xF000),
        0xF000,
        "TFFF clear, TXCTR reads 15"
    );
    let d = b.refused(PUSHR, Size::Long, Some(PCS1 | 0x05));
    assert!(
        d.contains("DSPI") && d.contains("TX FIFO full") && d.contains("EOQF"),
        "{d}"
    );
    assert_eq!(b.dspi().frames().len(), 1);
}

#[test]
fn unmodelled_modes_are_refused_also_when_a_queue_restarts() {
    // DCONF other than SPI (RM Table 40-4) and the modified timing format are not modelled
    for (mcr, what) in [(0x9003_0000, "DCONF"), (0x8403_0000, "MTFE")] {
        let mut b = board(&[], JEDEC_64K);
        b.w(MCR, mcr);
        let d = b.refused(PUSHR, Size::Long, Some(PCS1 | 0x9F));
        assert!(d.contains("DSPI") && d.contains(what), "{d}");
    }
    // frames waiting behind EOQF meet the mode in force when they go out: here slave mode
    let mut b = board(&[], JEDEC_64K);
    b.w(PUSHR, PCS1 | EOQ | 0x06);
    b.w(PUSHR, PCS1 | 0x05);
    b.w(MCR, 0x0003_0000);
    let d = b.refused(SR, Size::Long, Some(EOQF));
    assert!(d.contains("slave mode"), "{d}");
    assert_eq!(b.dspi().frames().len(), 1);
}

#[test]
fn take_frames_drains_and_record_off_logs_nothing() {
    let mut b = board(&[], JEDEC_64K);
    b.transfer(&[0x9F], 3);
    let n = b.dspi().frames().len();
    assert_eq!(n, 4);
    let taken = b.m.device_mut::<Dspi>(b.dspi).take_frames();
    assert_eq!(taken.len(), n);
    assert!(b.dspi().frames().is_empty());
    b.m.device_mut::<Dspi>(b.dspi).record(false);
    let got = b.transfer(&[0x9F], 3);
    assert_eq!(got, JEDEC_64K[..3], "the transfer still reaches the part");
    assert!(b.dspi().frames().is_empty());
}

fn frame(at: u64, data: u32, pcs: u8, cont: bool, ctas: u8) -> Frame {
    Frame {
        at,
        data,
        pcs,
        cont,
        ctas,
    }
}

#[test]
fn dspi_ctar2_to_7_select_frame_size_and_timing() {
    // RM Table 40-3: eight CTARs; PUSHR[CTAS] picks the one whose FMSZ sizes the frame (RM 40.4.2.1). The timing
    // fields are stored and read back; frames take no time whatever they say.
    let mut b = board(&[], JEDEC_64K);
    let ctars: [u32; 8] = [
        CTAR_8,
        CTAR_16,
        0x7E00_0335, // 16-bit, CPOL, CPHA, PBR 0, CSSCK 0, PASC 0, PDT 0, BR 5, DT 3, ASC 3
        0x3E00_1064, // 8-bit
        0x3801_2345,
        0x7802_3456,
        0x3A11_1111,
        0x7C22_2222,
    ];
    for (n, v) in ctars.into_iter().enumerate() {
        b.w(CTAR0 + 4 * n as u32, v);
    }
    for (n, v) in ctars.into_iter().enumerate() {
        assert_eq!(b.r(CTAR0 + 4 * n as u32), v, "CTAR{n}");
    }
    for ctas in 2..8u32 {
        b.w(PUSHR, ctas << 28 | 0xA55A);
    }
    let want: Vec<Frame> = (2..8u8)
        .map(|ctas| {
            let data = if ctars[usize::from(ctas)] >> 27 & 0xF == 15 {
                0xA55A
            } else {
                0x5A
            };
            frame(0, data, 0, false, ctas)
        })
        .collect();
    assert_eq!(b.dspi().frame_log(), want);
    // the 16-bit frames shift two bytes, the 8-bit ones one: the RX FIFO holds what the frames read back (no part
    // selected: all ones within the frame)
    let got: Vec<u32> = (0..6).map(|_| b.r(POPR)).collect();
    assert_eq!(got, [0xFFFF, 0xFF, 0xFF, 0xFFFF, 0xFF, 0xFFFF]);
    // the frame size check covers every CTAR
    b.w(CTAR0 + 4 * 6, 0x5800_0000); // FMSZ 11
    let d = b.refused(PUSHR, Size::Long, Some(6 << 28));
    assert!(d.contains("12-bit") && d.contains("CTAR6"), "{d}");
}

const EDMA: u32 = 0xFC04_4000;
/// DSPI0's RX FIFO drain and TX FIFO fill requests (RM Table 19-6).
const RX_CH: u8 = 12;
const TX_CH: u8 = 13;
const TFFF_DMA: u32 = 0x0300_0000;
const RFDF_DMA: u32 = 0x0003_0000;

/// RAM, the eDMA and DSPI0 on request lines 12 and 13, with a NOR on PCS1: 8-bit frames through CTAR0, running.
fn dma_board(contents: &[u8]) -> Board {
    let mut m = Machine::new();
    m.mem.map_ram(RAM, 0x0100_0000);
    m.add_bus_master(EDMA, 0x2000, Box::new(Edma::new()));
    let mut dspi = Dspi::with_dma(RX_CH, TX_CH);
    dspi.attach(1, Box::new(SpiNor::new(SIZE, JEDEC_64K, contents)));
    let dspi = m.add_device(DSPI, 0x4000, Box::new(dspi));
    let mut b = Board { m, dspi };
    b.w(CTAR0, CTAR_8);
    b.w(MCR, MCR_RUN);
    b
}

impl Board {
    /// Channel `ch`'s TCD: 32-bit both sides, NBYTES 4, CITER = BITER = `count`, D_REQ.
    fn tcd(&mut self, ch: u8, saddr: u32, soff: u32, daddr: u32, doff: u32, count: u32) {
        let t = EDMA + 0x1000 + 32 * u32::from(ch);
        for (off, size, v) in [
            (0x00, Size::Long, saddr),
            (0x04, Size::Word, 0x0202), // ATTR: 32-bit both sides
            (0x06, Size::Word, soff),
            (0x08, Size::Long, 4),
            (0x10, Size::Long, daddr),
            (0x16, Size::Word, doff),
            (0x14, Size::Word, count),
            (0x1C, Size::Word, count),
            (0x1E, Size::Word, 0x08), // CSR: D_REQ
        ] {
            self.m.access(t + off, size, Some(v)).unwrap();
        }
    }

    fn citer(&mut self, ch: u8) -> u32 {
        let a = EDMA + 0x1000 + 32 * u32::from(ch) + 0x14;
        self.m.access(a, Size::Word, None).unwrap()
    }

    fn serq(&mut self, ch: u8) {
        self.m
            .access(EDMA + 0x18, Size::Byte, Some(u32::from(ch)))
            .unwrap();
    }

    fn line(&self, ch: u8) -> bool {
        self.m.dma_lines().asserted() & 1 << ch != 0
    }
}

#[test]
fn dspi_tfff_dma_fills_the_tx_fifo() {
    // RM 40.4.6.2: TFFF_RE with TFFF_DIRS makes TFFF (the TX FIFO is not full) a DMA request (line 13 for DSPI0,
    // RM Table 19-6). The eDMA pushes PUSHR commands from memory. Frames take no time, so while the DSPI runs the
    // FIFO never fills; an EOQ frame stops it (RM 40.4.1), the pushes then fill the FIFO, and the push that fills
    // it drops the request. Clearing EOQF drains the FIFO and the request rises again.
    let mut b = dma_board(&[]);
    const CMDS: u32 = 0x4010_0000;
    // no chip select: the frames reach no part
    let mut cmds = vec![EOQ | 0x06]; // stops the queue
    cmds.extend((0..19u32).map(|i| CONT | i));
    cmds.push(EOQ | 0x55);
    let bytes: Vec<u8> = cmds.iter().flat_map(|c| c.to_be_bytes()).collect();
    b.m.mem.load(CMDS, &bytes).unwrap();
    b.tcd(TX_CH, CMDS, 4, DSPI + PUSHR, 0, cmds.len() as u32);
    assert!(!b.line(TX_CH), "RSER 0: no request");
    b.w(RSER, TFFF_DMA);
    assert_eq!(b.r(RSER), TFFF_DMA);
    assert!(b.line(TX_CH), "TFFF requests");
    // guess: TFFF follows the FIFO, so writing 1 to it (RM Table 40-7: w1c) does not lower the request
    b.w(SR, TFFF);
    assert!(b.line(TX_CH), "still requests");
    b.serq(TX_CH);
    // the EOQ frame went out; 16 more filled the FIFO
    assert_eq!(b.dspi().frame_log().len(), 1);
    let sr = b.r(SR);
    assert_eq!(sr & (TFFF | EOQF), EOQF, "full, stopped");
    assert_eq!(sr >> 12 & 0xF, 15, "TXCTR reads 15 with 16 held");
    assert!(
        !b.line(TX_CH),
        "the push that filled the FIFO dropped the request"
    );
    assert_eq!(b.citer(TX_CH), 21 - 17);
    b.w(SR, EOQF);
    // the 16 waiting frames, then the last 4 pushed by the eDMA, the final one with EOQ
    let log = b.dspi().frame_log();
    assert_eq!(log.len(), 21);
    let data: Vec<u32> = log.iter().map(|f| f.data).collect();
    let want: Vec<u32> = cmds.iter().map(|c| c & 0xFF).collect();
    assert_eq!(data, want, "every command once, in order");
    assert_ne!(b.r(SR) & EOQF, 0);
    assert_eq!(b.citer(TX_CH), 21, "major loop done, CITER reloaded");
    assert!(b.line(TX_CH), "TFFF still requests; D_REQ cleared ERQ");
}

#[test]
fn dspi_rfdf_dma_drains_the_rx_fifo() {
    // RM 40.4.6.5: RFDF_RE with RFDF_DIRS makes RFDF (the RX FIFO is not empty) a DMA request (line 12 for DSPI0,
    // RM Table 19-6). The eDMA pops POPR into memory; the pop that empties the FIFO drops the request.
    let mut b = dma_board(&[]);
    const BUF: u32 = 0x4020_0000;
    b.tcd(RX_CH, DSPI + POPR, 0, BUF, 4, 6);
    b.w(RSER, RFDF_DMA);
    // RDID: 0x9F then five reads; the request is up while the eDMA is not enabled for it
    b.w(PUSHR, PCS1 | CONT | 0x9F);
    b.w(PUSHR, PCS1 | CONT);
    assert!(b.line(RX_CH), "RFDF requests");
    assert_eq!(b.r(SR) >> 4 & 0xF, 2, "RXCTR");
    b.serq(RX_CH);
    assert!(!b.line(RX_CH), "the pop that emptied the FIFO dropped it");
    assert_eq!(b.citer(RX_CH), 4, "two minor loops");
    for i in 0..4 {
        b.w(PUSHR, PCS1 | if i == 3 { EOQ } else { CONT });
        assert!(!b.line(RX_CH), "drained after each frame");
    }
    assert_eq!(b.r(SR) & (RFDF | 0xF0), 0, "RX FIFO empty");
    let got: Vec<u32> = (0..6)
        .map(|i| b.m.mem.read(BUF + 4 * i, Size::Long).unwrap())
        .collect();
    assert_eq!(got[0], 0xFF);
    assert_eq!(
        got[1..],
        JEDEC_64K[..5]
            .iter()
            .map(|&x| u32::from(x))
            .collect::<Vec<_>>()
    );
    assert_eq!(b.citer(RX_CH), 6, "major loop done");
}

#[test]
fn dspi1_codec_setup_from_ref_t_runs() {
    // the codec setup's registers, with the TX-fill request served by the eDMA: CTAR2 0x7E000335 (16-bit), CTAR3
    // 0x3E001064 (8-bit), RSER 0x03000000 (the TFFF DMA request, line 15, RM Table 19-6), then the eDMA pushes the
    // commands, CTAS 3, the last with EOQ, and the code polls SR[EOQF]. Every frame is recorded with its CTAS.
    const DSPI1: u32 = 0xFC03_C000;
    const CMDS: u32 = 0x4030_0000;
    let src = "move.l #0xFC03C000,%a0\n\
               move.l #0x80030C00,(%a0)\n\
               move.l #0x7E000335,%d1\n\
               move.l %d1,0x14(%a0)\n\
               move.l #0x3E001064,%d1\n\
               move.l %d1,0x18(%a0)\n\
               move.l #0x03000000,%d1\n\
               move.l %d1,0x30(%a0)\n\
               move.l #0xFC044018,%a1\n\
               move.b #15,(%a1)\n\
               1: move.l 0x2C(%a0),%d0\n\
               btst #28,%d0\n\
               beq.s 1b\n\
               move.l #0x10000000,%d1\n\
               move.l %d1,0x2C(%a0)\n\
               end: nop";
    let (mut m, l) = common::prepare(src, &["end"]);
    m.add_bus_master(EDMA, 0x2000, Box::new(Edma::new()));
    let id = m.add_device(DSPI1, 0x4000, Box::new(Dspi::with_dma(14, 15)));
    let cmds: Vec<u32> = [0x12, 0x34, 0x56, 0x78, 0x9A]
        .iter()
        .enumerate()
        .map(|(i, &d)| 3 << 28 | PCS0 | if i == 4 { EOQ } else { CONT } | d)
        .chain([2 << 28 | PCS0 | EOQ | 0xBEEF])
        .collect();
    let bytes: Vec<u8> = cmds.iter().flat_map(|c| c.to_be_bytes()).collect();
    m.mem.load(CMDS, &bytes).unwrap();
    let t = EDMA + 0x1000 + 32 * 15;
    for (off, size, v) in [
        (0x00, Size::Long, CMDS),
        (0x04, Size::Word, 0x0202),
        (0x06, Size::Word, 4),
        (0x08, Size::Long, 4),
        (0x10, Size::Long, DSPI1 + PUSHR),
        (0x16, Size::Word, 0),
        (0x14, Size::Word, cmds.len() as u32),
        (0x1C, Size::Word, cmds.len() as u32),
        (0x1E, Size::Word, 0x08),
    ] {
        m.access(t + off, size, Some(v)).unwrap();
    }
    m.set_time_limit(Some(10_000));
    let r = m.run(common::until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    let log: Vec<(u32, u8, bool, u8)> = m
        .device::<Dspi>(id)
        .frame_log()
        .iter()
        .map(|f| (f.data, f.pcs, f.cont, f.ctas))
        .collect();
    assert_eq!(
        log[..5],
        [
            (0x12, 1, true, 3),
            (0x34, 1, true, 3),
            (0x56, 1, true, 3),
            (0x78, 1, true, 3),
            (0x9A, 1, false, 3),
        ],
        "the queue up to its EOQ, CTAS 3: 8-bit frames"
    );
    // the 16-bit frame waited in the TX FIFO behind EOQF until the code cleared it
    assert_eq!(log[5..], [(0xBEEF, 1, false, 2)]);
    let sr = m.access(DSPI1 + SR, Size::Long, None).unwrap();
    assert_eq!(
        sr & (EOQF | 0xF000),
        EOQF,
        "the second queue ended; TX FIFO empty"
    );
}
