//! The DSPI (RM ch. 40): DSPI0 with an SPI NOR flash on PCS1, driven through its registers the way a boot loader's
//! read routine drives it (up to 4 frames in flight, pop then push, CONT on all frames but the last, EOQ on it).

use mcf5441x_emu::parts::SpiNor;
use mcf5441x_emu::soc::Dspi;
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
    // the tripwires: CTAS >= 2, frame sizes other than 8 and 16, two attached parts selected at once, an empty
    // POPR, RSER requests, writes outside the writable registers
    let mut b = board(&[], JEDEC_64K);
    let d = b.refused(PUSHR, Size::Long, Some(PCS1 | 2 << 28));
    assert!(d.contains("CTAS 2"), "{d}");
    b.w(CTAR1, 0x5801_0000); // FMSZ 11: 12-bit frames
    let d = b.refused(PUSHR, Size::Long, Some(PCS1 | 1 << 28));
    assert!(d.contains("12-bit"), "{d}");
    let d = b.refused(POPR, Size::Long, None);
    assert!(d.contains("POPR") && d.contains("empty"), "{d}");
    let d = b.refused(RSER, Size::Long, Some(0x0002_0000));
    assert!(d.contains("RSER"), "{d}");
    assert_eq!(b.r(RSER), 0);
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
