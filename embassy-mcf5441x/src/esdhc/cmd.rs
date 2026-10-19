//! The eSDHC's command words, clock words and status decode (register layouts RM §25.3): pure, host-tested.

/// IRQSTAT: command complete, transfer complete.
pub(crate) const CC: u32 = 1 << 0;
pub(crate) const TC: u32 = 1 << 1;
const CTOE: u32 = 1 << 16;
const CCE: u32 = 1 << 17;
const CEBE: u32 = 1 << 18;
const CIE: u32 = 1 << 19;
const DTOE: u32 = 1 << 20;
const DCE: u32 = 1 << 21;
const DEBE: u32 = 1 << 22;
const AC12E: u32 = 1 << 24;
const DMAE: u32 = 1 << 28;
/// Every defined error bit of IRQSTAT (also valid in IRQSIGEN, unlike the reserved bits around them).
pub(crate) const ERR: u32 = CTOE | CCE | CEBE | CIE | DTOE | DCE | DEBE | AC12E | DMAE;

/// PRSSTAT: command inhibit (CMD line), command inhibit (DAT lines), DAT line active, buffer write / read enable.
pub(crate) const CIHB: u32 = 1 << 0;
pub(crate) const CDIHB: u32 = 1 << 1;
pub(crate) const DLA: u32 = 1 << 2;
/// PRSSTAT: the SD clock is stable after a divider change.
pub(crate) const SDSTB: u32 = 1 << 3;
/// PRSSTAT DLSL bit 0: the DAT0 line level. The card holds it low while busy (after an R1b command such as CMD6).
pub(crate) const DAT0: u32 = 1 << 24;
pub(crate) const BWEN: u32 = 1 << 10;
pub(crate) const BREN: u32 = 1 << 11;

/// SYSCTL: resets (all, command side, data side) and the 80 initialisation clocks, all self-clearing.
pub(crate) const RSTA: u32 = 1 << 24;
pub(crate) const RSTC: u32 = 1 << 25;
pub(crate) const RSTD: u32 = 1 << 26;
pub(crate) const INITA: u32 = 1 << 27;
pub(crate) const SDCLKEN: u32 = 1 << 3;
const PEREN: u32 = 1 << 2;

/// PROCTL: 4-bit data bus.
pub(crate) const DTW_4BIT: u32 = 1 << 1;

/// A failed command or transfer.
#[non_exhaustive]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Error {
    /// A command or data timeout, or a wait that ran out.
    Timeout,
    /// A command, data or auto-CMD12 CRC error.
    Crc,
    /// The response carried another command's index.
    CmdIndex,
    /// A command or data end-bit error.
    DataEnd,
    /// The controller reported a DMA error, or channel 59 hadn't finished when the controller had.
    Dma,
    /// CMD19/CMD14 did not read back the inverted pattern.
    BusTest,
    /// The card is byte-addressed (OCR access mode); only sector-addressed parts are supported.
    ByteAddressed,
    /// The read runs past the end of the card.
    OutOfRange,
    /// The buffer doesn't start on a 4-byte boundary: the eDMA writes it a 32-bit word at a time.
    Misaligned,
}

/// IRQSTAT to a result: timeouts first, then CRC, command index, DMA, and any other error bit as an end-bit error.
pub(crate) const fn check(s: u32) -> Result<(), Error> {
    if s & (CTOE | DTOE) != 0 {
        Err(Error::Timeout)
    } else if s & (CCE | DCE | AC12E) != 0 {
        Err(Error::Crc)
    } else if s & CIE != 0 {
        Err(Error::CmdIndex)
    } else if s & DMAE != 0 {
        Err(Error::Dma)
    } else if s & ERR != 0 {
        Err(Error::DataEnd)
    } else {
        Ok(())
    }
}

/// SD clock divider: SD clock = base / (2 × `prescaler` × (`divisor` + 1)), with SYSCTL SDCLKFS one-hot (0x01..0x80)
/// and DVS 0..15.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Divider {
    prescaler: u8,
    divisor: u8,
}

impl Divider {
    pub const fn new(prescaler: u8, divisor: u8) -> Self {
        assert!(prescaler.is_power_of_two(), "SDCLKFS is one-hot");
        assert!(divisor < 16, "DVS is 4 bits");
        Self { prescaler, divisor }
    }

    /// The SD clock from a `base_hz` eSDHC base clock (rounded down).
    pub const fn hz(self, base_hz: u32) -> u32 {
        base_hz / (2 * self.prescaler as u32 * (self.divisor as u32 + 1))
    }

    const fn bits(self) -> u32 {
        (self.prescaler as u32) << 8 | (self.divisor as u32) << 4
    }
}

/// The first SYSCTL word after a reset: the divider and the SD clock on, everything else clear (automatic clock gating).
pub(crate) const fn ident_sysctl(d: Divider) -> u32 {
    d.bits() | SDCLKEN
}

/// `s` with the data timeout at SDCLK × 2^27 (DTOCV 0xE) and the peripheral clock kept on (PEREN).
pub(crate) const fn with_timeout(s: u32) -> u32 {
    (s & !0x000F_0000) | 0x000E_0000 | PEREN
}

/// `s` with its divider replaced by `d` (switch it with SDCLKEN clear, then wait for SDSTB: RM §25.4.2).
pub(crate) const fn with_clock(s: u32, d: Divider) -> u32 {
    (s & 0xFFFF_000F) | d.bits()
}

/// Response types (XFERTYP RSPTYP, CCCEN, CICEN).
#[derive(Copy, Clone)]
enum Resp {
    None,
    /// 136 bits, CRC checked (CID, CSD).
    R2,
    /// 48 bits, no checks (OCR).
    R3,
    /// 48 bits, CRC and index checked.
    R1,
    /// R1 with busy.
    R1b,
}

/// Data phase (XFERTYP DPSEL, DTDSEL, MSBSEL, BCEN, AC12EN). DMAEN stays clear: the eDMA, not the controller's own
/// DMA, moves the data.
#[derive(Copy, Clone)]
enum Data {
    None,
    Read,
    Write,
    /// Several blocks, counted (BLKATTR), closed by an automatic CMD12.
    ReadMulti,
}

const fn xfertyp(index: u8, resp: Resp, data: Data, abort: bool) -> u32 {
    let r = match resp {
        Resp::None => 0,
        Resp::R2 => 1 << 19 | 0b01 << 16,
        Resp::R3 => 0b10 << 16,
        Resp::R1 => 1 << 20 | 1 << 19 | 0b10 << 16,
        Resp::R1b => 1 << 20 | 1 << 19 | 0b11 << 16,
    };
    let d = match data {
        Data::None => 0,
        Data::Read => 1 << 21 | 1 << 4,
        Data::Write => 1 << 21,
        Data::ReadMulti => 1 << 21 | 1 << 5 | 1 << 4 | 1 << 2 | 1 << 1,
    };
    (index as u32) << 24 | (if abort { 0b11 << 22 } else { 0 }) | r | d
}

/// RCA the driver assigns the card (CMD3) and addresses it by.
const RCA: u32 = 2 << 16;

/// The only commands the driver can send. None programs, erases or reconfigures the card: CMD6 switches only the
/// volatile HS_TIMING and BUS_WIDTH, and CMD12 only ends a read that failed or was cancelled.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Cmd {
    GoIdle,
    SendOpCond,
    AllSendCid,
    SetRca,
    SendCsd,
    SendCid,
    Select,
    SwitchHsTiming,
    SwitchBusWidth4,
    SendExtCsd,
    BusTestW,
    BusTestR,
    SetBlockLen,
    StopTransmission,
    ReadMultiple(u32),
}

impl Cmd {
    /// (XFERTYP, CMDARG).
    pub(crate) const fn encode(self) -> (u32, u32) {
        match self {
            Cmd::GoIdle => (xfertyp(0, Resp::None, Data::None, false), 0),
            // sector mode, 1.7–1.95 V and 2.7–3.6 V
            Cmd::SendOpCond => (xfertyp(1, Resp::R3, Data::None, false), 0x40FF_8000),
            Cmd::AllSendCid => (xfertyp(2, Resp::R2, Data::None, false), 0),
            Cmd::SetRca => (xfertyp(3, Resp::R1, Data::None, false), RCA),
            Cmd::SendCsd => (xfertyp(9, Resp::R2, Data::None, false), RCA),
            Cmd::SendCid => (xfertyp(10, Resp::R2, Data::None, false), RCA),
            Cmd::Select => (xfertyp(7, Resp::R1b, Data::None, false), RCA),
            // CMD6 write-byte: EXT_CSD[185] HS_TIMING = 1, EXT_CSD[183] BUS_WIDTH = 1 (4 bits)
            Cmd::SwitchHsTiming => (xfertyp(6, Resp::R1b, Data::None, false), 0x03B9_0100),
            Cmd::SwitchBusWidth4 => (xfertyp(6, Resp::R1b, Data::None, false), 0x03B7_0100),
            Cmd::SendExtCsd => (xfertyp(8, Resp::R1, Data::Read, false), 0),
            Cmd::BusTestW => (xfertyp(19, Resp::R1, Data::Write, false), 0),
            Cmd::BusTestR => (xfertyp(14, Resp::R1, Data::Read, false), 0),
            Cmd::SetBlockLen => (xfertyp(16, Resp::R1, Data::None, false), 512),
            Cmd::StopTransmission => (xfertyp(12, Resp::R1b, Data::None, true), 0),
            Cmd::ReadMultiple(lba) => (xfertyp(18, Resp::R1, Data::ReadMulti, false), lba),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_command_encodes_to_the_proven_word() {
        // (XFERTYP, CMDARG) as proven on hardware
        let want = [
            (Cmd::GoIdle, (0x0000_0000, 0)),
            (Cmd::SendOpCond, (0x0102_0000, 0x40FF_8000)),
            (Cmd::AllSendCid, (0x0209_0000, 0)),
            (Cmd::SetRca, (0x031A_0000, 2 << 16)),
            (Cmd::SendCsd, (0x0909_0000, 2 << 16)),
            (Cmd::SendCid, (0x0A09_0000, 2 << 16)),
            (Cmd::Select, (0x071B_0000, 2 << 16)),
            (Cmd::SwitchHsTiming, (0x061B_0000, 0x03B9_0100)),
            (Cmd::SwitchBusWidth4, (0x061B_0000, 0x03B7_0100)),
            (Cmd::SendExtCsd, (0x083A_0010, 0)),
            (Cmd::BusTestW, (0x133A_0000, 0)),
            (Cmd::BusTestR, (0x0E3A_0010, 0)),
            (Cmd::SetBlockLen, (0x101A_0000, 512)),
            (Cmd::StopTransmission, (0x0CDB_0000, 0)),
            (Cmd::ReadMultiple(0x1C_0000), (0x123A_0036, 0x1C_0000)),
        ];
        for (c, w) in want {
            assert_eq!(c.encode(), w, "{c:?}");
        }
    }

    #[test]
    fn sysctl_words() {
        let ident = Divider::new(0x10, 5);
        assert_eq!(ident_sysctl(ident), 0x1058);
        assert_eq!(with_timeout(0x1058), 0x000E_105C);
        assert_eq!(with_clock(0x000E_105C, Divider::new(0x01, 0)), 0x000E_010C);
    }

    #[test]
    fn divider_rates() {
        // a proven board's eSDHC base clock: 528 MHz / 7
        assert_eq!(Divider::new(0x10, 5).hz(75_428_571), 392_857);
        assert_eq!(Divider::new(0x01, 0).hz(75_428_571), 37_714_285);
    }

    #[test]
    fn status_decodes_to_the_first_matching_error() {
        assert_eq!(check(CC | TC), Ok(()));
        assert_eq!(check(1 << 16), Err(Error::Timeout)); // CTOE
        assert_eq!(check(1 << 20 | 1 << 21), Err(Error::Timeout)); // DTOE wins over DCE
        assert_eq!(check(1 << 21), Err(Error::Crc)); // DCE
        assert_eq!(check(1 << 24), Err(Error::Crc)); // AC12E
        assert_eq!(check(1 << 19), Err(Error::CmdIndex)); // CIE
        assert_eq!(check(1 << 28), Err(Error::Dma)); // DMAE
        assert_eq!(check(1 << 22), Err(Error::DataEnd)); // DEBE
        assert_eq!(check(1 << 18), Err(Error::DataEnd)); // CEBE
    }
}
