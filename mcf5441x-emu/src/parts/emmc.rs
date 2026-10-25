//! An eMMC card: the subset of the MMC command set a boot loader and a block driver use (JEDEC eMMC, JESD84): reads,
//! multiple-block writes and TRIM, behind the eSDHC ([`Esdhc`](crate::soc::Esdhc)).

use std::collections::BTreeMap;
use std::fmt;

/// CMD6 SWITCH arguments: access mode (bits 25:24), EXT_CSD index (23:16), value (15:8).
const WRITE_BYTE: u32 = 3;
const BUS_WIDTH: usize = 183;
const HS_TIMING: usize = 185;
const ERASE_GROUP_DEF: usize = 175;
/// EXT_CSD SEC_COUNT, bytes 212-215, little-endian.
const SEC_COUNT: usize = 212;
/// EXT_CSD ERASED_MEM_CONT: 1 when erased memory reads 0xFF, else 0x00.
const ERASED_MEM_CONT: usize = 181;
/// CMD38's TRIM argument (JESD84 6.6.10).
const TRIM: u32 = 1;
/// Card status ADDRESS_OUT_OF_RANGE and ERASE_PARAM (JESD84 Table 68).
const OUT_OF_RANGE: u32 = 1 << 31;
const ERASE_PARAM: u32 = 1 << 27;

/// The R1 card status the card answers with: CURRENT_STATE tran (4 << 9) and READY_FOR_DATA (bit 8).
const R1: u32 = 0x900;

/// The commands modelled: no reliable writes, erase-group erases, locks or write protection.
const MODELLED: [u8; 20] = [
    0, 1, 2, 3, 6, 7, 8, 9, 10, 12, 13, 14, 16, 18, 19, 23, 25, 35, 36, 38,
];

/// What the board decides about its card.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EmmcConfig {
    /// EXT_CSD SEC_COUNT: the user area in 512-byte sectors (EXT_CSD bytes 212-215).
    pub sec_count: u32,
    /// The CID as the controller's CMDRSP0-CMDRSP3 hold it after CMD2 or CMD10: bits 127:8 of the register, so
    /// that MID (CID bits 127:120) is bits 23:16 of `cid[3]` (RM Table 25-9).
    pub cid: [u32; 4],
    /// The CSD, after CMD9, in the same layout as `cid`.
    pub csd: [u32; 4],
    /// EXT_CSD bytes (index, value) set over the all-zero register and SEC_COUNT.
    pub ext_csd: Vec<(usize, u8)>,
    /// The OCR the card answers CMD1 with (bit 31: powered up; bits 30:29 = 10: sector addressing).
    pub ocr: u32,
}

/// A command's response, as the card sends it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Response {
    /// A 48-bit response: its 32 bits of content (R1, R3).
    Short(u32),
    /// A 136-bit response (R2): CMDRSP0-CMDRSP3.
    Long([u32; 4]),
}

/// How a command's data moves, when it has a data phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Transfer {
    /// Blocks from the card (CMD8, CMD18).
    Blocks,
    /// Blocks to the card (CMD25).
    BlocksWrite,
    /// The bus-test pattern from the card (CMD14).
    BusTestRead,
    /// The bus-test pattern to the card (CMD19).
    BusTestWrite,
}

/// An eMMC with sector addressing and 512-byte blocks behind an eSDHC.
///
/// Its contents are the `image` it was made with (sector `n` is bytes `512 n .. 512 n + 512`, and sectors past the end
/// of the image read 0), changed by writes and TRIM. The card is sparse: written sectors are held one by one and a
/// TRIM is held as a range, so a card of any SEC_COUNT costs only what was given or written
/// ([`Emmc::resident_sectors`]). [`Emmc::image`] builds the dense bytes on request.
/// The card answers:
/// - CMD0 (GO_IDLE): back to a 1-bit bus; CMD1 (SEND_OP_COND): the OCR; CMD2 and CMD10: the CID; CMD9: the CSD;
///   every other command R1 with the card in the transfer state, ready for data. The card has no state machine
///   beyond its bus width and the stuck-sending state below: no RCA, select or block length is checked.
/// - CMD6 (SWITCH): only writes of a byte to BUS_WIDTH (183; 0 for a 1-bit bus, 1 for a 4-bit bus), HS_TIMING
///   (185) and ERASE_GROUP_DEF (175; 0 or 1, stored, with no effect on the erase commands), which change the card's EXT_CSD. The controller holds DAT0 low after it (the card is busy).
/// - CMD8 (SEND_EXT_CSD): the 512-byte EXT_CSD, all zero but SEC_COUNT and the board's bytes; each 4-byte group
///   reversed, which is where a 32-bit DATPORT read puts each byte (measured on an MCF54415: byte `i` lands at
///   `i ^ 3`).
/// - CMD18 (READ_MULTIPLE_BLOCK): the blocks from the sector in its argument.
/// - CMD19 (BUS_TEST_W) takes a word, CMD14 (BUS_TEST_R) returns it inverted.
/// - CMD25 (WRITE_MULTIPLE_BLOCK): the blocks the controller sends land at the sector in its argument. One starting
///   or running past SEC_COUNT writes nothing and sets ADDRESS_OUT_OF_RANGE in the status.
/// - CMD35 and CMD36 (ERASE_GROUP_START and END) take the first and last sectors of a range; CMD38 (ERASE) with the
///   TRIM argument fills it with the erased value (0xFF when EXT_CSD ERASED_MEM_CONT, byte 181, is 1, else 0x00). A
///   range ending before its start or past SEC_COUNT trims nothing and sets ERASE_PARAM. Any other CMD38 argument
///   (erase-group ERASE, secure erase or trim, discard) is not modelled. The controller holds DAT0 low after CMD38.
/// - CMD13 (SEND_STATUS): R1 with the error bits set since the last CMD13, which it clears.
/// - CMD3, CMD7, CMD12, CMD16 and CMD23 (SET_BLOCK_COUNT, taken and ignored): R1. CMD12 ends the stuck-sending state.
///
/// guess: none of the write side is measured (the unit's card is not written); timings are the controller's
/// ([`EsdhcConfig`](crate::soc::EsdhcConfig)).
///
/// Any other command, and any other CMD6, is not modelled: the controller refuses the XFERTYP write that would
/// send it.
///
/// Faults ([`Emmc::faults`]): each CMD18 while the count is above 0 takes one off, fails its CRC and leaves the
/// card sending: until CMD12 it answers no command but CMD12 (the controller sees a command timeout). Each CMD25
/// while it is above 0 takes one off and fails its data's CRC: nothing is written (the controller sees DCE).
pub struct Emmc {
    /// The contents the card was made with, under everything below.
    base: Vec<u8>,
    /// TRIMs, in order: (first sector, last sector, fill byte); later ones win over earlier ones and the base.
    fills: Vec<(u32, u32, u8)>,
    /// Sectors written since the last TRIM covering them: they win over the fills and the base.
    sectors: BTreeMap<u32, Box<[u8; 512]>>,
    ocr: u32,
    cid: [u32; 4],
    csd: [u32; 4],
    ext_csd: [u8; 512],
    wide: bool,
    sending: bool,
    faults: u32,
    bus_test: u32,
    commands: Vec<(u8, u32)>,
    record: bool,
    /// The error bits for the next CMD13.
    status: u32,
    /// CMD35's and CMD36's sectors.
    erase_range: (u32, u32),
    written: Vec<(u32, u32)>,
    trimmed: Vec<(u32, u32)>,
}

impl fmt::Debug for Emmc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Emmc")
            .field("base", &self.base.len())
            .field("resident", &self.sectors.len())
            .field("wide", &self.wide)
            .field("sending", &self.sending)
            .field("faults", &self.faults)
            .field("commands", &self.commands.len())
            .finish_non_exhaustive()
    }
}

impl Emmc {
    /// A card holding `image`, configured by the board. Panics when an EXT_CSD index is 512 or more.
    pub fn new(image: Vec<u8>, config: EmmcConfig) -> Emmc {
        let mut ext_csd = [0; 512];
        ext_csd[SEC_COUNT..SEC_COUNT + 4].copy_from_slice(&config.sec_count.to_le_bytes());
        for (i, v) in config.ext_csd {
            assert!(i < 512, "EXT_CSD index {i} out of range");
            ext_csd[i] = v;
        }
        Emmc {
            base: image,
            fills: Vec::new(),
            sectors: BTreeMap::new(),
            ocr: config.ocr,
            cid: config.cid,
            csd: config.csd,
            ext_csd,
            wide: false,
            sending: false,
            faults: 0,
            bus_test: 0,
            commands: Vec::new(),
            record: true,
            status: 0,
            erase_range: (0, 0),
            written: Vec::new(),
            trimmed: Vec::new(),
        }
    }

    /// The card's contents as dense bytes: sector `n` at bytes `512 n ..`, up to the end of the image it was made
    /// with, the last sector written and the last sector trimmed (anything past that reads 0). A copy: the card
    /// is sparse, so this costs the size of that extent; use [`Emmc::sector`] on a big card.
    pub fn image(&self) -> Vec<u8> {
        let end = self
            .sectors
            .keys()
            .next_back()
            .map(|&n| n as usize + 1)
            .into_iter()
            .chain(self.fills.iter().map(|&(_, last, _)| last as usize + 1))
            .max()
            .unwrap_or(0)
            .max(self.base.len().div_ceil(512));
        (0..end as u32).flat_map(|n| self.sector(n)).collect()
    }

    /// Sector `n` as it reads.
    pub fn sector(&self, n: u32) -> [u8; 512] {
        if let Some(s) = self.sectors.get(&n) {
            return **s;
        }
        if let Some(&(_, _, fill)) = self.fills.iter().rev().find(|&&(a, b, _)| (a..=b).contains(&n)) {
            return [fill; 512];
        }
        let mut out = [0; 512];
        let start = n as usize * 512;
        if start < self.base.len() {
            let end = (start + 512).min(self.base.len());
            out[..end - start].copy_from_slice(&self.base[start..end]);
        }
        out
    }

    /// How many sectors the card holds one by one (written since the last TRIM over them).
    pub fn resident_sectors(&self) -> usize {
        self.sectors.len()
    }

    /// Every CMD25 that wrote, (first sector, sectors), in order.
    pub fn written(&self) -> &[(u32, u32)] {
        &self.written
    }

    /// Every CMD25 that wrote so far, emptying the log.
    pub fn take_written(&mut self) -> Vec<(u32, u32)> {
        std::mem::take(&mut self.written)
    }

    /// Every TRIM that trimmed, (first sector, last sector), in order.
    pub fn trimmed(&self) -> &[(u32, u32)] {
        &self.trimmed
    }

    /// Every TRIM that trimmed so far, emptying the log.
    pub fn take_trimmed(&mut self) -> Vec<(u32, u32)> {
        std::mem::take(&mut self.trimmed)
    }

    fn sectors(&self) -> u32 {
        u32::from_le_bytes(self.ext_csd[SEC_COUNT..SEC_COUNT + 4].try_into().unwrap())
    }

    /// The number of CMD18s and CMD25s still to fail: a CMD18 fails its CRC and leaves the card sending until CMD12;
    /// a CMD25's data fails its CRC and nothing is written.
    pub fn faults(&mut self) -> &mut u32 {
        &mut self.faults
    }

    /// Every command the card received, (index, argument), in order.
    pub fn commands(&self) -> &[(u8, u32)] {
        &self.commands
    }

    /// Every command the card received so far, emptying the log.
    pub fn take_commands(&mut self) -> Vec<(u8, u32)> {
        std::mem::take(&mut self.commands)
    }

    /// Whether commands are logged (default on). Off changes only the log, never the card.
    pub fn record(&mut self, on: bool) {
        self.record = on;
    }

    /// Why command `cmd` with `arg` is not modelled, if it is not.
    pub(crate) fn unmodelled(&self, cmd: u8, arg: u32) -> Option<String> {
        if !MODELLED.contains(&cmd) {
            return Some(format!(
                "eMMC: CMD{cmd} is not modelled (outside the card's command set)"
            ));
        }
        if cmd == 38 && arg != TRIM {
            return Some(format!(
                "eMMC: CMD38 {arg:#010x} is not modelled (only TRIM, argument 1)"
            ));
        }
        if cmd == 6 {
            let (access, index, value) =
                (arg >> 24 & 3, (arg >> 16 & 0xFF) as usize, arg >> 8 & 0xFF);
            let ok = access == WRITE_BYTE
                && match index {
                    BUS_WIDTH => value <= 1,
                    HS_TIMING => true,
                    ERASE_GROUP_DEF => value <= 1,
                    _ => false,
                };
            if !ok {
                return Some(format!(
                    "eMMC: CMD6 {arg:#010x} is not modelled (only byte writes of BUS_WIDTH 0 or 1, HS_TIMING and ERASE_GROUP_DEF 0 or 1)"
                ));
            }
        }
        None
    }

    /// How command `cmd`'s data moves, when it has a data phase.
    pub(crate) fn transfer(cmd: u8) -> Option<Transfer> {
        match cmd {
            8 | 18 => Some(Transfer::Blocks),
            25 => Some(Transfer::BlocksWrite),
            14 => Some(Transfer::BusTestRead),
            19 => Some(Transfer::BusTestWrite),
            _ => None,
        }
    }

    /// Receive a modelled command: its response, or `None` when the card answers nothing.
    pub(crate) fn command(&mut self, cmd: u8, arg: u32) -> Option<Response> {
        if self.record {
            self.commands.push((cmd, arg));
        }
        if self.sending && cmd != 12 {
            return None;
        }
        Some(match cmd {
            0 => {
                self.wide = false;
                Response::Short(0)
            }
            1 => Response::Short(self.ocr),
            2 | 10 => Response::Long(self.cid),
            9 => Response::Long(self.csd),
            6 => {
                let (index, value) = ((arg >> 16 & 0xFF) as usize, (arg >> 8) as u8);
                self.ext_csd[index] = value;
                if index == BUS_WIDTH {
                    self.wide = value == 1;
                }
                Response::Short(R1)
            }
            12 => {
                self.sending = false;
                Response::Short(R1)
            }
            13 => Response::Short(R1 | std::mem::take(&mut self.status)),
            35 => {
                self.erase_range.0 = arg;
                Response::Short(R1)
            }
            36 => {
                self.erase_range.1 = arg;
                Response::Short(R1)
            }
            38 => {
                self.trim();
                Response::Short(R1)
            }
            _ => Response::Short(R1),
        })
    }

    /// Whether the card's bus is 4 bits wide (after CMD6 BUS_WIDTH 1, until CMD0 or CMD6 BUS_WIDTH 0).
    pub(crate) fn wide(&self) -> bool {
        self.wide
    }

    /// The data of block command `cmd` (CMD8 or CMD18) with `arg`, `blocks` blocks, in the order the controller's
    /// DATPORT delivers it; `None` when it fails its CRC.
    pub(crate) fn read_blocks(&mut self, cmd: u8, arg: u32, blocks: u32) -> Option<Vec<u8>> {
        if cmd == 8 {
            // measured on an MCF54415: through the 32-bit DATPORT, EXT_CSD byte i lands at i ^ 3
            return Some(
                self.ext_csd
                    .chunks(4)
                    .flat_map(|w| w.iter().rev().copied())
                    .collect(),
            );
        }
        if self.faults > 0 {
            self.faults -= 1;
            self.sending = true;
            return None;
        }
        let mut out = Vec::with_capacity(blocks as usize * 512);
        for k in 0..blocks {
            out.extend_from_slice(&self.sector(arg.wrapping_add(k)));
        }
        Some(out)
    }

    /// CMD38 TRIM over CMD35..=CMD36's sectors.
    fn trim(&mut self) {
        let (start, end) = self.erase_range;
        if end < start || end >= self.sectors() {
            self.status |= ERASE_PARAM;
            return;
        }
        let erased = if self.ext_csd[ERASED_MEM_CONT] & 1 != 0 {
            0xFF
        } else {
            0
        };
        let covered: Vec<u32> = self.sectors.range(start..=end).map(|(&n, _)| n).collect();
        for n in covered {
            self.sectors.remove(&n);
        }
        self.fills.push((start, end, erased));
        if self.record {
            self.trimmed.push((start, end));
        }
    }

    /// CMD25 with `arg`: `data` (the blocks as the controller's DATPORT words carried them, big-endian) lands at its
    /// sector. `false` when an injected fault fails its CRC; nothing is written then.
    pub(crate) fn write_blocks(&mut self, arg: u32, data: &[u8]) -> bool {
        if self.faults > 0 {
            self.faults -= 1;
            return false;
        }
        let blocks = (data.len() / 512) as u32;
        if arg
            .checked_add(blocks)
            .is_none_or(|end| end > self.sectors())
        {
            self.status |= OUT_OF_RANGE;
            return true;
        }
        for (k, block) in data.chunks_exact(512).enumerate() {
            let mut sector = Box::new([0u8; 512]);
            sector.copy_from_slice(block);
            self.sectors.insert(arg + k as u32, sector);
        }
        if self.record {
            self.written.push((arg, blocks));
        }
        true
    }

    /// CMD19's word, written by the host.
    pub(crate) fn bus_test_write(&mut self, word: u32) {
        self.bus_test = word;
    }

    /// CMD14's word: the last CMD19 word inverted.
    pub(crate) fn bus_test_read(&self) -> u32 {
        !self.bus_test
    }
}
