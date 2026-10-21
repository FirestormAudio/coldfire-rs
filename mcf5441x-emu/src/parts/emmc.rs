//! An eMMC card: the read-only subset of the MMC command set a boot loader and a block driver use (JEDEC eMMC,
//! JESD84), behind the eSDHC ([`Esdhc`](crate::soc::Esdhc)).

use std::fmt;

/// CMD6 SWITCH arguments: access mode (bits 25:24), EXT_CSD index (23:16), value (15:8).
const WRITE_BYTE: u32 = 3;
const BUS_WIDTH: usize = 183;
const HS_TIMING: usize = 185;
/// EXT_CSD SEC_COUNT, bytes 212-215, little-endian.
const SEC_COUNT: usize = 212;

/// The R1 card status the card answers with: CURRENT_STATE tran (4 << 9) and READY_FOR_DATA (bit 8).
const R1: u32 = 0x900;

/// The commands modelled: no writes, erases or locks.
const MODELLED: [u8; 14] = [0, 1, 2, 3, 6, 7, 8, 9, 10, 12, 14, 16, 18, 19];

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
    /// The bus-test pattern from the card (CMD14).
    BusTestRead,
    /// The bus-test pattern to the card (CMD19).
    BusTestWrite,
}

/// An eMMC with sector addressing and 512-byte blocks, read-only, behind an eSDHC.
///
/// Its contents are `image`: sector `n` is bytes `512 n .. 512 n + 512`, and reads past the end of the image are 0.
/// The card answers:
/// - CMD0 (GO_IDLE): back to a 1-bit bus; CMD1 (SEND_OP_COND): the OCR; CMD2 and CMD10: the CID; CMD9: the CSD;
///   every other command R1 with the card in the transfer state, ready for data. The card has no state machine
///   beyond its bus width and the stuck-sending state below: no RCA, select or block length is checked.
/// - CMD6 (SWITCH): only writes of a byte to BUS_WIDTH (183; 0 for a 1-bit bus, 1 for a 4-bit bus) and HS_TIMING
///   (185), which change the card's EXT_CSD. The controller holds DAT0 low after it (the card is busy).
/// - CMD8 (SEND_EXT_CSD): the 512-byte EXT_CSD, all zero but SEC_COUNT and the board's bytes; each 4-byte group
///   reversed, which is where a 32-bit DATPORT read puts each byte (measured on an MCF54415: byte `i` lands at
///   `i ^ 3`).
/// - CMD18 (READ_MULTIPLE_BLOCK): the blocks from the sector in its argument.
/// - CMD19 (BUS_TEST_W) takes a word, CMD14 (BUS_TEST_R) returns it inverted.
/// - CMD3, CMD7, CMD12 and CMD16: R1. CMD12 ends the stuck-sending state.
///
/// Any other command, and any other CMD6, is not modelled: the controller refuses the XFERTYP write that would
/// send it.
///
/// Faults ([`Emmc::faults`]): each CMD18 while the count is above 0 takes one off, fails its CRC and leaves the
/// card sending: until CMD12 it answers no command but CMD12 (the controller sees a command timeout).
pub struct Emmc {
    image: Vec<u8>,
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
}

impl fmt::Debug for Emmc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Emmc")
            .field("image", &self.image.len())
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
            image,
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
        }
    }

    /// The number of CMD18s still to fail: each fails its CRC and leaves the card sending until CMD12.
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
                "eMMC: CMD{cmd} is not modelled (read-only command set)"
            ));
        }
        if cmd == 6 {
            let (access, index, value) =
                (arg >> 24 & 3, (arg >> 16 & 0xFF) as usize, arg >> 8 & 0xFF);
            let ok = access == WRITE_BYTE
                && match index {
                    BUS_WIDTH => value <= 1,
                    HS_TIMING => true,
                    _ => false,
                };
            if !ok {
                return Some(format!(
                    "eMMC: CMD6 {arg:#010x} is not modelled (only byte writes of BUS_WIDTH 0 or 1 and HS_TIMING)"
                ));
            }
        }
        None
    }

    /// How command `cmd`'s data moves, when it has a data phase.
    pub(crate) fn transfer(cmd: u8) -> Option<Transfer> {
        match cmd {
            8 | 18 => Some(Transfer::Blocks),
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
        let start = u64::from(arg) * 512;
        let len = u64::from(blocks) * 512;
        let mut out = vec![0; len as usize];
        let size = self.image.len() as u64;
        if start < size {
            let end = (start + len).min(size);
            out[..(end - start) as usize]
                .copy_from_slice(&self.image[start as usize..end as usize]);
        }
        Some(out)
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
