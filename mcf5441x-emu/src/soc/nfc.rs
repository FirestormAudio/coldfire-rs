//! The NAND flash controller, NFC (RM ch. 22), with a [`NandArray`] behind it: page reads into the buffer.

use std::collections::BTreeSet;

use crate::device::{Device, DeviceCx};
use crate::mem::Size;
use crate::stop::Stop;

/// The registers' offset in the window: the buffer is 0xFC0F_C000.., the registers 0xFC0F_FF00.. (RM Table 22-3).
const REGS: u32 = 0x3F00;
/// One buffer's size (RM 22.4.1).
const BUFFER_BYTES: usize = 0x900;

const CMD1: u32 = 0x00;
const CMD2: u32 = 0x04;
const CAR: u32 = 0x08;
const RAR: u32 = 0x0C;
const RPT: u32 = 0x10;
const RAI: u32 = 0x14;
const SR1: u32 = 0x18;
const SR2: u32 = 0x1C;
const DMA1: u32 = 0x20;
const DMACFG: u32 = 0x24;
const SWAP: u32 = 0x28;
const SECSZ: u32 = 0x2C;
const CFG: u32 = 0x30;
const DMA2: u32 = 0x34;
const ISR: u32 = 0x38;

/// Reset values (RM Table 22-3, all measured on an MCF54415).
const CMD1_RESET: u32 = 0x30FF_0000;
const CMD2_RESET: u32 = 0x007E_E000;
const RAR_RESET: u32 = 0x1100_0000;
const RAI_RESET: u32 = 1;
const SWAP_RESET: u32 = 0x0FFE_0FFE;
const SECSZ_RESET: u32 = 0x420;
const CFG_RESET: u32 = 0x000E_A631;
const ISR_RESET: u32 = DONE | IDLE;

/// NFC_CMD2 (RM Table 22-5): BYTE1, CODE, BUFNO and BUSY/START; bits 7:3 are reserved.
const CMD2_BITS: u32 = 0xFFFF_FF07;
const BUSY_START: u32 = 1;
/// NFC_CMD2\[CODE\] bits (RM Table 22-22).
const CODE_DMA_TO_SRAM: u32 = 1 << 15;
const CODE_COLUMN: u32 = 3 << 12;
const CODE_WRITE_DATA: u32 = 1 << 8;
const CODE_READ_DATA: u32 = 1 << 5;
const CODE_BYTE3: u32 = 1 << 4;
const CODE_READ_STATUS: u32 = 1 << 3;
const CODE_READ_ID: u32 = 1 << 2;
/// NFC_CFG (RM Table 22-17).
const ECCSRAM: u32 = 1 << 21;
const DMAREQ: u32 = 1 << 20;
const BTMD: u32 = 1 << 6;
const AIAD: u32 = 1 << 5;
const PAGECNT: u32 = 0xF;
/// NFC_ISR (RM Table 22-18).
const WERR: u32 = 1 << 31;
const DONE: u32 = 1 << 30;
const IDLE: u32 = 1 << 29;
const ENABLES: u32 = 7 << 20;
const WERRCLR: u32 = 1 << 19;
const DONECLR: u32 = 1 << 18;
const IDLECLR: u32 = 1 << 17;
/// NFC_SWAP ADDR1 and ADDR2 (RM Table 22-15).
const SWAP_BITS: u32 = 0x0FFE_0FFE;
/// ECC status byte CORFAIL (RM Table 22-19).
const CORFAIL: u8 = 0x80;

/// The NAND flash behind the controller: what a page read of a row returns.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct NandArray {
    /// The data of each row, from row 0; a row past the end reads erased (all 0xFF).
    pub pages: Vec<Vec<u8>>,
    /// The rows whose reads the ECC cannot correct.
    pub bad: BTreeSet<u32>,
    /// The bytes a page read puts in the buffer: the row's data, cut or padded with 0xFF to this length.
    pub payload: usize,
}

/// The NFC (RM ch. 22), mapped at 0xFC0F_C000 (0x4000 bytes): the buffer at +0..+0x3EFF and the registers at
/// +0x3F00, with a [`NandArray`] behind it. Page reads only; the NFC's own DMA, interrupt and boot mode are not
/// modelled.
///
/// The buffer is one byte array taking accesses of any size, big-endian (RM 22.4.1: the CPU sees each buffer at
/// a contiguous address outside boot mode; guess: the gaps between the buffers read and write like the rest).
///
/// Registers (32-bit accesses only; anything else is refused with [`Stop::Unmodelled`]), with their reset values
/// (RM Table 22-3, all measured on an MCF54415):
/// - CMD1 (0x30FF_0000), CAR (0), RAR (0x1100_0000), RPT (0), RAI (1), DMA1 (0), DMACFG (0), SECSZ (0x420), CFG
///   (0x000E_A631), DMA2 (0): stored, the bits the RM marks as reading 0 dropped in CMD1, CAR, RPT, RAI and SECSZ.
/// - CMD2 (0x007E_E000): stored; a write with BUSY/START runs the command (below), which is done at once:
///   BUSY/START reads 0.
/// - SR1, SR2 (0): read-only (nothing reads an ID or a status).
/// - SWAP (0x0FFE_0FFE): a write naming two different swap addresses is refused (the cache swap is not modelled).
/// - ISR (0x6000_0000, DONE and IDLE): WERRCLR, DONECLR and IDLECLR clear WERR, DONE and IDLE; writing the status
///   bits themselves does nothing. A write setting WERREN, DONEEN or IDLEEN is refused (the NFC interrupt is not
///   modelled).
///
/// The rest of the register window reads 0 and ignores writes.
///
/// A command is a page read (RM Figure 22-18) when its CODE reads data (bit 5) and neither writes (bits 15 and 8)
/// nor sends BYTE3, reads the status or reads the ID (bits 4, 3, 2). Any other command is refused, as are page
/// reads the model cannot do exactly: a repeat count (RPT) above 1, a column address other than 0 sent to the
/// flash, a BUFNO other than 0, CFG\[DMAREQ\] (the NFC's DMA), CFG\[PAGECNT\] above 1, and CFG\[BTMD\] (boot
/// mode's address hashing). A page read of row `RAR & 0xFF_FFFF`:
/// 1. records the row ([`Nfc::reads`]);
/// 2. puts the row's data, padded with 0xFF, in the buffer from byte 0 up to the array's `payload`;
/// 3. with CFG\[ECCSRAM\] set, writes the ECC status byte at buffer byte `CFG[ECCAD] * 8`: 0x80 (CORFAIL) for a
///    bad row, else 0 (RM 22.4.2 and Table 22-19); without it, writes nothing (RM Table 22-17);
/// 4. with CFG\[AIAD\] set, adds RAI to RAR's row bytes (guess: RM Figure 22-18 does it after each page, also
///    when there is only one);
/// 5. sets ISR\[DONE\] and ISR\[IDLE\]. The read takes no time.
#[derive(Debug)]
pub struct Nfc {
    array: NandArray,
    buffer: Vec<u8>,
    cmd1: u32,
    cmd2: u32,
    car: u32,
    rar: u32,
    rpt: u32,
    rai: u32,
    dma1: u32,
    dmacfg: u32,
    swap: u32,
    secsz: u32,
    cfg: u32,
    dma2: u32,
    isr: u32,
    reads: Vec<u32>,
}

impl Nfc {
    /// The NFC as it leaves reset, with `array` behind it.
    ///
    /// # Panics
    /// When `array.payload` is larger than one buffer (0x900 bytes, RM 22.4.1).
    pub fn new(array: NandArray) -> Nfc {
        assert!(
            array.payload <= BUFFER_BYTES,
            "NFC: a payload of {} bytes does not fit a 0x900-byte buffer",
            array.payload
        );
        Nfc {
            array,
            buffer: vec![0; REGS as usize],
            cmd1: CMD1_RESET,
            cmd2: CMD2_RESET,
            car: 0,
            rar: RAR_RESET,
            rpt: 0,
            rai: RAI_RESET,
            dma1: 0,
            dmacfg: 0,
            swap: SWAP_RESET,
            secsz: SECSZ_RESET,
            cfg: CFG_RESET,
            dma2: 0,
            isr: ISR_RESET,
            reads: Vec::new(),
        }
    }

    /// The array behind the controller.
    pub fn array(&self) -> &NandArray {
        &self.array
    }

    /// The rows read so far, in order.
    pub fn reads(&self) -> &[u32] {
        &self.reads
    }

    /// Why the command in `cmd2` cannot run, if it cannot.
    fn check(&self, cmd2: u32) -> Option<String> {
        let code = cmd2 >> 8 & 0xFFFF;
        let refused =
            CODE_DMA_TO_SRAM | CODE_WRITE_DATA | CODE_BYTE3 | CODE_READ_STATUS | CODE_READ_ID;
        if code & CODE_READ_DATA == 0 || code & refused != 0 {
            return Some(format!(
                "NFC: command code {code:#06x} (only page reads are modelled)"
            ));
        }
        if self.rpt > 1 {
            return Some(format!("NFC: command repeat (RPT {})", self.rpt));
        }
        if code & CODE_COLUMN != 0 && self.car != 0 {
            return Some(format!(
                "NFC: page read from column {:#x} (only column 0 is modelled)",
                self.car
            ));
        }
        let bufno = cmd2 >> 1 & 3;
        if bufno != 0 {
            return Some(format!(
                "NFC: page read into buffer {bufno} (only buffer 0 is modelled)"
            ));
        }
        if self.cfg & DMAREQ != 0 {
            return Some("NFC: page read with CFG[DMAREQ] (the NFC's DMA is not modelled)".into());
        }
        if self.cfg & PAGECNT > 1 {
            return Some(format!(
                "NFC: page read of {} virtual pages (CFG[PAGECNT] above 1 is not modelled)",
                self.cfg & PAGECNT
            ));
        }
        if self.cfg & BTMD != 0 {
            return Some(
                "NFC: page read in boot mode (CFG[BTMD] address hashing is not modelled)".into(),
            );
        }
        None
    }

    fn read_page(&mut self) {
        let row = self.rar & 0x00FF_FFFF;
        self.reads.push(row);
        let payload = self.array.payload;
        let data = self.array.pages.get(row as usize).map_or(&[][..], |p| p);
        let n = data.len().min(payload);
        self.buffer[..n].copy_from_slice(&data[..n]);
        self.buffer[n..payload].fill(0xFF);
        if self.cfg & ECCSRAM != 0 {
            let at = (self.cfg >> 22 & 0x1FF) as usize * 8;
            self.buffer[at] = if self.array.bad.contains(&row) {
                CORFAIL
            } else {
                0
            };
        }
        if self.cfg & AIAD != 0 {
            self.rar = self.rar & 0xFF00_0000 | (self.rar + self.rai) & 0x00FF_FFFF;
        }
        self.isr |= DONE | IDLE;
    }

    fn write_register(&mut self, reg: u32, value: u32, cx: &mut DeviceCx) {
        match reg {
            CMD1 => self.cmd1 = value & 0xFFFF_0000,
            CMD2 if value & BUSY_START != 0 => {
                if let Some(why) = self.check(value) {
                    return cx.refuse(Stop::Unmodelled, why);
                }
                self.cmd2 = value & CMD2_BITS & !BUSY_START;
                self.read_page();
            }
            CMD2 => self.cmd2 = value & CMD2_BITS,
            CAR => self.car = value & 0xFFFF,
            RAR => self.rar = value,
            RPT => self.rpt = value & 0xFFFF,
            RAI => self.rai = value & 0x00FF_FFFF,
            DMA1 => self.dma1 = value,
            DMACFG => self.dmacfg = value,
            SWAP => {
                let v = value & SWAP_BITS;
                if v >> 17 != v >> 1 & 0x7FF {
                    return cx.refuse(
                        Stop::Unmodelled,
                        format!("NFC: SWAP {value:#010x} (the cache swap is not modelled)"),
                    );
                }
                self.swap = v;
            }
            SECSZ => self.secsz = value & 0x1FFF,
            CFG => self.cfg = value,
            DMA2 => self.dma2 = value,
            ISR => {
                if value & ENABLES != 0 {
                    return cx.refuse(
                        Stop::Unmodelled,
                        format!("NFC: ISR {value:#010x} sets interrupt enables (the NFC interrupt is not modelled)"),
                    );
                }
                for (clear, bit) in [(WERRCLR, WERR), (DONECLR, DONE), (IDLECLR, IDLE)] {
                    if value & clear != 0 {
                        self.isr &= !bit;
                    }
                }
            }
            _ => {}
        }
    }
}

impl Device for Nfc {
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32 {
        if offset < REGS {
            let at = offset as usize;
            let bytes = &self.buffer[at..at + size.bytes() as usize];
            return bytes.iter().fold(0, |v, &b| v << 8 | b as u32);
        }
        if size != Size::Long {
            cx.refuse(
                Stop::Unmodelled,
                format!(
                    "NFC: {}-byte read at +{offset:#x} (32-bit register accesses only)",
                    size.bytes()
                ),
            );
            return 0;
        }
        match offset - REGS {
            CMD1 => self.cmd1,
            CMD2 => self.cmd2,
            CAR => self.car,
            RAR => self.rar,
            RPT => self.rpt,
            RAI => self.rai,
            SR1 | SR2 => 0,
            DMA1 => self.dma1,
            DMACFG => self.dmacfg,
            SWAP => self.swap,
            SECSZ => self.secsz,
            CFG => self.cfg,
            DMA2 => self.dma2,
            ISR => self.isr,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        if offset < REGS {
            let at = offset as usize;
            let n = size.bytes() as usize;
            for (i, b) in self.buffer[at..at + n].iter_mut().enumerate() {
                *b = (value >> (8 * (n - 1 - i))) as u8;
            }
            return;
        }
        if size != Size::Long {
            return cx.refuse(
                Stop::Unmodelled,
                format!(
                    "NFC: {}-byte write at +{offset:#x} (32-bit register accesses only)",
                    size.bytes()
                ),
            );
        }
        self.write_register(offset - REGS, value, cx);
    }

    fn poll_counted(&self) -> bool {
        false
    }

    /// No events, no interrupts.
    fn wake_sources(&self, _out: &mut Vec<(usize, u8)>) -> bool {
        true
    }
}
