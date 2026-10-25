//! An I2C module (RM ch. 42) as a master of instant transfers, with [`I2cTarget`] parts on the bus.

use crate::device::{Device, DeviceCx};
use crate::mem::Size;
use crate::stop::Stop;

const ADR: u32 = 0x0;
const FDR: u32 = 0x4;
const CR: u32 = 0x8;
const SR: u32 = 0xC;
const DR: u32 = 0x10;

/// I2CR (RM Table 42-4).
const IEN: u8 = 0x80;
const IIEN: u8 = 0x40;
const MSTA: u8 = 0x20;
const MTX: u8 = 0x10;
const RSTA: u8 = 0x04;
/// I2SR (RM Table 42-5).
const ICF: u8 = 0x80;
const IBB: u8 = 0x20;
const IAL: u8 = 0x10;
const IIF: u8 = 0x02;
const RXAK: u8 = 0x01;
/// I2SR at reset: ICF and RXAK (RM Table 42-1).
const SR_RESET: u8 = ICF | RXAK;

/// A part on the bus, seen a byte at a time. It answers to one 7-bit address.
pub trait I2cTarget: std::any::Any {
    /// The 7-bit address it answers to.
    fn address(&self) -> u8;
    /// A START or repeated START addressed it; `read` is the R/W bit.
    fn start(&mut self, read: bool);
    /// The master writes `byte`; returns whether it acknowledges.
    fn write(&mut self, byte: u8) -> bool;
    /// The master reads a byte.
    fn read(&mut self) -> u8;
    /// A STOP ended the transaction.
    fn stop(&mut self);
}

/// One I2C module in master mode. A byte takes no time: writing I2DR (transmit) or reading it (receive, after the
/// address) completes the byte at once, setting ICF and, with IIEN, IIF (RM 42.4.3: IIF needs IIEN). The first byte
/// after MSTA rises or after RSTA is the calling address; a part with that address acknowledges it (RXAK 0), and
/// no part leaves RXAK 1. MSTA rising sets IBB, falling clears it (a STOP). I2ADR and I2FDR are stored; the
/// transfer rate does not matter. IIF is cleared by writing 0 to it, and so is IAL (never set).
///
/// Not modelled, and refused with [`Stop::Unmodelled`]: accesses other than bytes, an interrupt request (IIEN
/// raises none; the interrupt controller is not driven), DMAEN, slave mode (I2ADR matching) and arbitration.
pub struct I2c {
    adr: u8,
    fdr: u8,
    cr: u8,
    sr: u8,
    dr: u8,
    /// The next byte written is the calling address.
    address_next: bool,
    /// The index in `targets` of the part addressed, and whether it is read.
    selected: Option<(usize, bool)>,
    targets: Vec<Box<dyn I2cTarget>>,
}

impl I2c {
    /// A module at reset, with no parts on the bus.
    pub fn new() -> I2c {
        I2c {
            adr: 0,
            fdr: 0,
            cr: 0,
            sr: SR_RESET,
            dr: 0,
            address_next: false,
            selected: None,
            targets: Vec::new(),
        }
    }

    /// Put a part on the bus.
    pub fn attach(&mut self, target: Box<dyn I2cTarget>) {
        self.targets.push(target);
    }

    /// The part on the bus at index `n` (in the order attached), as `T`; `None` if it is another type.
    pub fn target<T: 'static>(&self, n: usize) -> Option<&T> {
        let t: &dyn std::any::Any = self.targets.get(n)?.as_ref();
        t.downcast_ref()
    }

    fn done(&mut self) {
        self.sr |= ICF;
        if self.cr & IIEN != 0 {
            self.sr |= IIF;
        }
    }

    fn stop(&mut self) {
        if let Some((n, _)) = self.selected.take() {
            self.targets[n].stop();
        }
        self.sr &= !IBB;
        self.address_next = false;
    }

    fn write_cr(&mut self, v: u8) {
        let was = self.cr;
        self.cr = v & !RSTA;
        if v & IEN == 0 {
            // disabled: the module is reset (RM Table 42-4)
            self.stop();
            self.sr = SR_RESET;
            return;
        }
        if was & MSTA == 0 && v & MSTA != 0 {
            self.sr |= IBB;
            self.address_next = true;
        } else if was & MSTA != 0 && v & MSTA == 0 {
            self.stop();
        } else if v & MSTA != 0 && v & RSTA != 0 {
            self.address_next = true;
        }
    }

    fn write_dr(&mut self, v: u8) {
        self.dr = v;
        if self.cr & (MSTA | MTX) != MSTA | MTX {
            return;
        }
        self.sr &= !ICF;
        let acked = if self.address_next {
            self.address_next = false;
            let read = v & 1 != 0;
            self.selected = self
                .targets
                .iter()
                .position(|t| t.address() == v >> 1)
                .map(|n| (n, read));
            if let Some((n, _)) = self.selected {
                self.targets[n].start(read);
            }
            self.selected.is_some()
        } else {
            match self.selected {
                Some((n, false)) => self.targets[n].write(v),
                _ => false,
            }
        };
        self.sr = self.sr & !RXAK | if acked { 0 } else { RXAK };
        self.done();
    }

    fn read_dr(&mut self) -> u8 {
        let v = self.dr;
        if self.cr & (MSTA | MTX) == MSTA {
            // receive: this read starts the next byte
            self.sr &= !ICF;
            self.dr = match self.selected {
                Some((n, true)) => self.targets[n].read(),
                _ => 0xFF,
            };
            self.done();
        }
        v
    }
}

impl Default for I2c {
    fn default() -> I2c {
        I2c::new()
    }
}

impl Device for I2c {
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32 {
        if size != Size::Byte {
            cx.refuse(
                Stop::Unmodelled,
                format!("I2C {size:?} read at +{offset:#x} is not modelled (8-bit registers)"),
            );
            return 0;
        }
        u32::from(match offset {
            ADR => self.adr,
            FDR => self.fdr,
            CR => self.cr,
            SR => self.sr,
            DR => self.read_dr(),
            _ => 0,
        })
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        if size != Size::Byte {
            cx.refuse(
                Stop::Unmodelled,
                format!("I2C {size:?} write at +{offset:#x} is not modelled (8-bit registers)"),
            );
            return;
        }
        let v = value as u8;
        match offset {
            ADR => self.adr = v & 0xFE,
            FDR => self.fdr = v & 0x3F,
            CR => {
                if v & 0x01 != 0 {
                    cx.refuse(Stop::Unmodelled, "I2C DMAEN is not modelled".to_string());
                    return;
                }
                self.write_cr(v);
            }
            // IIF and IAL clear by writing 0 (RM Table 42-5); the other bits are read-only
            SR => self.sr &= v | !(IIF | IAL),
            DR => self.write_dr(v),
            _ => {}
        }
    }

    fn poll_counted(&self) -> bool {
        false
    }
}
