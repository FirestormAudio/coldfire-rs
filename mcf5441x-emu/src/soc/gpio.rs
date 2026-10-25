//! The GPIO ports' data registers (RM ch. 16), served inside the GPIO stub block ([`GpioPorts::RANGE`]).

use std::ops::Range;

use crate::device::{Device, DeviceCx};
use crate::mem::Size;

/// Ports A-K.
const PORTS: usize = 11;
const PODR: u32 = 0x00;
const PDDR: u32 = 0x0C;
const PPDSDR: u32 = 0x18;
const PCLRR: u32 = 0x24;
const PCR: u32 = 0x30;
/// PODR at reset, every port (measured on an MCF54415).
const PODR_RESET: u8 = 0xFF;
/// The PCRs at reset: PCR_A 0x0300, PCR_J 0x0008, PCR_K 0x0002, the rest 0 (measured on an MCF54415).
const PCR_RESET: [u16; PORTS] = [0x0300, 0, 0, 0, 0, 0, 0, 0, 0, 0x0008, 0x0002];

/// A write that changed a port's direction or output latch, as [`GpioPorts::changes`] records it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PinChange {
    /// The tick of the write.
    pub at: u64,
    /// The port: A = 0 ... K = 10.
    pub port: u8,
    /// PDDR after the write: bit n set makes pin n an output.
    pub pddr: u8,
    /// PODR after the write: the output latch.
    pub podr: u8,
}

/// The data registers of GPIO ports A-K (RM ch. 16), one byte per port, at their offsets from the GPIO base
/// 0xEC09_4000:
///
/// - PODR (+0x00 + port): the output latch.
/// - PDDR (+0x0C + port): the direction, a set bit an output.
/// - PPDSDR (+0x18 + port): a read gives the pin states ([`GpioPorts::level`]); a write sets the PODR bits written as
///   1 and leaves the rest.
/// - PCLRR (+0x24 + port): a write clears the PODR bits written as 0 and leaves the rest; it reads 0.
/// - PCR (+0x30 + 2 × port, 16 bits): pin n's PUE is bit 2n+1 and PUS bit 2n (1: pull up), stored as written.
///
/// Wider accesses are taken a byte at a time, big-endian. The gaps between the blocks (+0x0B, +0x17, +0x23, +0x2F,
/// +0x46 and +0x47) read 0 and drop writes. At reset PODR is 0xFF on every port, PDDR 0, PCR_A 0x0300 (PA4 pulled
/// up), PCR_J 0x0008 (PJ1 pulled down), PCR_K 0x0002 (PK0 pulled down) and every other PCR 0, so every pin starts
/// as an input (all measured on an MCF54415). The pin multiplexing (PAR) is not consulted: a pin's direction and
/// level are as if its pad were GPIO, where on the part PPDSDR reads 0 for a pad muxed away from GPIO (measured on
/// an MCF54415). The host drives inputs ([`GpioPorts::drive`]); an input it leaves undriven reads its pull, and
/// with no pull reads 1 (guess: the pad's keeper). Each write that changes a port's direction or latch from the
/// last logged state (the reset state at first) is logged ([`GpioPorts::changes`]) unless [`GpioPorts::record`]
/// turned the log off.
pub struct GpioPorts {
    podr: [u8; PORTS],
    pddr: [u8; PORTS],
    pcr: [u16; PORTS],
    /// The host's drive: (pins driven, their levels) per port.
    driven: [(u8, u8); PORTS],
    /// Board nets: (input pin, source pin), each as (port, mask).
    links: Vec<((usize, u8), (usize, u8))>,
    changes: Vec<PinChange>,
    record: bool,
}

impl Default for GpioPorts {
    fn default() -> GpioPorts {
        GpioPorts::new()
    }
}

impl GpioPorts {
    /// The GPIO offsets these registers take: PODR_A at +0x00 to PCR_K at +0x45, and the gap after it.
    pub const RANGE: Range<u32> = 0x00..0x48;

    /// Every register at its reset value and no pin driven.
    pub fn new() -> GpioPorts {
        GpioPorts {
            podr: [PODR_RESET; PORTS],
            pddr: [0; PORTS],
            pcr: PCR_RESET,
            driven: [(0, 0); PORTS],
            links: Vec::new(),
            changes: Vec::new(),
            record: true,
        }
    }

    /// Drive pin `bit` of `port` (A = 0 ... K = 10) at `level`, or leave it to its pull (`None`). An output ignores
    /// the drive until it is an input again. Panics on a port above 10 or a bit above 7.
    pub fn drive(&mut self, port: u8, bit: u8, level: Option<bool>) {
        let (p, mask) = pin(port, bit);
        let (driven, levels) = &mut self.driven[p];
        match level {
            Some(l) => {
                *driven |= mask;
                *levels = if l { *levels | mask } else { *levels & !mask };
            }
            None => *driven &= !mask,
        }
    }

    /// Wire pin `to` to pin `from` on the board: while `from` is an output and `to` an input, `to` reads `from`'s
    /// output latch; otherwise `to` reads the host's drive or its pull as usual. The pin multiplexing is not
    /// consulted. Panics on a port above 10 or a bit above 7.
    pub fn link(&mut self, to: (u8, u8), from: (u8, u8)) {
        self.links.push((pin(to.0, to.1), pin(from.0, from.1)));
    }

    /// The level the pin shows: its PODR bit when PDDR makes it an output, else the host's drive or its pull.
    pub fn level(&self, port: u8, bit: u8) -> bool {
        let (p, mask) = pin(port, bit);
        self.levels(p) & mask != 0
    }

    /// The pin's output level, `None` while it is an input.
    pub fn output(&self, port: u8, bit: u8) -> Option<bool> {
        let (p, mask) = pin(port, bit);
        (self.pddr[p] & mask != 0).then_some(self.podr[p] & mask != 0)
    }

    /// Every logged change so far, in order.
    pub fn changes(&self) -> &[PinChange] {
        &self.changes
    }

    /// Every logged change so far, emptying the log.
    pub fn take_changes(&mut self) -> Vec<PinChange> {
        std::mem::take(&mut self.changes)
    }

    /// Whether changes are logged (default on).
    pub fn record(&mut self, on: bool) {
        self.record = on;
    }

    /// Port `p`'s pin states, as PPDSDR reads them.
    fn levels(&self, p: usize) -> u8 {
        let (driven, levels) = self.driven[p];
        let mut inputs = 0;
        for n in 0..8 {
            let mask = 1 << n;
            let level = if driven & mask != 0 {
                levels & mask != 0
            } else {
                // PUE clear (no pull) reads 1, as pull up does.
                let pull = self.pcr[p] >> (2 * n) & 0b11;
                pull != 0b10
            };
            if level {
                inputs |= mask;
            }
        }
        for &((tp, tm), (sp, sm)) in &self.links {
            if tp == p && self.pddr[sp] & sm != 0 {
                inputs = if self.podr[sp] & sm != 0 { inputs | tm } else { inputs & !tm };
            }
        }
        (self.podr[p] & self.pddr[p]) | (inputs & !self.pddr[p])
    }

    fn read_byte(&self, offset: u32) -> u8 {
        let port = |base: u32| (offset - base) as usize;
        match offset {
            o if o < PODR + PORTS as u32 => self.podr[port(PODR)],
            o if (PDDR..PDDR + PORTS as u32).contains(&o) => self.pddr[port(PDDR)],
            o if (PPDSDR..PPDSDR + PORTS as u32).contains(&o) => self.levels(port(PPDSDR)),
            o if (PCR..PCR + 2 * PORTS as u32).contains(&o) => {
                let word = self.pcr[port(PCR) / 2];
                if (o - PCR) % 2 == 0 {
                    (word >> 8) as u8
                } else {
                    word as u8
                }
            }
            // PCLRR and the gaps
            _ => 0,
        }
    }

    fn write_byte(&mut self, offset: u32, value: u8, now: u64) {
        let port = |base: u32| (offset - base) as usize;
        let p = match offset {
            o if o < PODR + PORTS as u32 => {
                self.podr[port(PODR)] = value;
                port(PODR)
            }
            o if (PDDR..PDDR + PORTS as u32).contains(&o) => {
                self.pddr[port(PDDR)] = value;
                port(PDDR)
            }
            o if (PPDSDR..PPDSDR + PORTS as u32).contains(&o) => {
                self.podr[port(PPDSDR)] |= value;
                port(PPDSDR)
            }
            o if (PCLRR..PCLRR + PORTS as u32).contains(&o) => {
                self.podr[port(PCLRR)] &= value;
                port(PCLRR)
            }
            o if (PCR..PCR + 2 * PORTS as u32).contains(&o) => {
                let word = &mut self.pcr[port(PCR) / 2];
                *word = if (o - PCR) % 2 == 0 {
                    *word & 0x00FF | u16::from(value) << 8
                } else {
                    *word & 0xFF00 | u16::from(value)
                };
                return;
            }
            _ => return,
        };
        let (pddr, podr) = (self.pddr[p], self.podr[p]);
        let last = self.changes.iter().rev().find(|c| c.port as usize == p);
        let before = last.map_or((0, PODR_RESET), |c| (c.pddr, c.podr));
        if self.record && before != (pddr, podr) {
            self.changes.push(PinChange {
                at: now,
                port: p as u8,
                pddr,
                podr,
            });
        }
    }
}

/// The port index and bit mask of a pin.
fn pin(port: u8, bit: u8) -> (usize, u8) {
    assert!(
        (port as usize) < PORTS && bit < 8,
        "GPIO pin {port}.{bit}: ports 0-10, bits 0-7"
    );
    (port as usize, 1 << bit)
}

fn bytes(size: Size) -> u32 {
    match size {
        Size::Byte => 1,
        Size::Word => 2,
        Size::Long => 4,
    }
}

impl Device for GpioPorts {
    fn read(&mut self, offset: u32, size: Size, _cx: &mut DeviceCx) -> u32 {
        (0..bytes(size)).fold(0, |v, i| v << 8 | u32::from(self.read_byte(offset + i)))
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        let n = bytes(size);
        for i in 0..n {
            self.write_byte(offset + i, (value >> (8 * (n - 1 - i))) as u8, cx.now);
        }
    }

    fn poll_counted(&self) -> bool {
        false
    }
}
