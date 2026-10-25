//! The ADC (RM ch. 29), minimal: its registers hold what is written, the result registers hold what the host sets.

use crate::device::{Device, DeviceCx};
use crate::mem::Size;

/// The register window (RM Table 29-2): ADC_CR1 at +0 to ADC_ASDIV at +0x5A.
const LEN: usize = 0x5C;
/// ADC_RSLT0 (+0x12, RM Table 29-2), eight 16-bit results.
const RSLT: u32 = 0x12;
/// The reset values of the registers that have one other than 0 (RM Table 29-2): (offset, value).
const RESET: [(usize, u16); 12] = [
    (0x00, 0x5005), // CR1
    (0x02, 0x5080), // CR2
    (0x06, 0x3210), // LST1
    (0x08, 0x7654), // LST2
    (0x32, 0x7FF8), // HLMT0-7
    (0x34, 0x7FF8),
    (0x36, 0x7FF8),
    (0x38, 0x7FF8),
    (0x3A, 0x7FF8),
    (0x3C, 0x7FF8),
    (0x3E, 0x7FF8),
    (0x40, 0x7FF8),
];
/// The tail of the reset values: PWR, PWR2, DIV, ASDIV.
const RESET_TAIL: [(usize, u16); 4] = [
    (0x52, 0x3D8F),
    (0x56, 0x0005),
    (0x58, 0x0505),
    (0x5A, 0x0137),
];

/// The ADC. No conversion runs: a result register reads the value [`Adc::set_result`] gave it (0 until then), every
/// other register reads back what was written (its reset value at first; the status registers never change). The
/// host's value is a whole register: 16 bits, as the 12-bit sample sits in them (RM 29.4.9).
pub struct Adc {
    regs: [u8; LEN],
}

impl Adc {
    /// An ADC with the RM's reset values and results of 0.
    pub fn new() -> Adc {
        let mut regs = [0u8; LEN];
        for (at, v) in RESET.iter().chain(&RESET_TAIL) {
            regs[*at..*at + 2].copy_from_slice(&v.to_be_bytes());
        }
        Adc { regs }
    }

    /// Set result register `n` (0-7) to `value`, as the register reads. Panics on `n` above 7.
    pub fn set_result(&mut self, n: usize, value: u16) {
        assert!(n < 8, "ADC_RSLT{n}: results are 0-7");
        let at = (RSLT as usize) + 2 * n;
        self.regs[at..at + 2].copy_from_slice(&value.to_be_bytes());
    }

    /// Result register `n` (0-7) as it reads.
    pub fn result(&self, n: usize) -> u16 {
        let at = (RSLT as usize) + 2 * n;
        u16::from_be_bytes([self.regs[at], self.regs[at + 1]])
    }
}

impl Default for Adc {
    fn default() -> Adc {
        Adc::new()
    }
}

fn bytes(size: Size) -> u32 {
    match size {
        Size::Byte => 1,
        Size::Word => 2,
        Size::Long => 4,
    }
}

impl Device for Adc {
    fn read(&mut self, offset: u32, size: Size, _cx: &mut DeviceCx) -> u32 {
        (0..bytes(size)).fold(0, |v, i| {
            v << 8 | u32::from(self.regs.get((offset + i) as usize).copied().unwrap_or(0))
        })
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, _cx: &mut DeviceCx) {
        let n = bytes(size);
        for i in 0..n {
            // the result registers are the host's: a write does not change them
            let at = (offset + i) as usize;
            let in_results = (RSLT as usize..RSLT as usize + 16).contains(&at);
            if at < LEN && !in_results {
                self.regs[at] = (value >> (8 * (n - 1 - i))) as u8;
            }
        }
    }

    fn poll_counted(&self) -> bool {
        false
    }
}
