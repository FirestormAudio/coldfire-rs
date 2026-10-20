//! Serial boot's fetch: read the app part from the SPI flash the SBF booted from, over DSPI0 with PCS1 (the SBF's
//! pins, which return to GPIO after the SBF is done, RM 11.4.3 and Table 15-1). One frame in flight, polled.
#![cfg_attr(not(target_arch = "m68k"), allow(dead_code))] // the host builds the tables and their tests only
#[cfg(target_arch = "m68k")]
use super::ops::MARK_SPI;

/// `(register, mask, value)`: PAR_DSPIOWH puts SIN, SOUT and SCK on DSPI0, PAR_DSPIOWL puts PCS1 there (RM 15.3.6.6).
pub const SBF_PINS: [(u32, u8, u8); 2] = [(0xEC09_404E, 0xFC, 0xFC), (0xEC09_404F, 0x80, 0x80)];

const DSPI0: u32 = 0xFC05_C000;
pub(crate) const MCR: u32 = DSPI0;
pub(crate) const CTAR0: u32 = DSPI0 + 0x0C;
pub(crate) const SR: u32 = DSPI0 + 0x2C;
pub(crate) const PUSHR: u32 = DSPI0 + 0x34;
pub(crate) const POPR: u32 = DSPI0 + 0x38;
/// Master, every chip select idling high (PCSIS1), both FIFOs cleared, running (RM 40.3.1).
pub(crate) const MCR_VALUE: u32 = 1 << 31 | 1 << 17 | 1 << 11 | 1 << 10;
#[cfg(target_arch = "m68k")]
const PPMCR0: u32 = 0xFC04_002D;
#[cfg(target_arch = "m68k")]
/// DSPI0's clock slot (RM Table 9-5).
const DSPI0_SLOT: u8 = 23;
#[cfg(target_arch = "m68k")]
const RXCTR: u32 = 0xF << 4;
#[cfg(target_arch = "m68k")]
const CAP: u32 = 1 << 16;

/// READ (0x03) and a 24-bit big-endian address.
#[inline(always)]
pub const fn read_cmd(offset: u32) -> [u8; 4] {
    [0x03, (offset >> 16) as u8, (offset >> 8) as u8, offset as u8]
}

/// A PUSHR word: PCS1, CTAR0, and CONT (keep PCS1 asserted) on every frame but the last.
#[inline(always)]
pub const fn frame(byte: u8, last: bool) -> u32 {
    (!last as u32) << 31 | 1 << 17 | byte as u32
}

/// Exchange one frame.
#[cfg(target_arch = "m68k")]
#[inline(always)]
unsafe fn xfer(byte: u8, last: bool) -> u8 {
    unsafe {
        (PUSHR as *mut u32).write_volatile(frame(byte, last));
        let mut left = CAP;
        while super::exec::read32(SR) & RXCTR == 0 {
            left -= 1;
            if left == 0 {
                super::exec::halt(MARK_SPI);
            }
        }
        super::exec::read32(POPR) as u8
    }
}

#[cfg(target_arch = "m68k")]
#[inline(always)]
unsafe fn pin((r, mask, v): (u32, u8, u8)) {
    unsafe { (r as *mut u8).write_volatile(((r as *const u8).read_volatile() & !mask) | v) };
}

/// Copy `len` bytes from flash byte `offset` to `dst`.
#[cfg(target_arch = "m68k")]
#[unsafe(link_section = ".boot.text.coldfire_rt_spi")]
#[inline(never)]
pub(crate) unsafe fn read(offset: u32, ctar: u32, dst: u32, len: u32) {
    unsafe {
        (PPMCR0 as *mut u8).write_volatile(DSPI0_SLOT);
        // constant indices: the values fold into the code (a loop over the array would read it from .rodata, in DDR)
        pin(SBF_PINS[0]);
        pin(SBF_PINS[1]);
        // CTAR0 only changes while halted (RM 40.3.3)
        (MCR as *mut u32).write_volatile(MCR_VALUE | 1);
        (CTAR0 as *mut u32).write_volatile(ctar);
        (MCR as *mut u32).write_volatile(MCR_VALUE);
        let cmd = read_cmd(offset);
        let mut k = 0;
        while k < 4 {
            xfer(cmd[k], len == 0 && k == 3);
            k += 1;
        }
        let mut n = 0;
        while n < len {
            ((dst + n) as *mut u8).write_volatile(xfer(0, n + 1 == len));
            n += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pinmux::{par_update, Function, ParField, PAR_DSPIOWH, PAR_DSPIOWL};

    #[test]
    fn read_cmd_is_0x03_then_big_endian_24_bit() {
        assert_eq!(read_cmd(0x12_3456), [0x03, 0x12, 0x34, 0x56]);
    }

    #[test]
    fn pins_select_dspi0_for_pcs1_sck_sin_sout() {
        // the same fields the HAL's SPI driver muxes for DSPI0 (spi/pins.rs): SCK, SOUT, SIN in PAR_DSPIOWH, PCS1 in PAR_DSPIOWL
        let h = [(2, 2), (4, 2), (6, 2)].iter().fold(0u8, |v, &(s, w)| par_update(v, ParField::new(PAR_DSPIOWH, s, w), Function::Primary));
        let l = par_update(0, ParField::new(PAR_DSPIOWL, 7, 1), Function::Primary);
        assert_eq!(SBF_PINS, [(0xEC09_4000 + PAR_DSPIOWH as u32, h, h), (0xEC09_4000 + PAR_DSPIOWL as u32, l, l)]);
    }

    #[test]
    fn dspi0_addresses_match_the_pac() {
        use crate::pac;
        let a = |p: *mut u8| p as u32;
        assert_eq!(MCR, a(pac::DSPI0.mcr().as_ptr() as _));
        assert_eq!(CTAR0, a(pac::DSPI0.ctar0().as_ptr() as _));
        assert_eq!(SR, a(pac::DSPI0.sr().as_ptr() as _));
        assert_eq!(PUSHR, a(pac::DSPI0.pushr().as_ptr() as _));
        assert_eq!(POPR, a(pac::DSPI0.popr().as_ptr() as _));
        assert_eq!(0xEC09_4000, a(pac::GPIO.as_ptr() as _));
    }

    #[test]
    fn master_pcs1_idles_high_fifos_cleared_running() {
        assert_eq!(MCR_VALUE, 1 << 31 | 1 << 17 | 1 << 11 | 1 << 10);
        assert_eq!(MCR_VALUE & 1, 0, "not halted");
    }

    #[test]
    fn frames_select_pcs1_and_hold_it_until_the_last() {
        assert_eq!(frame(0x03, false), 1 << 31 | 1 << 17 | 0x03);
        assert_eq!(frame(0xFF, true), 1 << 17 | 0xFF);
    }
}
