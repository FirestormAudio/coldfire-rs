//! NAND boot's fetch: the app part follows the four boot pages in the same boot-page format (1056-byte pages, 996
//! payload bytes and 60 inverted BCH bytes, ECC mode 7), read one page at a time through buffer 0 (RM 22.4.3.1).
#![cfg_attr(not(target_arch = "m68k"), allow(dead_code))] // the host builds the tables and their tests only
use super::fetch::PAYLOAD;

const NFC: u32 = 0xFC0F_C000;
/// Buffer 0, as the CPU sees it outside boot mode (RM 22.4.1).
pub(crate) const BUF0: u32 = NFC;
pub(crate) const CMD1: u32 = NFC + 0x3F00;
pub(crate) const CMD2: u32 = NFC + 0x3F04;
pub(crate) const CAR: u32 = NFC + 0x3F08;
pub(crate) const RAR: u32 = NFC + 0x3F0C;
pub(crate) const SECSZ: u32 = NFC + 0x3F2C;
pub(crate) const CFG: u32 = NFC + 0x3F30;
pub(crate) const ISR: u32 = NFC + 0x3F38;
/// READ's second command byte (0x30), as `NFC_CMD1[BYTE2]`.
pub const CMD1_READ: u32 = 0x30 << 24;
/// READ (0x00) as `NFC_CMD2[BYTE1]`; CODE 0x7EE0: command, two column and three row bytes, the second command, wait
/// for R/B, read data (RM 22.4.3.1); buffer 0; START.
pub const CMD2_READ: u32 = 0x7EE0 << 8 | 1;
/// The boot geometry's page (RM 22.4.4).
pub const PAGE: u32 = 1056;
/// NFC_CFG: keep FAST, IDCNT, TIMEOUT and 16BIT; set ECC mode 7 (32-error BCH), write the ECC status to the buffer at
/// `ECCAD`, one page per command, no auto-increments, and leave boot mode (BTMD clear).
const CFG_KEEP: u32 = 1 << 16 | 0xFF << 8 | 1 << 7;
/// Where the ECC status lands: right after the page. Open question: RM 22.3.14 calls ECCAD a byte address but gives
/// it 9 bits; this takes it in 8-byte units.
const ECCAD: u32 = PAGE / 8;
const CFG_SET: u32 = ECCAD << 22 | 1 << 21 | 0b111 << 17 | 1;
const ISR_IDLE: u32 = 1 << 29;
const ISR_CLEAR: u32 = 1 << 18 | 1 << 17;
/// ECC status byte: page uncorrectable (RM Table 22-19).
const CORFAIL: u8 = 1 << 7;
const CAP: u32 = 1 << 20;

/// NFC_RAR for a row: bits 28 and 24 must be set (RM 22.3.4).
pub const fn rar(row: u32) -> u32 {
    1 << 28 | 1 << 24 | (row & 0xFF_FFFF)
}

/// After `done` of `len` bytes, starting `skip` bytes into the first page: (page index from the first page, offset in
/// its payload, bytes to take from it).
pub const fn chunk(skip: u32, len: u32, done: u32) -> (u32, u32, u32) {
    let pos = skip + done;
    let from = pos % PAYLOAD;
    let left = len - done;
    (pos / PAYLOAD, from, if PAYLOAD - from < left { PAYLOAD - from } else { left })
}

/// Copy `len` bytes from page `first_page` (skipping `skip` payload bytes) to `dst`.
#[cfg(target_arch = "m68k")]
#[unsafe(link_section = ".boot.text.coldfire_rt_nand")]
#[inline(never)]
pub(crate) unsafe fn read(first_page: u32, skip: u32, dst: u32, len: u32) {
    use super::exec::halt;
    use super::ops::{MARK_NAND_BUSY, MARK_NAND_ECC};
    unsafe {
        let w = |a: u32, v: u32| (a as *mut u32).write_volatile(v);
        let r = |a: u32| (a as *const u32).read_volatile();
        w(CFG, (r(CFG) & CFG_KEEP) | CFG_SET);
        w(SECSZ, PAGE);
        w(CMD1, CMD1_READ);
        w(CAR, 0);
        let mut done = 0;
        while done < len {
            let (page, from, n) = chunk(skip, len, done);
            w(ISR, ISR_CLEAR);
            w(RAR, rar(first_page + page));
            w(CMD2, CMD2_READ);
            let mut left = CAP;
            while r(ISR) & ISR_IDLE == 0 {
                left -= 1;
                if left == 0 {
                    halt(MARK_NAND_BUSY);
                }
            }
            if ((BUF0 + ECCAD * 8) as *const u8).read_volatile() & CORFAIL != 0 {
                halt(MARK_NAND_ECC);
            }
            let mut i = 0;
            while i < n {
                ((dst + done + i) as *mut u8).write_volatile(((BUF0 + from + i) as *const u8).read_volatile());
                i += 1;
            }
            done += n;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn page_of_zero_is_first_page() {
        assert_eq!(chunk(0, 2000, 0), (0, 0, 996));
    }

    #[test]
    fn page_of_996_is_next_page() {
        assert_eq!(chunk(0, 2000, 996), (1, 0, 996));
    }

    #[test]
    fn last_partial_page_length() {
        assert_eq!(chunk(0, 2000, 1992), (2, 0, 8));
    }

    #[test]
    fn a_skip_starts_mid_page() {
        assert_eq!(chunk(990, 20, 0), (0, 990, 6));
        assert_eq!(chunk(990, 20, 6), (1, 0, 14));
    }

    #[test]
    fn page_read_registers() {
        assert_eq!(CMD1_READ, 0x30 << 24);                       // BYTE2: READ's second command
        assert_eq!(CMD2_READ, 0x00 << 24 | 0x7EE0 << 8 | 1);     // BYTE1 0x00, RM 22.4.3.1's CODE, buffer 0, START
        assert_eq!(rar(0x01_0203), 1 << 28 | 1 << 24 | 0x01_0203);
    }

    #[test]
    fn nfc_addresses_match_the_pac() {
        use crate::pac;
        let a = |p: *mut u8| p as u32;
        assert_eq!(CMD1, a(pac::NFC.cmd1().as_ptr() as _));
        assert_eq!(CMD2, a(pac::NFC.cmd2().as_ptr() as _));
        assert_eq!(CAR, a(pac::NFC.car().as_ptr() as _));
        assert_eq!(RAR, a(pac::NFC.rar().as_ptr() as _));
        assert_eq!(SECSZ, a(pac::NFC.secsz().as_ptr() as _));
        assert_eq!(CFG, a(pac::NFC.cfg().as_ptr() as _));
        assert_eq!(ISR, a(pac::NFC.isr().as_ptr() as _));
        assert_eq!(BUF0, a(pac::NFC.as_ptr() as _));
    }
}
