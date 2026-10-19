//! Device-mode data structures (RM §24.5.2): the 64-byte dQH slot and the 32-byte dTD, as native big-endian words,
//! and the bit packing the driver needs.

/// dQH: 48 bytes of hardware fields in a 64-byte slot (the list is 2 KB aligned, so each slot is too).
#[repr(C, align(64))]
pub struct Dqh {
    pub cap: u32,
    pub cur: u32,
    /// Overlay of the active dTD: next pointer, token, buffer pages.
    pub next: u32,
    pub token: u32,
    pub buf: [u32; 5],
    reserved: u32,
    /// The setup packet, little-endian within each word.
    pub setup: [u32; 2],
    software: [u32; 4],
}

#[repr(C, align(32))]
pub struct Dtd {
    pub next: u32,
    pub token: u32,
    pub buf: [u32; 5],
    pad: u32,
}

const _: () = assert!(core::mem::size_of::<Dqh>() == 64 && core::mem::size_of::<Dtd>() == 32);

impl Dqh {
    pub const ZERO: Dqh = Dqh { cap: 0, cur: 0, next: T, token: 0, buf: [0; 5], reserved: 0, setup: [0; 2], software: [0; 4] };
}

impl Dtd {
    pub const ZERO: Dtd = Dtd { next: T, token: 0, buf: [0; 5], pad: 0 };
}

/// Next-pointer terminate bit.
pub const T: u32 = 1;
pub const ACTIVE: u32 = 0x80;
pub const HALTED: u32 = 0x40;
pub const BUFFER_ERROR: u32 = 0x20;
pub const TRANSACTION_ERROR: u32 = 0x08;
pub const IOC: u32 = 1 << 15;
/// dQH capabilities: 1 = no automatic zero-length packet (embassy-usb adds its own).
pub const ZLT: u32 = 1 << 29;
/// dQH capabilities: interrupt on setup.
pub const IOS: u32 = 1 << 15;

pub const fn cap(mps: u16, ios: bool) -> u32 {
    ZLT | ((mps as u32 & 0x7FF) << 16) | if ios { IOS } else { 0 }
}

pub const fn token(len: usize) -> u32 {
    ((len as u32 & 0x7FFF) << 16) | IOC | ACTIVE
}

pub const fn remaining(token: u32) -> usize {
    ((token >> 16) & 0x7FFF) as usize
}

/// Not a clean retirement (RM §24.5.3.6.4: Active, Halted, Data Buffer Error, Transaction Error all 0).
pub const fn failed(token: u32) -> bool {
    token & (ACTIVE | HALTED | BUFFER_ERROR | TRANSACTION_ERROR) != 0
}

/// A dTD's five page pointers for a buffer at `addr` (page 0 carries the offset).
pub const fn pages(addr: u32) -> [u32; 5] {
    let p = addr & !0xFFF;
    [addr, p + 0x1000, p + 0x2000, p + 0x3000, p + 0x4000]
}

/// The setup packet in wire order from the dQH's two words (byte 0 in bits 7..0 of word 0).
pub const fn setup_bytes(w0: u32, w1: u32) -> [u8; 8] {
    let (a, b) = (w0.to_le_bytes(), w1.to_le_bytes());
    [a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]]
}

/// dQH/dTD slot of an endpoint direction: OUT even, IN odd.
pub const fn index(ep: usize, is_in: bool) -> usize {
    2 * ep + is_in as usize
}

/// Its bit in EPPRIME/EPFLUSH/EPSR/EPCOMPLETE.
pub const fn bit(ep: usize, is_in: bool) -> u32 {
    1 << (ep + if is_in { 16 } else { 0 })
}

/// EPCRn after enabling one direction as `ep_type` (the 2-bit USB type) with a data-toggle reset. The
/// other direction, if it has no type yet, becomes bulk so it isn't left as control (RM §24.5.3.3.1).
pub const fn epcr_enable(old: u32, ep_type: u8, is_in: bool) -> u32 {
    let (sh, other) = if is_in { (16, 0) } else { (0, 16) };
    let half = 0x80 | 0x40 | ((ep_type as u32 & 3) << 2);
    let mut v = (old & !(0xFF << sh)) | (half << sh);
    let o = (v >> other) & 0xFF;
    if o & 0x80 == 0 && (o >> 2) & 3 == 0 {
        v |= (2 << 2) << other;
    }
    v
}

pub const fn epcr_disable(old: u32, is_in: bool) -> u32 {
    old & !(if is_in { 0x80 << 16 } else { 0x80 })
}

/// Whether `addr` is in the 64 KB on-chip SRAM (RAMBAR1 = 0x80000000; USB DMA reaches it through the
/// backdoor, RM §24.4.3 / RAMBAR1 BDE).
pub const fn in_sram(addr: u32) -> bool {
    addr >= 0x8000_0000 && addr < 0x8001_0000
}

/// The packet size an endpoint runs with: its max packet size, but 64 at full speed (the descriptors are
/// high-speed's; a full-speed host clamps bulk to 64), and 64 for an IN endpoint whose buffer is not in
/// SRAM: the device TX buffer is 64 bytes (RM §24.4.3) and the rest of a packet streams from memory while
/// on the wire; from DDR that underflows into a forced bit-stuff error the controller retries forever
/// (Table 24-55), which is what a 224-byte reply met on hardware. OUT may be 512 from anywhere (a receive
/// overflow is just NAKed and retried).
pub const fn packet_size(mps: u32, full_speed: bool, is_in: bool, buf: u32) -> u32 {
    if full_speed || (is_in && !in_sram(buf)) { if mps < 64 { mps } else { 64 } } else { mps }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layout_matches_the_controller() {
        assert_eq!(core::mem::offset_of!(Dqh, buf), 0x10);
        assert_eq!(core::mem::offset_of!(Dqh, setup), 0x28);
        assert_eq!(core::mem::align_of::<Dtd>(), 32);
    }

    #[test]
    fn setup_words_unpack_to_wire_order() {
        // GET_DESCRIPTOR(device, 64) as the controller stores it
        assert_eq!(setup_bytes(0x0100_0680, 0x0040_0000), [0x80, 6, 0, 1, 0, 0, 0x40, 0]);
    }

    #[test]
    fn tokens_and_capabilities() {
        assert_eq!(token(18), 0x0012_8080);
        assert_eq!(token(0), 0x0000_8080); // the status-stage token proven on hardware
        assert_eq!(remaining(0x01A4_8000), 0x1A4);
        assert!(!failed(0x01A4_8000) && failed(0x0000_8040) && failed(0x0000_8080));
        assert_eq!(cap(64, true), 0x2040_8000);
        assert_eq!(cap(512, false), 0x2200_0000); // high-speed bulk capabilities proven on hardware
    }

    #[test]
    fn endpoint_control_matches_the_proven_values() {
        let v = epcr_enable(0, 3, true); // CDC notification: interrupt IN
        assert_eq!(v, 0x00CC_0008);
        assert_eq!(epcr_enable(v, 2, false), 0x00CC_00C8); // + bulk OUT: a proven EPCR2
        assert_eq!(epcr_enable(epcr_enable(0, 2, true), 2, false), 0x00C8_00C8); // a proven EPCR1
        assert_eq!(epcr_disable(0x00C8_00C8, true), 0x0048_00C8);
        assert_eq!(epcr_disable(0x00C8_00C8, false), 0x00C8_0048);
    }

    #[test]
    fn in_packets_are_full_size_only_from_sram_at_high_speed() {
        let (sram, ddr) = (0x8000_9000, 0x4700_9000);
        assert_eq!(packet_size(512, false, true, sram), 512);
        assert_eq!(packet_size(512, false, true, ddr), 64);   // the 64-byte TX buffer underflows from DDR
        assert_eq!(packet_size(512, true, true, sram), 64);   // full speed: bulk is 64 at most
        assert_eq!(packet_size(512, false, false, ddr), 512); // OUT: an overflow is only NAKed
        assert_eq!(packet_size(512, true, false, sram), 64);
        assert_eq!(packet_size(16, false, true, ddr), 16);
        assert!(in_sram(0x8000_0000) && in_sram(0x8000_FFFF) && !in_sram(0x8001_0000) && !in_sram(0x7FFF_FFFF));
    }

    #[test]
    fn pages_cover_a_buffer_across_a_page_boundary() {
        assert_eq!(pages(0x4700_1FC0), [0x4700_1FC0, 0x4700_2000, 0x4700_3000, 0x4700_4000, 0x4700_5000]);
        assert_eq!((index(2, true), bit(2, true), bit(3, false)), (5, 1 << 18, 1 << 3));
    }
}
