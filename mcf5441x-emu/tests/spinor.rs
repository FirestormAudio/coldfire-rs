//! The SPI NOR flash part (an S25FL-S subset), driven through its byte protocol with no DSPI: its status, error and
//! protection behaviour, as a flash writer relies on it.

use mcf5441x_emu::parts::SpiNor;
use mcf5441x_emu::soc::dspi::SpiDevice;

const SIZE: u32 = 16 << 20;
const SECTOR: u32 = 0x40000;
/// ID byte 4 = 0x00: uniform 256 KiB sectors.
const JEDEC_256K: [u8; 6] = [0x01, 0x20, 0x18, 0x4D, 0x00, 0x80];
/// ID byte 4 = 0x01: 64 KiB sectors.
const JEDEC_64K: [u8; 6] = [0x01, 0x20, 0x18, 0x4D, 0x01, 0x80];

/// One command: select, shift `bytes`, deselect; returns the bytes shifted in.
fn cmd(nor: &mut SpiNor, bytes: &[u8]) -> Vec<u8> {
    nor.select(0);
    let out = bytes.iter().map(|&b| nor.transfer(b)).collect();
    nor.deselect(0);
    out
}

fn a3(addr: u32) -> [u8; 3] {
    [(addr >> 16) as u8, (addr >> 8) as u8, addr as u8]
}

fn a4(addr: u32) -> [u8; 4] {
    addr.to_be_bytes()
}

fn rdsr(nor: &mut SpiNor) -> u8 {
    cmd(nor, &[0x05, 0])[1]
}

fn with(op: u8, addr: &[u8], data: &[u8]) -> Vec<u8> {
    let mut v = vec![op];
    v.extend_from_slice(addr);
    v.extend_from_slice(data);
    v
}

fn nor256() -> SpiNor {
    SpiNor::new(SIZE, JEDEC_256K, &[])
}

#[test]
fn unit_id_selects_256k_sectors() {
    let mut nor = nor256();
    assert_eq!(nor.sector_size(), SECTOR);
    assert_eq!(cmd(&mut nor, &[0x9F, 0, 0, 0, 0, 0, 0])[1..], JEDEC_256K);
    assert_eq!(nor.take_unmodelled(), None);
}

#[test]
fn erase_reports_busy_then_done_and_erases_the_sector() {
    let mut contents = vec![0xFF; SECTOR as usize + 4];
    contents[SECTOR as usize..].fill(0);
    let mut nor = SpiNor::new(SIZE, JEDEC_256K, &contents);
    cmd(&mut nor, &[0x06]);
    cmd(&mut nor, &with(0xD8, &a3(SECTOR + 0x123), &[]));
    // guess: WIP lasts a number of status reads (2 by default), not a time
    let polls: Vec<u8> = (0..3).map(|_| rdsr(&mut nor) & 1).collect();
    assert_eq!(polls, [1, 1, 0]);
    assert_eq!(nor.memory()[SECTOR as usize..][..4], [0xFF; 4]);
}

#[test]
fn program_wraps_at_256_on_64k_parts() {
    let mut nor = SpiNor::new(SIZE, JEDEC_64K, &[]);
    assert_eq!(nor.page_size(), 256);
    cmd(&mut nor, &with(0x02, &a3(0x100), &[0x12])); // no WREN: ignored
    assert_eq!(nor.memory()[0x100], 0xFF);
    cmd(&mut nor, &[0x06]);
    cmd(&mut nor, &with(0x02, &a3(0x1FF), &[0xAA, 0xBB])); // wraps to 0x100
    assert_eq!((nor.memory()[0x1FF], nor.memory()[0x100]), (0xAA, 0xBB));
    assert_eq!(nor.memory()[0x200], 0xFF);
}

#[test]
fn program_wraps_at_512_on_256k_parts() {
    // S25FL-S datasheet: the uniform 256 KiB sector parts have 512-byte program pages
    let mut nor = nor256();
    assert_eq!(nor.page_size(), 512);
    cmd(&mut nor, &with(0x02, &a3(0x200), &[0x12])); // no WREN: ignored
    assert_eq!(nor.memory()[0x200], 0xFF);
    cmd(&mut nor, &[0x06]);
    cmd(&mut nor, &with(0x02, &a3(0x3FF), &[0xAA, 0xBB])); // wraps to 0x200, not 0x300
    let m = nor.memory();
    assert_eq!(
        (m[0x3FF], m[0x200], m[0x300], m[0x400]),
        (0xAA, 0xBB, 0xFF, 0xFF)
    );
    // a page program crosses no 256-byte boundary of its own
    cmd(&mut nor, &[0x06]);
    cmd(&mut nor, &with(0x02, &a3(0x4FF), &[0x01, 0x02]));
    assert_eq!((nor.memory()[0x4FF], nor.memory()[0x500]), (0x01, 0x02));
}

#[test]
fn dyb_protected_sector_rejects_erase_and_program_with_error_bits() {
    let mut nor = SpiNor::new(SIZE, JEDEC_256K, &vec![0; SECTOR as usize + 1]);
    cmd(&mut nor, &[0x06]);
    cmd(&mut nor, &with(0xE1, &a4(SECTOR), &[0x00])); // DYBWR: protect
    assert_eq!(
        *cmd(&mut nor, &with(0xE0, &a4(SECTOR), &[0]))
            .last()
            .unwrap(),
        0x00
    );
    cmd(&mut nor, &[0x06]);
    cmd(&mut nor, &with(0xD8, &a3(SECTOR), &[]));
    let sr = rdsr(&mut nor);
    assert!(
        sr & 0x20 != 0 && sr & 0x01 != 0,
        "E_ERR, and WIP until CLSR: {sr:#x}"
    );
    assert_eq!(nor.memory()[SECTOR as usize], 0x00);
    cmd(&mut nor, &[0x30]); // CLSR
    assert_eq!(rdsr(&mut nor) & 0x61, 0);
    cmd(&mut nor, &[0x06]);
    cmd(&mut nor, &with(0x02, &a3(SECTOR), &[0x00]));
    assert_ne!(rdsr(&mut nor) & 0x40, 0, "P_ERR");
    cmd(&mut nor, &[0x30]);
    cmd(&mut nor, &[0x06]);
    cmd(&mut nor, &with(0xE1, &a4(SECTOR), &[0xFF])); // unprotect
    assert_eq!(
        *cmd(&mut nor, &with(0xE0, &a4(SECTOR), &[0]))
            .last()
            .unwrap(),
        0xFF
    );
}

#[test]
fn ppb_protected_sector_reads_back_and_rejects_erase() {
    let mut nor = nor256();
    nor.faults().ppb.insert(0x38_0000);
    assert_eq!(
        *cmd(&mut nor, &with(0xE2, &a4(0x38_0000), &[0]))
            .last()
            .unwrap(),
        0x00
    );
    assert_eq!(
        *cmd(&mut nor, &with(0xE2, &a4(0x3C_0000), &[0]))
            .last()
            .unwrap(),
        0xFF
    );
    cmd(&mut nor, &[0x06]);
    cmd(&mut nor, &with(0xD8, &a3(0x38_0000), &[]));
    assert_ne!(rdsr(&mut nor) & 0x20, 0);
}

#[test]
fn injected_failures() {
    let mut nor = nor256();
    nor.faults().fail_program.insert(0x200);
    cmd(&mut nor, &[0x06]);
    cmd(&mut nor, &with(0x02, &a3(0x200), &[0x00]));
    assert_ne!(rdsr(&mut nor) & 0x40, 0);
    assert_eq!(nor.memory()[0x200], 0xFF);
    cmd(&mut nor, &[0x30]);
    nor.faults().stuck.insert(0x600, 0xFE); // bit 0 stuck at 0
    cmd(&mut nor, &[0x06]);
    cmd(&mut nor, &with(0x02, &a3(0x600), &[0xFF]));
    assert_eq!(nor.memory()[0x600], 0xFE);
    nor.faults().hang = true;
    cmd(&mut nor, &[0x06]);
    cmd(&mut nor, &with(0xD8, &a3(SECTOR), &[]));
    assert!((0..50).all(|_| rdsr(&mut nor) & 0x01 != 0));
}

#[test]
fn a_once_failure_fails_the_first_program_of_its_page_only() {
    let mut nor = nor256();
    nor.faults().fail_program_once.insert(0x200);
    cmd(&mut nor, &[0x06]);
    cmd(&mut nor, &with(0x02, &a3(0x200), &[0x00]));
    assert_ne!(rdsr(&mut nor) & 0x40, 0);
    assert_eq!(nor.memory()[0x200], 0xFF);
    cmd(&mut nor, &[0x30]);
    cmd(&mut nor, &[0x06]);
    cmd(&mut nor, &with(0x02, &a3(0x200), &[0x00]));
    assert_eq!(rdsr(&mut nor) & 0x40, 0);
    assert_eq!(nor.memory()[0x200], 0x00);
}

#[test]
fn wel_clears_after_every_program_or_erase_attempt() {
    // guess: a real S25FL-S keeps WEL after a failed program or erase until CLSR or WRDI
    let mut nor = nor256();
    nor.faults().fail_program.insert(0x200);
    cmd(&mut nor, &[0x06]);
    assert_eq!(rdsr(&mut nor), 0x02);
    cmd(&mut nor, &with(0x02, &a3(0x200), &[0x00]));
    assert_eq!(rdsr(&mut nor) & 0x02, 0);
    cmd(&mut nor, &[0x30]);
    cmd(&mut nor, &[0x06]);
    cmd(&mut nor, &[0x04]); // WRDI
    assert_eq!(rdsr(&mut nor), 0);
}

#[test]
fn commands_are_logged_with_their_tick_and_address() {
    let mut nor = SpiNor::new(SIZE, JEDEC_64K, &[1, 2, 3]);
    nor.select(5);
    for b in [0x03, 0, 0, 1, 0] {
        nor.transfer(b);
    }
    nor.deselect(9);
    cmd(&mut nor, &[0x06]);
    nor.select(20);
    for b in [0xDC, 0, 1, 0, 0] {
        nor.transfer(b);
    }
    nor.deselect(30);
    assert_eq!(
        nor.log(),
        [
            (9, 0x03, Some(1)),
            (0, 0x06, None),
            (30, 0xDC, Some(0x10000))
        ]
    );
    assert_eq!(nor.writes(), [(30, 0xDC, 0x10000)]);
}

#[test]
fn an_unmodelled_opcode_is_reported() {
    let mut nor = nor256();
    nor.select(0);
    assert_eq!(nor.transfer(0x0B), 0xFF); // fast read: not modelled
    assert_eq!(
        nor.take_unmodelled().as_deref(),
        Some("SPI NOR: opcode 0x0b is not modelled")
    );
    assert_eq!(nor.take_unmodelled(), None);
}

#[test]
fn reads_wrap_at_the_end_of_the_array() {
    let size = 0x2_0000;
    let mut contents = vec![0x55; size as usize];
    contents[0] = 0xAA;
    let mut nor = SpiNor::new(size, JEDEC_64K, &contents);
    let out = cmd(&mut nor, &with(0x03, &a3(size - 1), &[0, 0]));
    assert_eq!(out[4..], [0x55, 0xAA]);
}

#[test]
fn take_log_and_take_writes_drain_and_record_off_changes_only_the_logs() {
    let mut nor = nor256();
    cmd(&mut nor, &[0x06]);
    cmd(&mut nor, &with(0x02, &a3(0x100), &[0x12]));
    assert_eq!(nor.log().len(), 2);
    assert_eq!(nor.writes().len(), 1);
    let log = nor.take_log();
    assert_eq!(log.iter().map(|e| e.1).collect::<Vec<_>>(), [0x06, 0x02]);
    assert_eq!(nor.take_writes().len(), 1);
    assert!(nor.log().is_empty() && nor.writes().is_empty());
    nor.record(false);
    cmd(&mut nor, &[0x06]);
    cmd(&mut nor, &with(0x02, &a3(0x200), &[0x34]));
    assert_eq!(nor.memory()[0x200], 0x34, "the program still acts");
    assert!(nor.log().is_empty() && nor.writes().is_empty());
    nor.record(true);
    cmd(&mut nor, &[0x06]);
    assert_eq!(nor.log().len(), 1);
}

#[test]
fn a_latched_nor_error_reads_until_clsr() {
    // E_ERR, P_ERR and WIP as given (WEL and the rest of the byte are not latched), held across status reads and
    // commands until CLSR.
    let mut nor = nor256();
    nor.set_status(0xFF & !0x20);
    assert!((0..5).all(|_| rdsr(&mut nor) == 0x41));
    cmd(&mut nor, &[0x06]);
    assert_eq!(rdsr(&mut nor), 0x43);
    cmd(&mut nor, &[0x04]);
    cmd(&mut nor, &[0x30]);
    assert_eq!(rdsr(&mut nor), 0x00);
    nor.set_status(0x20);
    assert_eq!(rdsr(&mut nor), 0x20);
    cmd(&mut nor, &[0x30]);
    assert_eq!(rdsr(&mut nor), 0x00);
}
