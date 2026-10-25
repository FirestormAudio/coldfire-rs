//! The GPIO ports (RM ch. 16) inside the GPIO stub block: the data registers through the core's accesses, the host's
//! drive and the change log.

use mcf5441x_emu::soc::{Chip, ChipOptions, Mcf5441x, PinChange, StubBlock};
use mcf5441x_emu::{Machine, Size};

const GPIO: u32 = 0xEC09_4000;
const PODR: u32 = GPIO;
const PDDR: u32 = GPIO + 0x0C;
const PPDSDR: u32 = GPIO + 0x18;
const PCLRR: u32 = GPIO + 0x24;
const PCR: u32 = GPIO + 0x30;
/// Port E, an arbitrary port.
const E: u32 = 4;

fn chip() -> (Machine, Chip) {
    let mut m = Machine::new();
    let chip = Mcf5441x::build(&mut m, ChipOptions::default());
    (m, chip)
}

fn r(m: &mut Machine, addr: u32, size: Size) -> u32 {
    m.access(addr, size, None).unwrap()
}

fn w(m: &mut Machine, addr: u32, size: Size, value: u32) {
    m.access(addr, size, Some(value)).unwrap();
}

#[test]
fn podr_and_pddr_read_back() {
    let (mut m, _) = chip();
    w(&mut m, PODR + E, Size::Byte, 0xA5);
    assert_eq!(r(&mut m, PODR + E, Size::Byte), 0xA5);
    // a word takes two ports, big-endian
    w(&mut m, PDDR, Size::Word, 0x1234);
    assert_eq!(r(&mut m, PDDR, Size::Byte), 0x12);
    assert_eq!(r(&mut m, PDDR + 1, Size::Byte), 0x34);
    assert_eq!(r(&mut m, PDDR, Size::Long), 0x1234_0000);
}

#[test]
fn ppdsdr_write_sets_and_pclrr_write_zero_clears() {
    let (mut m, _) = chip();
    w(&mut m, PODR + E, Size::Byte, 0x0F);
    w(&mut m, PPDSDR + E, Size::Byte, 0x30);
    assert_eq!(r(&mut m, PODR + E, Size::Byte), 0x3F);
    w(&mut m, PCLRR + E, Size::Byte, !0x05 & 0xFF);
    assert_eq!(r(&mut m, PODR + E, Size::Byte), 0x3A);
}

#[test]
fn ppdsdr_reads_outputs_from_podr_and_inputs_from_the_host() {
    let (mut m, chip) = chip();
    w(&mut m, PDDR + E, Size::Byte, 0x0F);
    w(&mut m, PODR + E, Size::Byte, 0x05);
    let g = chip.gpio_mut(&mut m);
    g.drive(E as u8, 4, Some(false));
    g.drive(E as u8, 5, Some(false));
    g.drive(E as u8, 0, Some(false)); // an output: the drive does nothing
    // pins 6 and 7 are undriven with no pull: 1
    assert_eq!(r(&mut m, PPDSDR + E, Size::Byte), 0xC5);
    assert_eq!(chip.gpio(&m).output(E as u8, 0), Some(true));
    assert_eq!(chip.gpio(&m).output(E as u8, 4), None);
    assert!(!chip.gpio(&m).level(E as u8, 4));
    chip.gpio_mut(&mut m).drive(E as u8, 4, None);
    assert!(chip.gpio(&m).level(E as u8, 4));
}

#[test]
fn an_undriven_input_reads_its_pull() {
    let (mut m, chip) = chip();
    // PCR_E: pin 0 up (PUE|PUS = 11), pin 1 down (10), pin 2 none (00)
    w(&mut m, PCR + 2 * E, Size::Word, 0b00_10_11);
    assert_eq!(r(&mut m, PCR + 2 * E, Size::Word), 0b00_10_11);
    assert!(chip.gpio(&m).level(E as u8, 0));
    assert!(!chip.gpio(&m).level(E as u8, 1));
    assert!(chip.gpio(&m).level(E as u8, 2));
    assert_eq!(r(&mut m, PPDSDR + E, Size::Byte), 0xFD);
}

#[test]
fn pclrr_reads_zero() {
    let (mut m, _) = chip();
    w(&mut m, PODR + E, Size::Byte, 0xFF);
    assert_eq!(r(&mut m, PCLRR + E, Size::Byte), 0);
}

#[test]
fn reset_values_measured() {
    // measured on an MCF54415: PODR resets to 0xFF on every port, PDDR to 0, PCLRR reads 0; PCR_A resets to
    // 0x0300, PCR_J to 0x0008, PCR_K to 0x0002 and every other PCR to 0
    let (mut m, chip) = chip();
    for port in 0..11 {
        assert_eq!(r(&mut m, PODR + port, Size::Byte), 0xFF, "port {port}");
        assert_eq!(r(&mut m, PDDR + port, Size::Byte), 0, "port {port}");
        assert_eq!(r(&mut m, PCLRR + port, Size::Byte), 0, "port {port}");
        let pcr = match port {
            0 => 0x0300,
            9 => 0x0008,
            10 => 0x0002,
            _ => 0,
        };
        assert_eq!(r(&mut m, PCR + 2 * port, Size::Word), pcr, "port {port}");
    }
    // every pin an input: PA4 pulled up, PJ1 and PK0 pulled down, the rest with no pull
    assert_eq!(chip.gpio(&m).output(0, 0), None);
    assert_eq!(r(&mut m, PPDSDR, Size::Byte), 0xFF);
    assert_eq!(r(&mut m, PPDSDR + 9, Size::Byte), 0xFD);
    assert_eq!(r(&mut m, PPDSDR + 10, Size::Byte), 0xFE);
    // the first change is logged against the reset state: making a pin an output drives it high
    w(&mut m, PDDR + E, Size::Byte, 0x01);
    assert!(chip.gpio(&m).level(E as u8, 0));
    let changes = chip.gpio_mut(&mut m).take_changes();
    assert_eq!(
        changes
            .iter()
            .map(|c| (c.port, c.pddr, c.podr))
            .collect::<Vec<_>>(),
        [(4, 1, 0xFF)]
    );
    // a write of the reset latch changes nothing
    w(&mut m, PODR + 3, Size::Byte, 0xFF);
    assert!(chip.gpio(&m).changes().is_empty());
}

#[test]
fn the_gaps_read_zero_and_drop_writes_measured() {
    // measured on an MCF54415: the reserved bytes +0x0B, +0x17, +0x23, +0x2F, +0x46 and +0x47 read 0 and drop
    // writes
    let (mut m, _) = chip();
    for gap in [0x0B, 0x17, 0x23, 0x2F, 0x46, 0x47] {
        w(&mut m, GPIO + gap, Size::Byte, 0x5A);
        assert_eq!(r(&mut m, GPIO + gap, Size::Byte), 0, "+{gap:#x}");
    }
}

#[test]
fn pin_mux_offsets_stay_read_back() {
    let (mut m, _) = chip();
    for off in [0x48, 0x4B, 0x50, 0x6F] {
        w(&mut m, GPIO + off, Size::Byte, 0x15);
        assert_eq!(r(&mut m, GPIO + off, Size::Byte), 0x15, "+{off:#x}");
    }
}

#[test]
fn the_change_log_records_each_change_once() {
    let (mut m, chip) = chip();
    w(&mut m, PODR + E, Size::Byte, 0);
    w(&mut m, PDDR + E, Size::Byte, 0x01);
    w(&mut m, PPDSDR + E, Size::Byte, 0x01);
    w(&mut m, PPDSDR + E, Size::Byte, 0x01); // no change
    w(&mut m, PCLRR + E, Size::Byte, 0xFE);
    w(&mut m, PCR + 2 * E, Size::Word, 0x3); // not a direction or latch
    let changes: Vec<(u8, u8, u8)> = chip
        .gpio_mut(&mut m)
        .take_changes()
        .iter()
        .map(
            |&PinChange {
                 port, pddr, podr, ..
             }| (port, pddr, podr),
        )
        .collect();
    assert_eq!(changes, [(4, 0, 0), (4, 1, 0), (4, 1, 1), (4, 1, 0)]);
    assert!(chip.gpio(&m).changes().is_empty());
}

#[test]
fn chip_gpio_finds_the_model() {
    let (m, chip) = chip();
    assert_eq!(chip.gpio(&m).output(0, 0), None);
}

#[test]
fn the_stub_still_logs_port_writes() {
    let (mut m, chip) = chip();
    w(&mut m, PODR + E, Size::Byte, 0x80);
    let id = chip.stubs.iter().find(|(n, _)| *n == "GPIO").unwrap().1;
    let writes = m.device::<StubBlock>(id).writes();
    assert_eq!(writes.len(), 1);
    assert_eq!((writes[0].1, writes[0].3), (E, 0x80));
}

#[test]
fn a_linked_input_follows_its_source_while_the_source_is_an_output() {
    let (mut m, chip) = chip();
    // pin C3 wired to pin D4; C3 undriven reads 1 (no pull)
    chip.gpio_mut(&mut m).link((2, 3), (3, 4));
    assert_eq!(r(&mut m, PPDSDR + 2, Size::Byte) & 8, 8);
    // D4 an output: C3 follows its latch
    w(&mut m, PDDR + 3, Size::Byte, 0x10);
    w(&mut m, PODR + 3, Size::Byte, 0xEF);
    assert_eq!(r(&mut m, PPDSDR + 2, Size::Byte) & 8, 0);
    w(&mut m, PODR + 3, Size::Byte, 0x10);
    assert_eq!(r(&mut m, PPDSDR + 2, Size::Byte) & 8, 8);
    // D4 an input again: C3 reads what the host drives
    w(&mut m, PDDR + 3, Size::Byte, 0);
    chip.gpio_mut(&mut m).drive(2, 3, Some(false));
    assert_eq!(r(&mut m, PPDSDR + 2, Size::Byte) & 8, 0);
}
