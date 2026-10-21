mod common;
use mcf5441x_emu::{Fault, Memory, Size};

#[test]
fn ram_is_big_endian() {
    let mut m = Memory::default();
    m.map_ram(0x4010_0000, 0x1000);
    m.write(0x4010_0000, Size::Long, 0x1122_3344).unwrap();
    assert_eq!(m.read(0x4010_0000, Size::Byte).unwrap(), 0x11);
    assert_eq!(m.read(0x4010_0002, Size::Word).unwrap(), 0x3344);
}

#[test]
fn unmapped_read_faults() {
    let mut m = Memory::default();
    m.map_ram(0x4010_0000, 0x1000);
    assert_eq!(
        m.read(0x3000_0000, Size::Long),
        Err(Fault::Unmapped { addr: 0x3000_0000, size: Size::Long })
    );
}

#[test]
fn long_read_straddling_ram_end_is_unmapped() {
    let mut m = Memory::default();
    m.map_ram(0x4010_0000, 0x1000);
    assert_eq!(
        m.read(0x4010_0FFE, Size::Long),
        Err(Fault::Unmapped { addr: 0x4010_0FFE, size: Size::Long })
    );
    assert!(m.write(0x4010_0FFE, Size::Long, 0).is_err());
    assert_eq!(m.read(0x4010_0FFC, Size::Long), Ok(0));
}

#[test]
fn load_and_peek() {
    let mut m = Memory::default();
    m.map_ram(0x4010_0000, 0x1000);
    m.map_ram(0x2000_0000, 0x100);
    m.load(0x4010_0010, &[1, 2, 3]).unwrap();
    assert_eq!(m.peek(0x4010_0010, 3), Some(vec![1, 2, 3]));
    assert_eq!(m.peek(0x4010_0FFF, 2), None);
    assert!(m.load(0x4010_0FFF, &[1, 2]).is_err());
    assert_eq!(m.read(0x2000_0000, Size::Byte), Ok(0));
}

#[test]
fn assemble_helper_produces_code() {
    assert_eq!(common::assemble("nop\nrts", 0x4010_0000), [0x4E, 0x71, 0x4E, 0x75]);
}
