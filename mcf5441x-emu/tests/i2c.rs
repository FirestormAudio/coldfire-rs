//! The minimal I2C master (RM ch. 42) against a register-file part: the writes and reads a driver polls through.

use mcf5441x_emu::soc::{ChipOptions, I2c, I2cTarget, Mcf5441x};
use mcf5441x_emu::{Machine, Size};

const I2C: u32 = 0xFC05_8000;
const FDR: u32 = I2C + 4;
const CR: u32 = I2C + 8;
const SR: u32 = I2C + 0xC;
const DR: u32 = I2C + 0x10;
const IIF: u32 = 0x02;
const RXAK: u32 = 0x01;
const IBB: u32 = 0x20;

/// A part at 0x1A: the first byte written is a register pointer, later ones fill registers from it.
struct Regs {
    mem: [u8; 256],
    ptr: usize,
    first: bool,
    stops: u32,
}

impl I2cTarget for Regs {
    fn address(&self) -> u8 {
        0x1A
    }
    fn start(&mut self, read: bool) {
        self.first = !read;
    }
    fn write(&mut self, byte: u8) -> bool {
        if std::mem::take(&mut self.first) {
            self.ptr = byte as usize;
        } else {
            self.mem[self.ptr] = byte;
            self.ptr = (self.ptr + 1) % 256;
        }
        true
    }
    fn read(&mut self) -> u8 {
        self.ptr = (self.ptr + 1) % 256;
        self.mem[self.ptr - 1]
    }
    fn stop(&mut self) {
        self.stops += 1;
    }
}

fn rig() -> Machine {
    let mut m = Machine::new();
    let chip = Mcf5441x::build(&mut m, ChipOptions::default());
    m.device_mut::<I2c>(chip.i2c0).attach(Box::new(Regs {
        mem: [0; 256],
        ptr: 0,
        first: false,
        stops: 0,
    }));
    m
}

fn rb(m: &mut Machine, a: u32) -> u32 {
    m.access(a, Size::Byte, None).unwrap()
}

fn wb(m: &mut Machine, a: u32, v: u32) {
    m.access(a, Size::Byte, Some(v)).unwrap();
}

/// Send one byte as a driver does: write I2DR, wait for IIF, clear it; returns RXAK.
fn send(m: &mut Machine, v: u32) -> u32 {
    wb(m, DR, v);
    assert_eq!(rb(m, SR) & IIF, IIF, "byte done");
    wb(m, SR, 0);
    assert_eq!(rb(m, SR) & IIF, 0, "IIF clears by writing 0");
    rb(m, SR) & RXAK
}

#[test]
fn reset_values_and_a_register_write_and_read_back() {
    let mut m = rig();
    assert_eq!(rb(&mut m, SR), 0x81);
    wb(&mut m, FDR, 0x12);
    wb(&mut m, CR, 0x80);
    wb(&mut m, CR, 0xC0);
    // write register 0x2B = 0x50: START, address 0x1A write, pointer, value, STOP
    wb(&mut m, CR, 0xF0);
    assert_eq!(rb(&mut m, SR) & IBB, IBB);
    assert_eq!(send(&mut m, 0x34), 0, "acknowledged");
    assert_eq!(send(&mut m, 0x2B), 0);
    assert_eq!(send(&mut m, 0x50), 0);
    wb(&mut m, CR, 0xD0);
    assert_eq!(rb(&mut m, SR) & IBB, 0, "STOP frees the bus");
    // read it back: address+write, pointer, restart, address+read, one byte
    wb(&mut m, CR, 0xF0);
    send(&mut m, 0x34);
    send(&mut m, 0x2B);
    wb(&mut m, CR, 0xF4);
    assert_eq!(rb(&mut m, CR), 0xF0, "RSTA reads 0");
    assert_eq!(send(&mut m, 0x35), 0);
    wb(&mut m, CR, 0xE8); // receive, no ack
    rb(&mut m, DR); // the dummy read starts the byte
    assert_eq!(rb(&mut m, SR) & IIF, IIF);
    wb(&mut m, SR, 0);
    wb(&mut m, CR, 0xC8); // STOP before the last read
    assert_eq!(rb(&mut m, DR), 0x50);
    assert_eq!(rb(&mut m, SR) & IBB, 0);
}

#[test]
fn an_address_nobody_answers_is_not_acknowledged() {
    let mut m = rig();
    wb(&mut m, CR, 0xF0);
    assert_eq!(send(&mut m, 0x40), RXAK);
    assert_eq!(
        send(&mut m, 0x00),
        RXAK,
        "nothing selected, nothing acknowledges"
    );
}

#[test]
fn disabling_resets_and_wide_accesses_are_refused() {
    let mut m = rig();
    wb(&mut m, CR, 0xF0);
    wb(&mut m, CR, 0x00);
    assert_eq!(rb(&mut m, SR), 0x81);
    assert!(m.access(CR, Size::Long, None).is_err());
}
