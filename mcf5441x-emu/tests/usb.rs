//! The USB OTG controller in device mode (RM ch. 24) against hand-built dQHs and dTDs in RAM, driven by the host
//! primitives the way a USB host would: setup packets, IN tokens and OUT packets, bus resets, and scripts that
//! run on the machine's clock.

use std::cell::Cell;
use std::rc::Rc;

use mcf5441x_emu::soc::{Intc, Step, Token, UsbConfig, UsbOtg};
use mcf5441x_emu::{DeviceCx, DeviceId, Fault, Limit, Machine, Size, Stop};

const BASE: u32 = 0xFC0B_0000;
const INTC2: u32 = 0xFC05_0000;
const RAM: u32 = 0x4000_0000;
/// The dQH list, 2 KB aligned.
const QH: u32 = 0x4010_0800;
const TD: u32 = 0x4010_1000;
const BUF: u32 = 0x4010_2000;
/// On-chip memory that keeps up with a high-speed IN packet.
const SRAM: u32 = 0x8000_0000;
/// Ticks per microsecond: a test value.
const US: u64 = 100;
/// INTC2 source 17 (RM Table 17-17).
const SOURCE: u8 = 17;

const ID: u32 = 0x000;
const USBCMD: u32 = 0x140;
const USBSTS: u32 = 0x144;
const USBINTR: u32 = 0x148;
const DEVICEADDR: u32 = 0x154;
const EPLISTADDR: u32 = 0x158;
const BURSTSIZE: u32 = 0x160;
const PORTSC1: u32 = 0x184;
const OTGSC: u32 = 0x1A4;
const USBMODE: u32 = 0x1A8;
const EPSETUPSR: u32 = 0x1AC;
const EPPRIME: u32 = 0x1B0;
const EPFLUSH: u32 = 0x1B4;
const EPSR: u32 = 0x1B8;
const EPCOMPLETE: u32 = 0x1BC;
const EPCR0: u32 = 0x1C0;
const EPCR1: u32 = 0x1C4;
const EPCR2: u32 = 0x1C8;

const ZLT: u32 = 1 << 29;
const IOS: u32 = 1 << 15;
const IOC: u32 = 1 << 15;
const ACTIVE: u32 = 0x80;
const PR: u32 = 1 << 8;
const CCS: u32 = 1;

struct Rig {
    m: Machine,
    usb: DeviceId,
}

/// RAM, 64 KiB of SRAM, INTC2 and the USB OTG controller, with the DDR window as its slow memory.
fn rig() -> Rig {
    let mut m = Machine::new();
    m.mem.map_ram(RAM, 0x0100_0000);
    m.mem.map_ram(SRAM, 0x1_0000);
    let ([_, _, intc2], _) = Intc::chip();
    m.add_interrupt_controller(INTC2, 0x4000, Box::new(intc2));
    let usb = m.add_bus_master(
        BASE,
        0x1000,
        Box::new(UsbOtg::new(UsbConfig {
            controller: 2,
            source: SOURCE,
            ticks_per_us: US,
            slow_memory: Some(RAM..0x8000_0000),
        })),
    );
    Rig { m, usb }
}

impl Rig {
    fn w(&mut self, off: u32, v: u32) {
        let m = &mut self.m;
        m.access(BASE + off, Size::Long, Some(v))
            .unwrap_or_else(|f| panic!("write +{off:#x}: {f:?} {:?}", m.take_refusal()));
    }

    fn r(&mut self, off: u32) -> u32 {
        let m = &mut self.m;
        m.access(BASE + off, Size::Long, None)
            .unwrap_or_else(|f| panic!("read +{off:#x}: {f:?} {:?}", m.take_refusal()))
    }

    fn wr(&mut self, a: u32, v: u32) {
        self.m.mem.write(a, Size::Long, v).unwrap();
    }

    fn rd(&mut self, a: u32) -> u32 {
        self.m.mem.read(a, Size::Long).unwrap()
    }

    fn load(&mut self, a: u32, bytes: &[u8]) {
        self.m.mem.load(a, bytes).unwrap();
    }

    fn peek(&self, a: u32, n: u32) -> Vec<u8> {
        self.m.mem.peek(a, n).unwrap()
    }

    fn usb<R>(&mut self, f: impl FnOnce(&mut UsbOtg, &mut DeviceCx) -> R) -> R {
        self.m.with_device::<UsbOtg, R>(self.usb, f)
    }

    fn dev(&self) -> &UsbOtg {
        self.m.device::<UsbOtg>(self.usb)
    }

    fn irq(&self) -> bool {
        self.m.mem.irq_lines().asserted(2) & 1 << SOURCE != 0
    }

    /// What the driver does before RS: device mode with ES, the dQH list, EP0's capabilities, interrupts.
    fn start(&mut self) {
        self.w(USBMODE, 0x1E);
        self.w(EPLISTADDR, QH);
        self.wr(QH, 64 << 16 | ZLT | IOS); // EP0 OUT: MPS 64, ZLT, IOS
        self.wr(QH + 0x40, 64 << 16 | ZLT); // EP0 IN
        self.w(USBINTR, 0x157);
        self.w(USBCMD, 1);
    }

    /// A dTD at `at` of `len` bytes at `buf`, with IOC and Active.
    fn dtd(&mut self, at: u32, len: u32, buf: u32) {
        self.wr(at, 1);
        self.wr(at + 4, len << 16 | IOC | ACTIVE);
        for k in 0..5 {
            let page = if k == 0 {
                buf
            } else {
                (buf & !0xFFF) + 0x1000 * k
            };
            self.wr(at + 8 + 4 * k, page);
        }
    }

    fn prime(&mut self, ep: u32, dir: u32, at: u32) {
        self.wr(QH + 64 * (2 * ep + dir) + 8, at);
        self.w(EPPRIME, 1 << (ep + 16 * dir));
    }

    fn setup(&mut self, data: [u8; 8], addr: u8) -> Token {
        self.usb(|u, cx| u.setup(data, addr, cx))
    }

    fn in_token(&mut self, ep: u8, addr: u8) -> Token {
        self.usb(|u, cx| u.in_token(ep, addr, cx))
    }

    fn out_packet(&mut self, ep: u8, data: &[u8], addr: u8) -> Token {
        self.usb(|u, cx| u.out_packet(ep, data, addr, cx))
    }

    /// Runs `bra.s .` at the start of RAM until something stops it.
    fn spin(&mut self) -> mcf5441x_emu::StopReason {
        self.load(RAM, &[0x60, 0xFE]);
        self.m.cpu.pc = RAM;
        self.m.run(Limit {
            until_pc: None,
            max_instructions: Some(50_000_000),
        })
    }
}

fn data(bytes: impl IntoIterator<Item = u8>) -> Token {
    Token::Data(bytes.into_iter().collect())
}

#[test]
fn identity_registers_and_capability_length() {
    let mut t = rig();
    assert_eq!(t.r(ID), 0xE241_FA05);
    assert_eq!(t.m.access(BASE + 0x103, Size::Byte, None), Ok(0x40));
    assert_eq!(t.r(PORTSC1), 0xEC00_0004);
}

#[test]
fn hardware_parameters_follow_the_rm() {
    // RM Table 24-4: the identification, hardware and capability registers read their constant values.
    let mut t = rig();
    let want = [
        (0x004, 0x0000_07C5),
        (0x008, 0x1002_0001),
        (0x00C, 0x0000_0009),
        (0x010, 0x8004_0604),
        (0x014, 0x0000_0404),
        (0x100, 0x0100_0040),
        (0x104, 0x0001_0011),
        (0x108, 0x0000_0006),
        (0x120, 0x0000_0001),
        (0x124, 0x0000_0184),
        (0x180, 0x0000_0001),
    ];
    for (off, v) in want {
        assert_eq!(t.r(off), v, "+{off:#x}");
    }
    assert_eq!(t.m.access(BASE + 0x100, Size::Word, None), Ok(0x0100));
    assert_eq!(t.m.access(BASE + 0x122, Size::Word, None), Ok(1));
    // the rest as at reset (RM Table 24-4)
    let reset = [
        (USBCMD, 0x0008_0000),
        (USBSTS, 0x80),
        (USBINTR, 0),
        (DEVICEADDR, 0),
        (EPLISTADDR, 0),
        (BURSTSIZE, 0x0101),
        (OTGSC, 0x1020),
        (USBMODE, 0),
        (EPSETUPSR, 0),
        (EPPRIME, 0),
        (EPFLUSH, 0),
        (EPSR, 0),
        (EPCOMPLETE, 0),
        (EPCR0, 0x0080_0080),
        (EPCR1, 0),
    ];
    for (off, v) in reset {
        assert_eq!(t.r(off), v, "+{off:#x}");
    }
}

#[test]
fn setup_lands_little_endian_per_word_and_interrupts() {
    let mut t = rig();
    t.start();
    assert_eq!(t.setup([0x80, 6, 0, 1, 0, 0, 0x40, 0], 0), Token::Ack);
    assert_eq!(
        (t.rd(QH + 0x28), t.rd(QH + 0x2C)),
        (0x0100_0680, 0x0040_0000)
    );
    assert_eq!(t.r(EPSETUPSR), 1);
    assert_ne!(t.r(USBSTS) & 1, 0);
    assert!(t.irq());
    t.w(USBSTS, 1);
    t.w(EPSETUPSR, 1);
    assert!(!t.irq());
    assert_eq!(t.r(EPSETUPSR), 0);
}

#[test]
fn in_transfer_moves_the_buffer_and_retires_the_dtd() {
    let mut t = rig();
    t.start();
    t.load(BUF, &(0..18).collect::<Vec<u8>>());
    t.dtd(TD, 18, BUF);
    t.prime(0, 1, TD);
    assert_eq!((t.r(EPSR), t.r(EPPRIME)), (1 << 16, 0));
    assert_eq!(t.in_token(0, 0), data(0..18));
    assert_eq!(t.rd(TD + 4) & 0x7FFF_00FF, 0, "Total Bytes 0, Active clear");
    assert_eq!((t.r(EPCOMPLETE), t.r(EPSR)), (1 << 16, 0));
    assert_ne!(t.r(USBSTS) & 1, 0);
    assert_eq!(t.in_token(0, 0), Token::Nak);
}

#[test]
fn long_in_transfer_goes_in_max_packet_pieces_across_a_page() {
    let mut t = rig();
    t.start();
    let bytes: Vec<u8> = (0..150u32).map(|i| i as u8).collect();
    t.load(BUF + 0xFC0, &bytes); // runs into the next 4 KB page
    t.dtd(TD, 150, BUF + 0xFC0);
    t.prime(0, 1, TD);
    let mut got = Vec::new();
    let mut sizes = Vec::new();
    for _ in 0..3 {
        let Token::Data(p) = t.in_token(0, 0) else {
            panic!("no data")
        };
        sizes.push(p.len());
        got.extend(p);
    }
    assert_eq!(sizes, [64, 64, 22]);
    assert_eq!(got, bytes);
}

#[test]
fn out_packet_short_retires_with_the_remaining_count() {
    let mut t = rig();
    t.start();
    t.w(EPCR2, 0x00C8_00C8); // EP2 bulk both ways
    t.wr(QH + 64 * 4, 512 << 16 | ZLT);
    t.dtd(TD, 512, BUF);
    t.prime(2, 0, TD);
    assert_eq!(t.out_packet(2, &[b'x'; 100], 0), Token::Ack);
    assert_eq!(t.peek(BUF, 100), vec![b'x'; 100]);
    assert_eq!(t.rd(TD + 4) >> 16 & 0x7FFF, 412);
    assert_eq!(t.r(EPCOMPLETE), 1 << 2);
}

#[test]
fn out_packet_larger_than_the_packet_size_is_an_error() {
    let mut t = rig();
    t.start();
    t.dtd(TD, 64, BUF);
    t.prime(0, 0, TD);
    assert_eq!(t.out_packet(0, &[0; 65], 0), Token::Error);
    assert_eq!(t.rd(TD + 4) & 0xFF, 0x60, "Halted + Data Buffer Error");
    assert_ne!(t.r(USBSTS) & 2, 0);
    assert_eq!(t.r(EPSR) & 1, 0);
}

#[test]
fn unprimed_and_stalled_endpoints() {
    let mut t = rig();
    t.start();
    assert_eq!(t.in_token(0, 0), Token::Nak);
    assert_eq!(t.out_packet(0, &[], 0), Token::Nak);
    t.w(EPCR0, 0x0081_0081); // EP0 TXS | RXS (protocol stall)
    assert_eq!(t.in_token(0, 0), Token::Stall);
    assert_eq!(t.out_packet(0, &[], 0), Token::Stall);
    t.setup([0; 8], 0); // a setup clears it
    assert_eq!(t.r(EPCR0) & 0x1_0001, 0);
    assert_eq!(t.in_token(1, 0), Token::Nak, "EP1 isn't enabled");
}

#[test]
fn bus_reset_holds_pr_until_the_guest_flushes() {
    let mut t = rig();
    t.start();
    t.w(DEVICEADDR, 5 << 25);
    t.w(EPCR1, 0x00C8_00C8);
    t.usb(|u, cx| u.bus_reset_start(cx));
    assert_ne!(t.r(PORTSC1) & PR, 0);
    assert_eq!(t.r(DEVICEADDR), 0);
    assert_eq!(t.r(EPCR1) & 0x0080_0080, 0);
    assert_ne!(t.r(USBSTS) & 0x40, 0);
    assert!(!t.dev().reset_acked());
    t.w(EPFLUSH, 0xFFFF_FFFF);
    assert!(t.dev().reset_acked());
    t.usb(|u, cx| u.bus_reset_end(cx));
    let sc = t.r(PORTSC1);
    assert_eq!(sc & PR, 0);
    assert_eq!(sc >> 26 & 3, 2, "high speed");
    assert_ne!(sc & CCS, 0);
    assert_ne!(t.r(USBSTS) & 4, 0, "PCI");
}

#[test]
fn a_short_reset_drops_pr_at_once() {
    let mut t = rig();
    t.start();
    t.usb(|u, _| u.knobs().short_reset = true);
    t.usb(|u, cx| u.bus_reset_start(cx));
    assert_eq!(t.r(PORTSC1) & PR, 0);
    assert_ne!(t.r(USBSTS) & 0x40, 0);
}

#[test]
fn controller_reset_restores_defaults() {
    let mut t = rig();
    t.start();
    t.w(USBCMD, 2);
    assert_eq!(t.dev().stats().resets, 1);
    assert_eq!(
        (t.r(USBCMD), t.r(USBMODE), t.r(USBINTR)),
        (0x0008_0000, 0, 0)
    );
}

#[test]
fn byte_writes_are_unmodelled() {
    let mut t = rig();
    assert_eq!(
        t.m.access(BASE + USBCMD, Size::Byte, Some(1)),
        Err(Fault::Device(Stop::Unmodelled))
    );
    let r = t.m.take_refusal().expect("refused");
    assert_eq!(
        r.detail,
        "USB: 1-byte write at +0x140 (aligned 32-bit writes only)"
    );
    assert!(t.dev().stats().writes.is_empty());
}

#[test]
fn run_without_device_mode_is_unmodelled() {
    let mut t = rig();
    assert_eq!(
        t.m.access(BASE + USBCMD, Size::Long, Some(1)),
        Err(Fault::Device(Stop::Unmodelled))
    );
    let r = t.m.take_refusal().expect("refused");
    assert_eq!(
        r.detail,
        "USB: run (USBCMD RS) with USBMODE 0x0 (device mode with ES = 1 only)"
    );
    assert_eq!(t.r(USBCMD), 0x0008_0000);
}

#[test]
fn scripts_run_on_the_machine_clock() {
    let mut t = rig();
    let seen = Rc::new(Cell::new(0));
    let s = seen.clone();
    let mut state = 0;
    let script = move |_: &mut UsbOtg, cx: &mut DeviceCx| {
        state += 1;
        match state {
            1 => Step::Wait(100),
            2 => {
                s.set(cx.now);
                Step::Wait(100)
            }
            _ => Step::Done(42),
        }
    };
    t.usb(|u, _| u.run_script(Box::new(script), 1000));
    let stop = t.spin();
    assert_eq!(stop.kind, Stop::Halt("usb host done"), "{stop:?}");
    assert_eq!(t.dev().script_result(), Some(&Ok(42)));
    assert!(seen.get() >= 1000 + 100 * US, "{}", seen.get());
    assert!(t.m.now() >= 1000 + 200 * US);
}

#[test]
fn a_failing_script_stops_the_machine_with_its_error() {
    let mut t = rig();
    let mut first = true;
    let script = move |_: &mut UsbOtg, _: &mut DeviceCx| {
        if std::mem::take(&mut first) {
            Step::Wait(10)
        } else {
            Step::Fail("boom".into())
        }
    };
    t.usb(|u, _| u.run_script(Box::new(script), 0));
    let stop = t.spin();
    assert_eq!(stop.kind, Stop::Halt("usb host failed"), "{stop:?}");
    assert_eq!(stop.detail, "boom");
    assert_eq!(t.dev().script_result(), Some(&Err("boom".to_string())));
}

#[test]
fn a_running_script_lists_the_usb_interrupt_as_a_wake_source() {
    let mut t = rig();
    let mut out = Vec::new();
    use mcf5441x_emu::Device;
    assert!(t.dev().wake_sources(&mut out));
    assert!(out.is_empty());
    t.usb(|u, _| {
        u.run_script(
            Box::new(|_: &mut UsbOtg, _: &mut DeviceCx| Step::Done(0)),
            10,
        )
    });
    assert!(t.dev().wake_sources(&mut out));
    assert_eq!(out, [(2, SOURCE)]);
}

#[test]
fn tokens_reach_the_device_only_at_its_address_and_while_attached() {
    let mut t = rig();
    t.start();
    t.load(BUF, &(0..8).collect::<Vec<u8>>());
    t.dtd(TD, 8, BUF);
    t.prime(0, 1, TD);
    assert_eq!(t.in_token(0, 5), Token::NoAnswer, "still at address 0");
    assert_eq!(t.setup([0x80, 6, 0, 1, 0, 0, 8, 0], 5), Token::NoAnswer);
    assert_eq!(t.r(EPSETUPSR), 0, "not delivered");
    t.w(DEVICEADDR, 5 << 25);
    assert_eq!(t.in_token(0, 0), Token::NoAnswer);
    assert_eq!(t.in_token(0, 5), data(0..8));
    t.w(USBCMD, 0); // RS = 0: detached
    assert_eq!(t.out_packet(0, &[], 5), Token::NoAnswer);
}

#[test]
fn register_writes_are_recorded_in_order() {
    let mut t = rig();
    t.w(USBMODE, 0x1E);
    t.w(USBCMD, 2); // RST: the log survives the controller reset
    t.w(BURSTSIZE, 0x204);
    assert_eq!(
        t.dev().stats().writes,
        [(USBMODE, 0x1E), (USBCMD, 2), (BURSTSIZE, 0x204)]
    );
}

/// EP1 IN as high-speed bulk (MPS 512) with a 512-byte dTD at `buf`, the bus reset done at high speed.
fn bulk_in(t: &mut Rig, buf: u32) {
    t.start();
    t.usb(|u, cx| {
        u.bus_reset_start(cx);
        u.bus_reset_end(cx);
    });
    t.w(EPCR1, 0x00C8_00C8);
    t.wr(QH + 64 * 3, 512 << 16 | ZLT);
    t.load(buf, &[0xA5; 512]);
    t.dtd(TD, 512, buf);
    t.prime(1, 1, TD);
}

#[test]
fn a_long_high_speed_in_from_slow_memory_never_completes() {
    // Measured on an MCF54415: a high-speed IN of more than 64 bytes from DDR never completes.
    let mut t = rig();
    bulk_in(&mut t, BUF);
    for _ in 0..3 {
        assert_eq!(t.in_token(1, 0), Token::Nak);
    }
    assert_eq!(t.dev().stats().underflows, 3);
    assert!(t.dev().stats().in_sizes.is_empty());
    assert_eq!(t.r(EPSR), 1 << 17, "still primed");
    assert_eq!(t.r(EPCOMPLETE), 0);
}

#[test]
fn a_long_high_speed_in_from_sram_completes() {
    // Measured on an MCF54415: from SRAM the same IN completes.
    let mut t = rig();
    bulk_in(&mut t, SRAM + 0x1000);
    assert_eq!(t.in_token(1, 0), data([0xA5; 512]));
    assert_eq!(t.dev().stats().underflows, 0);
    assert_eq!(t.dev().stats().in_sizes, [512]);
    assert_eq!(t.r(EPCOMPLETE), 1 << 17);
}

#[test]
fn mute_in_naks_every_data_in() {
    let mut t = rig();
    bulk_in(&mut t, SRAM);
    t.usb(|u, _| u.knobs().mute_in = true);
    assert_eq!(t.in_token(1, 0), Token::Nak);
    assert_eq!(t.r(EPSR), 1 << 17);
}

#[test]
fn full_speed_bulk_in_over_64_bytes_is_unmodelled() {
    let mut t = rig();
    t.usb(|u, _| u.knobs().hs = false);
    bulk_in(&mut t, SRAM);
    assert_eq!(t.r(PORTSC1) >> 26 & 3, 0, "full speed");
    assert_eq!(t.in_token(1, 0), Token::Nak);
    let stop = t.spin();
    assert_eq!(stop.kind, Stop::Unmodelled);
    assert_eq!(
        stop.detail,
        "USB: EP1 IN with 512-byte packets at full speed (64 at most for bulk)"
    );
}

#[test]
fn in_without_zlt_is_unmodelled() {
    let mut t = rig();
    t.start();
    t.wr(QH + 0x40, 64 << 16); // EP0 IN without ZLT
    t.dtd(TD, 8, BUF);
    t.prime(0, 1, TD);
    assert_eq!(t.in_token(0, 0), Token::Nak);
    let stop = t.spin();
    assert_eq!(stop.kind, Stop::Unmodelled);
    assert_eq!(stop.detail, "USB: EP0 IN with ZLT = 0");
}

#[test]
fn run_cleared_detaches_after_100_us() {
    // guess: the PHY drops the pull-up 100 µs after RS clears
    let mut t = rig();
    t.start();
    t.usb(|u, cx| {
        u.bus_reset_start(cx);
        u.bus_reset_end(cx);
    });
    t.w(USBCMD, 0);
    let at = t.m.now();
    assert!(t.m.idle_until(at + 100 * US - 1).is_none());
    assert_ne!(t.r(PORTSC1) & CCS, 0);
    assert!(t.m.idle_until(at + 100 * US).is_none());
    assert_eq!(t.r(PORTSC1) & CCS, 0);
}

#[test]
fn a_controller_reset_before_the_detach_is_counted() {
    let mut t = rig();
    t.start();
    t.usb(|u, cx| {
        u.bus_reset_start(cx);
        u.bus_reset_end(cx);
    });
    t.w(USBCMD, 0);
    t.w(USBCMD, 2);
    assert_eq!(t.dev().stats().rst_while_attached, 1);
}

#[test]
fn a_chained_dtd_is_loaded_when_the_first_retires() {
    let mut t = rig();
    t.start();
    t.load(BUF, &[1, 2, 3, 4]);
    t.load(BUF + 0x100, &[5, 6]);
    t.dtd(TD, 4, BUF);
    t.dtd(TD + 0x20, 2, BUF + 0x100);
    t.wr(TD, TD + 0x20);
    t.prime(0, 1, TD);
    assert_eq!(t.in_token(0, 0), data([1, 2, 3, 4]));
    assert_eq!(t.r(EPSR), 1 << 16, "the next dTD keeps the endpoint primed");
    assert_eq!(t.rd(QH + 0x40 + 4), TD + 0x20);
    assert_eq!(t.in_token(0, 0), data([5, 6]));
    assert_eq!(t.r(EPSR), 0);
    assert_eq!(
        t.rd(QH + 0x40 + 8),
        1,
        "the overlay's next pointer terminates"
    );
}

#[test]
fn vbus_and_system_error_raise_their_interrupts() {
    let mut t = rig();
    t.start();
    t.w(OTGSC, 1 << 27); // BSVIE
    t.usb(|u, cx| u.vbus(true, cx));
    let otgsc = t.r(OTGSC);
    assert_ne!(otgsc & 1 << 11, 0, "BSV");
    assert_ne!(otgsc & 1 << 19, 0, "BSVIS");
    assert!(t.irq());
    t.w(OTGSC, otgsc & 0x007F_0000 | 1 << 27);
    assert!(!t.irq());
    t.usb(|u, cx| u.system_error(cx));
    assert_ne!(t.r(USBSTS) & 1 << 4, 0, "SEI");
    assert!(t.irq());
    // RM Table 24-20: in device mode a system error only interrupts; RS stays set.
    assert_eq!(t.r(USBCMD) & 1, 1);
}

#[test]
fn a_setup_before_the_prime_is_a_stale_prime() {
    let mut t = rig();
    t.start();
    t.usb(|u, _| u.knobs().setup_before_prime = Some([0x80, 6, 0, 1, 0, 0, 18, 0]));
    t.dtd(TD, 18, BUF);
    t.prime(0, 1, TD);
    assert_eq!(t.r(EPSETUPSR), 1);
    assert_eq!(t.rd(QH + 0x28), 0x0100_0680);
    let at = t.m.now();
    assert!(t.m.idle_until(at + 50).is_none());
    t.w(EPFLUSH, 1 << 16);
    assert_eq!(t.dev().stats().stale_primes, [50]);
}

#[test]
fn a_driver_bring_up_and_control_transfer_are_accepted() {
    // The register sequence of a device-mode driver: reset, ULPI, burst size, device mode, the dQH list, interrupts,
    // run; the bus reset clean-up; a setup read under the SUTW tripwire; a data stage; address; a protocol stall.
    let mut t = rig();
    t.w(USBCMD, 2);
    assert_eq!(t.r(USBCMD) & 2, 0);
    let sc = t.r(PORTSC1);
    t.w(PORTSC1, sc & !(0xC000_0000 | 1 << 24) | 0x8000_0000);
    t.w(BURSTSIZE, 0x204);
    t.w(USBMODE, 0x1E);
    t.wr(QH, 64 << 16 | ZLT | IOS);
    t.wr(QH + 0x40, 64 << 16 | ZLT);
    t.w(EPLISTADDR, QH);
    t.w(USBINTR, 0x157);
    let cmd = t.r(USBCMD);
    t.w(USBCMD, cmd & !0x00FF_0000 | 1);
    t.usb(|u, cx| {
        u.vbus(true, cx);
        u.bus_reset_start(cx);
    });
    let sts = t.r(USBSTS) & t.r(USBINTR);
    t.w(USBSTS, sts);
    let v = t.r(EPSETUPSR);
    t.w(EPSETUPSR, v);
    let v = t.r(EPCOMPLETE);
    t.w(EPCOMPLETE, v);
    assert_eq!(t.r(EPPRIME), 0);
    t.w(EPFLUSH, 0xFFFF_FFFF);
    assert_eq!(t.r(EPFLUSH), 0);
    assert_ne!(t.r(PORTSC1) & PR, 0);
    t.usb(|u, cx| u.bus_reset_end(cx));
    t.setup([0x80, 6, 0, 1, 0, 0, 18, 0], 0);
    t.w(EPSETUPSR, 1);
    let cmd = t.r(USBCMD);
    t.w(USBCMD, cmd | 1 << 13);
    let (w0, w1) = (t.rd(QH + 0x28), t.rd(QH + 0x2C));
    assert_ne!(t.r(USBCMD) & 1 << 13, 0);
    let cmd = t.r(USBCMD);
    t.w(USBCMD, cmd & !(1 << 13));
    assert_eq!((w0, w1), (0x0100_0680, 0x0012_0000));
    t.load(BUF, &[0x12; 18]);
    t.dtd(TD, 18, BUF);
    t.wr(QH + 0x4C, 0);
    t.prime(0, 1, TD);
    assert_eq!(t.in_token(0, 0), data([0x12; 18]));
    t.w(DEVICEADDR, 7 << 25);
    t.w(EPCR1, 0x00C8_00C8);
    let v = t.r(EPCR0);
    t.w(EPCR0, v | 0x0001_0001);
    assert_eq!(t.in_token(0, 7), Token::Stall);
    assert_eq!(t.dev().stats().writes.len(), 18, "every write accepted");
}
