//! The high-level USB host ([`UsbHost`]) against coldfire-rs's `examples` `usb-cdc` program (a CDC-ACM device that
//! echoes every packet), built with `toolchain/cargo-cf`; those tests are skipped (with a message) when the
//! ColdFire toolchain is not built. The rest run against a core that never starts the controller.

use std::path::PathBuf;
use std::process::Command;
use std::sync::Mutex;

use mcf5441x_emu::elf::load_elf;
use mcf5441x_emu::soc::usb::host::{POLL_US, TIMEOUT_US};
use mcf5441x_emu::soc::{Chip, ChipOptions, HostError, Mcf5441x, UsbHost, UsbOtg, endpoints};
use mcf5441x_emu::{DeviceId, Machine, Stop};

/// DDR as examples/memory.x has it: 128 MB at 0x40000000, the stack at its top.
const DDR: (u32, u32) = (0x4000_0000, 0x0800_0000);
const STACK_TOP: u32 = 0x4800_0000;
/// The SRAM where the program's RAMBAR puts it (its USB structures are in the upper half).
const SRAM: (u32, u32) = (0x8000_0000, 0x1_0000);
/// The test's time scale: ticks (instructions) per microsecond. guess: any scale works for the host; 100 keeps the
/// runs short.
const US: u64 = 100;
/// The example's idVendor and idProduct (pid.codes test IDs), little-endian as in the device descriptor.
const VID_PID: [u8; 4] = [0x09, 0x12, 0x01, 0x00];
const ADDRESS: u8 = 5;

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..")
}

static BUILD: Mutex<()> = Mutex::new(());

/// examples' `usb-cdc` as ELF bytes; `None` (with a message) when the ColdFire toolchain is not built.
fn build() -> Option<Vec<u8>> {
    let root = root();
    let rcf =
        std::env::var_os("RCF").map_or_else(|| root.join("build/rust-coldfire"), PathBuf::from);
    if !rcf
        .join("rustc_codegen_gcc/target/debug/librustc_codegen_gcc.so")
        .exists()
    {
        eprintln!("skipped: the ColdFire toolchain is not built (toolchain/setup.sh, or set RCF)");
        return None;
    }
    let _one = BUILD.lock().unwrap_or_else(|e| e.into_inner());
    let examples = root.join("examples");
    let target = examples.join("target").join("emu-usb-cdc");
    let mut cmd = Command::new(root.join("toolchain/cargo-cf"));
    // the outer cargo's settings are not the ColdFire build's
    for (k, _) in std::env::vars_os() {
        let k = k.to_string_lossy().into_owned();
        if (k.starts_with("CARGO_") && k != "CARGO_HOME")
            || k.starts_with("RUSTC")
            || k.starts_with("RUSTFLAGS")
            || k == "RUSTDOCFLAGS"
            || k == "RUSTUP_TOOLCHAIN"
        {
            cmd.env_remove(k);
        }
    }
    let out = cmd
        .arg(&examples)
        .args(["--bin", "usb-cdc", "--target-dir"])
        .arg(&target)
        .env("RCF", &rcf)
        .output()
        .expect("toolchain/cargo-cf runs");
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(
        out.status.success(),
        "cargo-cf usb-cdc failed:\n{}",
        &err[err.len().saturating_sub(4000)..]
    );
    Some(
        std::fs::read(target.join("m68k-coldfire-none-elf/release/usb-cdc"))
            .expect("usb-cdc built"),
    )
}

/// A machine with the chip, DDR and SRAM.
fn machine() -> (Machine, Chip) {
    let mut m = Machine::new();
    m.mem.map_ram(DDR.0, DDR.1);
    m.mem.map_ram(SRAM.0, SRAM.1);
    let chip = Mcf5441x::build(&mut m, ChipOptions::default().with_usb_ticks_per_us(US));
    (m, chip)
}

/// The example loaded and entered as a boot stage leaves it: supervisor mode, interrupts masked, the stack at the
/// top of DDR. Also the controller's id.
fn example() -> Option<(Machine, UsbHost, DeviceId)> {
    let elf = build()?;
    let (mut m, chip) = machine();
    m.cpu.pc = load_elf(&mut m, &elf).unwrap();
    m.cpu.set_sr(0x2700);
    m.cpu.a[7] = STACK_TOP;
    Some((m, UsbHost::new(chip.usb, US), chip.usb))
}

/// Connect, enumerate at [`ADDRESS`] and open the CDC port.
fn opened(m: &mut Machine, host: &mut UsbHost) {
    host.connect(m).unwrap();
    host.enumerate(m, ADDRESS).unwrap();
    host.cdc_open(m, 0).unwrap();
}

/// A machine whose core runs `code` from the start of DDR and never starts the USB controller.
fn silent(code: &[u8]) -> (Machine, UsbHost) {
    let (mut m, chip) = machine();
    m.mem.load(DDR.0, code).unwrap();
    m.cpu.pc = DDR.0;
    m.cpu.set_sr(0x2700);
    m.cpu.a[7] = STACK_TOP;
    (m, UsbHost::new(chip.usb, US))
}

/// `bra.s *`: a core busy forever.
const SPIN: [u8; 2] = [0x60, 0xFE];
/// `stop #0x2700`: a core waiting with every interrupt masked.
const STOP: [u8; 4] = [0x4E, 0x72, 0x27, 0x00];

fn data(n: u32) -> Vec<u8> {
    (0..n).map(|i| (i * 7 + i / 256) as u8).collect()
}

#[test]
fn enumerates_the_cdc_example() {
    let Some((mut m, mut host, usb)) = example() else {
        return;
    };
    host.connect(&mut m).unwrap();
    let (device, config) = host.enumerate(&mut m, ADDRESS).unwrap();
    assert_eq!(&device[..2], [18, 1], "bLength, bDescriptorType");
    assert_eq!(
        device[7], 64,
        "bMaxPacketSize0 (the example's max_packet_size_0)"
    );
    assert_eq!(&device[8..12], VID_PID);
    assert_eq!(&config[..2], [9, 2]);
    assert_eq!(
        usize::from(u16::from_le_bytes([config[2], config[3]])),
        config.len()
    );
    assert_eq!(
        config[4], 2,
        "a CDC-ACM function: communication and data interfaces"
    );
    // the notification endpoint, then the data interface's bulk OUT and IN (512 bytes at high speed)
    let eps = [(0x81, 3, 8), (0x01, 2, 512), (0x82, 2, 512)];
    assert_eq!(endpoints(&config), eps);
    assert_eq!(host.endpoint_list(), eps);
    assert_eq!(host.address(), ADDRESS);
    assert_eq!(m.device::<UsbOtg>(usb).address(), ADDRESS);
    assert!(m.device::<UsbOtg>(usb).high_speed());
}

#[test]
fn cdc_echo_round_trip() {
    let Some((mut m, mut host, usb)) = example() else {
        return;
    };
    opened(&mut m, &mut host);
    host.write(&mut m, b"hello").unwrap();
    assert_eq!(host.read(&mut m, 5).unwrap(), b"hello");
    // More than two packets: the second OUT is NAKed until the host has taken the first echo, which it does while
    // it waits (the outstanding read). With a 1024-byte URB, a full packet is held until the example's zero-length
    // packet after it ends the transfer.
    host.set_urb(Some(1024));
    let long = data(1100);
    host.write(&mut m, &long).unwrap();
    assert_eq!(host.read(&mut m, long.len()).unwrap(), long);
    assert_eq!(
        m.device::<UsbOtg>(usb).stats().in_sizes,
        [5, 512, 0, 512, 0, 76]
    );
}

#[test]
fn full_speed_clamps_bulk_packets_to_64_bytes() {
    let Some((mut m, mut host, usb)) = example() else {
        return;
    };
    m.device_mut::<UsbOtg>(usb).knobs().hs = false;
    opened(&mut m, &mut host);
    assert!(!m.device::<UsbOtg>(usb).high_speed());
    let short = data(100);
    host.write(&mut m, &short).unwrap();
    assert_eq!(host.read(&mut m, short.len()).unwrap(), short);
    // The configuration still says 512: the host sends 64 + 36 and the example echoes each packet; 64 is short of
    // the class's 512, so no zero-length packet follows it.
    assert_eq!(m.device::<UsbOtg>(usb).stats().in_sizes, [64, 36]);
}

#[test]
fn a_stalled_request_reports_stall() {
    let Some((mut m, mut host, _)) = example() else {
        return;
    };
    opened(&mut m, &mut host);
    // vendor requests the example has no handler for
    assert_eq!(
        host.control_in(&mut m, [0xC0, 0x99, 0, 0, 0, 0, 4, 0]),
        Err(HostError::Stall)
    );
    assert_eq!(
        host.control_out(&mut m, [0x40, 0x99, 0, 0, 0, 0, 0, 0], &[]),
        Err(HostError::Stall)
    );
    // the next setup clears EP0's stall
    let device = host
        .control_in(&mut m, [0x80, 6, 0, 1, 0, 0, 18, 0])
        .unwrap();
    assert_eq!(&device[8..12], VID_PID);
    host.write(&mut m, b"still here").unwrap();
    assert_eq!(host.read(&mut m, 10).unwrap(), b"still here");
}

#[test]
fn a_device_that_never_attaches_times_out() {
    let (mut m, mut host) = silent(&SPIN);
    host.set_timeouts(1_000, 10);
    assert_eq!(
        host.connect(&mut m),
        Err(HostError::Timeout("device pull-up (USBCMD[RS])"))
    );
    assert!(m.now() >= 1_000 * US && m.now() < 1_020 * US, "{}", m.now());
    assert_eq!(m.time_limit(), None, "the host's slice limit is removed");
}

#[test]
fn a_core_waiting_in_stop_waits_for_the_host_until_the_default_timeout() {
    // Nothing can wake the core: each slice moves the clock on instead of stopping with HaltedForever.
    // guess: TIMEOUT_US and POLL_US, the defaults.
    let (mut m, mut host) = silent(&STOP);
    assert_eq!(
        host.connect(&mut m),
        Err(HostError::Timeout("device pull-up (USBCMD[RS])"))
    );
    assert!(
        m.now() >= TIMEOUT_US * US && m.now() <= (TIMEOUT_US + POLL_US) * US,
        "{}",
        m.now()
    );
    assert_eq!(m.instructions(), 1, "only the STOP ran");
}

#[test]
fn the_callers_time_limit_stops_the_host() {
    let (mut m, mut host) = silent(&SPIN);
    m.set_time_limit(Some(55 * US));
    match host.connect(&mut m) {
        Err(HostError::Stopped(r)) => assert_eq!(r.kind, Stop::TimeLimit, "{r:?}"),
        other => panic!("{other:?}"),
    }
    assert_eq!(m.now(), 55 * US);
    assert_eq!(
        m.time_limit(),
        Some(55 * US),
        "the caller's limit is put back"
    );
}

#[test]
fn a_core_stop_stops_the_host() {
    // ILLEGAL with no vector table to take it
    let (mut m, mut host) = silent(&[0x4A, 0xFC]);
    match host.connect(&mut m) {
        Err(HostError::Stopped(r)) => assert_ne!(r.kind, Stop::TimeLimit, "{r:?}"),
        other => panic!("{other:?}"),
    }
}

#[test]
fn bulk_transfers_need_an_enumerated_configuration() {
    let (mut m, mut host) = silent(&SPIN);
    assert!(matches!(
        host.write(&mut m, b"x"),
        Err(HostError::Protocol(_))
    ));
    assert!(matches!(host.read(&mut m, 1), Err(HostError::Protocol(_))));
}

#[test]
fn endpoints_parse_a_configuration() {
    let config = [
        9, 2, 32, 0, 1, 1, 0, 0x80, 50, // configuration
        9, 4, 0, 0, 2, 0xFF, 0, 0, 0, // interface
        7, 5, 0x81, 2, 0x00, 0x02, 0, // bulk IN, 512
        7, 5, 0x02, 3, 0x40, 0x18,
        4, // interrupt OUT, 64 with 3 transactions per microframe (bits 12:11)
        0, 5, // a zero length ends the walk
    ];
    assert_eq!(endpoints(&config), [(0x81, 2, 512), (0x02, 3, 64)]);
    assert_eq!(
        endpoints(&config[..20]),
        [],
        "a cut-off descriptor is not read"
    );
}
