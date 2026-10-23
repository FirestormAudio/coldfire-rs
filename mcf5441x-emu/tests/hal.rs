//! embassy-mcf5441x's drivers on the emulated chip: the programs of coldfire-rs's `hal-tests` crate, built with
//! `toolchain/cargo-cf`, each run with the peers it needs under libgloss semihosting ([`Semihost`]). A program
//! passes by exiting 0; a failed check prints `FAIL file:line: ...` and exits 1, a panic exits 101. The tests are
//! skipped (with a message) when the ColdFire toolchain is not built.

mod programs;

use std::cell::RefCell;
use std::io::{self, Write};
use std::rc::Rc;

use mcf5441x_emu::parts::{Emmc, EmmcConfig};
use mcf5441x_emu::semihost::{Outcome, Semihost};
use mcf5441x_emu::soc::{
    ChipOptions, Dspi, Frame, PeerCx, PinChange, SerialPeer, SpiDevice, Ssi, Uart,
};
use mcf5441x_emu::{Limit, Machine, Stop};
use programs::{
    BUS_HZ, CHAR_TICKS, FRAME_TICKS, TICKS_PER_MS, machine, ssi_follows, ssi_frame, start,
};

/// Program `name` of `hal-tests`.
fn elf(name: &str) -> Option<Vec<u8>> {
    programs::elf("hal-tests", name)
}

/// A writer the test reads back.
#[derive(Clone, Default)]
struct Sink(Rc<RefCell<Vec<u8>>>);

impl Write for Sink {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.0.borrow_mut().extend_from_slice(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl Sink {
    fn text(&self) -> String {
        String::from_utf8_lossy(&self.0.borrow()).into_owned()
    }
}

/// How a program's run ended.
struct Ran {
    outcome: Outcome,
    stdout: String,
    stderr: String,
}

/// Load `elf` as a boot stage leaves a program (supervisor mode, interrupts masked, the stack at the top of DDR)
/// and run it under semihosting for at most `max_ms` of emulated time.
fn run(m: &mut Machine, elf: &[u8], max_ms: u64) -> Ran {
    run_slices(m, elf, &mut [], max_ms)
}

/// A change the host makes at a time of the run: (emulated ms from the start, the change).
type Change<'a> = (u64, &'a mut dyn FnMut(&mut Machine));

/// As [`run`], stopping at each of `changes`' times (in order) to make the change, then running on: the program
/// sees each change at its time, and may wait for it asleep.
fn run_slices(m: &mut Machine, elf: &[u8], changes: &mut [Change], max_ms: u64) -> Ran {
    start(m, elf, max_ms);
    let end = m.time_limit();
    let (out, err) = (Sink::default(), Sink::default());
    let mut host = Semihost::new(
        Box::new(io::empty()),
        Box::new(out.clone()),
        Box::new(err.clone()),
    );
    host.clock_hz = BUS_HZ;
    let mut outcome = None;
    for (ms, change) in changes.iter_mut() {
        m.set_time_limit(Some(*ms * TICKS_PER_MS));
        match host.run(m, Limit::default()) {
            Outcome::Stopped(s) if s.kind == Stop::TimeLimit => change(m),
            o => {
                outcome = Some(o);
                break;
            }
        }
    }
    let outcome = outcome.unwrap_or_else(|| {
        m.set_time_limit(end);
        host.run(m, Limit::default())
    });
    Ran {
        outcome,
        stdout: out.text(),
        stderr: err.text(),
    }
}

/// The program passed: it exited 0.
fn passed(r: &Ran) {
    assert_eq!(
        r.outcome,
        Outcome::Exit(0),
        "stdout:\n{}\nstderr:\n{}",
        r.stdout,
        r.stderr
    );
}

/// Port `port`'s logged (PDDR, PODR) pairs, masked to `mask`, without repeats.
fn port_states(changes: &[PinChange], port: u8, mask: u8) -> Vec<(u8, u8)> {
    let mut states: Vec<(u8, u8)> = Vec::new();
    for c in changes.iter().filter(|c| c.port == port) {
        let s = (c.pddr & mask, c.podr & mask);
        if states.last() != Some(&s) {
            states.push(s);
        }
    }
    states
}

#[test]
fn gpio_drives_reads_and_pulls() {
    let Some(elf) = elf("gpio") else { return };
    let (mut m, chip) = machine(ChipOptions::default());
    // PD0 driven high for the whole run; PC2 left to its pull
    chip.gpio_mut(&mut m).drive(3, 0, Some(true));
    let r = run(&mut m, &elf, 100);
    passed(&r);
    assert_eq!(r.stdout.lines().count(), 5, "{}", r.stdout);
    let changes = chip.gpio_mut(&mut m).take_changes();
    // PA0: high, then an output driving it, then low, then high again
    assert_eq!(
        port_states(&changes, 0, 1),
        [(0, 1), (1, 1), (1, 0), (1, 1)]
    );
    // PB0: low as it becomes an output, so no change of the latch
    assert_eq!(port_states(&changes, 1, 1), [(1, 0)]);
    // PC2: low, then an output
    assert_eq!(port_states(&changes, 2, 1 << 2), [(1 << 2, 0)]);
    assert_eq!(chip.gpio(&m).output(2, 2), Some(false));
    // PD0 stays an input
    assert_eq!(chip.gpio(&m).output(3, 0), None);
}

/// Echoes each byte UART0 sends, but answers the 11th with a framing error and the 13th twice, back to back.
#[derive(Default)]
struct Loopback {
    sent: Vec<u8>,
}

impl SerialPeer for Loopback {
    fn transmitted(&mut self, byte: u8, cx: &mut PeerCx) {
        self.sent.push(byte);
        match self.sent.len() {
            11 => cx.send(cx.now, None),
            13 => {
                cx.send(cx.now, Some(byte));
                cx.send(cx.now + CHAR_TICKS, Some(byte));
            }
            _ => cx.send(cx.now, Some(byte)),
        }
    }
}

#[test]
fn uart_blocking_reads_writes_and_errors() {
    let Some(elf) = elf("uart") else { return };
    let (mut m, chip) = machine(ChipOptions::default().with_uart_ticks(0, CHAR_TICKS));
    m.device_mut::<Uart>(chip.uart[0])
        .attach(Box::new(Loopback::default()));
    let r = run(&mut m, &elf, 100);
    passed(&r);
    let sent = &m.device::<Uart>(chip.uart[0]).peer::<Loopback>().sent;
    assert_eq!(sent.as_slice(), b"0123456789XYZ");
}

/// Records what UART0 sends; once it sends `!`, sends [`STREAM`] pattern bytes, one a character time.
#[derive(Default)]
struct Stream {
    sent: Vec<u8>,
    /// The next byte to send and its start tick, once started.
    next: Option<(usize, u64)>,
}

/// The bytes [`Stream`] sends: byte i is `(i * 7 + i / 256) as u8`.
const STREAM: usize = 1500;

impl SerialPeer for Stream {
    fn transmitted(&mut self, byte: u8, cx: &mut PeerCx) {
        self.sent.push(byte);
        if byte == b'!' && self.next.is_none() {
            // after the `!` has left the line
            self.next = Some((0, cx.now + CHAR_TICKS));
        }
    }

    fn next_event(&self) -> Option<u64> {
        self.next.filter(|&(i, _)| i < STREAM).map(|(_, at)| at)
    }

    fn advance(&mut self, cx: &mut PeerCx) {
        if let Some((i, at)) = self.next
            && i < STREAM
        {
            let start = at.max(cx.now);
            cx.send(start, Some((i * 7 + i / 256) as u8));
            self.next = Some((i + 1, start + CHAR_TICKS));
        }
    }
}

#[test]
fn uart_buffered_wraps_its_ring_and_keeps_commands_whole() {
    let Some(elf) = elf("uart-buffered") else {
        return;
    };
    let (mut m, chip) = machine(ChipOptions::default().with_uart_ticks(0, CHAR_TICKS));
    m.device_mut::<Uart>(chip.uart[0])
        .attach(Box::new(Stream::default()));
    let r = run(&mut m, &elf, 2_000);
    passed(&r);
    let sent = &m.device::<Uart>(chip.uart[0]).peer::<Stream>().sent;
    let block: Vec<u8> = (0..200u32).map(|i| i as u8).collect();
    assert_eq!(&sent[..200], block.as_slice());
    let rest = &sent[200..];
    assert_eq!(rest.len(), 100 + 120 + 1, "{rest:?}");
    for k in 0..3u8 {
        let c = b'A' + k;
        let at = rest
            .iter()
            .position(|&b| b == c)
            .expect("each command arrives");
        assert!(
            rest[at..at + 40].iter().all(|&b| b == c),
            "command {} is whole: {:?}",
            c as char,
            rest
        );
        assert_eq!(rest.iter().filter(|&&b| b == c).count(), 40);
    }
    let digits: Vec<u8> = rest.iter().copied().filter(u8::is_ascii_digit).collect();
    assert_eq!(digits, b"0123456789".repeat(10));
    assert_eq!(rest.last(), Some(&b'!'));
}

/// A part that answers each byte with its complement, counting its selects.
#[derive(Default)]
struct Echo {
    selects: usize,
    selected: bool,
}

impl SpiDevice for Echo {
    fn select(&mut self, _now: u64) {
        self.selects += 1;
        self.selected = true;
    }
    fn transfer(&mut self, byte: u8) -> u8 {
        assert!(self.selected, "a frame with the part deselected");
        !byte
    }
    fn deselect(&mut self, _now: u64) {
        self.selected = false;
    }
}

/// (data, PCS, CONT, CTAS) of each frame.
fn frames(log: &[Frame]) -> Vec<(u32, u8, bool, u8)> {
    log.iter()
        .map(|f| (f.data, f.pcs, f.cont, f.ctas))
        .collect()
}

#[test]
fn spi_frames_follow_the_transactions() {
    let Some(elf) = elf("spi") else { return };
    let (mut m, chip) = machine(ChipOptions::default());
    m.device_mut::<Dspi>(chip.dspi[0])
        .attach(0, Box::new(Echo::default()));
    let r = run(&mut m, &elf, 100);
    passed(&r);
    let log = frames(&m.device_mut::<Dspi>(chip.dspi[0]).take_frame_log());
    // the select is held from a transaction's first frame to its last (CONT); 8-bit frames through CTAR0
    let byte = |d: u32, cont: bool| (d, 1, cont, 0);
    let mut want = vec![
        byte(1, true),
        byte(2, true),
        byte(3, true),
        byte(0, true),
        byte(0, false),
        byte(0x10, true),
        byte(0x20, false),
        // wide reads: the write in 8-bit frames, the reads in 16-bit frames through CTAR1
        byte(1, true),
        byte(2, true),
        byte(3, true),
        (0, 1, true, 1),
        (0, 1, false, 1),
    ];
    // the bare bus: no chip select, no CONT
    want.extend([
        (0xA5, 0, false, 0),
        (0x5A, 0, false, 0),
        (1, 0, false, 0),
        (2, 0, false, 0),
    ]);
    assert_eq!(log, want);
    assert_eq!(m.device::<Dspi>(chip.dspi[0]).device::<Echo>(0).selects, 3);
    // PC2, the bare bus's select: high as it becomes an output, low around the frames, high again
    let changes = chip.gpio_mut(&mut m).take_changes();
    assert_eq!(
        port_states(&changes, 2, 1 << 2),
        [(0, 4), (4, 4), (4, 0), (4, 4)]
    );
}

#[test]
fn dma_copies_rings_links_and_interrupts() {
    let Some(elf) = elf("dma") else { return };
    let (mut m, _) = machine(ChipOptions::default());
    let r = run(&mut m, &elf, 100);
    passed(&r);
}

#[test]
fn esdhc_reads_the_card() {
    let Some(elf) = elf("esdhc") else { return };
    const SECTORS: u32 = 2048;
    let image: Vec<u8> = (0..SECTORS as usize * 512)
        .map(|i| (i * 13 + i / 512) as u8)
        .collect();
    // a card answering as tests/esdhc.rs's does: MID 0x15, powered up with sector addressing
    let card = Emmc::new(
        image,
        EmmcConfig {
            sec_count: SECTORS,
            cid: [0x0000_0001, 0, 0, 0x0015_0000],
            csd: [0x1111_1111, 0x2222_2222, 0x3333_3333, 0x0044_4444],
            ext_csd: vec![],
            ocr: 0xC0FF_8080,
        },
    );
    let (mut m, _) = machine(ChipOptions::default().with_card(card));
    let r = run(&mut m, &elf, 2_000);
    passed(&r);
}

#[test]
fn ssi_streams_render_output() {
    let Some(elf) = elf("ssi") else { return };
    let (mut m, chip) = machine(ChipOptions::default().with_ssi_ticks(0, FRAME_TICKS));
    m.device_mut::<Ssi>(chip.ssi[0])
        .set_source(Box::new(ssi_frame));
    let r = run(&mut m, &elf, 200);
    passed(&r);
    let capture = m.device_mut::<Ssi>(chip.ssi[0]).take_capture();
    // 20 blocks of 64 frames were rendered; half of them is well inside the stream
    ssi_follows(&capture, 1, 640);
}

#[test]
fn time_follows_the_emulated_clock() {
    let Some(elf) = elf("time") else { return };
    let (mut m, _) = machine(ChipOptions::default());
    let r = run(&mut m, &elf, 1_000);
    passed(&r);
    assert_eq!(r.stdout.lines().count(), 4, "{}", r.stdout);
}

#[test]
fn an_interrupt_executor_preempts_a_lower_one() {
    let Some(elf) = elf("executor-interrupt") else {
        return;
    };
    let (mut m, _) = machine(ChipOptions::default());
    let r = run(&mut m, &elf, 100);
    passed(&r);
}

#[test]
fn eport_waits_for_edges_and_levels() {
    let Some(elf) = elf("eport") else { return };
    let (mut m, chip) = machine(ChipOptions::default());
    // IRQ2 low from the start, IRQ6 high
    chip.drive_irq(&mut m, 2, false);
    chip.drive_irq(&mut m, 6, true);
    let c = &chip;
    let r = run_slices(
        &mut m,
        &elf,
        &mut [
            (10, &mut |m: &mut Machine| c.drive_irq(m, 2, true)),
            (20, &mut |m: &mut Machine| c.drive_irq(m, 6, false)),
            (30, &mut |m: &mut Machine| c.drive_irq(m, 6, true)),
            (40, &mut |m: &mut Machine| c.drive_irq(m, 2, false)),
            (60, &mut |m: &mut Machine| c.drive_irq(m, 6, false)),
            (70, &mut |m: &mut Machine| c.drive_irq(m, 6, true)),
        ],
        200,
    );
    passed(&r);
    assert_eq!(
        r.stdout.lines().filter(|l| l.starts_with("step")).count(),
        8 + 5,
        "{}",
        r.stdout
    );
}

#[test]
fn dtim_counts_the_bus_clock() {
    let Some(elf) = elf("dtim") else { return };
    let (mut m, _) = machine(ChipOptions::default());
    let r = run(&mut m, &elf, 90_000);
    passed(&r);
}
