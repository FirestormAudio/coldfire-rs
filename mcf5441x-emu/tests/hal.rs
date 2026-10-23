//! embassy-mcf5441x's drivers on the emulated chip: the programs of coldfire-rs's `hal-tests` crate, built with
//! `toolchain/cargo-cf`, each run with the peers it needs under libgloss semihosting ([`Semihost`]). A program
//! passes by exiting 0; a failed check prints `FAIL file:line: ...` and exits 1, a panic exits 101. The tests are
//! skipped (with a message) when the ColdFire toolchain is not built.

use std::cell::RefCell;
use std::io::{self, Write};
use std::path::PathBuf;
use std::process::Command;
use std::rc::Rc;
use std::sync::{Mutex, OnceLock};

use mcf5441x_emu::elf::load_elf;
use mcf5441x_emu::parts::{Emmc, EmmcConfig};
use mcf5441x_emu::semihost::{Outcome, Semihost};
use mcf5441x_emu::soc::{
    Chip, ChipOptions, Dspi, Frame, Mcf5441x, PeerCx, PinChange, SerialPeer, SpiDevice, Ssi, Uart,
};
use mcf5441x_emu::{Limit, Machine};

/// DDR as hal-tests/memory.x has it: 128 MB at 0x40000000, the stack at its top.
const DDR: (u32, u32) = (0x4000_0000, 0x0800_0000);
const STACK_TOP: u32 = 0x4800_0000;
/// The SRAM where the programs' RAMBAR puts it.
const SRAM: (u32, u32) = (0x8000_0000, 0x1_0000);
/// The bus clock the HAL finds: the chip's reset PLL (FBKDIV 9, REFDIV 0, OUTDIV2 3) from the programs' 24 MHz
/// reference, a 240 MHz VCO over 4. One tick is one bus clock (the PIT counts one per tick).
const BUS_HZ: u64 = 60_000_000;
const TICKS_PER_MS: u64 = BUS_HZ / 1000;

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..")
}

/// The directory of the built programs, or `None` (with a message) when the ColdFire toolchain is not built.
fn built() -> Option<PathBuf> {
    static BUILT: OnceLock<Option<PathBuf>> = OnceLock::new();
    static BUILD: Mutex<()> = Mutex::new(());
    let _one = BUILD.lock().unwrap_or_else(|e| e.into_inner());
    BUILT
        .get_or_init(|| {
            let root = root();
            let rcf = std::env::var_os("RCF")
                .map_or_else(|| root.join("build/rust-coldfire"), PathBuf::from);
            if !rcf
                .join("rustc_codegen_gcc/target/debug/librustc_codegen_gcc.so")
                .exists()
            {
                return None;
            }
            let crate_dir = root.join("hal-tests");
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
            // cargo-cf runs in the crate's directory: the target directory is relative to it
            let out = cmd
                .arg(&crate_dir)
                .args(["--target-dir", "target/emu"])
                .env("RCF", &rcf)
                .output()
                .expect("toolchain/cargo-cf runs");
            let err = String::from_utf8_lossy(&out.stderr);
            assert!(
                out.status.success(),
                "cargo-cf hal-tests failed:\n{}",
                &err[err.len().saturating_sub(4000)..]
            );
            Some(crate_dir.join("target/emu/m68k-coldfire-none-elf/release"))
        })
        .clone()
}

/// Program `name` as ELF bytes; `None` (with a message) when the ColdFire toolchain is not built.
fn elf(name: &str) -> Option<Vec<u8>> {
    let Some(dir) = built() else {
        eprintln!("skipped: the ColdFire toolchain is not built (toolchain/setup.sh, or set RCF)");
        return None;
    };
    Some(std::fs::read(dir.join(name)).unwrap_or_else(|e| panic!("{name} built: {e}")))
}

/// A machine with the chip (`options`), DDR and SRAM.
fn machine(options: ChipOptions) -> (Machine, Chip) {
    let mut m = Machine::new();
    m.mem.map_ram(DDR.0, DDR.1);
    m.mem.map_ram(SRAM.0, SRAM.1);
    let chip = Mcf5441x::build(&mut m, options);
    (m, chip)
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
    m.cpu.pc = load_elf(m, elf).unwrap();
    m.cpu.set_sr(0x2700);
    m.cpu.a[7] = STACK_TOP;
    m.set_time_limit(Some(m.now() + max_ms * TICKS_PER_MS));
    let (out, err) = (Sink::default(), Sink::default());
    let mut host = Semihost::new(
        Box::new(io::empty()),
        Box::new(out.clone()),
        Box::new(err.clone()),
    );
    host.clock_hz = BUS_HZ;
    let outcome = host.run(m, Limit::default());
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

/// 115,200 baud in ticks: ten bits a character.
const CHAR_TICKS: u64 = BUS_HZ * 10 / 115_200;

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

/// The SSI0 line's frame `n`: two 16-bit slots.
fn ssi_frame(n: u64) -> Vec<i32> {
    let x = (n & 0x7FFF) as i32;
    vec![x, -x]
}

/// From the first frame `capture` sends that isn't silence, each frame is the line's frame a fixed delay earlier
/// plus `add` (16-bit), for `frames` frames; returns the delay.
fn ssi_follows(capture: &[(u64, Vec<i32>)], add: i32, frames: usize) -> usize {
    let first = capture
        .iter()
        .position(|(_, f)| f.iter().any(|&w| w != 0))
        .expect("the stream sends something");
    let x = (capture[first].1[0] as i16).wrapping_sub(add as i16) as i64;
    let delay = first - x as usize;
    for (k, (_, f)) in capture[first..first + frames].iter().enumerate() {
        let want: Vec<i16> = ssi_frame((first + k - delay) as u64)
            .iter()
            .map(|&w| (w as i16).wrapping_add(add as i16))
            .collect();
        let got: Vec<i16> = f.iter().map(|&w| w as i16).collect();
        assert_eq!(got, want, "frame {}", first + k);
    }
    delay
}

/// 48 kHz frames at the bus clock.
const FRAME_TICKS: u64 = BUS_HZ / 48_000;

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
