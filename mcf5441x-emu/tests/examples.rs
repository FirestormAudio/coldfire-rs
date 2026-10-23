//! coldfire-rs's `examples`, built with `toolchain/cargo-cf`, on the emulated chip: `uart-echo`, `ticker` and
//! `ssi-loop`, each checked by what its peer sees (`usb-cdc` is `usb_host.rs`'s). The tests are skipped (with a
//! message) when the ColdFire toolchain is not built.

mod programs;

use mcf5441x_emu::soc::{ChipOptions, PeerCx, SerialPeer, Ssi, Uart};
use mcf5441x_emu::{Limit, Machine, Stop};
use programs::{
    BUS_HZ, CHAR_TICKS, FRAME_TICKS, TICKS_PER_MS, machine, ssi_follows, ssi_frame, start,
};

/// Example `name`.
fn elf(name: &str) -> Option<Vec<u8>> {
    programs::elf("examples", name)
}

/// Run until the time limit [`start`] set; anything else is a failure.
fn run_out(m: &mut Machine) {
    let r = m.run(Limit::default());
    assert_eq!(r.kind, Stop::TimeLimit, "{r:?}");
}

const BANNER: &[u8] = b"uart-echo: type something\r\n";
const ECHOED: usize = 1000;

/// Records what UART0 sends; once the banner is out, sends [`ECHOED`] pattern bytes, one a character time.
#[derive(Default)]
struct Typist {
    sent: Vec<u8>,
    next: Option<(usize, u64)>,
}

impl SerialPeer for Typist {
    fn transmitted(&mut self, byte: u8, cx: &mut PeerCx) {
        self.sent.push(byte);
        if self.next.is_none() && self.sent == BANNER {
            self.next = Some((0, cx.now + CHAR_TICKS));
        }
    }

    fn next_event(&self) -> Option<u64> {
        self.next.filter(|&(i, _)| i < ECHOED).map(|(_, at)| at)
    }

    fn advance(&mut self, cx: &mut PeerCx) {
        if let Some((i, at)) = self.next
            && i < ECHOED
        {
            let start = at.max(cx.now);
            cx.send(start, Some(typed(i)));
            self.next = Some((i + 1, start + CHAR_TICKS));
        }
    }
}

fn typed(i: usize) -> u8 {
    (i * 7 + i / 256) as u8
}

#[test]
fn uart_echo_echoes() {
    let Some(elf) = elf("uart-echo") else { return };
    let (mut m, chip) = machine(ChipOptions::default().with_uart_ticks(0, CHAR_TICKS));
    m.device_mut::<Uart>(chip.uart[0])
        .attach(Box::new(Typist::default()));
    start(&mut m, &elf, 200);
    run_out(&mut m);
    let sent = &m.device::<Uart>(chip.uart[0]).peer::<Typist>().sent;
    let want: Vec<u8> = BANNER
        .iter()
        .copied()
        .chain((0..ECHOED).map(typed))
        .collect();
    assert_eq!(sent, &want);
}

#[test]
fn ticker_ticks_every_second() {
    let Some(elf) = elf("ticker") else { return };
    let (mut m, chip) = machine(ChipOptions::default());
    start(&mut m, &elf, 5_500);
    run_out(&mut m);
    let sent = m.device::<Uart>(chip.uart[0]).sent().to_vec();
    let text: String = sent.iter().map(|&(_, b)| b as char).collect();
    let lines: Vec<&str> = text.split_terminator("\r\n").collect();
    assert_eq!(lines.len(), 5, "{text:?}");
    let mut at = 0;
    for (n, line) in (1u64..).zip(&lines) {
        // the line's first byte, n s ± 1 ms from the start
        let tick = sent[at].0;
        assert!(
            tick.abs_diff(n * BUS_HZ) <= TICKS_PER_MS,
            "line {n} at tick {tick}"
        );
        let ms: u64 = line
            .strip_prefix(&format!("tick {n} at "))
            .and_then(|r| r.strip_suffix(" ms"))
            .and_then(|r| r.parse().ok())
            .unwrap_or_else(|| panic!("{line:?}"));
        assert!(ms.abs_diff(n * 1000) <= 1, "{line:?}");
        at += line.len() + 2;
    }
}

#[test]
fn ssi_loop_sends_what_it_receives() {
    let Some(elf) = elf("ssi-loop") else { return };
    let (mut m, chip) = machine(ChipOptions::default().with_ssi_ticks(0, FRAME_TICKS));
    m.device_mut::<Ssi>(chip.ssi[0])
        .set_source(Box::new(ssi_frame));
    start(&mut m, &elf, 100);
    run_out(&mut m);
    let capture = m.device_mut::<Ssi>(chip.ssi[0]).take_capture();
    ssi_follows(&capture, 0, 1000);
}
