//! The UARTs UART0-UART9 (RM ch. 41).

use std::fmt;

use crate::device::{Device, DeviceCx};
use crate::mem::Size;

const UMR: u32 = 0x00;
const USR: u32 = 0x04;
const UCR: u32 = 0x08;
const URB: u32 = 0x0C;
const UIPCR: u32 = 0x10;
const UISR: u32 = 0x14;
const UIP: u32 = 0x34;

/// USR bits (RM Table 41-6).
const RXRDY: u8 = 0x01;
const TXRDY: u8 = 0x04;
const TXEMP: u8 = 0x08;
const OE: u8 = 0x10;
const FE: u8 = 0x40;

/// UMR1\[ERR\]: block error mode (RM Table 41-4).
const ERR: u8 = 0x20;

/// How a UART is wired and clocked: board values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UartConfig {
    /// The interrupt controller its request goes to (its [`Intc::index`](super::Intc::index)).
    pub controller: usize,
    /// Its source on that controller (RM Table 17-15, Table 17-16: UART0-UART3 are INTC0
    /// sources 26-29, UART4-UART9 INTC1 sources 48-53).
    pub source: u8,
    /// The DMA request line of its receive request (RM Table 19-6), if it is wired to the eDMA.
    pub rx_dma: Option<u8>,
    /// The ticks one character takes on the line, start bit to stop bit: the board's baud rate in ticks. 0 makes
    /// sending and receiving instant.
    pub ticks_per_byte: u64,
}

/// One UART (RM ch. 41), with a 1-deep receiver and a transmitter whose timing is one character per
/// [`UartConfig::ticks_per_byte`].
///
/// The host drives the receive line with [`Uart::receive`] and sees what is sent in [`Uart::sent`] (or through a
/// hook, [`Uart::on_byte`]). The receiver and the transmitter start disabled (RM 41.3.5): a board whose boot
/// stages left them on writes UCR.
///
/// Registers (byte registers; the access size is not checked and only the low byte of a write is used):
/// - UMR1/UMR2 (+0x00, reset 0): stored and read back through the mode register pointer (RM 41.3.1, RM
///   41.3.2). Of their fields only UMR1\[ERR\] acts. RxIRQ (FFULL as the request source), parity, the looping
///   modes and RTS/CTS control are not modelled.
/// - USR (+0x04 read): RXRDY, TXRDY, TXEMP, OE, FE. FFULL, PE and RB are never set. UCSR (+0x04 write) is ignored:
///   the timing is [`UartConfig::ticks_per_byte`].
/// - UCR (+0x08 write): the RC and TC commands, then MISC (so 0x21 ends with the receiver disabled). MISC 1 resets
///   the mode register pointer, 2 resets the receiver (disables it and empties the holding register; bytes still
///   on the line arrive later and are lost), 3 resets the transmitter (disables it and abandons the character
///   being sent), 4 resets the error status (OE and FE); the break commands are ignored.
/// - URB (+0x0C read): the received character, which the read pops with its status. An empty URB reads the last
///   character again (guess: RM Table 41-3 gives only its reset value, 0xFF).
/// - UTB (+0x0C write): starts sending the character when the transmitter is on; it is dropped
///   ([`Uart::dropped`]) when it is off.
/// - UISR (+0x14 read): TXRDY (bit 0) and RXRDY (bit 1), unmasked; COS and DB are never set. UIMR (+0x14 write):
///   bit 0 TXRDY and bit 1 RXRDY are the interrupt mask.
/// - UIPCR (+0x10) reads 0x0F and UIP (+0x34) reads 0xFF: CTS is never asserted (guess: no device drives it).
///   UACR, UBG1/UBG2 and UOP1/UOP0 take writes and do nothing. The rest of the window reads 0.
///
/// The transmitter: a UTB write makes TXRDY and TXEMP read 0 until the character is sent, `ticks_per_byte`
/// after it starts; characters written while one is being sent are queued behind it. The holding register and
/// the shift register are not told apart: TXRDY and TXEMP are the same. The transmit DMA request is not driven.
///
/// The receiver: a character arrives `ticks_per_byte` after its start bit, and lands in the holding register,
/// setting RXRDY, when the receiver is on and the register is empty. Otherwise it is lost ([`Uart::lost`]): with
/// the receiver off, with no status (RM 41.3.5), or over a held character, setting OE (RM Table 41-6); the framing
/// error of a lost character is lost with it. The three-deep FIFO is not modelled. A framing error is received as 0xFF with FE: in character
/// mode FE belongs to the character held and is popped with it, in block mode (UMR1\[ERR\]) it holds until RESET
/// ERROR STATUS (RM 41.4.2.3).
///
/// Requests: the interrupt (source [`UartConfig::source`] of controller [`UartConfig::controller`]) is
/// `UIMR[0] & TXRDY | UIMR[1] & RXRDY`; the receive DMA request (line [`UartConfig::rx_dma`]) is RXRDY (RM
/// 41.5.1.2), so the eDMA's read of URB empties the holding register and lowers it.
pub struct Uart {
    config: UartConfig,
    umr: [u8; 2],
    /// The mode register pointer is on UMR2.
    umr2: bool,
    uimr: u8,
    rx_on: bool,
    /// The holding register: the character and its framing error.
    rbr: Option<(u8, bool)>,
    /// The last character read from URB.
    last: u8,
    oe: bool,
    /// Block mode's FE: any character with a framing error since RESET ERROR STATUS.
    block_fe: bool,
    /// Characters on the line: (arrival tick, character or `None` for a framing error), in arrival order.
    rx_pending: Vec<(u64, Option<u8>)>,
    tx_on: bool,
    /// The tick at which the transmitter has sent everything written.
    busy_until: u64,
    /// The tick TXRDY rises at, while a character is being sent.
    tx_edge: Option<u64>,
    sent: Vec<(u64, u8)>,
    hook: Option<Box<dyn FnMut(u64, u8)>>,
    lost: u64,
    dropped: u64,
}

impl fmt::Debug for Uart {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Uart")
            .field("config", &self.config)
            .field("rx_on", &self.rx_on)
            .field("tx_on", &self.tx_on)
            .field("rbr", &self.rbr)
            .field("sent", &self.sent.len())
            .field("lost", &self.lost)
            .field("dropped", &self.dropped)
            .finish_non_exhaustive()
    }
}

impl Uart {
    /// A UART at reset, wired and clocked as `config` says.
    pub fn new(config: UartConfig) -> Uart {
        Uart {
            config,
            umr: [0; 2],
            umr2: false,
            uimr: 0,
            rx_on: false,
            rbr: None,
            last: 0xFF,
            oe: false,
            block_fe: false,
            rx_pending: Vec::new(),
            tx_on: false,
            busy_until: 0,
            tx_edge: None,
            sent: Vec::new(),
            hook: None,
            lost: 0,
            dropped: 0,
        }
    }

    /// Characters on the receive line: (tick of the start bit, character), `None` for a character with a framing
    /// error. Each arrives [`UartConfig::ticks_per_byte`] after its start; characters arriving at the same tick
    /// arrive in the order given.
    pub fn receive(&mut self, bytes: &[(u64, Option<u8>)]) {
        for &(start, b) in bytes {
            let at = start + self.config.ticks_per_byte;
            let i = self.rx_pending.partition_point(|&(t, _)| t <= at);
            self.rx_pending.insert(i, (at, b));
        }
    }

    /// The characters sent, each with the tick of its UTB write.
    pub fn sent(&self) -> &[(u64, u8)] {
        &self.sent
    }

    /// The characters sent, as UTF-8 (invalid sequences replaced).
    pub fn text(&self) -> String {
        let bytes: Vec<u8> = self.sent.iter().map(|&(_, b)| b).collect();
        String::from_utf8_lossy(&bytes).into_owned()
    }

    /// Call `f` with the tick and the character of every character sent from now on: a board's peer on the line.
    pub fn on_byte(&mut self, f: Box<dyn FnMut(u64, u8)>) {
        self.hook = Some(f);
    }

    /// Received characters lost: arriving with the receiver off or over a held character.
    pub fn lost(&self) -> u64 {
        self.lost
    }

    /// Characters written to UTB with the transmitter off.
    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    fn usr(&self, now: u64) -> u8 {
        let mut v = 0;
        if let Some((_, fe)) = self.rbr {
            v |= RXRDY;
            if fe && self.umr[0] & ERR == 0 {
                v |= FE;
            }
        }
        if self.block_fe && self.umr[0] & ERR != 0 {
            v |= FE;
        }
        if self.oe {
            v |= OE;
        }
        if self.tx_on && now >= self.busy_until {
            v |= TXRDY | TXEMP;
        }
        v
    }

    fn uisr(&self, now: u64) -> u8 {
        let usr = self.usr(now);
        u8::from(usr & TXRDY != 0) | u8::from(usr & RXRDY != 0) << 1
    }

    fn drive(&self, cx: &mut DeviceCx) {
        let on = self.uisr(cx.now) & self.uimr & 3 != 0;
        cx.irq.set(self.config.controller, self.config.source, on);
        if let Some(ch) = self.config.rx_dma {
            cx.dma.set(ch, self.rbr.is_some());
        }
    }

    fn arrive(&mut self, b: Option<u8>) {
        if !self.rx_on {
            self.lost += 1;
        } else if self.rbr.is_some() {
            self.oe = true;
            self.lost += 1;
        } else {
            let fe = b.is_none();
            self.rbr = Some((b.unwrap_or(0xFF), fe));
            self.block_fe |= fe;
        }
    }

    fn command(&mut self, v: u8) {
        match v & 3 {
            1 => self.rx_on = true,
            2 => self.rx_on = false,
            _ => {}
        }
        match v >> 2 & 3 {
            1 => self.tx_on = true,
            2 => self.tx_on = false,
            _ => {}
        }
        match v >> 4 & 7 {
            1 => self.umr2 = false,
            2 => {
                self.rx_on = false;
                self.rbr = None;
            }
            3 => {
                self.tx_on = false;
                self.busy_until = 0;
                self.tx_edge = None;
            }
            4 => {
                self.oe = false;
                self.block_fe = false;
                if let Some((_, fe)) = &mut self.rbr {
                    *fe = false;
                }
            }
            _ => {}
        }
    }

    fn send(&mut self, b: u8, now: u64) {
        if !self.tx_on {
            self.dropped += 1;
            return;
        }
        self.busy_until = self.busy_until.max(now) + self.config.ticks_per_byte;
        self.tx_edge = (self.busy_until > now).then_some(self.busy_until);
        self.sent.push((now, b));
        if let Some(f) = &mut self.hook {
            f(now, b);
        }
    }
}

impl Device for Uart {
    fn read(&mut self, offset: u32, _size: Size, cx: &mut DeviceCx) -> u32 {
        let v = match offset {
            UMR => {
                let i = usize::from(self.umr2);
                self.umr2 = true;
                self.umr[i]
            }
            USR => self.usr(cx.now),
            URB => {
                if let Some((b, _)) = self.rbr.take() {
                    self.last = b;
                }
                self.drive(cx);
                self.last
            }
            UIPCR => 0x0F,
            UISR => self.uisr(cx.now),
            UIP => 0xFF,
            _ => 0,
        };
        u32::from(v)
    }

    fn write(&mut self, offset: u32, _size: Size, value: u32, cx: &mut DeviceCx) {
        let v = value as u8;
        match offset {
            UMR => {
                let i = usize::from(self.umr2);
                self.umr2 = true;
                self.umr[i] = v;
            }
            UCR => self.command(v),
            URB => self.send(v, cx.now),
            UISR => self.uimr = v,
            _ => {}
        }
        self.drive(cx);
    }

    fn next_event(&self) -> Option<u64> {
        let rx = self.rx_pending.first().map(|&(t, _)| t);
        let tx = self.tx_edge.filter(|_| self.uimr & 1 != 0);
        match (rx, tx) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    fn advance(&mut self, now: u64, cx: &mut DeviceCx) {
        let due = self.rx_pending.partition_point(|&(t, _)| t <= now);
        let later = self.rx_pending.split_off(due);
        for (_, b) in std::mem::replace(&mut self.rx_pending, later) {
            self.arrive(b);
        }
        if self.tx_edge.is_some_and(|t| t <= now) {
            self.tx_edge = None;
        }
        self.drive(cx);
    }

    fn poll_counted(&self) -> bool {
        false
    }

    /// Its own source, when UIMR enables the request a pending character or a character being sent will raise.
    /// The receive DMA's interrupts are listed by the eDMA.
    fn wake_sources(&self, out: &mut Vec<(usize, u8)>) -> bool {
        let rx = self.uimr & 2 != 0 && !self.rx_pending.is_empty();
        let tx = self.uimr & 1 != 0 && self.tx_edge.is_some();
        if rx || tx {
            out.push((self.config.controller, self.config.source));
        }
        true
    }
}
