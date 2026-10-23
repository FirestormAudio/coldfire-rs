//! Interrupts: levels (INTC ICRn, 1–6), masking (CIMR/SIMR, IMRH/L), software pending (INTFRCH/L), and
//! embassy's compile-time binding of handlers (`bind_interrupts!`).
//!
//! Vector v (64..=255) is controller (v − 64) / 64, source (v − 64) % 64. Level 7 is non-maskable: critical
//! sections (SR = 0x2700) can't exclude it, so it isn't offered.
pub use crate::pac::Interrupt;

/// An interrupt level a driver may use.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Priority {
    P1 = 1,
    P2 = 2,
    P3 = 3,
    P4 = 4,
    P5 = 5,
    P6 = 6,
}

impl Priority {
    /// Panics outside 1..=6.
    pub const fn from_level(level: u8) -> Self {
        match level {
            1 => Self::P1,
            2 => Self::P2,
            3 => Self::P3,
            4 => Self::P4,
            5 => Self::P5,
            6 => Self::P6,
            _ => panic!("interrupt level must be 1..=6"),
        }
    }
}

/// Controller and source of a CPU vector 64..=255.
pub const fn split(vector: u16) -> (u8, u8) {
    assert!(vector >= 64 && vector < 256, "not an INTC vector");
    let v = vector - 64;
    ((v / 64) as u8, (v % 64) as u8)
}

/// The 32-bit word (high: sources 32..=63) and bit of a source in IPR/IMR/INTFRC.
pub(crate) const fn word_bit(source: u8) -> (bool, u32) {
    if source >= 32 { (true, 1 << (source - 32)) } else { (false, 1 << source) }
}

/// `$r` bound to the PAC block of controller `$n` (the three are separate types with the same registers).
macro_rules! with_intc {
    ($n:expr, $r:ident => $e:expr) => {
        match $n {
            0 => {
                let $r = crate::pac::INTC0;
                $e
            }
            1 => {
                let $r = crate::pac::INTC1;
                $e
            }
            2 => {
                let $r = crate::pac::INTC2;
                $e
            }
            n => panic!("no INTC{}", n),
        }
    };
}

/// Interrupt-controller operations on anything with a vector number.
pub trait InterruptExt: Copy {
    fn vector(self) -> u16;

    /// Set the source's level (ICR).
    fn set_priority(self, p: Priority) {
        let (c, s) = split(self.vector());
        with_intc!(c, r => r.icr(s as usize).write(|w| w.set_level(p as u8)))
    }

    /// The source's level, 0..=7 as the ICR holds it.
    fn priority(self) -> u8 {
        let (c, s) = split(self.vector());
        with_intc!(c, r => r.icr(s as usize).read().level())
    }

    /// Unmask the source (CIMR).
    ///
    /// # Safety
    /// The handler may run from here on: whatever it touches must be ready, and a critical section that isn't
    /// the `critical-section` crate's (raising SR) doesn't exclude it.
    unsafe fn enable(self) {
        let (c, s) = split(self.vector());
        with_intc!(c, r => r.cimr().write(|w| w.set_cimr(s)))
    }

    /// Mask the source (SIMR).
    fn disable(self) {
        let (c, s) = split(self.vector());
        with_intc!(c, r => r.simr().write(|w| w.set_simr(s)))
    }

    fn is_enabled(self) -> bool {
        let (c, s) = split(self.vector());
        let (high, bit) = word_bit(s);
        with_intc!(c, r => if high { r.imrh().read() & bit == 0 } else { r.imrl().read().int_mask() & bit == 0 })
    }

    /// Whether the source is requesting: its peripheral's request (IPR) or a software pend (INTFRC). IPR doesn't
    /// show forced sources (checked on hardware), so both are read. The mask doesn't affect either.
    fn is_pending(self) -> bool {
        let (c, s) = split(self.vector());
        let (high, bit) = word_bit(s);
        with_intc!(c, r => if high {
            (r.iprh().read() | r.intfrch().read()) & bit != 0
        } else {
            (r.iprl().read().int() | r.intfrcl().read().intfrcl()) & bit != 0
        })
    }

    /// Pend the source from software (its INTFRC bit).
    fn pend(self) {
        force(self.vector(), true)
    }

    /// Clear the software pend. A peripheral's own request clears at the peripheral.
    fn unpend(self) {
        force(self.vector(), false)
    }
}

fn force(vector: u16, on: bool) {
    let (c, s) = split(vector);
    let (high, bit) = word_bit(s);
    // A higher-level pender may run in the middle of the read-modify-write.
    critical_section::with(|_| {
        with_intc!(c, r => if high {
            r.intfrch().modify(|v| if on { *v |= bit } else { *v &= !bit })
        } else {
            r.intfrcl().modify(|w| {
                let v = w.intfrcl();
                w.set_intfrcl(if on { v | bit } else { v & !bit })
            })
        })
    })
}

impl InterruptExt for Interrupt {
    fn vector(self) -> u16 {
        self as u16
    }
}

/// An INTC source no peripheral drives, used as a software interrupt (pended through INTFRCH/L).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct SoftIrq {
    intc: u8,
    source: u8,
}

impl SoftIrq {
    pub const fn new(intc: u8, source: u8) -> Self {
        assert!(intc <= 2 && source < 64, "SoftIrq: INTC 0..=2, source 0..=63");
        Self { intc, source }
    }

    /// CPU vector: 64 + 64 × controller + source.
    pub const fn vector(self) -> u16 {
        64 + 64 * self.intc as u16 + self.source as u16
    }
}

impl InterruptExt for SoftIrq {
    fn vector(self) -> u16 {
        SoftIrq::vector(self)
    }
}

/// Embassy's type-level interrupts: drivers name `typelevel::UART9`, apps prove a handler with
/// `bind_interrupts!`.
pub mod typelevel {
    use super::{InterruptExt, Priority};

    trait SealedInterrupt {}

    #[allow(private_bounds)]
    pub trait Interrupt: SealedInterrupt {
        const IRQ: super::Interrupt;
        fn set_priority(p: Priority) {
            Self::IRQ.set_priority(p)
        }
        /// # Safety
        /// As [`InterruptExt::enable`].
        unsafe fn enable() {
            unsafe { Self::IRQ.enable() }
        }
        fn disable() {
            Self::IRQ.disable()
        }
        fn is_pending() -> bool {
            Self::IRQ.is_pending()
        }
        fn pend() {
            Self::IRQ.pend()
        }
        fn unpend() {
            Self::IRQ.unpend()
        }
    }

    /// A driver's interrupt handler.
    pub trait Handler<I: Interrupt> {
        /// # Safety
        /// Called only from the bound interrupt.
        unsafe fn on_interrupt();
    }

    /// Proof, made only by `bind_interrupts!`, that `H` runs on `I`.
    ///
    /// # Safety
    /// Implement only through `bind_interrupts!`.
    pub unsafe trait Binding<I: Interrupt, H: Handler<I>>: Copy {}

    macro_rules! irqs {
        ($($(#[$m:meta])* $name:ident),* $(,)?) => {$(
            $(#[$m])*
            #[allow(non_camel_case_types)]
            #[derive(Debug)]
            pub enum $name {}
            $(#[$m])*
            impl SealedInterrupt for $name {}
            $(#[$m])*
            impl Interrupt for $name {
                const IRQ: super::Interrupt = super::Interrupt::$name;
            }
        )*};
    }

    irqs!(
        EPORT_EPF1, EPORT_EPF2, EPORT_EPF3, EPORT_EPF4, EPORT_EPF5, EPORT_EPF6, EPORT_EPF7,
        PIT0, PIT1, PIT2, PIT3,
        UART0, UART1,
        #[cfg(any(feature = "mcf54415", feature = "mcf54416", feature = "mcf54417", feature = "mcf54418"))] UART2,
        UART3, UART4, UART5,
        #[cfg(any(feature = "mcf54415", feature = "mcf54416", feature = "mcf54417", feature = "mcf54418"))] UART6,
        UART7, UART8,
        #[cfg(any(feature = "mcf54415", feature = "mcf54416", feature = "mcf54417", feature = "mcf54418"))] UART9,
        #[cfg(any(feature = "mcf54415", feature = "mcf54416", feature = "mcf54417", feature = "mcf54418"))] USBOTG,
        EDMA_INTR_INT00, EDMA_INTR_INT01, EDMA_INTR_INT02, EDMA_INTR_INT03, EDMA_INTR_INT04, EDMA_INTR_INT05,
        EDMA_INTR_INT06, EDMA_INTR_INT07, EDMA_INTR_INT08, EDMA_INTR_INT09, EDMA_INTR_INT10, EDMA_INTR_INT11,
        EDMA_INTR_INT12, EDMA_INTR_INT13, EDMA_INTR_INT14, EDMA_INTR_INT15, EDMA_INTR_INT16, EDMA_INTR_INT17,
        EDMA_INTR_INT18, EDMA_INTR_INT19, EDMA_INTR_INT20, EDMA_INTR_INT21, EDMA_INTR_INT22, EDMA_INTR_INT23,
        EDMA_INTR_INT24, EDMA_INTR_INT25, EDMA_INTR_INT26, EDMA_INTR_INT27, EDMA_INTR_INT28, EDMA_INTR_INT29,
        EDMA_INTR_INT30, EDMA_INTR_INT31, EDMA_INTR_INT32, EDMA_INTR_INT33, EDMA_INTR_INT34, EDMA_INTR_INT35,
        EDMA_INTR_INT36, EDMA_INTR_INT37, EDMA_INTR_INT38, EDMA_INTR_INT39, EDMA_INTR_INT40, EDMA_INTR_INT41,
        EDMA_INTR_INT42, EDMA_INTR_INT43, EDMA_INTR_INT44, EDMA_INTR_INT45, EDMA_INTR_INT46, EDMA_INTR_INT47,
        EDMA_INTR_INT48, EDMA_INTR_INT49, EDMA_INTR_INT50, EDMA_INTR_INT51, EDMA_INTR_INT52, EDMA_INTR_INT53,
        EDMA_INTR_INT54, EDMA_INTR_INT55, EDMA_INTR_INT56_63, EDMA_ERR_ERR_N, ESDHC,
    );
}

/// Bind handlers to interrupts: `bind_interrupts!(struct Irqs { UART9 => uart::InterruptHandler<UART9>; });`
/// Each interrupt gets a `#[coldfire_rt::interrupt]` handler that runs its handlers in order; the app must depend
/// on `coldfire-rt` (the attribute's expansion names it).
#[macro_export]
macro_rules! bind_interrupts {
    ($(#[$attr:meta])* $vis:vis struct $name:ident { $($irq:ident => $($handler:ty),*;)* }) => {
        #[derive(Copy, Clone)]
        $(#[$attr])*
        $vis struct $name;
        $(
            #[$crate::rt::interrupt(vectors = $crate::pac::Interrupt)]
            fn $irq() {
                $(unsafe {
                    <$handler as $crate::interrupt::typelevel::Handler<$crate::interrupt::typelevel::$irq>>::on_interrupt()
                };)*
            }
            $(unsafe impl $crate::interrupt::typelevel::Binding<$crate::interrupt::typelevel::$irq, $handler> for $name {})*
        )*
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn split_vectors_into_controller_and_source() {
        assert_eq!(split(64), (0, 0));
        assert_eq!(split(64 + 40), (0, 40)); // INTC0 source 40
        assert_eq!(split(181), (1, 53)); // UART9
        assert_eq!(split(190), (1, 62)); // INTC1 source 62, a software interrupt
        assert_eq!(split(205), (2, 13)); // PIT0
        assert_eq!(split(Interrupt::ESDHC as u16), (2, 31)); // eSDHC
        #[cfg(any(feature = "mcf54415", feature = "mcf54416", feature = "mcf54417", feature = "mcf54418"))]
        assert_eq!(split(Interrupt::USBOTG as u16), (2, 17)); // USB OTG
        assert_eq!(split(255), (2, 63));
    }

    #[test]
    #[should_panic]
    fn split_refuses_cpu_exceptions() {
        split(63);
    }

    #[test]
    fn word_bit_picks_the_high_word_from_source_32() {
        assert_eq!(word_bit(0), (false, 1));
        assert_eq!(word_bit(31), (false, 1 << 31));
        assert_eq!(word_bit(32), (true, 1));
        assert_eq!(word_bit(62), (true, 1 << 30));
    }

    #[test]
    fn soft_irq_vector() {
        assert_eq!(SoftIrq::new(1, 62).vector(), 190);
        assert_eq!(SoftIrq::new(0, 5).vector(), 69);
    }

    #[test]
    fn priorities_are_levels_one_to_six() {
        assert_eq!(Priority::from_level(1) as u8, 1);
        assert_eq!(Priority::from_level(6), Priority::P6);
    }

    #[test]
    #[should_panic(expected = "1..=6")]
    fn level_seven_is_refused() {
        Priority::from_level(7);
    }
}
