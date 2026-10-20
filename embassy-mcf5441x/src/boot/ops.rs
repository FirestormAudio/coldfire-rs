//! The cold-boot sequence as data: a `const fn` turns a [`BootConfig`] into register operations in the reference
//! manual's order; the boot part's executor runs them from reset. Host tests check the list, so what the chip
//! sees is what the tests read.
use super::config::{settings, BootConfig, Ddr2Clk, ExtraWrite, Phase, Pll, Width, START};

/// One operation, 16 bytes. `width` is the access width in bytes (1, 2 or 4). A failed `POLL` or `CHECK` halts
/// with marker `0xB007_0000 | mark`; a `POLL` reads at most `1 << cap_log2` times.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Op {
    pub kind: u8,
    pub width: u8,
    pub mark: u8,
    pub cap_log2: u8,
    pub addr: u32,
    pub a: u32,
    pub b: u32,
}

/// `*addr = a`.
pub const KIND_WRITE: u8 = 1;
/// `*addr = (*addr & a) | b`.
pub const KIND_RMW: u8 = 2;
/// Read until `*addr & a == b`, at most `1 << cap_log2` reads; then halt.
pub const KIND_POLL: u8 = 3;
/// `a` iterations of `nop`.
pub const KIND_DELAY: u8 = 4;
/// `*addr & a == b`, or halt.
pub const KIND_CHECK: u8 = 5;
/// `movec a, RAMBAR`.
pub const KIND_RAMBAR: u8 = 6;
/// `movec a, VBR`.
pub const KIND_VBR: u8 = 7;
/// `movec a, CACR`.
pub const KIND_CACR: u8 = 8;

/// Halt markers (in D0 and after the `halt`, as `.short 0x51FB; .long marker`).
pub const MARK_PLL_MISMATCH: u32 = 0xB007_0001;
pub const MARK_PLL_LOCK: u32 = 0xB007_0002;
pub const MARK_DDR_INIT: u32 = 0xB007_0003;
pub const MARK_SPI: u32 = 0xB007_0004;
pub const MARK_NAND_ECC: u32 = 0xB007_0005;
pub const MARK_NAND_BUSY: u32 = 0xB007_0006;

/// Room for the longest sequence while it is built (the list a board links holds only its own length).
pub const OPS_MAX: usize = 160;

pub struct OpList {
    pub len: usize,
    pub ops: [Op; OPS_MAX],
}

// Register addresses (a const fn can't take them from the PAC's pointers; a host test checks they agree).
pub(crate) const MISCCR: u32 = 0xEC09_000E;
pub(crate) const MISCCR2: u32 = 0xEC09_001A;
pub(crate) const PLL_CR: u32 = 0xFC0C_0000;
pub(crate) const PLL_DR: u32 = 0xFC0C_0004;
pub(crate) const PLL_SR: u32 = 0xFC0C_0008;
pub(crate) const PPMCR0: u32 = 0xFC04_002D;
pub(crate) const MSCR_SDRAMC: u32 = 0xEC09_4060;
pub(crate) const DDR_CR: u32 = 0xFC0B_8000;
pub(crate) const DDR_RCR: u32 = 0xFC0B_8180;
pub(crate) const DDR_PADCR: u32 = 0xFC0B_81AC;
pub(crate) const CSAR0: u32 = 0xFC00_8000;
pub(crate) const CSMR0: u32 = 0xFC00_8004;
pub(crate) const CSCR0: u32 = 0xFC00_8008;

/// MISCCR[LIMP] (RM 10.3.4): the core runs from the reference while set.
pub(crate) const LIMP: u32 = 1 << 12;
/// MISCCR2[DDR2CLK] (RM 10.3.10): 1 takes the DDR 2× clock from the VCO, 0 from the core clock.
pub(crate) const DDR2CLK: u32 = 1 << 14;
const LOCK: u32 = 1 << 4;
/// SRAM at 0x8000_0000, backdoor enabled (other masters such as DMA reach it), RM Table 7-3's instructions-and-data
/// attributes.
const RAMBAR: u32 = 0x8000_0000 | 1 << 9 | 0x21;
/// CACR: invalidate the data, branch and instruction caches; enable nothing (RM 6: reset doesn't invalidate them).
const CACR_INVALIDATE: u32 = 1 << 24 | 1 << 18 | 1 << 8;
/// PPMCR0 slot of the DDR controller's clock (RM Table 9-5).
const DDR_SLOT: u32 = 46;
/// DDR_RCR[RST] (RM 21.4): resets the read clock recovery.
const RCR_RST: u32 = 1 << 30;
/// CR27 INTSTATUS bit 3: DRAM initialisation complete.
const INIT_DONE: u32 = 1 << 3;
/// RM 9.3: after leaving limp mode, wait 256 clocks before touching the DDR controller.
const AFTER_LIMP: u32 = 256;
const CAP: u8 = 20;

const CR_MASK: u32 = 0x3F | 0x7 << 8;
const fn cr_value(s: super::PllSettings) -> u32 {
    s.fbkdiv as u32 | (s.refdiv as u32) << 8
}
const DR_OUT1: u32 = 0x1F;
const DR_OUT2: u32 = 0x1F << 5;
const DR_OUT3: u32 = 0x1F << 10;
const DR_OUT4: u32 = 0x1F << 16;
const DR_OUT5: u32 = 0x1F << 21;
const fn dr_value(s: super::PllSettings) -> u32 {
    s.outdiv1 as u32 | (s.outdiv2 as u32) << 5 | (s.outdiv5 as u32) << 21
}

const fn op(kind: u8, width: u8, addr: u32, a: u32, b: u32) -> Op {
    Op { kind, width, mark: 0, cap_log2: 0, addr, a, b }
}

struct Builder {
    l: OpList,
}

impl Builder {
    const fn push(mut self, o: Op) -> Self {
        if self.l.len == OPS_MAX {
            panic!("BootConfig: the boot sequence needs more than OPS_MAX operations");
        }
        self.l.ops[self.l.len] = o;
        self.l.len += 1;
        self
    }
    const fn write(self, width: u8, addr: u32, v: u32) -> Self {
        self.push(op(KIND_WRITE, width, addr, v, 0))
    }
    const fn rmw(self, width: u8, addr: u32, keep: u32, set: u32) -> Self {
        self.push(op(KIND_RMW, width, addr, keep, set))
    }
    const fn poll(self, addr: u32, mask: u32, value: u32, mark: u32) -> Self {
        self.push(Op { kind: KIND_POLL, width: 4, mark: mark as u8, cap_log2: CAP, addr, a: mask, b: value })
    }
    const fn check(self, addr: u32, mask: u32, value: u32) -> Self {
        self.push(Op { kind: KIND_CHECK, width: 4, mark: MARK_PLL_MISMATCH as u8, cap_log2: 0, addr, a: mask, b: value })
    }
    const fn extras(mut self, w: &[ExtraWrite], phase: Phase) -> Self {
        let mut i = 0;
        while i < w.len() {
            let e = w[i];
            if matches!(
                (e.phase, phase),
                (Phase::InLimpWindow, Phase::InLimpWindow)
                    | (Phase::AfterPll, Phase::AfterPll)
                    | (Phase::BeforeDdrController, Phase::BeforeDdrController)
            ) {
                let width = match e.width {
                    Width::W8 => 1,
                    Width::W16 => 2,
                    Width::W32 => 4,
                };
                self = self.write(width, e.addr, e.value);
            }
            i += 1;
        }
        self
    }
}

/// The boot sequence for `c`, in the reference manual's order.
pub const fn sequence(c: &BootConfig) -> OpList {
    let empty = op(0, 0, 0, 0, 0);
    let mut b = Builder { l: OpList { len: 0, ops: [empty; OPS_MAX] } };
    b = b.push(op(KIND_RAMBAR, 4, 0, RAMBAR, 0)).push(op(KIND_VBR, 4, 0, 0x8000_0000, 0)).push(op(KIND_CACR, 4, 0, CACR_INVALIDATE, 0));
    let s = settings(c.pll);
    let (mut dr_keep, mut dr_set) = (!0u32, 0u32);
    if let Some(v) = c.outdiv3 {
        dr_keep &= !DR_OUT3;
        dr_set |= (v as u32) << 10;
    }
    if let Some(v) = c.outdiv4 {
        dr_keep &= !DR_OUT4;
        dr_set |= (v as u32) << 16;
    }
    let set = matches!(c.pll, Pll::Set(_));
    if !set {
        // the boot hardware set the PLL: it must be what the board says, before anything relies on it
        b = b.check(PLL_CR, CR_MASK, cr_value(s)).check(PLL_DR, DR_OUT1 | DR_OUT2 | DR_OUT5, dr_value(s));
    }
    if set {
        // every divider explicitly: OUTDIV3/OUTDIV4 off (0) unless the board sets them (their reset values are unsure)
        dr_keep &= !(DR_OUT1 | DR_OUT2 | DR_OUT3 | DR_OUT4 | DR_OUT5);
        dr_set |= dr_value(s);
    }
    let dr = dr_keep != !0u32;
    let mut in_limp = false;
    let mut i = 0;
    while i < c.extra.len() {
        in_limp |= matches!(c.extra[i].phase, Phase::InLimpWindow);
        i += 1;
    }
    if set || dr || c.ddr2clk.is_some() || in_limp {
        // RM 8.1.3.3: change the PLL in limp mode, running from SRAM or the boot flash; PLL_DR in one 32-bit write
        b = b.rmw(2, MISCCR, 0xFFFF, LIMP);
        if set {
            b = b.rmw(4, PLL_CR, !CR_MASK, cr_value(s));
        }
        if dr {
            b = b.rmw(4, PLL_DR, dr_keep, dr_set);
        }
        if let Some(d) = c.ddr2clk {
            b = b.rmw(2, MISCCR2, 0xFFFF & !DDR2CLK, if matches!(d, Ddr2Clk::Vco) { DDR2CLK } else { 0 });
        }
        b = b.extras(c.extra, Phase::InLimpWindow);
        b = b.rmw(2, MISCCR, 0xFFFF & !LIMP, 0).poll(PLL_SR, LOCK, LOCK, MARK_PLL_LOCK).push(op(KIND_DELAY, 0, 0, AFTER_LIMP, 0));
    }
    b = b.extras(c.extra, Phase::AfterPll).extras(c.extra, Phase::BeforeDdrController);
    // RM 21.4/21.6: clock, pads, read clock recovery, PADCR, the CRs with START clear, START, then wait for init
    b = b.write(1, PPMCR0, DDR_SLOT).write(1, MSCR_SDRAMC, c.ddr.full_strength as u32);
    b = b.write(4, DDR_RCR, 0).write(4, DDR_RCR, RCR_RST).write(4, DDR_PADCR, c.ddr.padcr);
    let mut i = 0;
    while i < c.ddr.cr.len() {
        let (n, v) = c.ddr.cr[i];
        b = b.write(4, DDR_CR + 4 * n as u32, v);
        i += 1;
    }
    b = b.rmw(4, DDR_CR + 9 * 4, !0, START).poll(DDR_CR + 27 * 4, INIT_DONE, INIT_DONE, MARK_DDR_INIT);
    if let Some(cs) = c.cs0 {
        // NOR stays at 0: CSAR0 = 0, then the device's timing, then CSMR0[V] ends CS0's global decode (RM 20.4.1.3)
        b = b.write(4, CSAR0, 0).write(4, CSCR0, cs.cscr).write(4, CSMR0, (cs.size - 1) & 0xFFFF_0000 | 1);
    }
    b.l
}

impl BootConfig {
    /// How many operations the boot sequence has.
    pub const fn ops_len(&self) -> usize {
        sequence(self).len
    }

    /// The boot sequence, exactly `N = ops_len()` operations (what `#[boot_config]` links into the boot part).
    pub const fn ops<const N: usize>(&self) -> [Op; N] {
        let l = sequence(self);
        if N != l.len {
            panic!("BootConfig::ops: N must be ops_len()");
        }
        let mut out = [op(0, 0, 0, 0, 0); N];
        let mut i = 0;
        while i < N {
            out[i] = l.ops[i];
            i += 1;
        }
        out
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::boot::*;
    use std::vec::Vec;

    const MANUAL: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };
    const CR: [(u8, u32); 3] = [(0, 0x0000_0101), (9, 0x0000_0100), (27, 0)];
    const DDR: Ddr = Ddr { cr: &CR, padcr: 1 << 9 | 1, full_strength: false };

    fn builder(p: Pll) -> BootConfigBuilder {
        BootConfig::builder(RefClock::Crystal(25_000_000), p, DDR)
            .app(AppSource::Mapped(0x8000_0000))
            .cs0(FlexBusCs0 { cscr: 0x0010_0140, size: 0x100_0000 })
    }

    fn list(c: &BootConfig) -> Vec<Op> {
        let l = sequence(c);
        l.ops[..l.len].to_vec()
    }

    fn at(ops: &[Op], kind: u8, addr: u32) -> Vec<usize> {
        ops.iter().enumerate().filter(|(_, o)| o.kind == kind && o.addr == addr).map(|(i, _)| i).collect()
    }

    fn first(ops: &[Op], kind: u8, addr: u32) -> usize {
        *at(ops, kind, addr).first().unwrap_or_else(|| panic!("no op {kind} at {addr:#x}"))
    }

    #[test]
    fn addresses_match_the_pac() {
        use crate::pac;
        let a = |p: *mut u8| p as u32;
        assert_eq!(MISCCR, a(pac::CCM.misccr().as_ptr() as _));
        assert_eq!(MISCCR2, a(pac::CCM.misccr2().as_ptr() as _));
        assert_eq!(PLL_CR, a(pac::PLL.cr().as_ptr() as _));
        assert_eq!(PLL_DR, a(pac::PLL.dr().as_ptr() as _));
        assert_eq!(PLL_SR, a(pac::PLL.sr().as_ptr() as _));
        assert_eq!(PPMCR0, a(pac::SCM.ppmcr0().as_ptr() as _));
        assert_eq!(MSCR_SDRAMC, a(pac::GPIO.mscr_sdramc().as_ptr() as _));
        assert_eq!(DDR_RCR, a(pac::DDRMC.rcr().as_ptr() as _));
        assert_eq!(DDR_PADCR, a(pac::DDRMC.padcr().as_ptr() as _));
        assert_eq!(DDR_CR, a(pac::DDRMC.cr00().as_ptr() as _));
        assert_eq!(DDR_CR + 27 * 4, a(pac::DDRMC.cr27().as_ptr() as _));
        assert_eq!(CSMR0, a(pac::FLEXBUS.csmr0().as_ptr() as _));
        assert_eq!(CSCR0, a(pac::FLEXBUS.cscr0().as_ptr() as _));
    }

    #[test]
    fn rambar_vbr_cacr_first() {
        let o = list(&builder(Pll::Set(MANUAL)).build());
        assert_eq!((o[0].kind, o[0].a), (KIND_RAMBAR, 0x8000_0221)); // SRAM at 0x80000000, BDE, Table 7-3's 0x21
        assert_eq!((o[1].kind, o[1].a), (KIND_VBR, 0x8000_0000));
        assert_eq!((o[2].kind, o[2].a), (KIND_CACR, 1 << 24 | 1 << 18 | 1 << 8)); // DCINVA, BCINVA, ICINVA only
    }

    #[test]
    fn set_pll_in_one_dr_write_inside_limp() {
        let o = list(&builder(Pll::Set(MANUAL)).ddr2clk(Ddr2Clk::Vco).build());
        let limp_on = first(&o, KIND_RMW, MISCCR);
        assert_eq!(o[limp_on].b, LIMP);
        let cr = first(&o, KIND_RMW, PLL_CR);
        assert_eq!((o[cr].a & 0x73F, o[cr].b), (0, 19)); // FBKDIV/REFDIV replaced, the rest kept
        let dr = at(&o, KIND_WRITE, PLL_DR).len() + at(&o, KIND_RMW, PLL_DR).len();
        assert_eq!(dr, 1, "PLL_DR is written once (RM 8.2.2)");
        let dr_at = at(&o, KIND_RMW, PLL_DR).first().or(at(&o, KIND_WRITE, PLL_DR).first()).copied().unwrap();
        assert_eq!(o[dr_at].width, 4);
        let ddr2 = first(&o, KIND_RMW, MISCCR2);
        assert_eq!(o[ddr2].b, DDR2CLK);
        let limp_off = at(&o, KIND_RMW, MISCCR)[1];
        assert_eq!((o[limp_off].a & LIMP, o[limp_off].b), (0, 0));
        let lock = first(&o, KIND_POLL, PLL_SR);
        assert!(limp_on < cr && cr < dr_at && dr_at < ddr2 && ddr2 < limp_off && limp_off < lock);
    }

    #[test]
    fn set_writes_outdiv3_and_outdiv4_explicitly() {
        // RM 8.2.2: 0 turns the eSDHC (OUTDIV3) and USB (OUTDIV4) clocks off; Set writes both, off unless given
        let o = list(&builder(Pll::Set(MANUAL)).build());
        let dr = at(&o, KIND_RMW, PLL_DR)[0];
        assert_eq!(o[dr].a & (0x1F << 10 | 0x1F << 16), 0);
        assert_eq!(o[dr].b & (0x1F << 10 | 0x1F << 16), 0);
        let o = list(&builder(Pll::Set(MANUAL)).outdiv3(3).build());
        assert_eq!(o[at(&o, KIND_RMW, PLL_DR)[0]].b & 0x1F << 10, 3 << 10);
    }

    #[test]
    fn keep_checks_and_writes_no_pll() {
        let o = list(&builder(Pll::Keep(MANUAL)).build());
        let checks = [first(&o, KIND_CHECK, PLL_CR), first(&o, KIND_CHECK, PLL_DR)];
        assert!(at(&o, KIND_RMW, PLL_CR).is_empty() && at(&o, KIND_RMW, PLL_DR).is_empty() && at(&o, KIND_WRITE, PLL_DR).is_empty());
        let firstwrite = o.iter().position(|x| x.kind == KIND_WRITE || x.kind == KIND_RMW).unwrap();
        assert!(checks.iter().all(|&c| c < firstwrite));
        assert_eq!(o[checks[0]].b, 19); // FBKDIV 19, REFDIV 0
        assert_eq!(o[checks[0]].mark, MARK_PLL_MISMATCH as u8);
    }

    #[test]
    fn keep_with_outdiv3_writes_dr_once_inside_limp() {
        let o = list(&builder(Pll::Keep(MANUAL)).outdiv3(6).build());
        let dr = at(&o, KIND_RMW, PLL_DR);
        assert_eq!(dr.len(), 1);
        let dr = dr[0];
        assert_eq!((o[dr].a & 0x1F << 10, o[dr].b), (0, 6 << 10)); // OUTDIV3 cleared, then 6
        let m = at(&o, KIND_RMW, MISCCR);
        assert!(m[0] < dr && dr < m[1] && m[1] < first(&o, KIND_POLL, PLL_SR));
    }

    #[test]
    fn ddr_order() {
        static BOTH: [ExtraWrite; 2] =
            [ExtraWrite { phase: Phase::BeforeDdrController, width: Width::W32, addr: 0xFC00_4000, value: 0x1234 },
             ExtraWrite { phase: Phase::AfterPll, width: Width::W8, addr: 0xEC09_4000, value: 1 }];
        let o = list(&builder(Pll::Set(MANUAL)).extra(&BOTH).build());
        let lock = first(&o, KIND_POLL, PLL_SR);
        let after_pll = first(&o, KIND_WRITE, 0xEC09_4000);
        let before = first(&o, KIND_WRITE, 0xFC00_4000);
        let ppmcr = first(&o, KIND_WRITE, PPMCR0);
        assert_eq!((o[ppmcr].a, o[ppmcr].width), (46, 1));
        let mscr = first(&o, KIND_WRITE, MSCR_SDRAMC);
        assert!(o[mscr].a < 2, "never MSCR_SDRAMC's reserved reset value");
        let rcr = at(&o, KIND_WRITE, DDR_RCR);
        assert_eq!(rcr.len(), 2);
        assert_eq!((o[rcr[0]].a, o[rcr[1]].a), (0, 0x4000_0000));
        let padcr = first(&o, KIND_WRITE, DDR_PADCR);
        let crs: Vec<usize> = CR.iter().map(|&(n, _)| first(&o, KIND_WRITE, DDR_CR + 4 * n as u32)).collect();
        let start = first(&o, KIND_RMW, DDR_CR + 9 * 4);
        assert_eq!(o[start].b, 1 << 24);
        let done = first(&o, KIND_POLL, DDR_CR + 27 * 4);
        assert_eq!((o[done].a, o[done].b, o[done].mark), (1 << 3, 1 << 3, MARK_DDR_INIT as u8));
        assert!(lock < after_pll && after_pll < before && before < ppmcr && ppmcr < mscr && mscr < rcr[0]);
        assert!(rcr[1] < padcr && padcr < crs[0] && crs.windows(2).all(|w| w[0] < w[1]));
        assert!(*crs.last().unwrap() < start && start < done);
    }

    #[test]
    fn in_limp_extras_run_inside_the_window() {
        static W: [ExtraWrite; 1] = [ExtraWrite { phase: Phase::InLimpWindow, width: Width::W16, addr: MISCCR2, value: 0x409F }];
        let o = list(&builder(Pll::Keep(MANUAL)).extra(&W).build());
        let m = at(&o, KIND_RMW, MISCCR);
        let w = first(&o, KIND_WRITE, MISCCR2);
        assert_eq!((o[w].width, o[w].a), (2, 0x409F));
        assert!(m.len() == 2 && m[0] < w && w < m[1] && m[1] < first(&o, KIND_POLL, PLL_SR));
    }

    #[test]
    fn every_poll_capped() {
        let o = list(&builder(Pll::Set(MANUAL)).build());
        for x in o.iter().filter(|x| x.kind == KIND_POLL) {
            assert!(x.cap_log2 > 0 && x.mark != 0, "{x:?}");
        }
    }

    #[test]
    fn leaving_limp_waits_before_the_ddr_controller() {
        let o = list(&builder(Pll::Set(MANUAL)).build());
        let lock = first(&o, KIND_POLL, PLL_SR);
        let delay = o.iter().position(|x| x.kind == KIND_DELAY).unwrap();
        assert!(lock < delay && delay < first(&o, KIND_WRITE, PPMCR0) && o[delay].a >= 256);
    }

    #[cfg(feature = "boot-nor")]
    #[test]
    fn cs0_after_ddr_for_nor() {
        let o = list(&builder(Pll::Set(MANUAL)).build());
        let cscr = first(&o, KIND_WRITE, CSCR0);
        let csmr = first(&o, KIND_WRITE, CSMR0);
        assert_eq!(o[cscr].a, 0x0010_0140);
        assert_eq!(o[csmr].a, 0x00FF_0000 | 1);
        assert!(first(&o, KIND_POLL, DDR_CR + 27 * 4) < cscr && cscr < csmr);
    }

    #[test]
    fn fits() {
        static MANY: [(u8, u32); 41] = [(0, 0); 41];
        static XBS: [ExtraWrite; 12] = [ExtraWrite { phase: Phase::BeforeDdrController, width: Width::W32, addr: 0xFC00_4000, value: 0 }; 12];
        let c = BootConfig::builder(RefClock::Crystal(25_000_000), Pll::Keep(MANUAL), Ddr { cr: &MANY, ..DDR })
            .app(AppSource::Mapped(0))
            .cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 })
            .outdiv3(6)
            .ddr2clk(Ddr2Clk::Vco)
            .extra(&XBS)
            .build();
        assert!(c.ops_len() < OPS_MAX);
        const C: BootConfig = BootConfig::builder(RefClock::Crystal(25_000_000), Pll::Keep(MANUAL), DDR)
            .app(AppSource::Mapped(0))
            .cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 })
            .build();
        let trimmed = C.ops::<{ C.ops_len() }>();
        assert_eq!((trimmed.len(), trimmed[0].kind), (C.ops_len(), KIND_RAMBAR));
    }
}
