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
        Err(Fault::Unmapped {
            addr: 0x3000_0000,
            size: Size::Long
        })
    );
}

#[test]
fn long_read_straddling_ram_end_is_unmapped() {
    let mut m = Memory::default();
    m.map_ram(0x4010_0000, 0x1000);
    assert_eq!(
        m.read(0x4010_0FFE, Size::Long),
        Err(Fault::Unmapped {
            addr: 0x4010_0FFE,
            size: Size::Long
        })
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
    assert_eq!(
        common::assemble("nop\nrts", 0x4010_0000),
        [0x4E, 0x71, 0x4E, 0x75]
    );
}

#[test]
fn access_ending_at_the_top_of_the_address_space() {
    // The last byte of the last RAM page is 0xFFFFFFFF: the end of the access must not overflow.
    let mut m = Memory::default();
    m.map_ram(0xFFFF_F000, 0x1000);
    m.write(0xFFFF_FFFC, Size::Long, 0xCAFE_BABE).unwrap();
    assert_eq!(m.read(0xFFFF_FFFC, Size::Long).unwrap(), 0xCAFE_BABE);
    m.write(0xFFFF_FFFF, Size::Byte, 0x5A).unwrap();
    assert_eq!(m.read(0xFFFF_FFFF, Size::Byte).unwrap(), 0x5A);
}

/// Where the mirrors in these tests are.
const MIRROR: u32 = 0x2010_0000;

#[test]
fn mirror_reads_and_writes_the_same_bytes() {
    let mut m = Memory::default();
    m.map_ram(0x4010_0000, 0x1000);
    m.map_mirror(MIRROR, 0x1000, 0x4010_0000);
    // A part of the RAM, and a mirror of the mirror.
    m.map_mirror(0x3000_0000, 0x100, 0x4010_0800);
    m.map_mirror(0x5000_0000, 0x10, MIRROR + 0x810);

    m.write(MIRROR + 4, Size::Long, 0x1122_3344).unwrap();
    assert_eq!(m.read(0x4010_0004, Size::Long), Ok(0x1122_3344));
    m.write(0x4010_0006, Size::Word, 0xABCD).unwrap();
    assert_eq!(m.read(MIRROR + 4, Size::Long), Ok(0x1122_ABCD));
    m.write(0x3000_0010, Size::Byte, 0x5A).unwrap();
    assert_eq!(m.read(0x4010_0810, Size::Byte), Ok(0x5A));
    assert_eq!(m.read(MIRROR + 0x810, Size::Byte), Ok(0x5A));
    assert_eq!(m.read(0x5000_0000, Size::Byte), Ok(0x5A));

    m.load(MIRROR + 0x20, &[1, 2, 3]).unwrap();
    assert_eq!(m.peek(0x4010_0020, 3), Some(vec![1, 2, 3]));
    m.load(0x4010_0FFE, &[7, 8]).unwrap();
    assert_eq!(m.peek(MIRROR + 0xFFE, 2), Some(vec![7, 8]));

    // Each mirror ends where its size says.
    assert_eq!(
        m.read(0x3000_00FE, Size::Long),
        Err(Fault::Unmapped {
            addr: 0x3000_00FE,
            size: Size::Long
        })
    );
    assert_eq!(m.peek(0x5000_000F, 2), None);
    assert!(m.load(MIRROR + 0xFFF, &[1, 2]).is_err());
}

#[test]
fn code_runs_from_the_mirror() {
    use mcf5441x_emu::{Limit, Machine, Stop};
    let until = |pc| Limit {
        until_pc: Some(pc),
        ..Limit::default()
    };
    // Running from the mirror, the program patches its subroutine through the RAM's own addresses.
    let src = format!(
        "bsr.w sub\n\
         lea sub,%a3\n\
         move.l %a3,%d3\n\
         add.l #{delta},%d3\n\
         move.l %d3,%a3\n\
         move.l #0x4e714e71,(%a3)\n\
         bsr.w sub\n\
         bra.w end\n\
         sub: moveq #1,%d0\n\
         add.l %d0,%d1\n\
         rts\n\
         end: nop\n\
         .long end\n",
        delta = 0x4010_0000u32 - MIRROR
    );
    let code = common::assemble(&src, MIRROR);
    let (code, tail) = code.split_at(code.len() - 4);
    let end = u32::from_be_bytes(tail.try_into().unwrap());
    let mut m = Machine::new();
    m.mem.map_ram(0x4010_0000, 0x10_0000);
    m.mem.map_mirror(MIRROR, 0x10_0000, 0x4010_0000);
    m.mem.load(0x4010_0000, code).unwrap();
    m.cpu.pc = MIRROR;
    m.cpu.a[7] = MIRROR + 0x8_0000;
    m.profile(true);
    let r = m.run(until(end));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    // The second call ran `nop; nop; rts`.
    assert_eq!(m.cpu.d[1], 1);
    // The stack the mirror holds is the RAM's.
    assert_eq!(
        m.mem.peek(0x4010_0000 + 0x8_0000 - 4, 4),
        m.mem.peek(MIRROR + 0x8_0000 - 4, 4)
    );
    let profile = m.take_profile();
    assert!(profile.iter().any(|&(pc, _)| pc == MIRROR), "{profile:x?}");
    assert!(
        profile.iter().all(|&(pc, _)| pc >> 24 == MIRROR >> 24),
        "{profile:x?}"
    );

    // Code cached at the RAM's own addresses, rewritten through the mirror by the host: the next run sees it.
    let loop_src = "loop: addq.l #1,%d1\nsubq.l #1,%d2\nbne.s loop\nend: nop";
    let code = common::assemble(loop_src, 0x4010_0000);
    m.mem.load(0x4010_0000, &code).unwrap();
    let end = 0x4010_0000 + code.len() as u32 - 2;
    m.cpu.pc = 0x4010_0000;
    m.cpu.d[1] = 0;
    m.cpu.d[2] = 3;
    let r = m.run(until(end));
    assert_eq!((r.kind, m.cpu.d[1]), (Stop::UntilPc, 3), "{r:?}");
    m.mem.write(MIRROR, Size::Word, 0x4e71).unwrap();
    m.cpu.pc = 0x4010_0000;
    m.cpu.d[2] = 3;
    let r = m.run(until(end));
    // `nop` replaced `addq`: one pass, then the count runs out from 3 to 0 by `subq`, so D1 stays 3.
    assert_eq!((r.kind, m.cpu.d[1]), (Stop::UntilPc, 3), "{r:?}");
    assert_eq!(m.cpu.d[2], 0);

    // Code cached before the mirror is mapped, rewritten through the new mirror.
    let mut m = Machine::new();
    m.mem.map_ram(0x4010_0000, 0x10_0000);
    m.mem.load(0x4010_0000, &code).unwrap();
    m.cpu.pc = 0x4010_0000;
    m.cpu.d[2] = 3;
    let r = m.run(until(end));
    assert_eq!((r.kind, m.cpu.d[1]), (Stop::UntilPc, 3), "{r:?}");
    m.mem.map_mirror(MIRROR, 0x1000, 0x4010_0000);
    m.mem.write(MIRROR, Size::Word, 0x4e71).unwrap();
    m.cpu.pc = 0x4010_0000;
    m.cpu.d[2] = 3;
    let r = m.run(until(end));
    assert_eq!((r.kind, m.cpu.d[1]), (Stop::UntilPc, 3), "{r:?}");
}

/// A bus master with no registers of its own, for reaching the bus from the host.
struct Master;

impl mcf5441x_emu::Device for Master {
    fn read(&mut self, _: u32, _: Size, _: &mut mcf5441x_emu::DeviceCx) -> u32 {
        0
    }
    fn write(&mut self, _: u32, _: Size, _: u32, _: &mut mcf5441x_emu::DeviceCx) {}
}

#[test]
fn dma_through_the_mirror() {
    use mcf5441x_emu::{Limit, Machine, Stop};
    let mut m = Machine::new();
    m.mem.map_ram(0x4010_0000, 0x10_0000);
    m.mem.map_mirror(MIRROR, 0x10_0000, 0x4010_0000);
    let master = m.add_bus_master(0xFC01_0000, 0x100, Box::new(Master));
    m.with_device::<Master, _>(master, |_, cx| {
        let mut bus = cx.bus().expect("a bus master has a bus");
        assert!(bus.is_ram(MIRROR + 0x100, Size::Long));
        bus.write(MIRROR + 0x100, Size::Long, 0xCAFE_F00D).unwrap();
        assert_eq!(bus.read(0x4010_0100, Size::Long), Ok(0xCAFE_F00D));
        bus.write_bytes(0x4010_0200, &[1, 2, 3, 4]).unwrap();
        let mut out = [0; 4];
        bus.read_bytes(MIRROR + 0x200, &mut out).unwrap();
        assert_eq!(out, [1, 2, 3, 4]);
    });
    assert_eq!(
        m.mem.peek(0x4010_0100, 4),
        Some(vec![0xCA, 0xFE, 0xF0, 0x0D])
    );
    // The host's accesses, as the core's.
    assert_eq!(m.access(MIRROR + 0x100, Size::Long, None), Ok(0xCAFE_F00D));
    m.access(0x4010_0300, Size::Word, Some(0x1234)).unwrap();
    assert_eq!(m.access(MIRROR + 0x300, Size::Word, None), Ok(0x1234));

    // A loop cached at the mirror, rewritten by the master through the RAM's own addresses.
    let code = common::assemble(
        "loop: addq.l #1,%d1\nsubq.l #1,%d2\nbne.s loop\nend: nop",
        MIRROR,
    );
    m.mem.load(MIRROR, &code).unwrap();
    let limit = Limit {
        until_pc: Some(MIRROR + code.len() as u32 - 2),
        ..Limit::default()
    };
    m.cpu.pc = MIRROR;
    m.cpu.d[2] = 3;
    let r = m.run(limit);
    assert_eq!((r.kind, m.cpu.d[1]), (Stop::UntilPc, 3), "{r:?}");
    m.with_device::<Master, _>(master, |_, cx| {
        cx.bus()
            .unwrap()
            .write_bytes(0x4010_0000, &[0x4e, 0x71])
            .unwrap();
    });
    m.cpu.pc = MIRROR;
    m.cpu.d[2] = 3;
    let r = m.run(limit);
    assert_eq!((r.kind, m.cpu.d[1]), (Stop::UntilPc, 3), "{r:?}");
}

#[test]
fn mirror_overlap_panics() {
    use std::panic::{AssertUnwindSafe, catch_unwind};
    let setup = || {
        let mut m = mcf5441x_emu::Machine::new();
        m.mem.map_ram(0x4010_0000, 0x1000);
        m.mem.map_ram(0x4020_0000, 0x1000);
        m.mem.map_mirror(MIRROR, 0x1000, 0x4010_0000);
        m.add_device(0xFC00_0000, 0x100, Box::new(Master));
        m
    };
    let panics = |what: &str, f: &dyn Fn(&mut mcf5441x_emu::Machine)| {
        let mut m = setup();
        assert!(
            catch_unwind(AssertUnwindSafe(|| f(&mut m))).is_err(),
            "{what} did not panic"
        );
    };
    panics("over RAM", &|m| {
        m.mem.map_mirror(0x4020_0800, 0x1000, 0x4010_0000)
    });
    panics("over a mirror", &|m| {
        m.mem.map_mirror(MIRROR + 0xFFF, 0x10, 0x4010_0000)
    });
    panics("over a device", &|m| {
        m.mem.map_mirror(0xFBFF_FF00, 0x200, 0x4010_0000)
    });
    panics("of no RAM", &|m| {
        m.mem.map_mirror(0x3000_0000, 0x10, 0x4030_0000)
    });
    panics("of too little RAM", &|m| {
        m.mem.map_mirror(0x3000_0000, 0x1001, 0x4010_0000)
    });
    panics("of two regions", &|m| {
        m.mem.map_ram(0x4010_1000, 0x1000);
        m.mem.map_mirror(0x3000_0000, 0x2000, 0x4010_0000)
    });
    panics("empty", &|m| m.mem.map_mirror(0x3000_0000, 0, 0x4010_0000));
    panics("RAM over a mirror", &|m| {
        m.mem.map_ram(MIRROR + 0x800, 0x1000)
    });
    panics("a device over a mirror", &|m| {
        m.add_device(MIRROR + 0x800, 0x100, Box::new(Master));
    });
    // Next to everything is fine.
    let mut m = setup();
    m.mem.map_mirror(MIRROR + 0x1000, 0x1000, 0x4020_0000);
    m.mem.map_mirror(0xFBFF_F000, 0x1000, 0x4010_0000);
}

/// `loop: addq.l #1,%d1; subq.l #1,%d2; bne.s loop; nop`.
const LOOP: [u8; 8] = [0x52, 0x81, 0x53, 0x82, 0x66, 0xFA, 0x4E, 0x71];

/// Run [`LOOP`] at `at` with D2 = 3; returns D1, 3 when the `addq` ran and 0 when a `nop` replaced it.
fn run_loop(m: &mut mcf5441x_emu::Machine, at: u32) -> u32 {
    use mcf5441x_emu::{Limit, Stop};
    m.cpu.pc = at;
    m.cpu.d[1] = 0;
    m.cpu.d[2] = 3;
    let r = m.run(Limit {
        until_pc: Some(at + 6),
        ..Limit::default()
    });
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    m.cpu.d[1]
}

#[test]
fn a_write_through_a_mirror_drops_code_that_shares_a_page_only_by_way_of_it() {
    // One mirror page holding the halves of two RAM pages, each with a cached loop. A data write in the first RAM
    // page, then a write over the second loop through the mirror: the second loop's blocks must go.
    let mut m = mcf5441x_emu::Machine::new();
    m.mem.map_ram(0x4010_0000, 0x2000);
    m.mem.map_mirror(MIRROR, 0x1000, 0x4010_0800);
    m.mem.load(0x4010_0800, &LOOP).unwrap();
    m.mem.load(0x4010_1000, &LOOP).unwrap();
    assert_eq!(run_loop(&mut m, 0x4010_0800), 3);
    assert_eq!(run_loop(&mut m, 0x4010_1000), 3);
    m.mem.write(0x4010_0C00, Size::Long, 0).unwrap();
    m.mem.write(MIRROR + 0x800, Size::Word, 0x4e71).unwrap();
    assert_eq!(run_loop(&mut m, 0x4010_1000), 0);

    // Two small mirrors of two RAM pages, side by side in one page.
    let mut m = mcf5441x_emu::Machine::new();
    m.mem.map_ram(0x4010_0000, 0x2000);
    m.mem.map_mirror(0x3000_0000, 0x100, 0x4010_0000);
    m.mem.map_mirror(0x3000_0100, 0x100, 0x4010_1000);
    m.mem.load(0x4010_0000, &LOOP).unwrap();
    m.mem.load(0x4010_1000, &LOOP).unwrap();
    assert_eq!(run_loop(&mut m, 0x4010_0000), 3);
    assert_eq!(run_loop(&mut m, 0x4010_1000), 3);
    m.mem.write(0x4010_0800, Size::Long, 0).unwrap();
    m.mem.write(0x3000_0100, Size::Word, 0x4e71).unwrap();
    assert_eq!(run_loop(&mut m, 0x4010_1000), 0);
}
