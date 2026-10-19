//! The boot part's executor: runs the linked operation list from reset, with volatile accesses and `movec` only.
//! Everything here is inlined into, or placed in, the boot part (`.boot.text`): DDR holds nothing yet.
use super::ops::*;

/// Stop with `marker` in D0 (a debugger reads it at the halt), and stay stopped.
#[unsafe(link_section = ".boot.text.coldfire_rt_halt")]
#[inline(never)]
pub(crate) fn halt(marker: u32) -> ! {
    loop {
        unsafe { core::arch::asm!("halt", in("d0") marker, options(nomem, nostack)) };
    }
}

#[inline(always)]
unsafe fn read(width: u8, addr: u32) -> u32 {
    unsafe {
        match width {
            1 => (addr as *const u8).read_volatile() as u32,
            2 => (addr as *const u16).read_volatile() as u32,
            _ => (addr as *const u32).read_volatile(),
        }
    }
}

#[inline(always)]
unsafe fn write(width: u8, addr: u32, v: u32) {
    unsafe {
        match width {
            1 => (addr as *mut u8).write_volatile(v as u8),
            2 => (addr as *mut u16).write_volatile(v as u16),
            _ => (addr as *mut u32).write_volatile(v),
        }
    }
}

/// Run `n` operations from `ops`.
#[unsafe(link_section = ".boot.text.coldfire_rt_run")]
#[inline(never)]
pub(crate) unsafe fn run(ops: *const Op, n: usize) {
    let mut i = 0;
    while i < n {
        let o = unsafe { ops.add(i).read() };
        unsafe {
            match o.kind {
                KIND_WRITE => write(o.width, o.addr, o.a),
                KIND_RMW => write(o.width, o.addr, (read(o.width, o.addr) & o.a) | o.b),
                KIND_POLL => {
                    let mut left = 1u32 << o.cap_log2;
                    while read(o.width, o.addr) & o.a != o.b {
                        left -= 1;
                        if left == 0 {
                            halt(0xB007_0000 | o.mark as u32);
                        }
                    }
                }
                KIND_DELAY => {
                    let mut k = o.a;
                    while k != 0 {
                        core::arch::asm!("nop", options(nomem, nostack));
                        k -= 1;
                    }
                }
                KIND_CHECK => {
                    if read(o.width, o.addr) & o.a != o.b {
                        halt(0xB007_0000 | o.mark as u32);
                    }
                }
                KIND_RAMBAR => core::arch::asm!("movec {0},%rambar1", in(reg_data) o.a, options(nostack)),
                KIND_VBR => core::arch::asm!("movec {0},%vbr", in(reg_data) o.a, options(nostack)),
                KIND_CACR => core::arch::asm!("movec {0},%cacr", "nop", in(reg_data) o.a, options(nostack)),
                _ => {}
            }
        }
        i += 1;
    }
}
