//! Libcalls GCC emits for ColdFire ISA_C: `__atomic_*` (single core: each runs with interrupts masked) and byte swaps.
use core::arch::asm;

#[inline(always)]
fn masked<R>(f: impl FnOnce() -> R) -> R {
    let sr: u32;
    unsafe { asm!("move.w %sr,{0}", "move.w #0x2700,%sr", out(reg_data) sr, options(nostack)) };
    let r = f();
    unsafe { asm!("move.w {0},%sr", in(reg_data) sr, options(nostack)) };
    r
}

macro_rules! atomics {
    ($t:ty, $load:ident, $store:ident, $xchg:ident, $cas:ident,
     $add:ident, $sub:ident, $and:ident, $or:ident, $xor:ident, $nand:ident) => {
        #[unsafe(no_mangle)] pub unsafe extern "C" fn $load(p: *const $t, _mo: i32) -> $t { masked(|| unsafe { p.read_volatile() }) }
        #[unsafe(no_mangle)] pub unsafe extern "C" fn $store(p: *mut $t, v: $t, _mo: i32) { masked(|| unsafe { p.write_volatile(v) }) }
        #[unsafe(no_mangle)] pub unsafe extern "C" fn $xchg(p: *mut $t, v: $t, _mo: i32) -> $t {
            masked(|| unsafe { let o = p.read_volatile(); p.write_volatile(v); o })
        }
        // GCC's libcall drops the `weak` flag of the builtin: (ptr, expected, desired, success, failure).
        #[unsafe(no_mangle)] pub unsafe extern "C" fn $cas(p: *mut $t, e: *mut $t, d: $t, _s: i32, _f: i32) -> bool {
            masked(|| unsafe {
                let c = p.read_volatile();
                if c == e.read() { p.write_volatile(d); true } else { e.write(c); false }
            })
        }
        atomics!(@rmw $t, $add, |o: $t, v: $t| o.wrapping_add(v));
        atomics!(@rmw $t, $sub, |o: $t, v: $t| o.wrapping_sub(v));
        atomics!(@rmw $t, $and, |o: $t, v: $t| o & v);
        atomics!(@rmw $t, $or, |o: $t, v: $t| o | v);
        atomics!(@rmw $t, $xor, |o: $t, v: $t| o ^ v);
        atomics!(@rmw $t, $nand, |o: $t, v: $t| !(o & v));
    };
    (@rmw $t:ty, $name:ident, $f:expr) => {
        #[unsafe(no_mangle)] pub unsafe extern "C" fn $name(p: *mut $t, v: $t, _mo: i32) -> $t {
            masked(|| unsafe { let o = p.read_volatile(); p.write_volatile(($f)(o, v)); o })
        }
    };
}

atomics!(u8, __atomic_load_1, __atomic_store_1, __atomic_exchange_1, __atomic_compare_exchange_1,
         __atomic_fetch_add_1, __atomic_fetch_sub_1, __atomic_fetch_and_1, __atomic_fetch_or_1, __atomic_fetch_xor_1, __atomic_fetch_nand_1);
atomics!(u16, __atomic_load_2, __atomic_store_2, __atomic_exchange_2, __atomic_compare_exchange_2,
         __atomic_fetch_add_2, __atomic_fetch_sub_2, __atomic_fetch_and_2, __atomic_fetch_or_2, __atomic_fetch_xor_2, __atomic_fetch_nand_2);
atomics!(u32, __atomic_load_4, __atomic_store_4, __atomic_exchange_4, __atomic_compare_exchange_4,
         __atomic_fetch_add_4, __atomic_fetch_sub_4, __atomic_fetch_and_4, __atomic_fetch_or_4, __atomic_fetch_xor_4, __atomic_fetch_nand_4);

// Byte swaps. compiler_builtins' Rust `__bswapsi2` is compiled by GCC, which lowers `swap_bytes()` back
// into a call to the `__bswapsi2` libcall, i.e. itself (infinite recursion, hit by `{:02}` formatting).
// These strong definitions use ColdFire's ISA_C `byterev` and take precedence over the weak ones.
core::arch::global_asm!(
    ".global __bswapsi2",
    "__bswapsi2:",
    "move.l 4(%sp),%d0",
    "byterev.l %d0",
    "rts",
    ".global __bswapdi2",
    "__bswapdi2:",          // u64 in 4(sp) high, 8(sp) low; returned in d0 high, d1 low
    "move.l 8(%sp),%d0",
    "byterev.l %d0",
    "move.l 4(%sp),%d1",
    "byterev.l %d1",
    "rts",
);
