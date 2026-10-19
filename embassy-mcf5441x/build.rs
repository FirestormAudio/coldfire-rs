//! With a cold boot feature, write boot-region.x (where the boot part runs and what each boot source gives it) for
//! coldfire-rt's cold link script, and put it on the linker search path.
use std::{env, fs, path::PathBuf};

/// Serial boot (RM 11): the SBF copies the image to SRAM offset 0; the boot part runs there (the stack takes the
/// top 4 KB of the 64 KB), and MEDIUM (memory.x) is the image as it continues past what the SBF copied.
const SERIAL: &str = "MEMORY { BOOT (rwx) : ORIGIN = 0x80000000, LENGTH = 64K }
REGION_ALIAS(\"HEAD\", BOOT);
REGION_ALIAS(\"HEAD_LOAD\", MEDIUM);
REGION_ALIAS(\"BOOT_LOAD\", MEDIUM);
__boot_stack_top = 0x80010000;
__boot_budget = 60K;
__boot_entry = _start;
";
/// NOR on FlexBus CS0 (RM 20.4.1.3): the boot part runs in place from address 0, MEDIUM (memory.x) being the flash.
const NOR: &str = "REGION_ALIAS(\"BOOT\", MEDIUM);
REGION_ALIAS(\"HEAD\", MEDIUM);
REGION_ALIAS(\"HEAD_LOAD\", MEDIUM);
REGION_ALIAS(\"BOOT_LOAD\", MEDIUM);
__boot_stack_top = 0x80010000;
__boot_budget = LENGTH(MEDIUM);
__boot_entry = _start;
";
/// NAND (RM 22.4.4): the NFC maps 3984 bytes of the four boot pages at 0. The boot part keeps to the first 3968:
/// whole 8-byte chunks of each page's 996 payload bytes, where the interleave is unambiguous. The relocator there
/// copies the boot part to SRAM and runs it; the app part is the page stream after the boot pages, offset 0 at
/// MEDIUM's origin (a numbering of the payload bytes, not an address the CPU reads).
const NAND: &str = "MEMORY {
  NFCBUF (rx) : ORIGIN = 0, LENGTH = 3968
  BOOT (rwx) : ORIGIN = 0x80000000, LENGTH = 64K
  MEDIUM (r) : ORIGIN = 0x10000000, LENGTH = 256M
}
REGION_ALIAS(\"HEAD\", NFCBUF);
REGION_ALIAS(\"HEAD_LOAD\", NFCBUF);
REGION_ALIAS(\"BOOT_LOAD\", NFCBUF);
__boot_stack_top = 0x80010000;
__boot_budget = 3968;
__boot_entry = __coldfire_rt_reloc;
";

fn main() {
    let f = |n: &str| env::var_os(format!("CARGO_FEATURE_{n}")).is_some();
    let region = if f("BOOT_SERIAL") {
        SERIAL
    } else if f("BOOT_NOR") {
        NOR
    } else if f("BOOT_NAND") {
        NAND
    } else {
        return;
    };
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR"));
    fs::write(out.join("boot-region.x"), region).expect("write boot-region.x");
    println!("cargo::rustc-link-search={}", out.display());
}
