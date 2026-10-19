//! Put this crate's link.x on the linker search path, so a binary links with `-Tlink.x` against the version of the
//! script that matches this version of the runtime (its handler table and vector-table rules).
use std::{env, fs, path::PathBuf};

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR"));
    // the cold layout (feature `cold`) replaces the one for an image a bootloader loads
    let script = if env::var_os("CARGO_FEATURE_COLD").is_some() { "link-cold.x" } else { "link.x" };
    fs::copy(script, out.join("link.x")).expect("copy the link script");
    println!("cargo::rustc-link-search={}", out.display());
    println!("cargo::rerun-if-changed=link.x");
    println!("cargo::rerun-if-changed=link-cold.x");
}
