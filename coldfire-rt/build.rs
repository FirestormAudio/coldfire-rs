//! Put this crate's link.x on the linker search path, so a binary links with `-Tlink.x` against the version of the
//! script that matches this version of the runtime (its handler table and vector-table rules).
use std::{env, fs, path::PathBuf};

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR"));
    fs::copy("link.x", out.join("link.x")).expect("copy link.x");
    println!("cargo::rustc-link-search={}", out.display());
    println!("cargo::rerun-if-changed=link.x");
}
