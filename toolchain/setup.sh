#!/usr/bin/env bash
# Build the Rust-for-ColdFire toolchain under $RCF (default <repo>/build/rust-coldfire): libgccjit for
# m68k-elf (default CPU 54418) and rustc_codegen_gcc. Nothing is installed system-wide. Idempotent: re-running
# it on a built tree makes no network access and rebuilds nothing (pins are checked against what's present).
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
RCF=${RCF:-$ROOT/build/rust-coldfire}
mkdir -p "$RCF"
# Everything below writes $RCF into links and config.toml: use its physical path, so a run through a
# symlink (a worktree's build/ points at the main checkout's) doesn't leave paths that die with the link.
RCF=$(cd "$RCF" && pwd -P)
CG_PIN=8c858fb5fbb077eaa776302b9601cf084f12aa46
NIGHTLY=nightly-2026-09-29

have_commit() { git -C "$1" rev-parse -q --verify "$2^{commit}" > /dev/null 2>&1; }

have_toolchain() { # the nightly with the components the backend and -Zbuild-std need (offline check)
  local installed
  installed=$(rustup component list --toolchain "$NIGHTLY" --installed 2>/dev/null) || return 1
  for c in rust-src rustc-dev llvm-tools; do grep -q "^$c" <<< "$installed" || return 1; done
}

checkout_pin() { # repo dir, url, commit: fetch only if the commit isn't already present
  if [ ! -d "$1/.git" ]; then
    git init -q "$1"
    git -C "$1" remote add origin "$2"
  fi
  have_commit "$1" "$3" || git -C "$1" fetch -q --depth 1 origin "$3"
  [ "$(git -C "$1" rev-parse HEAD 2>/dev/null)" = "$3" ] || git -C "$1" checkout -q "$3"
}

# 1. rustc_codegen_gcc at its pin (it names the GCC commit it needs)
checkout_pin "$RCF/rustc_codegen_gcc" https://github.com/rust-lang/rustc_codegen_gcc.git "$CG_PIN"
GCC_PIN=$(cat "$RCF/rustc_codegen_gcc/libgccjit.version")

# 2. libgccjit for m68k-elf, default CPU 54418
if [ ! -f "$RCF/gcc-install/lib/libgccjit.so" ] || [ "$(cat "$RCF/gcc-install/.pin" 2>/dev/null)" != "$GCC_PIN" ]; then
  checkout_pin "$RCF/gcc" https://github.com/rust-lang/gcc.git "$GCC_PIN"
  rm -rf "$RCF/gcc-build" "$RCF/gcc-install"   # never mix an install from another pin
  mkdir -p "$RCF/gcc-build" "$RCF/gcc-install"
  (cd "$RCF/gcc-build" && ../gcc/configure --target=m68k-elf --with-cpu=54418 --enable-host-shared \
      --enable-languages=jit --enable-checking=release --disable-bootstrap --disable-multilib --disable-nls \
      --without-headers --with-newlib --prefix="$RCF/gcc-install" > configure.log 2>&1 \
   && make -j"$(nproc)" all-gcc > make.log 2>&1 && make install-gcc > install.log 2>&1)
  echo "$GCC_PIN" > "$RCF/gcc-install/.pin"
fi
mkdir -p "$RCF/gcc-install/m68k-elf/bin"
for t in as ld ar nm objcopy ranlib; do ln -sf "$(command -v m68k-elf-$t)" "$RCF/gcc-install/m68k-elf/bin/$t"; done

# 3. backend + cross sysroot preparation, skipped when already built for these pins
have_toolchain || rustup toolchain install "$NIGHTLY" --profile minimal -c rust-src -c rustc-dev -c llvm-tools-preview > /dev/null
STAMP="$CG_PIN $GCC_PIN $NIGHTLY"
BACKEND=$RCF/rustc_codegen_gcc/target/debug/librustc_codegen_gcc.so
if [ ! -f "$BACKEND" ] || [ "$(cat "$RCF/.backend-stamp" 2>/dev/null)" != "$STAMP" ]; then
  (cd "$RCF/rustc_codegen_gcc" \
   && printf 'gcc-path = "%s/gcc-install/lib"\n' "$RCF" > config.toml \
   && ./y.sh prepare --cross > "$RCF/prepare.log" 2>&1 \
   && ./y.sh build > "$RCF/build-backend.log" 2>&1)
  echo "$STAMP" > "$RCF/.backend-stamp"
fi

# 4. sysroot symlinks the backend and -Zbuild-std need
SR=$RCF/rustc_codegen_gcc/build/build_sysroot/sysroot
TC=$(rustup run "$NIGHTLY" rustc --print sysroot)                    # honours RUSTUP_HOME
HOST=$(rustup run "$NIGHTLY" rustc -vV | sed -n 's/^host: //p')
BK=$SR/lib/rustlib/$HOST/codegen-backends/lib
mkdir -p "$BK/m68k-coldfire-none-elf" "$BK/m68k" "$SR/lib/rustlib/src" "$SR/lib/rustlib/$HOST"
ln -sf "$RCF/gcc-install/lib/libgccjit.so" "$BK/m68k-coldfire-none-elf/libgccjit.so"
ln -sf "$RCF/gcc-install/lib/libgccjit.so" "$BK/m68k/libgccjit.so"
ln -sfnT "$TC/lib/rustlib/src/rust" "$SR/lib/rustlib/src/rust"     # -T: never nest inside a directory
ln -sfnT "$TC/lib/rustlib/$HOST/lib" "$SR/lib/rustlib/$HOST/lib"
echo "rust-coldfire toolchain ready in $RCF (backend $CG_PIN, gcc $GCC_PIN)"
