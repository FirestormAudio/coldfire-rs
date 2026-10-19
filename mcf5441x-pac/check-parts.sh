#!/usr/bin/env bash
# Part gating, checked by compiling: under mcf54410, UART2 (removed from the 196 MAPBGA package, RM ch. 2) and its
# Interrupt variant must not exist and UART7 must; under mcf54418 UART2 exists. Each probe is a tiny crate outside
# the workspace.
set -euo pipefail
P=$(cd "$(dirname "$0")" && pwd)
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
export CARGO_TARGET_DIR=${CARGO_TARGET_DIR:-$P/../target/part-probe}

probe() {   # <name> <part feature> <expression>: 0 if `mcf5441x_pac::<expression>` compiles
  mkdir -p "$T/$1/src"
  printf '[package]\nname = "%s"\nversion = "0.0.0"\nedition = "2021"\n\n[dependencies]\nmcf5441x-pac = { path = "%s", features = ["%s"] }\n\n[workspace]\n' \
    "$1" "$P" "$2" > "$T/$1/Cargo.toml"
  printf '#![no_std]\npub fn probe() -> usize { mcf5441x_pac::%s as usize }\n' "$3" > "$T/$1/src/lib.rs"
  cargo check -q --offline --manifest-path "$T/$1/Cargo.toml" 2> "$T/$1.err"
}

fail=0
probe present_54410 mcf54410 'UART7.as_ptr()' || { echo "mcf54410: UART7 should exist"; cat "$T/present_54410.err"; fail=1; }
probe present_54418 mcf54418 'UART2.as_ptr()' || { echo "mcf54418: UART2 should exist"; cat "$T/present_54418.err"; fail=1; }
probe irq_54418 mcf54418 'Interrupt::UART2' || { echo "mcf54418: Interrupt::UART2 should exist"; cat "$T/irq_54418.err"; fail=1; }
if probe absent_54410 mcf54410 'UART2.as_ptr()'; then
  echo "mcf54410: UART2 should not exist"; fail=1
elif ! grep -q 'cannot find value `UART2`' "$T/absent_54410.err"; then
  echo "mcf54410: UART2 probe failed for another reason:"; cat "$T/absent_54410.err"; fail=1
fi
if probe irq_54410 mcf54410 'Interrupt::UART2'; then
  echo "mcf54410: Interrupt::UART2 should not exist"; fail=1
elif ! grep -q 'no variant.*`UART2`' "$T/irq_54410.err"; then
  echo "mcf54410: Interrupt::UART2 probe failed for another reason:"; cat "$T/irq_54410.err"; fail=1
fi
[ "$fail" = 0 ] && echo "part gates: ok"
exit "$fail"
