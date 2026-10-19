#!/usr/bin/env bash
# mcf5441x.svd → src/lib.rs: chiptool, rustfmt, then postgen.py (no Cortex-M glue, part-choice compile_error!,
# per-part cfg gates on instances and their Interrupt variants from pac-gen/parts_map.json, checked against
# raw.svd's Table 1-1 availability).
set -euo pipefail
P=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$P/.." && pwd)
PY=${CF_PYTHON:-python3}
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
"$ROOT/build/tools/bin/chiptool" generate --svd "$P/svd/mcf5441x.svd" --transform "$P/transform.yaml" \
  --no-defmt --output "$T"
rustfmt --edition 2021 "$T/lib.rs"
mkdir -p "$P/src"
"$PY" "$P/postgen.py" "$T/lib.rs" "$P/svd/raw.svd" "$ROOT/pac-gen/parts_map.json" "$P/src/lib.rs" \
  "$P/svd/mcf5441x.svd"
rustfmt --edition 2021 "$P/src/lib.rs"
