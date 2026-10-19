#!/usr/bin/env bash
# raw.svd (from the RM, only when RM=<markdown dir> is set) → patched mcf5441x.svd → verify against the oracles.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
PY=${CF_PYTHON:-python3}
S=$ROOT/mcf5441x-pac/svd
if [ -n "${RM:-}" ]; then
  PYTHONPATH=$ROOT/pac-gen "$PY" -m rm2svd "$RM" -o "$S/raw.svd" --coverage "$S/coverage.txt"
fi
"$ROOT/build/tools/bin/svdtools" patch --post-validate "$S/patches/mcf5441x.yaml" "$S/mcf5441x.svd"
PYTHONPATH=$ROOT/pac-gen "$PY" "$ROOT/pac-gen/verify.py" "$S/mcf5441x.svd" "$ROOT"/pac-gen/oracles/*.json
