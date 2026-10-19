#!/usr/bin/env bash
# Install the PAC generators into <repo>/build/tools (nothing system-wide). Idempotent:
# a tool is reinstalled unless it is already at its pinned version.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
T=$ROOT/build/tools
SVDTOOLS_VERSION=0.5.0
CHIPTOOL_REV=bcf538a2e7b8584ae874ee9ab72efb1576fc6152
mkdir -p "$T"
# svdtools --version prints "svdtools X.Y.Z"
[ "$("$T/bin/svdtools" --version 2>/dev/null)" = "svdtools $SVDTOOLS_VERSION" ] ||
  cargo install --locked --root "$T" svdtools --version "=$SVDTOOLS_VERSION"
# cargo install --list shows a git install as "chiptool v… (<url>?rev=<rev>#<short rev>):"; the pinned rev must be on
# the chiptool line. The listing is captured first: a `grep -q` reading the pipe could exit early and kill cargo
# with SIGPIPE, which pipefail would turn into a needless reinstall.
# --force: cargo's metadata can outlive a deleted binary, and it would refuse to reinstall.
installed=$(cargo install --list --root "$T")
if ! { [ -x "$T/bin/chiptool" ] && grep -qE "^chiptool v[^ ]+ \(.*#${CHIPTOOL_REV:0:8}\):" <<< "$installed"; }; then
  cargo install --locked --force --root "$T" --git https://github.com/embassy-rs/chiptool --rev "$CHIPTOOL_REV" chiptool
fi
"$T/bin/svdtools" --version
"$T/bin/chiptool" --help | sed -n 1,3p          # sed reads to the end: no SIGPIPE for a long help
