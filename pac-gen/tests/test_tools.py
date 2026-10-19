"""pac-gen/tools.sh with fake cargo and tools: a pinned install is left alone, whatever the output sizes."""
import os
import shutil
import subprocess
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1] / "tools.sh"
REV = "bcf538a2e7b8584ae874ee9ab72efb1576fc6152"


def setup(tmp_path, listing: str, help_lines: int = 3):
    (tmp_path / "pac-gen").mkdir()
    shutil.copy(TOOLS, tmp_path / "pac-gen/tools.sh")
    bin_ = tmp_path / "build/tools/bin"
    bin_.mkdir(parents=True)
    (bin_ / "svdtools").write_text("#!/bin/sh\necho 'svdtools 0.5.0'\n")
    # a long --help: `| head -3` would make it die of SIGPIPE under pipefail
    (bin_ / "chiptool").write_text(f"#!/bin/sh\nseq 1 {help_lines}\n")
    fake = tmp_path / "fake"
    fake.mkdir()
    (fake / "listing").write_text(listing)
    # `cargo install --list` prints the listing (long: an early `grep -q` exit would SIGPIPE it); `cargo install`
    # records that it ran
    (fake / "cargo").write_text(f"""#!/bin/sh
if [ "$2" = "--list" ]; then cat {fake}/listing; else echo "$@" >> {fake}/installs; fi
""")
    for f in (bin_ / "svdtools", bin_ / "chiptool", fake / "cargo"):
        f.chmod(0o755)
    env = {**os.environ, "PATH": f"{fake}:{os.environ['PATH']}"}
    r = subprocess.run(["bash", str(tmp_path / "pac-gen/tools.sh")], capture_output=True, text=True, env=env)
    installs = (fake / "installs").read_text() if (fake / "installs").exists() else ""
    return r, installs


PAD = "".join(f"pad{i} v0.1.0:\n    pad{i}\n" for i in range(20000))


def test_pinned_tools_are_left_alone_with_long_outputs(tmp_path):
    listing = (f"chiptool v0.1.0 (https://github.com/embassy-rs/chiptool?rev={REV}#{REV[:8]}):\n    chiptool\n"
               "svdtools v0.5.0:\n    svdtools\n" + PAD)
    r, installs = setup(tmp_path, listing, help_lines=200000)
    assert r.returncode == 0, r.stderr
    assert installs == ""
    assert r.stdout.splitlines()[:4] == ["svdtools 0.5.0", "1", "2", "3"]


def test_the_pinned_revision_must_be_on_the_chiptool_line(tmp_path):
    # another crate installed from the pinned revision doesn't make chiptool pinned
    listing = (f"chiptool v0.1.0 (https://github.com/embassy-rs/chiptool?rev=0123456789#01234567):\n    chiptool\n"
               f"other v0.1.0 (https://example.org/other?rev={REV}#{REV[:8]}):\n    other\n")
    r, installs = setup(tmp_path, listing)
    assert r.returncode == 0, r.stderr
    assert "chiptool" in installs and REV in installs
