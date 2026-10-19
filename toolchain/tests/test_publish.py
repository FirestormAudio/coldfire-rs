"""What publishing the coldfire-rs crates needs: licence and metadata, versions on path dependencies, package contents,
board-neutral wording, a host doc build, and a clean `cargo publish --dry-run` (nothing is uploaded)."""
import json
import re
import subprocess
from functools import cache
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CRATES = ["coldfire", "coldfire-rt", "coldfire-rt-macros", "mcf5441x-pac", "embassy-mcf5441x"]


def run(cmd, **kw):
    return subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, **kw)


@cache
def _metadata():
    r = run(["cargo", "metadata", "--format-version", "1", "--no-deps"])
    assert r.returncode == 0, r.stderr
    return {p["name"]: p for p in json.loads(r.stdout)["packages"]}


def metadata(crate):
    return _metadata()[crate]


def test_every_crate_has_publish_metadata():
    for c in CRATES:
        m = metadata(c)
        assert m["license"] == "MIT OR Apache-2.0" and m["description"] and m["readme"], c
        assert m["keywords"] and m["categories"], c
        assert (Path(m["manifest_path"]).parent / m["readme"]).exists(), c
    assert (ROOT / "LICENSE-MIT").exists() and (ROOT / "LICENSE-APACHE").exists()


def test_path_dependencies_carry_versions():
    for c in CRATES:
        for d in metadata(c)["dependencies"]:
            if d.get("path"):
                assert d["req"] == "^0.1.0", (c, d["name"], d["req"])


@cache
def package_list():
    """Every crate's packaged files: {crate: [paths]} (`cargo package -p CRATE --list`)."""
    out = {}
    for c in CRATES:
        r = run(["cargo", "package", "-p", c, "--list", "--allow-dirty"])
        assert r.returncode == 0, (c, r.stderr)
        out[c] = r.stdout.split()
    return out


def test_packages_hold_no_generator_or_manual_files():
    # pac-gen and the manual-derived inputs (the raw SVD, the patches, generator scripts and data) stay in the repo;
    # a package carries its sources, the final SVD, its tests, the licences and its README
    banned = re.compile(r"^(pac-gen|svd/raw|svd/patches|svd/coverage)|oracles|\.(py|sh)$|^transform\.yaml$|^pins?(mux)?\.json$")
    for c, paths in package_list().items():
        assert [p for p in paths if banned.search(p)] == [], c
        assert {"LICENSE-MIT", "LICENSE-APACHE", "README.md"} <= set(paths), c


# (an OS code address is 0x400xxxxx; the DDR base 0x40000000 is a chip address and allowed)
# board names, the board's firmware ("the OS", "the vendor's ..." values, its code addresses), our RE notes and their
# "§N" pointers, and "the unit" (one board); "RM §N" (the manual's sections) is fine
BOARD = re.compile(r"(?i:digitakt|elektron|stock os|vendor os|the os\b|the vendor'?s?\b|on the unit|re/notes|_os_|_panel|_vendor_)"
                   r"|(?<!RM )(?<!RM ch\. )§\d|\b0x400(?!00000\b)[0-9A-Fa-f]{5}\b")
TEXT = (".rs", ".md", ".json", ".yaml", ".toml", ".x", ".svd")
# pac-gen's own rule: no firmware, OS code addresses or RE notes (it may name digitakt as its oracles' source)
FIRMWARE = re.compile(r"(?i:elektron|stock os|vendor os|the os\b|on the unit|re/notes)|\b0x400(?!00000\b)[0-9A-Fa-f]{5}\b")


def test_published_files_are_board_neutral():
    # a value proven on a board says so, without naming the board, its firmware or the notes made reverse-engineering it
    hits = []
    for c, paths in package_list().items():
        base = Path(metadata(c)["manifest_path"]).parent
        for p in paths:
            # the packaged Cargo.toml is cargo's normalised copy; Cargo.toml.orig is the manifest as written
            f = base / ("Cargo.toml" if p == "Cargo.toml.orig" else p)
            if (p.endswith(TEXT) or p == "Cargo.toml.orig") and p != "Cargo.toml" and f.exists():
                hits += _hits(f, f"{c}/{p}", BOARD)
    for f in ("README.md", "TODO.md"):
        hits += _hits(ROOT / f, f, BOARD)
    # public in the repository though not published as crates: the examples in full, and pac-gen except that it may name
    # digitakt as the source of its oracle data
    for f in sorted((ROOT / "examples").rglob("*")):
        if f.is_file() and "target" not in f.parts and (f.suffix in TEXT or f.name in ("memory.x", "Cargo.toml")):
            hits += _hits(f, str(f.relative_to(ROOT)), BOARD)
    for f in sorted((ROOT / "pac-gen").rglob("*")):
        if f.is_file() and "__pycache__" not in f.parts and f.suffix in (".py", ".md", ".sh"):
            hits += _hits(f, str(f.relative_to(ROOT)), FIRMWARE)
    assert hits == [], "\n".join(hits)


def _hits(f, name, pattern):
    return [f"{name}:{n}" for n, line in enumerate(f.read_text().splitlines(), 1) if pattern.search(line)]


def docs_rs_features(crate):
    # [package.metadata.docs.rs] is a dotted key: cargo reports it as {"docs": {"rs": {...}}}
    return (((metadata(crate).get("metadata") or {}).get("docs") or {}).get("rs") or {}).get("features", [])


def test_docs_build_on_the_host_with_the_docs_rs_features():
    # docs.rs builds on a host target; the PAC and HAL need a part feature to build at all
    assert docs_rs_features("mcf5441x-pac") == ["mcf54418"]
    assert "mcf54418" in docs_rs_features("embassy-mcf5441x")
    import os
    env = {**os.environ, "RUSTDOCFLAGS": "--cfg docsrs"}   # as docs.rs builds
    for c in CRATES:
        feats = docs_rs_features(c)
        r = run(["cargo", "doc", "--no-deps", "-p", c, *(["--features", ",".join(feats)] if feats else [])], env=env)
        assert r.returncode == 0 and "warning" not in r.stderr, (c, r.stderr)
    # a crate that builds but renders nothing is no documentation: each shows its main items
    doc = ROOT / "target/doc"
    for page in ("coldfire_rt/fn.set_vector.html", "coldfire_rt/struct.Handler.html", "coldfire_rt/macro.exception_entry.html",
                 "coldfire_rt/attr.entry.html", "coldfire/emac/index.html", "coldfire/mmio/index.html",
                 "mcf5441x_pac/constant.UART0.html", "embassy_mcf5441x/usb/struct.Driver.html",
                 "embassy_mcf5441x/fn.init.html", "embassy_mcf5441x/boot/struct.BootConfig.html"):
        assert (doc / page).exists(), page


def _toolchain():
    """The rust-coldfire backend toolchain/setup.sh builds (RCF, default build/rust-coldfire), if it is there."""
    import os
    rcf = Path(os.environ.get("RCF", ROOT / "build/rust-coldfire"))
    return (rcf / "rustc_codegen_gcc/target/debug/librustc_codegen_gcc.so").exists()


import pytest  # noqa: E402


@pytest.mark.skipif(not _toolchain(), reason="no ColdFire toolchain (toolchain/setup.sh; RCF=)")
def test_examples_build_for_coldfire_warning_free():
    # the chip-generic starting points: a standalone package (its binaries can't build on a host, so it stays out of
    # the workspace), built for ColdFire and gated by cfgate
    r = run([str(ROOT / "toolchain/cargo-cf"), str(ROOT / "examples"), "--features", "mcf54415"])
    assert r.returncode == 0, r.stderr
    # the known noise: the toolchain's 'retain' note, the RWX-segment linker note, and cargo's per-binary count of it
    known = re.compile(r"'retain' attribute ignored|has a LOAD segment with RWX permissions$|generated 1 warning$")
    assert [l for l in r.stderr.splitlines() if l.startswith("warning") and not known.search(l)] == []
    for name in ("uart-echo", "ticker", "usb-cdc"):
        assert (ROOT / "examples/target/m68k-coldfire-none-elf/release" / name).exists(), name


def test_publish_dry_run_of_the_workspace_passes():
    # uploads nothing; the PAC and HAL refuse to build without a part, so verification needs one (as the real publish
    # will). --workspace handles crates whose siblings aren't on crates.io yet, through a temporary local registry
    # whose crates cargo extracts under ~/.cargo/registry/src/-<hash>/ and never refreshes while the version stays
    # 0.1.0, and whose builds it treats as immutable: drop those extractions and builds of our crates first, or the
    # check verifies against an old copy
    import os
    import shutil
    src = Path(os.environ.get("CARGO_HOME", Path.home() / ".cargo")) / "registry/src"
    for d in src.glob("-*/*-0.1.0"):
        if d.name.removesuffix("-0.1.0") in CRATES:
            shutil.rmtree(d)
    target = ROOT / "target/publish-check"
    names = {c.replace("-", "_") for c in CRATES} | set(CRATES)
    for d in target.glob("debug/*/*"):
        if d.name.split("-")[0].removeprefix("lib") in names or "-".join(d.name.split("-")[:-1]).removeprefix("lib") in names:
            shutil.rmtree(d) if d.is_dir() else d.unlink()
    r = run(["cargo", "publish", "--dry-run", "--workspace", "--allow-dirty", "--target-dir", str(target),
             "--features", "mcf5441x-pac/mcf54418,embassy-mcf5441x/mcf54418,embassy-mcf5441x/bootloader"])
    assert r.returncode == 0, r.stderr[-3000:]
    assert r.stderr.count("aborting upload due to dry run") == len(CRATES), r.stderr[-3000:]


def test_coldfire_rt_hands_its_link_script_to_the_linker():
    # a user of the published coldfire-rt must link with that crate's link.x (its handler table and vector rules), not
    # with whatever link.x a toolchain checkout happens to hold
    assert "build.rs" in package_list()["coldfire-rt"]
    cargo_cf = (ROOT / "toolchain/cargo-cf").read_text()
    assert "coldfire-rt/link.x" not in cargo_cf and "LINKX=link.x" in cargo_cf   # -T$LINKX: found on the search path


def test_keywords_and_categories_fit_each_crate():
    for c in CRATES:
        m = metadata(c)
        assert not set(m["keywords"]) & set(m["categories"]), c          # a keyword repeating a category adds nothing
        assert len(m["keywords"]) <= 5, c
    # a proc-macro crate runs on the host: it isn't a no-std crate
    assert "no-std" not in metadata("coldfire-rt-macros")["categories"]
    assert "development-tools::procedural-macro-helpers" in metadata("coldfire-rt-macros")["categories"]


PROCESS = re.compile(r"(?i:superpowers|claude|\bllm\b|subagent|\bledger\b|\bruling\b|final review|review #\d|\btask \d)"
                     r"|\b3[a-e](\.2)?'s\b|\(3[a-e](\.2)?\)|embassy-mcf5441x 3[a-e]\b|docs/superpowers|§7")


def test_no_development_process_references_in_the_repository():
    # what goes public is the code and its own docs: no pointers to plans, review findings, sub-project names or
    # agent tooling
    files = run(["git", "ls-files"]).stdout.split()
    hits = []
    for name in files:
        f = ROOT / name
        if f.suffix in (".rs", ".md", ".py", ".sh", ".toml", ".yaml", ".x", ".json") and f.is_file() and not f.is_symlink():
            if name == "toolchain/tests/test_publish.py":
                continue   # holds the patterns themselves
            hits += _hits(f, name, PROCESS)
    assert hits == [], "\n".join(hits)


def test_editor_settings_with_local_paths_are_not_tracked():
    files = run(["git", "ls-files"]).stdout.split()
    assert not [f for f in files if f.startswith((".vscode/", ".zed/"))]


REPOSITORY = "https://github.com/FirestormAudio/coldfire-rs"


def test_every_crate_names_the_public_repository():
    for c in CRATES:
        assert metadata(c)["repository"] == REPOSITORY, c
    # the crate READMEs that point at the repository link it
    for c in CRATES:
        text = (Path(metadata(c)["manifest_path"]).parent / "README.md").read_text()
        if "repository" in text:
            assert REPOSITORY in text, c


# register values and the operating point read off one particular board: test values come from the reference manual
DIGITAKT_VALUES = re.compile(r"0x1015\b|0x071861|528_000_000|264_000_000|132_000_000|0xBE01_?0020|0xFE01_?0020|0xA115_?0061")
MISCCR2_VALUE = re.compile(r"MISCCR2.*0xE000\b|0xE000\b.*MISCCR2")


def test_no_digitakt_values():
    files = run(["git", "ls-files"]).stdout.split()
    hits = []
    for name in files:
        f = ROOT / name
        if f.suffix in (".rs", ".md", ".py", ".sh", ".toml", ".x") and f.is_file() and not f.is_symlink():
            if name == "toolchain/tests/test_publish.py":
                continue   # holds the patterns themselves
            hits += _hits(f, name, DIGITAKT_VALUES) + _hits(f, name, MISCCR2_VALUE)
    assert hits == [], "\n".join(hits)
