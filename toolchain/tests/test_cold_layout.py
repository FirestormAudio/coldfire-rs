"""The cold layout: one ELF per boot source from examples/cold, read back with binutils. Needs the ColdFire toolchain
(toolchain/setup.sh); skipped without it, like the examples build."""
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import coldelf  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bootcheck import boot_references  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
COLD = ROOT / "examples/cold"
RCF = Path(os.environ.get("RCF", ROOT / "build/rust-coldfire"))
SOURCES = ("serial", "nor", "nand")

pytestmark = pytest.mark.skipif(not (RCF / "rustc_codegen_gcc/target/debug/librustc_codegen_gcc.so").exists(),
                                reason="ColdFire toolchain not built (toolchain/setup.sh)")


def build(source, memory_dir=None, check=True):
    env = {**os.environ, "RCF": str(RCF), "CF_MEMORY_DIR": str(memory_dir or COLD / "memory" / source)}
    r = subprocess.run([str(ROOT / "toolchain/cargo-cf"), str(COLD), "--no-default-features", "--features", f"boot-{source},mcf54415",
                        "--target-dir", str(COLD / "target" / (source if memory_dir is None else f"{source}-alt"))],
                       env=env, capture_output=True, text=True)
    if check:
        assert r.returncode == 0, r.stderr[-4000:]
    return r


@pytest.fixture(scope="module")
def elfs():
    out = {}
    for s in SOURCES:
        build(s)
        out[s] = COLD / "target" / s / "m68k-coldfire-none-elf/release/cold-hello"
    return out


def flat(elf, *sections):
    out = elf.with_suffix(".head.bin")
    subprocess.run(["m68k-elf-objcopy", "-O", "binary", *[a for s in sections for a in ("-j", s)], str(elf), str(out)], check=True)
    return out.read_bytes()


def test_sp_pc_at_offset_0(elfs):
    for s, elf in elfs.items():
        sym = coldelf.symbols(elf)
        image = flat(elf, ".boot.vectors")
        sp, pc = int.from_bytes(image[0:4], "big"), int.from_bytes(image[4:8], "big")
        assert (sp, pc) == (sym["__boot_stack_top"], sym["__boot_entry"]), s
        assert sym["__boot_entry"] == (sym["__coldfire_rt_reloc"] if s == "nand" else sym["_start"]), s


def test_boot_part_within_budget(elfs):
    for s, elf in elfs.items():
        sym = coldelf.symbols(elf)
        assert 8 < sym["__boot_size"] <= sym["__boot_budget"], s


def test_app_lma_follows_boot_vma_in_ddr(elfs):
    for s, elf in elfs.items():
        sym = coldelf.symbols(elf)
        assert sym["__app_vma"] == 0x4000_0400, s                       # memory.x's DDR + 0x400, as today
        if s == "nand":
            assert sym["__app_offset"] == 0                            # the first app page's first payload byte
        else:
            assert sym["__app_lma"] == sym["__image_start"] + sym["__boot_size"], s
            assert sym["__app_offset"] == sym["__boot_size"], s
        loads = [seg for seg in coldelf.segments(elf) if seg[0] >= 0x4000_0000 and seg[0] < 0x8000_0000 and seg[2]]
        assert loads and min(p for _, p, _, _ in loads) == sym["__app_lma"], s
        assert sum(f for _, _, f, _ in loads) <= sym["__app_len"], s


def test_boot_part_references_nothing_outside(elfs):
    for s, elf in elfs.items():
        assert boot_references(elf) == [], s


def test_bootloader_layout_unchanged():
    env = {**os.environ, "RCF": str(RCF)}
    r = subprocess.run([str(ROOT / "toolchain/cargo-cf"), str(ROOT / "examples"), "--features", "mcf54415"], env=env,
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr[-4000:]
    golden = [l.split() for l in (Path(__file__).parent / "golden/bootloader-sections.txt").read_text().split("\n")[:-1]]
    got = [l.split() for l in coldelf.golden_lines(ROOT / "examples/target/m68k-coldfire-none-elf/release/ticker")]

    def stable(rows):
        # .rodata holds panic locations' file paths, so its size (and .bss's address after it) depend on the checkout
        # path; the vector table, the code and where the constants start don't
        out = []
        for name, addr, size in rows:
            out.append((name, addr, size if name in (".vectors", ".text") else None) if name != ".bss" else (name, None, size))
        return out

    assert stable(got) == stable(golden)


def test_overflow_is_a_link_error(tmp_path):
    shutil.copy(COLD / "memory/serial/memory.x", tmp_path / "memory.x")
    m = tmp_path / "memory.x"
    m.write_text(re.sub(r"(MEDIUM\s*\([a-z]*\)\s*:\s*ORIGIN\s*=\s*0x[0-9A-Fa-f_]+\s*,\s*LENGTH\s*=\s*)\S+", r"\g<1>16", m.read_text()))
    assert "LENGTH = 16" in m.read_text()
    r = build("serial", memory_dir=tmp_path, check=False)
    assert r.returncode != 0 and "MEDIUM" in r.stderr, r.stderr[-3000:]


def test_nand_relocator_runs_in_place(elfs):
    # the reset PC runs out of the NFC buffer at its link address, before the boot part is in SRAM
    elf = elfs["nand"]
    sym = coldelf.symbols(elf)
    head = {n: (a, sz) for n, a, sz in coldelf.sections(elf)}[".boot.head"]
    assert sym["__image_start"] == 0 and head[0] == 8 and 8 < head[1]
    assert head[0] <= sym["__coldfire_rt_reloc"] < head[0] + head[1]
    assert sym["__boot_lma"] == head[0] + head[1] and sym["__boot_vma"] == 0x8000_0000
    assert sym["__boot_size"] <= 3968


def test_boot_part_register_reads_keep_their_width(elfs):
    # a 32-bit register read folded into a byte-wide bit test on memory reads the wrong width (the NFC's registers
    # answer 32-bit accesses only)
    for s, elf in elfs.items():
        out = subprocess.run(["m68k-elf-objdump", "-d", "-m", "m68k:cfv4e", "-j", ".boot.head", "-j", ".boot", str(elf)],
                             capture_output=True, text=True, check=True).stdout
        assert [l for l in out.splitlines() if re.search(r"\b(btst|bset|bclr|bchg)\s+#\d+,%a\d@", l)] == [], s


def test_app_part_is_one_block(elfs):
    # the boot part copies __app_len bytes from __app_lma to __app_vma: every loaded app section must sit at the same
    # load-to-run offset, alignment gaps included
    for s, elf in elfs.items():
        sym = coldelf.symbols(elf)
        offsets = {p - v for v, p, f, _ in coldelf.segments(elf) if 0x4000_0000 <= v < 0x8000_0000 and f}
        assert offsets == {sym["__app_lma"] - sym["__app_vma"]}, (s, [hex(o) for o in offsets])


def test_cargo_cf_refuses_a_boot_part_that_reaches_ddr():
    # unoptimised, the boot part's helpers are outlined into .text (DDR): the image must not link quietly and die at
    # reset; cargo-cf's post-link check names what the boot part reaches
    env = {**os.environ, "RCF": str(RCF), "CF_MEMORY_DIR": str(COLD / "memory/serial"), "CF_RUSTFLAGS": "-C opt-level=0"}
    r = subprocess.run([str(ROOT / "toolchain/cargo-cf"), str(COLD), "--no-default-features", "--features", "boot-serial,mcf54415",
                        "--target-dir", str(COLD / "target/serial-o0")], env=env, capture_output=True, text=True)
    out = r.stdout + r.stderr
    assert r.returncode != 0 and "reaches outside the boot part" in out, out[-3000:]


def test_an_app_part_past_its_limit_is_a_link_error(tmp_path):
    # NAND's MEDIUM is a numbering, not the device: a board says how many app bytes its NAND holds (__app_part_max)
    m = tmp_path / "memory.x"
    m.write_text((COLD / "memory/nand/memory.x").read_text() + "__app_part_max = 16;\n")
    env = {**os.environ, "RCF": str(RCF), "CF_MEMORY_DIR": str(tmp_path)}
    r = subprocess.run([str(ROOT / "toolchain/cargo-cf"), str(COLD), "--no-default-features", "--features", "boot-nand,mcf54415",
                        "--target-dir", str(COLD / "target/nand-alt")], env=env, capture_output=True, text=True)
    assert r.returncode != 0 and "__app_part_max" in r.stderr, r.stderr[-3000:]
