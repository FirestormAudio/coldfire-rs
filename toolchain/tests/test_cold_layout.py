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

ROOT = Path(__file__).resolve().parents[2]
COLD = ROOT / "examples/cold"
RCF = Path(os.environ.get("RCF", ROOT / "build/rust-coldfire"))
SOURCES = ("serial", "nor", "nand")
# what the boot part may name outside itself: where it hands over, and the app part it copies
EXITS = {"coldfire_rt_start", "__stack_top", "__boot_stack_top", "__app_vma", "__app_lma", "__app_len", "__app_offset", "__sbss", "__ebss"}

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


def boot_references(elf):
    """(instruction, address) of every address the boot part's code names outside [boot start, boot end)."""
    sym = coldelf.symbols(elf)
    by_addr = {}
    for n, a in sym.items():
        by_addr.setdefault(a, set()).add(n)
    lo, hi = min(sym["__image_start"], sym["__boot_vma"]), sym["__boot_vma"] + sym["__boot_vma_len"]
    head_lo, head_hi = sym["__image_start"], sym["__image_start"] + sym["__head_size"]
    out = subprocess.run(["m68k-elf-objdump", "-d", "-m", "m68k:cfv4e", "-j", ".boot.head", "-j", ".boot", str(elf)],
                         capture_output=True, text=True, check=True).stdout
    bad = []
    for line in out.splitlines():
        m = re.match(r"\s*[0-9a-f]+:\s+(?:[0-9a-f]{4} )+\s*(\S+)\s*(.*)", line)
        if not m:
            continue
        operands = re.sub(r"<[^>]*>", "", m.group(2))                         # objdump's symbol annotations
        imms = [int(v, 0) & 0xFFFF_FFFF for v in re.findall(r"#(-?(?:0x[0-9a-f]+|\d+))", operands)]
        addrs = [int(v, 16) for v in re.findall(r"\b([0-9a-f]{4,8})\b", re.sub(r"#-?(?:0x[0-9a-f]+|\d+)", "", operands))]
        for a, is_imm in [(a, False) for a in addrs] + [(a, True) for a in imms]:
            inside = lo <= a < hi or head_lo <= a < head_hi
            exit_ = bool(by_addr.get(a, set()) & EXITS)
            sram, peripheral, ddr = 0x8000_0000 <= a <= 0x8001_0000, a >= 0xE000_0000, 0x4000_0000 <= a < 0x8000_0000
            # an immediate is a value unless it points into DDR; an absolute address must be the boot part's own
            if not (inside or exit_ or sram or peripheral) and (ddr if is_imm else a >= 0x1000):
                bad.append((line.strip(), hex(a), sorted(by_addr.get(a, []))))
    return bad


def test_boot_part_references_nothing_outside(elfs):
    for s, elf in elfs.items():
        assert boot_references(elf) == [], s


def test_bootloader_layout_unchanged():
    env = {**os.environ, "RCF": str(RCF)}
    r = subprocess.run([str(ROOT / "toolchain/cargo-cf"), str(ROOT / "examples"), "--features", "mcf54415"], env=env,
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr[-4000:]
    golden = (Path(__file__).parent / "golden/bootloader-sections.txt").read_text().split("\n")[:-1]
    assert coldelf.golden_lines(ROOT / "examples/target/m68k-coldfire-none-elf/release/ticker") == golden


def test_overflow_is_a_link_error(tmp_path):
    shutil.copy(COLD / "memory/serial/memory.x", tmp_path / "memory.x")
    m = tmp_path / "memory.x"
    m.write_text(re.sub(r"(MEDIUM\s*\([a-z]*\)\s*:\s*ORIGIN\s*=\s*0x[0-9A-Fa-f_]+\s*,\s*LENGTH\s*=\s*)\S+", r"\g<1>16", m.read_text()))
    assert "LENGTH = 16" in m.read_text()
    r = build("serial", memory_dir=tmp_path, check=False)
    assert r.returncode != 0 and "MEDIUM" in r.stderr, r.stderr[-3000:]
