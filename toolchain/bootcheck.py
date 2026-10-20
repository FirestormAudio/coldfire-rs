"""Cold-boot check, run by cargo-cf on every ELF with a boot part (`__coldfire_rt_boot`): the boot part runs before DDR
is up, so its code may name nothing outside itself except where it hands over (coldfire-rt's startup, the app part it
copies, the stacks) and the on-chip SRAM and peripherals. An outlined helper, a library call or a jump table in
`.text`/`.rodata` would be in DDR: the image would link and then die at reset."""
import re
import subprocess
import sys

# what the boot part may name outside itself
EXITS = {"coldfire_rt_start", "__stack_top", "__boot_stack_top", "__app_vma", "__app_lma", "__app_len", "__app_offset",
         "__sbss", "__ebss"}


def symbols(elf) -> dict[str, int]:
    out = subprocess.run(["m68k-elf-readelf", "-sW", str(elf)], capture_output=True, text=True, check=True).stdout
    return {f[7]: int(f[1], 16) for f in (l.split() for l in out.splitlines())
            if len(f) >= 8 and f[0].endswith(":") and f[0][:-1].isdigit()}


def boot_references(elf) -> list[tuple[str, str, list[str]]]:
    """(instruction, address, symbols there) of every address the boot part's code names outside itself."""
    sym = symbols(elf)
    by_addr: dict[int, set[str]] = {}
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


def main(argv: list[str]) -> int:
    failed = 0
    for elf in argv:
        if "__coldfire_rt_boot" not in symbols(elf):
            continue
        bad = boot_references(elf)
        for line, a, names in bad:
            print(f"{elf}: the boot part reaches outside the boot part (DDR isn't up yet): {line} -> {a} {' '.join(names)}",
                  file=sys.stderr)
        failed |= bool(bad)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
