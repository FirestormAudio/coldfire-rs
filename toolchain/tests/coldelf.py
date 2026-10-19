"""Reading linked ColdFire ELFs with GNU binutils (m68k-elf-readelf/objdump): sections, symbols, segments."""
import re
import subprocess

SKIP = re.compile(r"^\.(debug|comment|symtab|strtab|shstrtab)")


def sections(elf):
    """(name, address, size) of each allocated or loaded section, in file order."""
    out = subprocess.run(["m68k-elf-readelf", "-SW", str(elf)], capture_output=True, text=True, check=True).stdout
    rows = []
    for line in out.splitlines():
        m = re.match(r"\s*\[\s*\d+\]\s+(\S+)\s+\S+\s+([0-9a-f]+)\s+[0-9a-f]+\s+([0-9a-f]+)", line)
        if m and m.group(1) != "NULL" and not SKIP.match(m.group(1)):
            rows.append((m.group(1), int(m.group(2), 16), int(m.group(3), 16)))
    return rows


def symbols(elf):
    out = subprocess.run(["m68k-elf-readelf", "-sW", str(elf)], capture_output=True, text=True, check=True).stdout
    syms = {}
    for line in out.splitlines():
        f = line.split()
        if len(f) >= 8 and f[0].endswith(":") and f[0][:-1].isdigit():
            syms[f[7]] = int(f[1], 16)
    return syms


def segments(elf):
    """(vaddr, paddr, filesz, memsz) of each PT_LOAD."""
    out = subprocess.run(["m68k-elf-readelf", "-lW", str(elf)], capture_output=True, text=True, check=True).stdout
    return [tuple(int(x, 16) for x in m.groups())
            for m in re.finditer(r"LOAD\s+0x[0-9a-f]+\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)", out)]


def golden_lines(elf):
    return [f"{n} {a:08x} {s:06x}" for n, a, s in sections(elf)]
