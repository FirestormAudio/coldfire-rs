"""ColdFire gate: (1) objdump must decode every instruction (CF_OBJDUMP_ARCH, default m68k:isa-c:emac, the
MCF5441x core); (2) GNU as (-mcpu=CF_CPU, default 54418) must accept the decoded text. The objdump->GAS
rewriting below only fixes syntax; any remaining rejection is a real ISA violation."""
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

_LINE = re.compile(r"^\s*([0-9a-f]+):\s+(?:[0-9a-f]{4} ?)+\s+(.*)$")
_BRANCH = re.compile(r"^b(ra|sr|hi|ls|cc|cs|ne|eq|vc|vs|pl|mi|ge|lt|gt|le)[sbwl]?\s")


def normalise(ins: str) -> str:
    ins = re.sub(r"\s*<[^>]*>", "", ins).strip()
    ins = re.sub(r"^(cpushl)\s+(nc|dc|ic|bc),", r"\1 %\2,", ins)  # cache operand (objdump drops the %)
    ins = re.sub(r"%pc@\([0-9a-fx]+", "%pc@(0", ins)                       # PC-relative: legality only
    ins = re.sub(r"(?<![\w%#.$(-])([0-9a-f]{5,8})\b", r"0x\1", ins)        # bare absolute addresses
    ins = re.sub(r"^(\w+\s+)([0-9a-f]{1,4})(,|$)", r"\g<1>0x\2\3", ins)    # short bare absolute operand
    ins = re.sub(r"@\((-?)([0-9a-f]+),", r"@(\g<1>0x\2,", ins)             # indexed-mode displacement
    if _BRANCH.match(ins):
        ins = ins.split()[0] + " ."                                        # target irrelevant to ISA check
    return ins


def _objdump_arch() -> str:
    """objdump's machine for the decode pass: the MCF5441x core (ISA_C + EMAC) unless CF_OBJDUMP_ARCH says."""
    return os.environ.get("CF_OBJDUMP_ARCH", "m68k:isa-c:emac")


def _cpu() -> str:
    """GAS's -mcpu for the re-assembly pass: 54418 unless CF_CPU says."""
    return os.environ.get("CF_CPU", "54418")


@dataclass
class GateResult:
    undecodable: list[str] = field(default_factory=list)
    rejected: list[str] = field(default_factory=list)
    instructions: int = 0

    @property
    def ok(self) -> bool:
        return self.instructions > 0 and not self.undecodable and not self.rejected  # no code: nothing checked


_TABLEJUMP = re.compile(r"^jmp %pc@\(([0-9a-f]+)\s*(?:<[^>]*>)?,")


def _disassemble(elf: Path, start: int | None = None) -> list[tuple[int, str, str]]:
    cmd = ["m68k-elf-objdump", "-d", "-m", _objdump_arch(), str(elf)]
    if start is not None:
        cmd.append(f"--start-address={start:#x}")
    out = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout
    rows = []
    for l in out.splitlines():
        m = _LINE.match(l)
        if m:
            rows.append((int(m.group(1), 16), m.group(0).split("\t")[1].strip() if "\t" in m.group(0) else "", m.group(2).strip()))
    return rows


def _functions(elf: Path) -> list[tuple[int, int]]:
    """(start, end) of every sized FUNC symbol."""
    out = subprocess.run(["m68k-elf-readelf", "-sW", str(elf)], check=True, capture_output=True, text=True).stdout
    fns = []
    for l in out.splitlines():
        f = l.split()
        if len(f) >= 8 and f[3] == "FUNC" and f[2].isdigit() and int(f[2]):
            start = int(f[1], 16)
            fns.append((start, start + int(f[2])))
    return fns


def _code_rows(elf: Path) -> list[tuple[int, str]]:
    """Disassembly with GCC's inline jump tables (`jmp %pc@(T,Xn)` + 16-bit offsets at T) removed.
    A table ends at the lowest forward target T + offset, or at the first word that can't be an entry: its
    target falls outside the enclosing function (when its symbol has a size), or it points backwards at
    something that isn't an instruction start. A table whose cases all come before it has no forward
    target. Disassembly restarts at the end, since objdump may have decoded table words as
    instructions and lost alignment."""
    rows, done = _disassemble(elf), set()
    fns = _functions(elf)
    while True:
        byte_at = {}
        for a, raw, _ in rows:
            for k, w in enumerate(raw.split()):
                byte_at[a + 2 * k] = int(w, 16)
        table = next((int(m.group(1), 16) for a, _, t in rows
                      if (m := _TABLEJUMP.match(t)) and int(m.group(1), 16) not in done), None)
        if table is None:
            return [(a, t) for a, _, t in rows]
        done.add(table)
        fn = next(((a, b) for a, b in fns if a <= table < b), None)
        starts = {a for a, _, _ in rows if a < table}        # decoded before the table: reliable boundaries
        end, p = None, table
        while (end is None or p < end) and p in byte_at:
            off = byte_at[p] - 0x10000 if byte_at[p] & 0x8000 else byte_at[p]
            outside = fn is not None and not fn[0] <= table + off < fn[1]
            if outside or (off < 0 and table + off not in starts):
                end = p                                       # not an entry: the table ended before it
                break
            if off > 0:
                end = table + off if end is None else min(end, table + off)
            p += 2
        if end is None:
            return [(a, t) for a, _, t in rows]
        rows = [r for r in rows if r[0] < table] + _disassemble(elf, start=end)


# libgloss m68k semihosting: a NOP at a 4-byte boundary, HALT, then the sentinel longword 0x4E7BF000, which objdump
# decodes as `movec %sp,%sfc`. The host resumes after the sentinel, so it is data, never executed.
_SENTINEL = "movec %sp,%sfc"


def _semihosting_sentinels(rows: list[tuple[int, str]]) -> set[int]:
    """The addresses of the sentinels that follow a semihosting trap's NOP and HALT."""
    return {a for (n, nop), (h, halt), (a, t) in zip(rows, rows[1:], rows[2:])
            if t == _SENTINEL and halt == "halt" and nop == "nop" and n % 4 == 0 and h == n + 2 and a == n + 4}


def check(elf: Path) -> GateResult:
    res, lines = GateResult(), []
    rows = _code_rows(elf)
    sentinels = _semihosting_sentinels(rows)
    for a, text in rows:
        addr = f"{a:x}"
        if a in sentinels:
            continue
        if text.startswith(".short"):
            res.undecodable.append(f"{addr}: {text}")
            continue
        if text.startswith((".long", ".byte")):
            continue
        lines.append((addr, normalise(text)))
    res.instructions = len(lines) + len(res.undecodable)
    with tempfile.NamedTemporaryFile("w", suffix=".s", delete=False) as f:
        f.write("".join(f"    {t}\n" for _, t in lines))
    err = subprocess.run(["m68k-elf-as", f"-mcpu={_cpu()}", f.name, "-o", "/dev/null"],
                         capture_output=True, text=True).stderr
    Path(f.name).unlink()
    for e in err.splitlines():
        m = re.match(r".*:(\d+): Error: (.*)", e)
        if m:
            a, t = lines[int(m.group(1)) - 1]
            res.rejected.append(f"{a}: {t}  [{m.group(2)}]")
    return res


def main(argv: list[str]) -> int:
    bad = 0
    for p in argv:
        r = check(Path(p))
        print(f"{p}: {r.instructions} instructions, {len(r.undecodable)} undecodable, {len(r.rejected)} rejected")
        for x in (r.undecodable + r.rejected)[:20]:
            print("   ", x)
        bad += not r.ok
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
