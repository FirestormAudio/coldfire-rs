"""RM ch. 17 source tables → interrupts named after their flag, valued by vector (INTCn source s → 64·(n+1) + s)."""
import re

from .model import Interrupt
from .parts import row_cells

_TABLE = re.compile(r"Interrupt Source Assignment for INTC(\d)", re.I)
_FIELD = re.compile(r"([A-Za-z][A-Za-z0-9_]*)\s*\[([^\]]+)\]")
_REGISTER = re.compile(r"^([A-Za-z][A-Za-z0-9_]*) register$")
_OR = re.compile(r"\s+or\s+", re.I)
_IDENT = re.compile(r"[A-Za-z][A-Za-z0-9_]*")


def _name(flag: str) -> str | None:
    """REG_FIELD for `REG[FIELD]` (first field of `A or B`, range `56–63` becomes `56_63`), REG for `REG register`."""
    s = re.sub(r"\s+", " ", flag.replace("<br>", " ")).strip()
    m = _FIELD.search(s)
    if m:
        field = re.sub(r"[^A-Za-z0-9]+", "_", _OR.split(m.group(2))[0]).strip("_")
        return f"{m.group(1)}_{field}".upper()
    if _IDENT.fullmatch(s):
        return s.upper()
    r = _REGISTER.match(s)
    return r.group(1).upper() if r else None


def interrupts_with_problems(ch17: str) -> tuple[list[Interrupt], list[str]]:
    out, problems, seen, intc, module = [], [], set(), None, ""
    for line in ch17.splitlines():
        t = _TABLE.search(line)
        if t:
            if int(t.group(1)) != intc:       # a module label never carries into another INTC; a continued page keeps it
                module = ""
            intc = int(t.group(1))
            continue
        cells = row_cells(line)
        if intc is None or cells is None or len(cells) < 4 or not cells[0].isdigit():
            continue
        src, mod, flag = int(cells[0]), cells[1].replace("<br>", ""), cells[2]
        module = mod or module
        if not flag or "not used" in cells[3].lower():
            continue
        value = 64 * (intc + 1) + src
        name = _name(flag)
        if name is None:
            problems.append(f"INTC{intc} source {src} (vector {value}): flag {flag!r} has no name")
            continue
        if name in seen:
            name = f"{name}_{value}"
        seen.add(name)
        out.append(Interrupt(name, value, module))
    return out, problems


def interrupts(ch17: str) -> list[Interrupt]:
    return interrupts_with_problems(ch17)[0]
