"""Cross-check an SVD against oracles: every oracle register address must exist with the same width, and every
oracle bit constant must lie in the SVD's fields there (named fields where the oracle names them)."""
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def _dim(e) -> list[tuple[str, int]]:
    """(index, byte step) for each element of a `dim` array; one unnamed element otherwise."""
    if e.findtext("dim") is None:
        return [("", 0)]
    n, step = int(e.findtext("dim"), 0), int(e.findtext("dimIncrement"), 0)
    idx = (e.findtext("dimIndex") or "").split(",")
    if len(idx) != n:                                   # absent, or a range like 0-63
        lo = idx[0].split("-")[0]
        idx = [str(int(lo) + i) for i in range(n)] if lo.isdigit() else [str(i) for i in range(n)]
    return [(s, i * step) for i, s in enumerate(idx)]


def _walk(block, base: int, prefix: str, out: dict, alts: dict):
    for r in block.findall("register"):
        for s, step in _dim(r):
            name = r.findtext("name").replace("[%s]", s).replace("%s", s)
            a = base + int(r.findtext("addressOffset"), 0) + step
            out[a] = (prefix + name, int(r.findtext("size") or "32", 0))
            alts.setdefault(a, []).append((prefix + name, r))
    for c in block.findall("cluster"):
        for s, step in _dim(c):
            name = c.findtext("name").replace("[%s]", s).replace("%s", s)
            _walk(c, base + int(c.findtext("addressOffset"), 0) + step, f"{prefix}{name}_", out, alts)


def _registers(svd: Path, alts: dict | None = None) -> dict[int, tuple[str, int]]:
    """address → (name, width); `alts` gets address → every (name, register element) there (alternates too)."""
    root = ET.parse(svd).getroot()
    peris = {p.findtext("name"): p for p in root.iter("peripheral")}
    out = {}
    alts = {} if alts is None else alts
    for p in peris.values():
        base = int(p.findtext("baseAddress"), 0)
        src = peris.get(p.get("derivedFrom"), p)
        regs = src.find("registers")
        if regs is not None:
            _walk(regs, base, f"{p.findtext('name')}_", out, alts)
        out.setdefault(base, (p.findtext("name"), 0))
    return out


def _same_field(field: str, want: str) -> bool:
    """A field is the oracle's when the names agree without underscores or case, trailing index included
    (`PCS1` isn't `PCS2`); an indexed field (`PCSn`, `EN_n_`) holds any index (`PCS1`)."""
    f, w = field.replace("_", ""), want.replace("_", "")
    if f.upper() == w.upper():
        return True
    marker = re.fullmatch(r"(.*[^a-z])n", f)             # rm2svd's index marker: a lone lowercase n after the name
    idx = re.fullmatch(r"(.+?)\d+", w)
    return bool(marker and idx and idx.group(1).upper() == marker.group(1).upper())


def _bit_problems(reg, entry: dict) -> list[str]:
    """What an SVD register gets wrong about one oracle bit constant (empty: it agrees)."""
    fields = [(f.findtext("name"), int(f.findtext("bitOffset")), int(f.findtext("bitWidth")))
              for f in reg.iter("field")]
    mask = lambda lsb, w: ((1 << w) - 1) << lsb
    out = []
    value = entry["value"]
    for b in range(64):
        if value >> b & 1 and not any(lsb <= b < lsb + w for _, lsb, w in fields):
            out.append(f"bit {b} lies in no field")
    for want in entry["fields"]:
        hits = [(n, lsb, w) for n, lsb, w in fields if _same_field(n, want["name"])]
        if not hits:
            out.append(f"no field {want['name']} (fields: {', '.join(n for n, _, _ in fields) or 'none'})")
            continue
        n, lsb, w = hits[0]
        if "mask" in want and mask(lsb, w) != want["mask"]:
            out.append(f"{n} is bits {lsb + w - 1}-{lsb}, oracle mask {want['mask']:#x}")
        if want.get("within") and not mask(lsb, w) & value:
            out.append(f"{n} (bits {lsb + w - 1}-{lsb}) has none of the oracle's bits {value:#x}")
        if "value" in want and (value >> lsb) & ((1 << w) - 1) != want["value"]:
            out.append(f"{n} holds {(value >> lsb) & ((1 << w) - 1)} in {value:#x}, oracle {want['value']}")
        if "lsb" in want and lsb != want["lsb"]:
            out.append(f"{n} starts at bit {lsb}, oracle {want['lsb']}")
    return out


def _named(cands: list, register: str | None) -> list:
    """The alternates at an address that are the register the oracle names (`uart::UART0+UIMR` → UART0_UIMR;
    `USR_UCSR` names both), or all of them when none is (the oracle's name differs from the manual's)."""
    if not register:
        return cands
    short = register.split("::")[-1]
    last = short.split("+")[-1]
    want = {short.replace("+", "_"), last, *last.split("_")}
    own = [(n, r) for n, r in cands if n in want or n.split("_", 1)[-1] in want]
    return own or cands


def verify(svd: Path, oracles: list[Path]) -> list[str]:
    """Findings, one line each; empty = pass. Entries marked `"excluded": true` carry a `note` and are skipped;
    `"excluded": "width"` skips only the width check (the address must still be a register)."""
    alts: dict = {}
    regs, out = _registers(svd, alts), []
    for o in oracles:
        data = json.loads(Path(o).read_text())
        for b in data.get("bits", []):
            if b.get("excluded"):
                continue
            cands = alts.get(b["address"], [])
            if not cands:
                out.append(f"{data['source']}: bits {b['name']}: no register at {b['address']:#010x} ({b['register']})")
                continue
            found = [(n, _bit_problems(r, b)) for n, r in _named(cands, b.get("register"))]
            if all(p for _, p in found):                # no register there (alternates included) agrees
                n, p = found[0]
                out.append(f"{data['source']}: bits {b['name']} in {n}: {'; '.join(p)}")
        for r in data["registers"]:
            if r.get("excluded") is True:
                continue
            got = regs.get(r["address"])
            if got is None:
                out.append(f"{data['source']}: missing {r['name']} at {r['address']:#010x}")
                continue
            # every register at the address (alternates too); the width agrees if any of them has it
            widths = {int(e.findtext("size") or "32", 0): n for n, e in alts.get(r["address"], [])} or {got[1]: got[0]}
            if r.get("excluded") != "width" and r.get("width") and 0 not in widths and r["width"] not in widths:
                names = ", ".join(n for n, _ in alts.get(r["address"], [])) or got[0]
                out.append(f"{data['source']}: width {r['name']} {names} is "
                           f"{'/'.join(map(str, sorted(widths)))}, oracle {r['width']}")
    return out


def excluded(oracles: list[Path]) -> int:
    datas = [json.loads(Path(o).read_text()) for o in oracles]
    return sum(bool(r.get("excluded")) for d in datas for r in d["registers"] + d.get("bits", []))


def checked_bits(oracles: list[Path]) -> int:
    return sum(not b.get("excluded") for o in oracles for b in json.loads(Path(o).read_text()).get("bits", []))


def summary(oracles: list[Path]) -> str:
    """The exclusion counts by kind. A bit constant with no register known when the oracle was made is
    excluded with "no register known": counted apart, so a constant left unchecked shows up here."""
    datas = [json.loads(Path(o).read_text()) for o in oracles]
    regs = [r for d in datas for r in d["registers"]]
    bits = [b for d in datas for b in d.get("bits", [])]
    whole = sum(r.get("excluded") is True for r in regs)
    width = sum(r.get("excluded") == "width" for r in regs)
    xbits = [b for b in bits if b.get("excluded")]
    unplaced = sum(b.get("note") == "no register known for this constant" for b in xbits)
    return (f"oracle entries excluded (see their notes): {excluded(oracles)} (registers: {whole}, widths only: "
            f"{width}; bit constants: {len(xbits)}, of which no register known: {unplaced}); bit constants checked: "
            f"{checked_bits(oracles)}")


if __name__ == "__main__":
    oracles = [Path(p) for p in sys.argv[2:]]
    problems = verify(Path(sys.argv[1]), oracles)
    print(summary(oracles))
    print("\n".join(problems) or "verify: ok")
    sys.exit(1 if problems else 0)
