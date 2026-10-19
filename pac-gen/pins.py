"""GPIO pins per package from the RM's signal table (Table 15-2): pin names, and whether the 196 and 256 MAPBGA
packages have each. Nothing else from the table is kept.
Usage: pins.py <RM markdown dir> -o <pins.json>"""
import argparse
import json
import re
from pathlib import Path

_GPIO = re.compile(r"^(?:RGPIO\d+/)?P([A-K])(?:(\d)|\[(\d):(\d)\])$")


_NOTE = re.compile(r"<sup>.*?</sup>|~~[^~]*~~")


def _cells(line: str) -> list[str]:
    s = line.strip()
    # a <br> inside a name is a line wrap ("FB_BE/BWE<br>[1:0]"): join without a space; other tags become spaces
    return [" ".join(re.sub(r"<[^>]+>", " ", _NOTE.sub("", c).replace("<br>", "")).split()) for c in s[1:-1].split("|")]


def _name(cell: str) -> str | None:
    return None if cell in ("—", "-", "") else cell


def _expand(name: str | None, n: int) -> list[str | None]:
    """Per-pin names for a range row of `n` pins (highest first): `SIG[1:0]` → `SIG1`, `SIG0`."""
    m = re.search(r"\[(\d+):(\d+)\]", name or "")
    if not m or int(m.group(1)) - int(m.group(2)) + 1 != n:
        return [name] * n
    h = int(m.group(1))
    return [name[:m.start()] + str(h - i) + name[m.end():] for i in range(n)]


def pins(chapter: str) -> dict[str, dict]:
    out: dict[str, dict] = {}
    last: list[str] = []                       # the previous row's pins, for names that wrap onto the next row
    for line in chapter.splitlines():
        if not line.startswith("|") or not line.rstrip().endswith("|"):
            continue
        c = _cells(line)
        if len(c) != 10:
            continue
        if not (m := _GPIO.match(c[1])):
            # a name ending in "/" wraps onto the next row, whose GPIO cell is empty; drop a section heading after it
            if last and not c[1]:
                for key, col in (("primary", 0), ("alt1", 2), ("alt2", 3)):
                    more = re.sub(r"\*\*[^*]*\*\*", "", c[col]).strip()
                    for name in last:
                        if (out[name][key] or "").endswith("/") and more:
                            out[name][key] += more
            last = []
            continue
        port, one, hi, lo = m.groups()
        bits = [int(one)] if one is not None else list(range(int(hi), int(lo) - 1, -1))
        packages = [pkg for pkg, cell in (("196", c[8]), ("256", c[9])) if cell not in ("—", "-", "")]
        prim, a1, a2 = (_expand(_name(x), len(bits)) for x in (c[0], c[2], c[3]))
        for i, b in enumerate(bits):
            name = f"P{port}{b}"
            if name in out:
                raise SystemExit(f"pins: {name} listed twice")
            out[name] = {"packages": packages, "primary": prim[i], "alt1": a1[i], "alt2": a2[i]}
        last = [f"P{port}{b}" for b in bits]
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("rm")
    ap.add_argument("-o", required=True)
    a = ap.parse_args()
    got = pins(next(Path(a.rm).glob("ch15-*.md")).read_text())
    want = {f"P{p}{b}" for p in "ABCDEFGHIJK" for b in range(8)}
    if set(got) != want:
        raise SystemExit(f"pins: missing {sorted(want - set(got))}, unexpected {sorted(set(got) - want)}")
    Path(a.o).write_text(json.dumps(dict(sorted(got.items())), indent=1) + "\n")


if __name__ == "__main__":
    main()
