"""Registers from the manual's markdown: memory-map rows (table and picture-text shapes) and register figures.

Table rows are read cell by cell, so the manual's several shapes parse alike: the width in the name cell
(`Name (ABBR)<br>16`) or in its own cell, no width cell at all (taken from the reset value's digits), extra
columns between name and width/access (USB's `N`, `H/D`; FlexCAN's reset-domain `Y`/`N`), stride rows
(`ADDR<br>+ (_n_0xC)` with `_n_= 0 – 5`), address ranges with letter ranges (`PODR_A – K`), offset-only rows
(`0x04`, relative to a base-address table) and blank-address rows that share the previous row's address."""
import re
from dataclasses import dataclass

from .parts import row_cells

ADDR = r"0x[0-9A-Fa-f]{4}_?[0-9A-Fa-f]{4}"
_ROW_PIC = re.compile(rf"({ADDR})\s+([^()<>|]*?)\(\s*([A-Za-z0-9_ ]+?)\s*\)\s*(\d+)\s+(R/W\w*|R|W)\s+(0x[0-9A-Fa-f_]+|—|-|See[^<]*?|Undefined)\s+[\d.]+/[\d-]+")
_PAIR = re.compile(rf"({ADDR})\s*\(([A-Za-z0-9_]+)\)")
_BITS = re.compile(r"(?<![\dx_])(63|31|15|7)(?:\s+|\s*(?:<br>|\|)\s*)(62|30|14|6)(?![\d_])")
_ACCESS = re.compile(r"(R/W1C|R/W|RW|R|W)(?:\d*)$")
_RANGE_N = re.compile(r"\bn\s*=\s*(\d+)\s*(?:–|-|:|to)\s*(\d+)")
_STRIDE_HEX = re.compile(r"\+\s*hex\(\s*(0x[0-9A-Fa-f]+|\d+)?\s*[^\w\s()]*\s*_?n_?\s*\)")
_STRIDE_PAREN = re.compile(r"\+\s*\(?\s*_?n_?\s*[^\w\s()]*\s*(0x[0-9A-Fa-f]+|\d+)\s*\)?")
_STRIDE = re.compile(r"\+\s*(0x[0-9A-Fa-f]+|\d+)?\s*[^\w\s()]*\s*_?n(?:_|\b)")
_ADDR_RANGE = re.compile(rf"({ADDR})\s*(?:<br>)?\s*[–-]\s*(?:<br>)?\s*({ADDR})")
_LETTERS = re.compile(r"^(.*?_)([A-Z])\s*[–-]\s*([A-Z])$")
_NUMBERS = re.compile(r"^(.*?)(\d+)\s*[–-]\s*(\d+)$")
_IDENT = re.compile(r"[A-Za-z][A-Za-z0-9_]*")
_RESET_CELL = re.compile(r"0x(?:[0-9A-Fa-f-]{2}|[0-9A-Fa-f-]{4})")     # an 8- or 16-bit reset value


def _addr(s: str) -> int:
    return int(s.replace("_", ""), 16)


def norm_abbrev(s: str) -> str:
    s = s.replace("_n_", "n").strip()
    return re.sub(r"\s+n\s*$", "n", s).replace(" ", "")


def access_of(s: str) -> str:
    """SVD access of a memory-map access cell; ValueError for anything else (`—`, `Undefined`)."""
    a = {"R": "read-only", "W": "write-only", "R/W": "read-write", "RW": "read-write", "R/W1C": "read-write"}
    k = s.strip().upper()
    if k not in a:
        raise ValueError(f"access not understood: {s!r}")
    return a[k]


def _reset(s: str) -> int | None:
    s = s.strip().replace("_", "")
    return int(s, 16) if re.fullmatch(r"0x[0-9A-Fa-f]+", s) else None


def _plain(s: str) -> str:
    """Cell text without bold, superscripts (footnote marks) or HTML tags other than <br>."""
    s = re.sub(r"<sup>.*?</sup>", "", s)
    return re.sub(r"<(?!br>)[^>]*>", "", s).replace("**", "").strip()


def _template(raw: str) -> str:
    """Abbreviation with its index markers explicit: `_n_` / ` n ` → `{n}`, `_x_` → `{x}`."""
    s = _plain(raw).replace("<br>", "")
    s = s.replace("_x_", "{x}").replace("_n_", "{n}")
    s = re.sub(r"\s+n(?:\s+|$)", "{n}", s)
    return s.replace(" ", "")


@dataclass
class MapRow:
    abbrev: str
    addresses: list[int]
    width: int | None
    access: str
    reset: int | None
    template: str = ""          # abbrev with `{n}` (instance marker) and `{x}` markers; array indices filled in
    relative: bool = False      # addresses are offsets from a base-address table
    array: bool = False         # expanded from a stride or range row (the index is in the template)
    section: int | None = None  # chapter number of the Section/Page column
    index: str | None = None    # the array index (or letter) an expanded row stands for


@dataclass
class Figure:
    addresses: list[tuple[int, str]]
    access: str | None
    one_to_clear_bits: set[int]
    text_after: str
    width: int | None = None
    rows: tuple[list[str], list[str]] | None = None     # the diagram's R and W cells


def _abbrev_cell(cell: str) -> tuple[list[str], int | None]:
    """(raw abbreviations, width embedded as `<br>16`) of a register-name cell; `(UMR1_n_), (UMR2_n_)` gives both."""
    s = re.sub(r"<sup>.*?</sup>", "", cell).replace("**", "")
    width = None
    w = re.search(r"<br>\s*(\d+)\s*$", s)
    if w:
        width, s = int(w.group(1)), s[:w.start()]
    found = []
    for m in re.finditer(r"\(([^()]*)\)", s):
        inner = m.group(1).replace("<br>", "").strip()
        if re.match(r"[A-Za-z_]", inner) and not inner.lower().startswith(("see", "bits", "channels")):
            found.append(re.split(r",|<br>", inner)[0].strip())
    if found:
        return found, width
    bare = s.split("<br>")[0].strip()
    if _IDENT.fullmatch(bare) and ("_" in bare or bare.isupper()):
        return [bare], width
    return [], width


def _width_from_reset(s: str) -> int | None:
    m = re.fullmatch(r"0x([0-9A-Fa-f_-]+)", s.strip())
    n = len(m.group(1).replace("_", "")) if m else 0
    return {2: 8, 4: 16, 8: 32}.get(n)


@dataclass
class _Raw:
    addrs: list[int]
    relative: bool
    stride: int | None
    span: tuple[int, int] | None
    nrange: tuple[int, int] | None
    abbrev: str
    width: int | None
    access: str
    reset: int | None
    section: int | None


def _parse_row(cells: list[str], prev: "_Raw | None") -> "list[_Raw] | str | None":
    """The register rows of a table row (one per name in its name cell), a problem, or None (not a map row)."""
    c0 = _plain(cells[0])
    span = None
    rm = _ADDR_RANGE.search(c0)
    if rm:
        span, addrs, relative = (_addr(rm.group(1)), _addr(rm.group(2))), [_addr(rm.group(1))], False
    elif re.search(ADDR, c0):
        addrs, relative = [_addr(a) for a in re.findall(ADDR, c0)], False
    elif re.fullmatch(r"0x[0-9A-Fa-f]{1,4}", c0):
        addrs, relative = [int(c0, 16)], True
    elif not c0 and prev is not None and prev.stride is None and prev.span is None:
        addrs, relative = list(prev.addrs), prev.relative
    else:
        return None
    rest = [c for c in cells[1:] if _plain(c)]
    if not rest:
        # an absolute address and nothing else (a stray row the conversion split off): not silently skipped
        return f"unread map row: {addrs[0]:#010x} no register in the row" if c0 and not relative else None
    abbrevs, width = _abbrev_cell(rest[0])
    if not abbrevs:
        words = _plain(rest[0]).replace("<br>", " ").split()
        firsts = [_plain(c).split("<br>")[0].strip() for c in rest[1:]]
        # a register row has an access cell; without one (a spelling this parser doesn't know), a reset value that
        # can't be a base address (8 or 16 bits) still marks it. Otherwise: a base-address table or a region list.
        register = any(_ACCESS.fullmatch(p) for p in firsts) or (
            not any(re.fullmatch(ADDR, _plain(c)) for c in cells[1:]) and any(_RESET_CELL.fullmatch(p) for p in firsts))
        if relative or not c0 or not words or words[0].lower().startswith("reserved") or not register:
            return None
        return f"unread map row: {addrs[0]:#010x} {' '.join(words[:5])}"
    abbrev = abbrevs[0]
    access = reset = None
    for k, c in enumerate(rest[1:], 1):
        p = _plain(c).split("<br>")[0].strip()
        if width is None and access is None and re.fullmatch(r"\d+", p):
            width = int(p)
        elif access is None and _ACCESS.fullmatch(p):
            access = _ACCESS.fullmatch(p).group(1)
            reset = _plain(rest[k + 1]).split("<br>")[0] if k + 1 < len(rest) else ""
            break
    if access is None:
        return None if relative or len(cells) < 5 or not c0 else f"unread map row: {addrs[0]:#010x} {_plain(abbrev)}"
    if width is None:
        width = _width_from_reset(reset)
    stride = None
    s = _STRIDE_HEX.search(c0) or _STRIDE_PAREN.search(c0) or _STRIDE.search(c0)
    if s:
        stride = int(s.group(1), 0) if s.group(1) else 1
    text = (c0 + " " + rest[0]).replace("_n_", " n ").replace("_", "").replace("**", "")
    n = _RANGE_N.search(text)
    if stride is None and span is None and ("+" in c0 or n):
        return f"stride not understood: {addrs[0]:#010x} {_plain(abbrev)}"
    sec = re.search(r"(\d+)\.\d+(?:\.\d+)*/", " ".join(cells[2:]))
    return [_Raw(addrs, relative, stride, span, (int(n.group(1)), int(n.group(2))) if n else None, a, width,
                 access, _reset(reset), int(sec.group(1)) if sec else None) for a in abbrevs]


def _expand(r: _Raw) -> list[MapRow] | str:
    """MapRows of a parsed row (one per array index or letter), or a problem."""
    tpl = _template(r.abbrev)
    try:
        access = access_of(r.access)
    except ValueError:
        return f"access not understood: {_plain(r.abbrev)} {r.access!r}"
    base = dict(width=r.width, access=access, reset=r.reset, relative=r.relative, section=r.section)
    if r.span:
        lm = _LETTERS.match(_plain(r.abbrev).replace("<br>", "").strip())
        if not lm:
            return f"address range without a letter range: {r.abbrev}"
        letters = [chr(c) for c in range(ord(lm.group(2)), ord(lm.group(3)) + 1)]
        step = (r.span[1] - r.span[0]) // max(len(letters) - 1, 1)
        if len(letters) < 2 or step * (len(letters) - 1) != r.span[1] - r.span[0]:
            return f"address range does not divide over {r.abbrev}"
        stem = _template(lm.group(1))
        return [MapRow(stem + c, [r.addrs[0] + k * step], template=stem + c, array=True, index=c, **base)
                for k, c in enumerate(letters)]
    nr = _NUMBERS.match(tpl)
    if nr and r.stride is None:                     # `ESW_PSNP1–8`: consecutive registers of the row's width
        if not r.width or r.width > 64:
            return f"register range wider than a register: {r.abbrev} ({r.width} bits)"
        out = []
        for i in range(int(nr.group(2)), int(nr.group(3)) + 1):
            t = nr.group(1) + str(i)
            step = (i - int(nr.group(2))) * r.width // 8
            out.append(MapRow(t, [a + step for a in r.addrs], template=t, array=True, index=str(i), **base))
        return out
    if r.stride is None or (r.width or 0) > 64:      # a wide stride row (eDMA TCDs) stays whole: build reports it
        return [MapRow(norm_abbrev(r.abbrev), r.addrs, template=tpl, **base)]
    if r.nrange is None:
        return f"stride row without an index range: {r.abbrev}"
    out = []
    for i in range(r.nrange[0], r.nrange[1] + 1):
        t = tpl.replace("{n}", str(i)) if "{n}" in tpl else tpl + str(i)
        out.append(MapRow(t.replace("{x}", "x"), [a + r.stride * i for a in r.addrs], template=t, array=True,
                          index=str(i), **base))
    return out


def memory_map_rows_with_problems(chapter: str) -> tuple[list[MapRow], list[str]]:
    rows, problems = [], []
    raws: list[_Raw] = []
    pending_range = None
    last: _Raw | None = None                                        # the register row on the line above
    for line in chapter.splitlines():
        cells = row_cells(line)
        above, last = last, None
        if cells is not None and len(cells) >= 4:
            r = _parse_row(cells, above)
            if isinstance(r, list):
                for x in r:
                    if x.stride is not None and x.nrange is None:
                        x.nrange = pending_range
                raws.extend(r)
                last = r[0]
                continue
            if isinstance(r, str):
                problems.append(r)
                continue
            n = _RANGE_N.search(re.sub(r"[*_|]", "", line))
            if n:
                rng = (int(n.group(1)), int(n.group(2)))
                if raws and raws[-1].stride is not None and \
                        re.match(r"\(?\s*n\s*=", _plain(cells[0]).replace("_", "")):
                    raws[-1].nrange = rng                           # `(_n_=1:7)` on the row below its stride row
                else:
                    pending_range = rng                             # a section header naming the index range
            continue
        if cells is not None:
            continue
        for seg in re.finditer(rf"{_ROW_PIC.pattern}((?:<br>{ADDR}(?=<|$))*)", line):
            more = [_addr(a) for a in re.findall(ADDR, seg.group(7))]
            raws.append(_Raw([_addr(seg.group(1)), *more], False, None, None, None, seg.group(3), int(seg.group(4)),
                             seg.group(5), _reset(seg.group(6)), None))
    for r in raws:
        e = _expand(r)
        if isinstance(e, str):
            problems.append(e)
        else:
            rows.extend(e)
    return rows, problems


def memory_map_rows(chapter: str) -> list[MapRow]:
    return memory_map_rows_with_problems(chapter)[0]


def _instance_name(label: str) -> str | None:
    s = _plain(label).replace("<br>", "")
    s = re.sub(r"[\s-]+", "", s)
    return s.upper() if re.fullmatch(r"[A-Za-z][A-Za-z0-9]*\d", s) else None


def instance_bases(chapter: str) -> list[tuple[str, int]]:
    """Base-address tables: rows of (address, `NAME k`) or (`NAME k`, address) pairs → [(NAMEk, base)]."""
    out = []
    for line in chapter.splitlines():
        cells = row_cells(line)
        if not cells or len(cells) % 2:
            continue
        found = []
        for a, b in ((cells[k], cells[k + 1]) for k in range(0, len(cells), 2)):
            if not a and not b:
                continue
            addr, label = (a, b) if re.fullmatch(ADDR, _plain(a)) else (b, a)
            name = _instance_name(label)
            if not re.fullmatch(ADDR, _plain(addr)) or name is None:
                found = None
                break
            found.append((name, _addr(_plain(addr))))
        if found:
            out.extend(found)
    return out


def instance_headers(chapter: str) -> list[str]:
    """The memory map's instance header cell (`**PIT 0**<br>**PIT 1**`) → ["PIT0", "PIT1"]; [] if none."""
    for line in chapter.splitlines():
        cells = row_cells(line)
        if not cells or "**" not in cells[0]:
            continue
        parts = [p for p in cells[0].split("<br>") if _plain(p) and _plain(p).lower() != "address"]
        names = [_instance_name(p) for p in parts]
        if len(names) >= 2 and all(names):
            return names
    return []


def figures(chapter: str) -> list[Figure]:
    from .fields import figure_rows, figure_w1c_bits    # fields imports this module
    out = []
    # `Address: 0x…`, `Address 0x…` (the colon lost in conversion) or `Offset 0x…` heads a figure
    ms = list(re.finditer(rf"\b(?:Address|Offset)\s*:?\s*{ADDR}", chapter))
    for k, m in enumerate(ms):
        end = ms[k + 1].start() if k + 1 < len(ms) else len(chapter)
        body = chapter[m.start():end]
        bits = _BITS.search(body[:1500])
        head_end = bits.start() if bits else min(len(body), 600)
        head = body[:head_end]
        pairs = [(_addr(a), n) for a, n in _PAIR.findall(head)]
        if not pairs:
            continue
        rest = re.sub(_PAIR, " ", head).replace("<br>", " ")
        acc = re.search(r"Access:\s*(?:User|Supervisor)?\s*(?:or\s+\w+\s+)?"
                        r"(read/write|read-write|read-only|write-only|read only|write only)", rest, re.I)
        word = acc.group(1).lower() if acc else None
        access = None if not acc else {"read/write": "read-write"}.get(word, word.replace(" ", "-"))
        width = int(bits.group(1)) + 1 if bits else None
        # the figure's own diagram: up to its caption, or its field table; a later figure headed "Offset" isn't ours
        own = re.search(r"\*\*Figure [\d-]+|Field Descriptions?", body)
        diagram = body[:own.start()] if own else body
        out.append(Figure(pairs, access, figure_w1c_bits(diagram, width or 32), chapter[m.start() + head_end:end], width,
                          figure_rows(body[bits.start() if bits else head_end:len(diagram)])))
    return out
