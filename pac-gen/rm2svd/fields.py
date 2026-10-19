"""Fields from a 'Field Descriptions' table: bit ranges, names, write-1-to-clear, enumerated values."""
import re

from .model import Field
from .registers import norm_abbrev

_NAME_ROW = re.compile(r"^\|\s*([A-Za-z][A-Za-z0-9_]*)\s*\|(.*)\|\s*$")
_ENUM = re.compile(r"(?:^|<br>)\s*([01]{1,6})\s+([A-Za-z][^<|]{0,60})([^<|]?)")
# write-1-to-clear of the field itself: not a register type list (`RW1C`), not another field's clearing
_ONE = r"(?:a )?(?:1|one)"
_W1C = re.compile(rf"\bw1c\b|write {_ONE} to clear(?!\s+the\s+corresponding)"
                  rf"|write {_ONE} to (?:it|this bit|each bit) to clear\b"
                  rf"|cleared by writing {_ONE} to (?:it|this bit)\b"
                  rf"|writing {_ONE}(?: to (?:it|this bit))? clears (?:it|this bit)\b", re.I)


def _const(text: str) -> str:
    words = re.sub(r"[^A-Za-z0-9]+", "_", text.strip()).strip("_").upper()
    return words if words and not words[0].isdigit() else "V_" + words


def label_is_name(label: str, truncated: bool = False) -> bool:
    """Content rule: an enum label from the manual is kept only if it reads as a name, not a sentence:
    at most 24 characters as a constant, or at most 4 words, and not cut at the length cap."""
    name = _const(label)                                # words as the constant spells them (On-chip → ON_CHIP)
    return not truncated and (len(name) <= 24 or len(name.split("_")) <= 4)


def field_table_title(text: str) -> str | None:
    """The register a figure's field table is titled for (`Table 40-6. DSPI_x_ _CTAR_n_ Field Description` →
    `DSPI CTAR`), or None when the first `Field Description(s)` isn't in a table title."""
    m = re.search(r"Field Descriptions?", text)
    if not m:
        return None
    line = text[text.rfind("\n", 0, m.start()) + 1:m.start()]
    t = re.match(r"\W*Table [\d-]+\.(.*)$", line.replace("**", ""))
    if not t:
        return None
    s = re.sub(r"<[^>]+>", "", t.group(1)).replace("_x_", " ").replace("_n_", " ")
    return " ".join(re.sub(r"(?<![A-Za-z])[xn](?![A-Za-z])", " ", s).split()) or None


def title_names(title: str, register: str) -> bool:
    """Whether a field table's title names this register. Letters are compared; the title may list several
    (`A & B`, `ESW_ING{S,D}AL`), may lack the register's module prefix or index letter (`PODR x` for PODR_A),
    and may leave out a short infix the register has (`PWM_IER` for PWM_SM0IER, `RX_xx_MASK` for RXGMASK)."""
    key = lambda s: re.sub(r"[^A-Z]", "", s.upper())
    t = re.sub(r"_?xx_?", " ", title)
    while (b := re.search(r"(\S*?)\{([^{}]*)\}(\S*)", t)):
        t = t[:b.start()] + " & ".join(b.group(1) + o.strip() + b.group(3) for o in b.group(2).split(",")) \
            + t[b.end():]
    alts = [key(a) for a in re.split(r"&|\band\b|,|/", t) if key(a)]
    regs = {key(c) for c in (register, re.sub(r"_[A-Z]$", "", register), register.split("_", 1)[-1]) if key(c)}
    for a in alts:
        for r in regs:
            if a in r or r in a:
                return True
            if len(r) > len(a) and any(r.startswith(a[:i]) and r.endswith(a[i:]) and len(r) - len(a) <= 3
                                       for i in range(1, len(a))):
                return True
    return False


def diagram_has_fields(diagram: str, names: list[str]) -> bool:
    """Whether every field name appears as a word of the figure's bit diagram (`PIFn` as PIF)."""
    words = set(re.findall(r"[A-Za-z][A-Za-z0-9]*", re.sub(r"_?<br>", "", diagram).replace("_", "")))
    return bool(names) and all(n.replace("_", "") in words or n.rstrip("n").replace("_", "") in words
                               for n in names)


_DASH = "–—−-"                                                       # en dash, em dash, minus sign, hyphen
_BITS = rf"\d{{1,2}}(?:\s*[{_DASH}]+\s*\d{{1,2}})?"
_BITLIST = re.compile(rf"{_BITS}(?:\s*,\s*{_BITS})*\s*,?")
_BIT_CELL = re.compile(rf"^\|\s*\d{{1,2}}\s*(?:[,{_DASH}]|<br>|\||or\b)")       # a row whose first cell is a bit number
_NAME = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")


def _ranges(s: str) -> list[tuple[int, int]]:
    """`18–17, 16–15` → [(18, 17), (16, 15)] as (hi, lo)."""
    out = []
    for part in filter(None, (p.strip() for p in s.split(","))):
        b = [int(x) for x in re.split(rf"\s*[{_DASH}]+\s*", part)]
        out.append((max(b), min(b)))
    return out


def _bit_groups(cell: str) -> list[tuple[list[tuple[int, int]], list[str]]] | None:
    """A first cell's bit lists, each with the name lines after it (`13<br>CFA1<br>12<br>CFA0` gives two);
    None if the cell doesn't start with a bit list (`11 or 10`)."""
    cell = re.sub(r"<sup>.*?</sup>|\*\*Field\*\*", "", cell).replace("**", "")
    groups: list[tuple[list[tuple[int, int]], list[str]]] = []
    for frag in (f.strip() for f in cell.split("<br>")):
        if not frag:
            continue
        if _BITLIST.fullmatch(frag):
            if groups and groups[-1][0] and not groups[-1][1] and groups[-1][2]:
                groups[-1] = (groups[-1][0] + _ranges(frag), [], frag.endswith(","))   # a list continued
            else:
                groups.append((_ranges(frag), [], frag.endswith(",")))
        elif groups:
            groups[-1][1].append(frag)
        else:
            return None
    return [(r, n) for r, n, _ in groups]


def _joined(frags: list[str]) -> str | None:
    """Name lines of one field joined where the converter split a name (`RDBAS`+`E`, `DLY_`+`CTRL`); None when
    the lines are separate names or text."""
    name = frags[0]
    for f in frags[1:]:
        if name.endswith("_") or f.startswith("_") or re.fullmatch(r"[A-Z0-9]{1,3}", f):
            name += f
        else:
            return None
    return name


def _field_names(raw: str, count: int) -> list[str] | None:
    """One SVD name per listed bit range: `EN_n_` over two ranges → EN1, EN0; `BUSY/START` → BUSY_START,
    `16BIT` → _16BIT, `FIFO[3:0]` → FIFO. None if the text isn't a name."""
    indexed = "_n_" in raw or bool(re.search(r"\s+n\s*$", raw))
    name = norm_abbrev(re.sub(r"\[[\d:]+\]$", "", raw.strip())).replace("/", "_").rstrip("_")
    if name[:1].isdigit():
        name = "_" + name.upper()
    if not _NAME.fullmatch(name or "-"):
        return None
    if count == 1:
        return [name]
    if not indexed or not name.endswith("n"):
        return None
    return [name[:-1] + str(k) for k in range(count - 1, -1, -1)]


def _cell_names(cell: str) -> str:
    """A first cell for a problem line: its name-like lines, other text (a description run into it) as `…`."""
    return "<br>".join(f if len(f.split()) <= 3 else "…" for f in (x.strip() for x in cell.strip().split("<br>")))


def fields_after(text: str, width: int, access: str, problems: list[str] | None = None) -> list[Field]:
    """Fields of the first field-description table in `text`. A row starting with a bit number that can't be
    read becomes a `field row unread:` line in `problems`."""
    start = re.search(r"Field Descriptions?", text)
    if not start:
        return []
    out, seen_rows, pending = [], False, None
    unread = problems if problems is not None else []
    for line in text[start.end():].splitlines():
        s = line.strip()
        if re.match(r"\*\*Table [\d-]+\..*Field Descriptions?\*\*", s) and seen_rows:   # `(continued)` ends otherwise
            break                                                       # the next register's table
        if seen_rows and (re.match(r"#{1,5}\s", s) or (re.match(r"(?:#+\s*)?\*\*Table [\d-]+\.", s)
                                                and not re.search(r"Field Descriptions?.*continued", s))):
            break                                                       # a heading or another kind of table
        rows: list[tuple[list[tuple[int, int]], str]] = []              # (bit ranges, raw name) per field
        if _BIT_CELL.match(s) and s.endswith("|"):
            seen_rows = True
            first, desc = s[1:-1].split("|", 1)
            groups = _bit_groups(first)
            if groups is None:
                pending = None
                unread.append(f"field row unread: {_cell_names(first)}")
                continue
            if pending and pending[2]:                                  # `7, 6,` above: the list goes on here
                groups[0] = (pending[0] + groups[0][0], groups[0][1])
            pending = None
            for k, (ranges, frags) in enumerate(groups):
                if not frags:
                    if k == 0 and len(groups) == 1 and not desc.strip().lower().startswith("reserved"):
                        pending = (ranges, desc, first.rstrip().endswith(","))   # the name may be on the next row
                    continue                                            # else a reserved bit after the name
                raw = _joined(frags)
                if raw is None:
                    unread.append(f"field row unread: {_cell_names(first)}")
                    rows = []
                    break
                rows.append((ranges, raw))
        else:
            n = _NAME_ROW.match(s) if pending else None
            if not n:
                pending = None
                continue
            (ranges, desc, _), raw = pending, n.group(1)
            desc, pending = desc + "<br>" + n.group(2), None
            rows.append((ranges, raw))
        for ranges, raw in rows:
            if re.fullmatch(r"reserved", raw.strip(), re.I):
                continue
            names = _field_names(raw, len(ranges))
            if names is None:
                unread.append(f"field row unread: {raw.strip()} at bits {', '.join(f'{h}-{l}' for h, l in ranges)}")
                continue
            for name, (hi, lo) in zip(names, ranges):
                if hi >= width:
                    unread.append(f"field beyond width: {name} bits {hi}-{lo} in a {width}-bit register")
                    continue
                w1c = bool(_W1C.search(re.sub(r"\s*<br>\s*", " ", desc)))
                out.append(Field(name, lo, hi - lo + 1, access, w1c, _enum(desc, hi - lo + 1)))
    return out


def _enum(desc: str, width: int) -> dict[str, int]:
    """Enumerated values from `<br>`-separated `code label` lines whose code spells every bit of the field (a
    `10 MHz` in a 1-bit field's text isn't one). At least two values, distinct, and every label name-like."""
    enum, names = {}, True
    for code, label, more in _ENUM.findall(desc):
        if len(code) == width:
            names = names and label_is_name(label.strip(), truncated=bool(more))
            enum.setdefault(_const(label), int(code, 2))
    return _shorten(enum) if len(enum) >= 2 and len(set(enum.values())) == len(enum) and names else {}


def _shorten(enum: dict[str, int]) -> dict[str, int]:
    """Drop the words every value's name starts with (`PIF_INTERRUPT_REQUESTS_ENABLED`/`..._DISABLED` becomes
    `ENABLED`/`DISABLED`), keeping at least one word per name; unchanged if that would collide or start with a digit."""
    words = [n.split("_") for n in enum]
    k = 0
    while all(len(w) > k + 1 for w in words) and len({w[k] for w in words}) == 1:
        k += 1
    out = {"_".join(w[k:]): v for w, v in zip(words, enum.values())}
    if len(out) != len(enum) or any(n[0].isdigit() for n in out):
        return enum
    return out


def w1c_bits(figure_text: str, width: int) -> set[int]:
    m = re.search(r"\bW\b(.*?)\bReset\b", figure_text)
    cells = m.group(1).split() if m else []
    if len(cells) != width:
        return set()
    return {width - 1 - i for i, c in enumerate(cells) if c.lower() == "w1c"}


def _cells(line: str) -> list[str]:
    s = line.strip()
    return [c.strip() for c in s[1:-1 if s.endswith("|") else None].split("|")]


def figure_w1c_bits(figure_text: str, width: int) -> set[int]:
    """Write-1-to-clear bits of a register figure: `w1c` cells of a markdown table's W rows, each under the bit
    number row above it (a row whose header cells aren't all bit numbers places nothing), else the picture-text
    diagram's W cells (`w1c_bits`)."""
    out, bits = set(), None
    for line in figure_text.splitlines():
        if not line.lstrip().startswith("|"):
            continue
        cells = _cells(line)
        if cells and cells[0] == "" and len(cells) > 1:
            bits = [int(c) for c in cells[1:]] if all(c.isdigit() for c in cells[1:]) else None
        elif cells and cells[0] == "W" and bits is not None and len(cells) - 1 == len(bits):
            out |= {b for b, c in zip(bits, cells[1:]) if "w1c" in c.lower() and b < width}
    return out or w1c_bits(figure_text, width)


def figure_rows(diagram: str) -> tuple[list[str], list[str]] | None:
    """The cells of a register diagram's R rows and W rows (picture text or markdown table): every token after
    an `R` (or `W`) up to the next `R`, `W` or `Reset`. None when the diagram has no R row and no W row."""
    rows: tuple[list[str], list[str]] = ([], [])
    cur, seen = None, set()
    for t in (t.strip("*") for t in re.sub(r"<br>|\|", " ", diagram).split()):
        if t in ("R", "W"):
            cur = rows[t == "W"]
            seen.add(t)
        elif t in ("Reset", "Reset:"):
            cur = None
        elif cur is not None:
            cur.append(t)
    return rows if seen else None
