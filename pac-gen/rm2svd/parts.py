"""RM Table 1-1: which modules each MCF5441x part has (a checkmark, U+F0B7, or an empty cell is present, a dash
absent, a bare number an instance count)."""
import re

PARTS = ["mcf54410", "mcf54415", "mcf54416", "mcf54417", "mcf54418"]
_SEPARATOR = re.compile(r"^:?-+:?$")
_CHECK = "\uf0b7"                       # the checkmark, kept as the PDF's Symbol-font private-use character


def row_cells(line: str) -> list[str] | None:
    """Cells of a markdown table row, or None for a non-row. Removes exactly one leading and one trailing pipe so
    trailing empty cells survive; separator rows (all dashes or colons) are None too."""
    s = line.strip()
    if not s.startswith("|"):
        return None
    s = s[1:-1] if s.endswith("|") else s[1:]
    cells = [c.strip() for c in s.split("|")]
    if all(_SEPARATOR.match(c) for c in cells):
        return None
    return cells


def availability_with_problems(ch01: str) -> tuple[dict[str, dict[str, int]], list[str]]:
    """Table 1-1 by row label: per part, 0 for a dash, the number for a bare number (UARTs 6/10), and 1 (present)
    for an empty cell or any other text: a number with a unit counts ports, channels, Kbytes or a package, not
    modules, and the merged cells of the clock and memory rows land in one column. Such rows are reported by
    position, as is a chapter without the table."""
    out = {p: {} for p in PARTS}
    problems, heads = [], list(re.finditer(r"^\|\*\*Module\*\*\|\*\*MCF54410\*\*.*$", ch01, re.M))
    if not heads:
        return out, ["availability: Table 1-1 not found"]
    # Table 1-1 runs over several pages, each with its own header: read every one.
    for head in heads:
        for line in ch01[head.end():].splitlines()[1:]:
            if not line.strip().startswith("|"):
                break
            cells = row_cells(line)
            if cells is None:          # separator row
                continue
            if len(cells) != 6:
                break
            label = cells[0].replace("<br>", " ")
            other = False
            for p, c in zip(PARTS, cells[1:]):
                if c in ("—", "–", "-"):
                    out[p][label] = 0
                elif re.fullmatch(r"\d+", c):
                    out[p][label] = int(c)
                else:
                    out[p][label] = 1
                    other = other or c not in ("", _CHECK)
            if other:
                problems.append(f"availability: Table 1-1 row {list(out[PARTS[0]]).index(label) + 1}: "
                                "cells that aren't a count taken as present")
    return out, problems


def availability(ch01: str) -> dict[str, dict[str, int]]:
    return availability_with_problems(ch01)[0]


def _label(s: str) -> str:
    """A row label without markup tags and non-ASCII marks (®)."""
    return re.sub(r"[^\x00-\x7f]", "", re.sub(r"<[^>]+>", "", s))


def keyed(avail: dict[str, dict[str, int]], groups: dict[str, dict]) -> tuple[dict[str, dict[str, int]], list[str]]:
    """Availability by parts_map.json key instead of Table 1-1 row label: a row belongs to the one group whose
    `match` text it contains. The labels themselves are never kept; an unmatched row is reported by position."""
    out: dict[str, dict[str, int]] = {p: {} for p in avail}
    problems = []
    labels = list(next(iter(avail.values()), {}))
    seen = set()
    for k, label in enumerate(labels, 1):
        keys = [g for g, spec in groups.items() if spec["match"] in _label(label)]
        if len(keys) != 1:
            problems.append(f"availability: Table 1-1 row {k} matches " +
                            (f"several parts_map.json groups: {', '.join(keys)}" if keys else "no parts_map.json group"))
            continue
        seen.add(keys[0])
        for p in avail:
            out[p][keys[0]] = avail[p][label]
    if labels:
        problems += [f"availability: parts_map.json group {g} matches no Table 1-1 row" for g in groups if g not in seen]
    return out, problems
