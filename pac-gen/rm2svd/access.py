"""A field never has more access than its register, unless the register's figure shows cells for it.

A memory-map row gives the register's access; a figure's `Access:` line gives its fields'. Where the two disagree
(a field read-write in a read-only register) the figure's R and W rows decide: a W row with cells (a name, `w1c`)
means the register can be written, a blank one means the field cannot; the same for R cells and a write-only
register."""
import re

from .model import Register

_MORE = {"read-only": ("read-write", "write-only"), "write-only": ("read-write", "read-only")}


def _is_cell(token: str) -> bool:
    return bool(re.search(r"[A-Za-z]", token)) and not token.startswith("(") and token.lower() not in ("x", "reset")


def _cells(row: list[str], other: list[str], fields: list[str]) -> list[str]:
    """The row's cells that are not the end of a field name wrapped over from the other row (`SW` above `PORT`)."""
    heads = {t.upper() for t in other}
    names = {n.replace("_", "").upper() for n in fields}
    return [t for t in row if _is_cell(t) and not any(h + t.upper() == n for h in heads for n in names)]


def settle_access(reg: Register, rows: tuple[list[str], list[str]] | None, figure_access: str | None,
                  paired: bool) -> str | None:
    """Bring `reg`'s fields within its access; returns a problem line (without chapter and name) when it changed
    anything. `rows` are the figure's (R cells, W cells), None when the figure has none; `paired` when another
    register of the other direction shares the address and the figure (UISR/UIMR): each keeps its own."""
    over = [f for f in reg.fields if f.access in _MORE.get(reg.access, ())]
    if not over:
        return None
    names = ", ".join(f.name for f in over)

    def clamp() -> None:
        for f in over:
            f.access = reg.access
    if paired:
        clamp()
        return f"shares its address and figure with a {_MORE[reg.access][1]} register; {names} made {reg.access}"
    if rows is None:
        clamp()
        return f"figure has no R and W rows; {names} made {reg.access}"
    r, w = rows
    writing = reg.access == "read-only"
    cells = _cells(w, r, [f.name for f in reg.fields]) if writing else _cells(r, w, [f.name for f in reg.fields])
    row = "W" if writing else "R"
    if not cells:
        clamp()
        return f"figure's {row} row is blank; {names} made {reg.access}"
    was = reg.access
    explicit = figure_access if figure_access in _MORE[was] else None   # the figure says which way, when it does
    reg.access = explicit or "read-write"
    return f"figure's {row} row has cells ({' '.join(cells[:4])}); register {reg.access}, the memory map says {was}"
