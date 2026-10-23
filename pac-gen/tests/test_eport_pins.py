"""Every edge port pad the HAL offers carries that IRQ signal, in that function's column of RM Table 15-2."""
import os
import re
from pathlib import Path

import pytest

HAL = Path(__file__).resolve().parents[2] / "embassy-mcf5441x"
ROW = re.compile(r"^\s*irq!\((?:#\[[^\]]*\]\s*)?(\d), EPORT_EPF\1, (P[A-K]\d), (PAR_\w+), (\d), (\d), (\w+)\);", re.M)
COLUMN = {"Primary": 0, "Alt1": 2, "Alt2": 3}


def rows():
    return ROW.findall((HAL / "src/eport/pins.rs").read_text())


def test_every_row_is_an_irq_pad():
    assert rows() and all(1 <= int(n) <= 7 for n, *_ in rows())


RM = os.environ.get("RM")


@pytest.mark.skipif(not RM, reason="RM=<manual markdown dir> not set")
def test_pins_carry_their_irq_signal_in_the_table():
    text = next(Path(RM).glob("ch15-*.md")).read_text()
    cells = []
    for line in text.splitlines():
        if line.startswith("|") and line.rstrip().endswith("|"):
            c = [re.sub(r"<[^>]+>", " ", x).strip() for x in line.strip()[1:-1].split("|")]
            if len(c) == 10:
                cells.append(c)
    for n, pin, _reg, _shift, _width, func in rows():
        row = next((c for c in cells if re.fullmatch(rf"(?:RGPIO\d+/)?{pin}", c[1])), None)
        assert row is not None, pin
        assert re.search(rf"\bIRQ{n}\b", row[COLUMN[func]]), (n, pin, func, row)
