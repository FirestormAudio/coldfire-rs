"""Every DSPI pin the HAL offers carries that DSPI signal, in that function's column of RM Table 15-2."""
import os
import re
from pathlib import Path

import pytest

HAL = Path(__file__).resolve().parents[2] / "embassy-mcf5441x"
ROW = re.compile(r"^\s*(pin|pcs)!\((?:#\[[^\]]*\]\s*)?(DSPI\d), (\w+), (P[A-K]\d), (PAR_\w+), (\d), (\d), (\w+)\);", re.M)
COLUMN = {"Primary": 0, "Alt1": 2, "Alt2": 3}
SIGNAL = {"SckPin": "SCK", "SoutPin": "SOUT", "SinPin": "SIN"}


def rows():
    return ROW.findall((HAL / "src/spi/pins.rs").read_text())


def test_every_dspi_has_sck_sout_sin_and_pcs0():
    got = {(d, k if m == "pin" else f"PCS{k}") for m, d, k, *_ in rows()}
    for n in range(4):
        assert {(f"DSPI{n}", s) for s in ("SckPin", "SoutPin", "SinPin", "PCS0")} <= got


RM = os.environ.get("RM")


@pytest.mark.skipif(not RM, reason="RM=<manual markdown dir> not set")
def test_pins_carry_their_dspi_signal_in_the_table():
    text = next(Path(RM).glob("ch15-*.md")).read_text()
    cells = []
    for line in text.splitlines():
        if line.startswith("|") and line.rstrip().endswith("|"):
            c = [re.sub(r"<[^>]+>", " ", x).strip() for x in line.strip()[1:-1].split("|")]
            if len(c) == 10:
                cells.append(c)
    for macro, dspi, kind, pin, _reg, _shift, _width, func in rows():
        sig = SIGNAL[kind] if macro == "pin" else f"PCS{kind}"
        row = next((c for c in cells if re.fullmatch(rf"(?:RGPIO\d+/)?{pin}", c[1])), None)
        assert row is not None, pin
        assert re.search(rf"\b{dspi}_{sig}\b", row[COLUMN[func]]), (dspi, kind, pin, func, row)
