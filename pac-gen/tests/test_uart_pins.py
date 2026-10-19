"""Every UART pin the HAL offers carries that UART signal, in that function's column of RM Table 15-2."""
import os
import re
from pathlib import Path

import pytest

HAL = Path(__file__).resolve().parents[2] / "embassy-mcf5441x"
ROW = re.compile(r"^\s*pin!\((?:#\[[^\]]*\]\s*)?(UART\d), (TxPin|RxPin), (P[A-K]\d), (PAR_\w+), (\d), (\d), (\w+)\);", re.M)
COLUMN = {"Primary": 0, "Alt1": 2, "Alt2": 3}


def rows():
    return ROW.findall((HAL / "src/uart/pins.rs").read_text())


def test_every_uart_has_a_tx_and_an_rx_pin():
    got = {(u, k) for u, k, *_ in rows()}
    assert got == {(f"UART{n}", k) for n in range(10) for k in ("TxPin", "RxPin")}


RM = os.environ.get("RM")


@pytest.mark.skipif(not RM, reason="RM=<manual markdown dir> not set")
def test_pins_carry_their_uart_signal_in_the_table():
    text = next(Path(RM).glob("ch15-*.md")).read_text()
    cells = []
    for line in text.splitlines():
        if line.startswith("|") and line.rstrip().endswith("|"):
            c = [re.sub(r"<[^>]+>", " ", x).strip() for x in line.strip()[1:-1].split("|")]
            if len(c) == 10:
                cells.append(c)
    for uart, kind, pin, _reg, _shift, _width, func in rows():
        n, sig = uart[4:], "TXD" if kind == "TxPin" else "RXD"
        row = next((c for c in cells if re.fullmatch(rf"(?:RGPIO\d+/)?{pin}", c[1])), None)
        assert row is not None, pin
        want = rf"\bU(?:ART)?{n}_?{sig}\b"
        assert re.search(want, row[COLUMN[func]]), (uart, kind, pin, func, row)


PAR_ADDR = {"PAR_CANI2C": 0x4B, "PAR_DSPIOWH": 0x4E, "PAR_UART2": 0x51, "PAR_UART1": 0x52, "PAR_UART0": 0x53,
            "PAR_SSI0H": 0x58, "PAR_SSI0L": 0x59}


def par_fields(text: str, reg: str) -> list[str]:
    """A PAR register's field names, high bits first, from its figure (from its address to the figure's end)."""
    at = text.index(f"0xEC09_40{PAR_ADDR[reg]:02X} ({reg})")
    block = text[at:text.index("<!-- End of picture text -->", at)]
    return [t for t in re.findall(r"PAR_[A-Z0-9_]+", block) if t not in PAR_ADDR]


@pytest.mark.skipif(not RM, reason="RM=<manual markdown dir> not set")
def test_pins_use_their_pads_par_field():
    text = next(Path(RM).glob("ch15-*.md")).read_text()
    cells = []
    for line in text.splitlines():
        if line.startswith("|") and line.rstrip().endswith("|"):
            c = [re.sub(r"<[^>]+>", " ", x).strip() for x in line.strip()[1:-1].split("|")]
            if len(c) == 10:
                cells.append(c)
    for uart, kind, pin, reg, shift, _width, _func in rows():
        fields = par_fields(text, reg)
        pad = next(c for c in cells if re.fullmatch(rf"(?:RGPIO\d+/)?{pin}", c[1]))[0]
        hits = [i for i, f in enumerate(fields) if re.search(rf"(?:^|[_/ ]){re.escape(f[4:])}(?:$|[/ ])", pad)]
        assert len(hits) == 1, (pin, pad, reg, fields)
        assert int(shift) == 2 * (len(fields) - 1 - hits[0]), (uart, kind, pin, reg, fields, pad)
