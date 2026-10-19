"""embassy-mcf5441x/pinmux.json: every pin's PAR field, checked against pins.json (Table 15-2) and the RM's PAR
register table and figures."""
import json
import os
import re
from pathlib import Path

import pytest

HAL = Path(__file__).resolve().parents[2] / "embassy-mcf5441x"
MAP = json.loads((HAL / "pinmux.json").read_text())
PINS = json.loads((HAL / "pins.json").read_text())
RM = os.environ.get("RM")


def test_every_pin_is_mapped_once():
    seen = [f["pin"] for f in MAP["fields"]] + [p for g in MAP["groups"] for p in g["pins"]]
    assert sorted(seen) == sorted(PINS)


def test_fields_fit_their_registers_without_overlap():
    for reg in MAP["registers"]:
        used = set()
        for f in (f for f in MAP["fields"] if f["reg"] == reg):
            bits = set(range(f["shift"], f["shift"] + f["width"]))
            assert f["width"] in (1, 2) and max(bits) < 8, f
            assert not used & bits, f
            used |= bits


def test_one_bit_fields_are_pins_without_alternates():
    for f in MAP["fields"]:
        if f["width"] == 1:
            assert PINS[f["pin"]]["alt1"] is None and PINS[f["pin"]]["alt2"] is None, f


def test_each_field_names_its_pins_primary_signal():
    for f in MAP["fields"]:
        assert (PINS[f["pin"]]["primary"] or "").startswith(f["signal"]), (f, PINS[f["pin"]])


def test_hal_consts_match_the_register_offsets():
    src = (HAL / "src/pinmux.rs").read_text()
    consts = {m[0]: m[1].lower() for m in re.findall(r"pub\(crate\) const (PAR_\w+): u8 = (0x[0-9A-Fa-f]+);", src)}
    assert consts == {k: v.lower() for k, v in MAP["registers"].items()}


def _figure(text: str, reg: str) -> str:
    """A register's figure: the body before the caption that names it (the address line inside may be split
    across table cells, as PAR_SIMP0L's is), else from its address line to the next caption (PAR_UART1 shares
    PAR_UART2's figure and has no caption of its own)."""
    cap = re.search(rf"\*\*Figure[^\n]*\({reg}\)", text)
    if cap:
        start = text.rfind("**Figure", 0, cap.start())
        return text[max(start, 0):cap.start()]
    at = text.index(f"({reg})")
    return text[at:text.index("**Figure", at)]


def _norm(s):
    """A field name without its PAR prefix, letters and digits only (some figures split "PAR_" from the name
    across rows, PAR_DEBUGH1's among them, or drop the underscore, as PAR_SIMP0L's "PARCLK")."""
    return re.sub(r"^PAR", "", re.sub(r"[^A-Z0-9]", "", s.upper()))


@pytest.mark.skipif(not RM, reason="RM=<manual markdown dir> not set")
def test_registers_sit_at_their_table_addresses():
    text = next(Path(RM).glob("ch15-*.md")).read_text()
    for reg, off in MAP["registers"].items():
        assert re.search(rf"\|0xEC09_40{int(off, 16):02X}\|{reg}\|", text), reg


@pytest.mark.skipif(not RM, reason="RM=<manual markdown dir> not set")
def test_verified_fields_follow_their_figures_order():
    text = next(Path(RM).glob("ch15-*.md")).read_text()
    for reg in MAP["registers"]:
        fields = sorted((f for f in MAP["fields"] if f["reg"] == reg and f["verified"]), key=lambda f: -f["shift"])
        if not fields:
            continue
        block = _figure(text, reg)
        tokens = _figure_names(block)
        pos = -1
        for f in fields:                                   # high shift first = leftmost in the figure
            nxt = tokens.index(_norm(f["field"]), pos + 1)
            assert nxt > pos, (reg, f["field"])
            pos = nxt


def test_reserved_fields_are_settled_and_say_why():
    reserved = [f for f in MAP["fields"] if f.get("reserved")]
    assert reserved and all(f["verified"] and isinstance(f["reserved"], str) for f in reserved), reserved
    pcs1 = next(f for f in MAP["fields"] if f["pin"] == "PC0")
    assert pcs1["verified"] and "SBF_CS" in pcs1["reserved"]


def test_pins_without_alternates_have_one_bit_fields():
    for f in MAP["fields"]:
        if PINS[f["pin"]]["alt1"] is None and PINS[f["pin"]]["alt2"] is None:
            assert f["width"] == 1, f


def test_flexbus_control_fields_are_settled():
    # widths 1+2+2+1+2 fill PAR_FBCTL exactly, in figure order: FBCLK 7, ALE 6:5, OE 4:3, R/W 2, TA 1:0
    fb = {f["field"]: (f["shift"], f["width"], f["verified"]) for f in MAP["fields"] if f["reg"] == "PAR_FBCTL"}
    assert fb == {"PAR_FBCLK": (7, 1, True), "PAR_ALE": (5, 2, True), "PAR_OE": (3, 2, True),
                  "PAR_RW": (2, 1, True), "PAR_TA": (0, 2, True)}


ENC = {"Gpio": 0, "Alt2": 1, "Alt1": 2, "Primary": 3}

# digitakt's hardware-proven PAR writes: register, {pin: function}, value of those pins' bits
PROVEN = [
    ("PAR_CANI2C", {"PB2": "Alt1", "PB1": "Alt1"}, 0xA0),                       # panel link (stub, panel.rs)
    ("PAR_CANI2C", {"PB0": "Alt1", "PC7": "Alt1"}, 0x0A),                       # MIDI / flashdump
    ("PAR_DSPIOWH", {"PD5": "Primary", "PD4": "Primary", "PD6": "Primary", "PD7": "Gpio"}, 0xFC),  # SPI NOR
    ("PAR_DSPIOWH", {"PD4": "Alt2", "PD6": "Alt2", "PD7": "Alt2"}, 0x15),        # eMMC on the shared pads
    ("PAR_DSPIOWL", {"PC0": "Primary"}, 0x80),                                  # SBF_CS
    ("PAR_TIMER", {"PD0": "Alt2"}, 0x04),                                       # eMMC SDHC_DAT1
    ("PAR_TIMER", {"PD2": "Alt2", "PE7": "Alt2"}, 0x41),                        # USB ULPI_DIR, ULPI_NXT
    ("PAR_SDHCH", {"PF1": "Primary"}, 0x30),                                    # eMMC DAT2
    ("PAR_SDHCL", {"PG6": "Primary"}, 0x0C),                                    # eMMC CMD
    ("PAR_IRQ0L", {"PC3": "Primary"}, 0x30),                                    # eMMC
    ("PAR_IRQ0L", {"PC5": "Alt2"}, 0x40),                                       # USB_CLKIN
    ("PAR_UART2", {"PE6": "Alt2", "PE5": "Alt2", "PE4": "Alt2", "PE3": "Alt2"}, 0x55),  # SSI1 audio
    ("PAR_SSI0H", {"PH4": "Alt1"}, 0x02),                                       # SSI_CLKIN
]


@pytest.mark.parametrize("reg, funcs, value", PROVEN)
def test_the_map_reproduces_digitakts_proven_writes(reg, funcs, value):
    got = 0
    for pin, func in funcs.items():
        f = next(f for f in MAP["fields"] if f["pin"] == pin)
        assert f["reg"] == reg, (pin, f)
        v = (func == "Primary") if f["width"] == 1 else ENC[func]
        got |= v << f["shift"]
    assert got == value, (reg, funcs, hex(got))


def _figure_names(block: str) -> list[str]:
    """Field names in figure order. A bare "PAR_" is a name wrapped onto a later line (PAR_FBCTL's FBCLK, the
    debug registers' DDATAn/PSTn): it holds a slot that the next name without a PAR prefix fills."""
    names: list[str | None] = []
    for raw in re.findall(r"[A-Z][A-Z0-9_]+", block):
        if raw == "PAR_":
            names.append(None)
        elif not raw.startswith("PAR") and None in names:
            names[names.index(None)] = _norm(raw)
        elif _norm(raw):
            names.append(_norm(raw))
    return [n for n in names if n]


def _cell_bits(header: list[str]) -> list[list[int]]:
    return [[int(b) for b in re.findall(r"\d", h)] for h in header]


@pytest.mark.skipif(not RM, reason="RM=<manual markdown dir> not set")
def test_figures_with_bit_cells_put_each_field_where_the_map_does():
    text = next(Path(RM).glob("ch15-*.md")).read_text()
    checked = 0
    for reg in MAP["registers"]:
        rows = [l for l in _figure(text, reg).splitlines() if l.startswith("|") and l.rstrip().endswith("|")]
        if len(rows) < 3:
            continue                                       # a picture-text figure: no bit cells
        cells = [[c.strip() for c in r.strip()[1:-1].split("|")] for r in rows]
        header = next((c for c in cells if c and c[0] == "" and all(re.fullmatch(r"\d(<br>\d)?", x) for x in c[1:] if x)), None)
        if header is None:
            continue
        bits = _cell_bits(header[1:])
        fields = [f for f in MAP["fields"] if f["reg"] == reg and f["verified"]]
        for f in fields:
            name = _norm(f["field"])
            cols = {i for c in cells for i, x in enumerate(c[1:]) if name and _norm(x) == name}
            if not cols:
                continue
            got = sorted(b for i in cols for b in bits[i])
            assert got == list(range(f["shift"], f["shift"] + f["width"])), (reg, f["field"], got)
            checked += 1
    assert checked >= 8, checked
