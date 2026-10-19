"""pins.py: GPIO pins, package availability and Table 15-2 functions from the signal table (synthetic rows in its
shape)."""
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pins  # noqa: E402

ROWS = """
|**Signal name**|**GPIO**|**Alt 1**|**Alt 2**|**Pull**|**Dir**|**V**|**Pad**|**196**|**256**|
|SIG_A|PA7|ALT_X|—|—|O|EVDD|fsr|E2|D2|
|SIG_B|PB6|—|—|—|I|EVDD|msr|—|F2|
|SIG_C|RGPIO3/PD0|—|ALT_Y<sup>12</sup>|U|I/O|EVDD|msr|H13|H13|
|SIG_D[1:0]|PA[1:0]|ALT_Z[1:0]|—|—|O|EVDD|fsr|F2, F1|D1, F4|
|SIG_E<sup>9</sup>~~5~~|PH2|—|—|—|O|EVDD|fsr|K12|—|
|SIG_F|—|—|—|—|O|EVDD|fsr|A1|A1|
"""


def test_rows_give_pins_packages_and_functions():
    got = pins.pins(ROWS)
    assert got["PA7"] == {"packages": ["196", "256"], "primary": "SIG_A", "alt1": "ALT_X", "alt2": None}
    assert got["PB6"] == {"packages": ["256"], "primary": "SIG_B", "alt1": None, "alt2": None}
    assert got["PD0"] == {"packages": ["196", "256"], "primary": "SIG_C", "alt1": None, "alt2": "ALT_Y"}
    assert got["PA1"]["primary"] == "SIG_D1" and got["PA0"]["primary"] == "SIG_D0"
    assert got["PA1"]["alt1"] == "ALT_Z1" and got["PA0"]["alt1"] == "ALT_Z0"
    assert got["PH2"] == {"packages": ["196"], "primary": "SIG_E", "alt1": None, "alt2": None}
    assert set(got) == {"PA7", "PB6", "PD0", "PA1", "PA0", "PH2"}


def test_a_pin_listed_twice_is_an_error():
    with pytest.raises(SystemExit):
        pins.pins(ROWS + "|SIG_G|PA7|—|—|—|O|EVDD|fsr|E2|D2|\n")


RM = os.environ.get("RM")


@pytest.mark.skipif(not RM, reason="RM=<manual markdown dir> not set")
def test_the_manual_lists_every_pin_once():
    got = pins.pins(next(Path(RM).glob("ch15-*.md")).read_text())
    assert sorted(got) == sorted(f"P{p}{b}" for p in "ABCDEFGHIJK" for b in range(8))
    assert got["PA7"]["packages"] == ["196", "256"] and got["PD3"]["packages"] == ["196", "256"]
    assert got["PH2"]["packages"] == ["196"]
    for p in ("PB6", "PB5", "PB0", "PC7", "PJ0", "PH1", "PH0", *(f"PG{b}" for b in range(8)),
              *(f"PK{b}" for b in range(8)), *(f"PI{b}" for b in range(2, 8))):
        assert got[p]["packages"] == ["256"], p


@pytest.mark.skipif(not RM, reason="RM=<manual markdown dir> not set")
def test_the_manual_names_each_pins_functions():
    got = pins.pins(next(Path(RM).glob("ch15-*.md")).read_text())
    assert (got["PB0"]["primary"], got["PB0"]["alt1"], got["PB0"]["alt2"]) == ("CAN1_TX", "UART9_TXD", "I2C1_SCL")
    assert got["PB2"]["alt1"] == "UART8_TXD"
    assert got["PA1"]["primary"] == "FB_BE/BWE1" and got["PA0"]["primary"] == "FB_BE/BWE0"
    assert got["PI5"]["primary"] == "PST3" and got["PI2"]["primary"] == "PST0"
    assert got["PJ6"]["primary"] == "MII0_RXD1" and got["PJ5"]["primary"] == "MII0_RXD0"
    assert got["PC5"]["alt2"] == "USB_CLKIN"                 # footnote marker removed
    assert got["PB7"]["alt1"] is None and got["PB7"]["alt2"] is None


CONT = """
|SIG_G/|PB7|ALT_A/|—|—|O|EVDD|fsr|E1|E1|
|SIG_H|||**Section**|||||||
|SIG_J|PB3|ALT_B|ALT_C/<br>|—|O|EVDD|fsr|E3|E3|
||||ALT_D<sup>17</sup><br>**Next section**|||||||
"""


def test_a_name_ending_in_a_slash_continues_on_the_next_row():
    got = pins.pins(CONT)
    assert got["PB7"]["primary"] == "SIG_G/SIG_H"
    assert got["PB7"]["alt1"] == "ALT_A/"                # nothing below it in that column: left as is
    assert got["PB3"]["alt2"] == "ALT_C/ALT_D"


@pytest.mark.skipif(not RM, reason="RM=<manual markdown dir> not set")
def test_the_manual_names_that_wrap_onto_the_next_row():
    got = pins.pins(next(Path(RM).glob("ch15-*.md")).read_text())
    assert got["PD4"]["primary"] == "DSPI0_SOUT/SBF_DO"
    assert got["PE7"]["alt2"] == "USBO_VBUS_OC/ULPI_NXT"
