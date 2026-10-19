"""The USB driver names its ULPI pads as concrete pins: each ULPI signal must sit on exactly one pad, in Alt 2."""
import json
from pathlib import Path

HAL = Path(__file__).resolve().parents[2] / "embassy-mcf5441x"
ULPI = {"USB_CLKIN": "PC5", "ULPI_DIR": "PD2", "ULPI_NXT": "PE7", "ULPI_STP": "PJ0",
        **{f"ULPI_DATA{n}": p for n, p in zip(range(8), ["PK4", "PK5", "PK1", "PK2", "PK7", "PK6", "PK3", "PK0"])}}


def pins():
    return json.loads((HAL / "pins.json").read_text())


def test_each_ulpi_signal_has_exactly_one_pad_and_it_is_alt2():
    found = {}
    for pin, row in pins().items():
        for col in ("primary", "alt1", "alt2"):
            for sig in (row[col] or "").split("/"):
                if sig in ULPI:
                    found.setdefault(sig, []).append((pin, col))
    assert found == {s: [(p, "alt2")] for s, p in ULPI.items()}


def test_ulpi_pads_are_on_the_256_package_only():
    assert all(pins()[p]["packages"] == ["256"] for p in ("PC5", "PJ0", "PK0", "PK7"))


def test_the_driver_names_the_same_pads():
    src = (HAL / "src/usb/mod.rs").read_text()
    for sig, pad in ULPI.items():
        assert f"Peri<'d, {pad}>" in src, (sig, pad)
