"""halgen.py renders embassy-mcf5441x/src/peripherals.rs from pins.json and parts_map.json."""
import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import halgen  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
HAL = ROOT / "embassy-mcf5441x"
P256 = 'any(feature = "mcf54415", feature = "mcf54416", feature = "mcf54417", feature = "mcf54418")'


def render():
    return halgen.render(json.loads((HAL / "pins.json").read_text()),
                         json.loads((ROOT / "pac-gen/parts_map.json").read_text()),
                         json.loads((HAL / "pinmux.json").read_text()))


def render_uart():
    return halgen.render_uart_pins(json.loads((HAL / "pins.json").read_text()),
                                   json.loads((ROOT / "pac-gen/parts_map.json").read_text()),
                                   json.loads((HAL / "pinmux.json").read_text()))


def test_halgen_output_is_committed():
    assert (HAL / "src/peripherals.rs").read_text() == render()


def test_halgen_gates_tokens_absent_on_mcf54410():
    out = render()
    for t in ("UART2", "UART6", "UART9", "I2C1"):
        assert f"#[cfg({P256})] {t}," in out
        assert f"#[cfg({P256})] impl_regs!({t}," in out
    assert "\n    UART8,\n" in out and "\n    I2C0,\n" in out


def test_halgen_gates_pins_by_package():
    out = render()
    assert f"#[cfg({P256})] PB6," in out
    assert '#[cfg(feature = "mcf54410")] PH2,' in out
    assert "\n    PA7,\n" in out
    assert f"#[cfg({P256})] impl_pin!(PB6, 1, 6, None);" in out


def test_halgen_leaves_the_time_driver_pit_out():
    out = render()
    assert '#[cfg(not(feature = "time-driver-pit0"))] PIT0,' in out
    assert '#[cfg(not(feature = "time-driver-pit3"))] impl_regs!(PIT3, pit0::Pit0);' in out


def test_halgen_rejects_a_token_present_on_no_part():
    import pytest
    parts_map = {"absent": {p: [] for p in halgen.PARTS}}
    with pytest.raises(SystemExit, match="PA0"):
        halgen.render({"PA0": {"packages": ["no-such-package"]}}, parts_map, {"registers": {}, "fields": [], "groups": []})
    parts_map["absent"] = {p: list(halgen.BLOCKS) for p in halgen.PARTS}
    with pytest.raises(SystemExit, match="PIT0"):
        halgen.render({}, parts_map, {"registers": {}, "fields": [], "groups": []})


def test_halgen_gives_every_dma_channel_its_interrupt():
    out = render()
    assert "impl_dma_channel!(DMA_CH0, 0, EDMA_INTR_INT00);" in out
    assert "impl_dma_channel!(DMA_CH36, 36, EDMA_INTR_INT36);" in out
    assert "impl_dma_channel!(DMA_CH55, 55, EDMA_INTR_INT55);" in out
    assert "impl_dma_channel!(DMA_CH60, 60, EDMA_INTR_INT56_63);" in out
    assert out.count("impl_dma_channel!(") == 64


def test_uart_pins_output_is_committed():
    assert (HAL / "src/uart/pins.rs").read_text() == render_uart()


def test_pins_carry_their_par_field():
    out = render()
    assert "impl_pin!(PB0, 1, 0, Some(crate::pinmux::ParField::new(crate::pinmux::PAR_CANI2C, 2, 2)));" in out
    assert "impl_pin!(PI2, 8, 2, Some(crate::pinmux::ParField::new(crate::pinmux::PAR_DEBUGH0, 0, 1)));" in out


def test_unverified_and_group_pins_get_no_par():
    out = render()
    for p, port, bit in (("PC0", 2, 0), ("PB3", 1, 3), ("PC4", 2, 4), ("PD3", 3, 3), ("PJ3", 9, 3), ("PK0", 10, 0), ("PI1", 8, 1)):
        assert re.search(rf"impl_pin!\({p}, {port}, {bit}, None\);", out), p


PROVEN = {   # the hand-written UART pin table the generated one replaced, proven on hardware (console, MIDI, panel)
    ("UART0", "TxPin", "PF3", "PAR_UART0", "0", "Primary"), ("UART0", "RxPin", "PF4", "PAR_UART0", "2", "Primary"),
    ("UART1", "TxPin", "PF7", "PAR_UART1", "0", "Primary"), ("UART1", "RxPin", "PE0", "PAR_UART1", "2", "Primary"),
    ("UART2", "TxPin", "PE3", "PAR_UART2", "0", "Primary"), ("UART2", "RxPin", "PE4", "PAR_UART2", "2", "Primary"),
    ("UART3", "TxPin", "PD4", "PAR_DSPIOWH", "4", "Alt1"), ("UART3", "RxPin", "PD5", "PAR_DSPIOWH", "6", "Alt1"),
    ("UART4", "TxPin", "PF6", "PAR_UART0", "6", "Alt1"), ("UART4", "RxPin", "PF5", "PAR_UART0", "4", "Alt1"),
    ("UART5", "TxPin", "PE2", "PAR_UART1", "6", "Alt1"), ("UART5", "RxPin", "PE1", "PAR_UART1", "4", "Alt1"),
    ("UART6", "TxPin", "PE6", "PAR_UART2", "6", "Alt1"), ("UART6", "RxPin", "PE5", "PAR_UART2", "4", "Alt1"),
    ("UART7", "TxPin", "PH5", "PAR_SSI0H", "2", "Alt1"), ("UART7", "RxPin", "PH3", "PAR_SSI0L", "0", "Alt1"),
    ("UART8", "TxPin", "PB2", "PAR_CANI2C", "6", "Alt1"), ("UART8", "RxPin", "PB1", "PAR_CANI2C", "4", "Alt1"),
    ("UART9", "TxPin", "PB0", "PAR_CANI2C", "2", "Alt1"), ("UART9", "RxPin", "PC7", "PAR_CANI2C", "0", "Alt1"),
}


def test_generated_uart_pins_match_the_proven_table():
    rows = re.findall(r"pin!\((?:#\[[^\]]*\]\s*)?(UART\d), (TxPin|RxPin), (P[A-K]\d), (PAR_\w+), (\d), \d, (\w+)\);", render_uart())
    assert set(rows) == PROVEN


def test_uart_pins_are_gated_by_part_and_package():
    out = render_uart()
    assert re.search(r"pin!\(#\[cfg\([^\]]*mcf54418[^\]]*\)\] UART9, TxPin, PB0,", out)
    assert re.search(r"^pin!\(UART8, TxPin, PB2,", out, re.M)


def test_reserved_pads_keep_their_mux_for_gpio():
    assert re.search(r"impl_pin!\(PC0, 2, 0, None\);", render())


def test_a_driver_signal_on_a_pad_without_a_settled_field_is_an_error():
    import pytest
    parts_map = {"absent": {p: [] for p in halgen.PARTS}}
    pins = {"PA0": {"packages": ["196", "256"], "primary": "UART0_TXD", "alt1": None, "alt2": None}}
    pinmux = {"registers": {"PAR_X": "0x48"}, "groups": [],
              "fields": [{"reg": "PAR_X", "field": "PAR_X", "pin": "PA0", "shift": 0, "width": 2, "signal": "UART0_TXD",
                          "verified": False}]}
    with pytest.raises(SystemExit, match="PA0"):
        halgen.render_uart_pins(pins, parts_map, pinmux)


def test_flexbus_control_pads_carry_their_field():
    assert "impl_pin!(PA7, 0, 7, Some(crate::pinmux::ParField::new(crate::pinmux::PAR_FBCTL, 5, 2)));" in render()
    assert "impl_pin!(PB7, 1, 7, Some(crate::pinmux::ParField::new(crate::pinmux::PAR_FBCTL, 7, 1)));" in render()


def render_spi():
    return halgen.render_spi_pins(json.loads((HAL / "pins.json").read_text()),
                                  json.loads((ROOT / "pac-gen/parts_map.json").read_text()),
                                  json.loads((HAL / "pinmux.json").read_text()))


SPI_ROW = re.compile(r"(pin|pcs)!\((?:#\[[^\]]*\]\s*)?(DSPI\d), (\w+), (P[A-K]\d), (PAR_\w+), (\d), (\d), (\w+)\);")

# Table 15-2's DSPI signals with their pads' PAR fields (pinmux.json); pcs rows carry the chip-select index
RM_SPI = {
    ("pcs", "DSPI0", "0", "PD7", "PAR_DSPIOWH", "0", "2", "Primary"),
    ("pcs", "DSPI0", "1", "PC0", "PAR_DSPIOWL", "7", "1", "Primary"),
    ("pcs", "DSPI0", "2", "PC2", "PAR_IRQ0L", "2", "2", "Alt1"),
    ("pcs", "DSPI0", "3", "PC3", "PAR_IRQ0L", "4", "2", "Alt1"),
    ("pin", "DSPI0", "SckPin", "PD6", "PAR_DSPIOWH", "2", "2", "Primary"),
    ("pin", "DSPI0", "SinPin", "PD5", "PAR_DSPIOWH", "6", "2", "Primary"),
    ("pin", "DSPI0", "SoutPin", "PD4", "PAR_DSPIOWH", "4", "2", "Primary"),
    ("pcs", "DSPI1", "0", "PF2", "PAR_SDHCH", "6", "2", "Alt2"),
    ("pcs", "DSPI1", "1", "PF0", "PAR_SDHCH", "2", "2", "Alt2"),
    ("pcs", "DSPI1", "2", "PF1", "PAR_SDHCH", "4", "2", "Alt2"),
    ("pin", "DSPI1", "SckPin", "PG5", "PAR_SDHCL", "0", "2", "Alt2"),
    ("pin", "DSPI1", "SinPin", "PG6", "PAR_SDHCL", "2", "2", "Alt2"),
    ("pin", "DSPI1", "SoutPin", "PG7", "PAR_SDHCH", "0", "2", "Alt2"),
    ("pcs", "DSPI2", "0", "PF5", "PAR_UART0", "4", "2", "Alt2"),
    ("pin", "DSPI2", "SckPin", "PF6", "PAR_UART0", "6", "2", "Alt2"),
    ("pin", "DSPI2", "SinPin", "PF4", "PAR_UART0", "2", "2", "Alt2"),
    ("pin", "DSPI2", "SoutPin", "PF3", "PAR_UART0", "0", "2", "Alt2"),
    ("pcs", "DSPI3", "0", "PE1", "PAR_UART1", "4", "2", "Alt2"),
    ("pin", "DSPI3", "SckPin", "PE2", "PAR_UART1", "6", "2", "Alt2"),
    ("pin", "DSPI3", "SinPin", "PE0", "PAR_UART1", "2", "2", "Alt2"),
    ("pin", "DSPI3", "SoutPin", "PF7", "PAR_UART1", "0", "2", "Alt2"),
}


def test_spi_pins_output_is_committed():
    assert (HAL / "src/spi/pins.rs").read_text() == render_spi()


def test_generated_spi_pins_are_table_15_2s_dspi_signals():
    assert set(SPI_ROW.findall(render_spi())) == RM_SPI


def test_the_boot_flash_chip_select_is_offered_though_gpio_leaves_it():
    # PC0 (SBF_CS) is reserved: GPIO never takes it, but DSPI0 may select it as PCS1
    assert "pcs!(DSPI0, 1, PC0, PAR_DSPIOWL, 7, 1, Primary);" in render_spi()
    assert re.search(r"impl_pin!\(PC0, 2, 0, None\);", render())


def test_spi_pins_are_gated_by_package():
    out = render_spi()
    assert f"pin!(#[cfg({P256})] DSPI1, SckPin, PG5," in out
    assert f"pcs!(#[cfg({P256})] DSPI0, 2, PC2," in out
    assert re.search(r"^pin!\(DSPI0, SckPin, PD6,", out, re.M)


def test_a_signal_with_a_boot_alias_is_matched_and_a_longer_name_is_not():
    parts_map = {"absent": {p: [] for p in halgen.PARTS}}
    pins = {"PA0": {"packages": ["196", "256"], "primary": "DSPI0_SCK/SBF_CK", "alt1": "DSPI0_SCKX", "alt2": None}}
    pinmux = {"registers": {"PAR_X": "0x48"}, "groups": [],
              "fields": [{"reg": "PAR_X", "field": "PAR_X", "pin": "PA0", "shift": 0, "width": 2, "signal": "DSPI0_SCK",
                          "verified": True}]}
    rows = SPI_ROW.findall(halgen.render_spi_pins(pins, parts_map, pinmux))
    assert rows == [("pin", "DSPI0", "SckPin", "PA0", "PAR_X", "0", "2", "Primary")]


def render_esdhc():
    return halgen.render_esdhc_pins(json.loads((HAL / "pins.json").read_text()),
                                    json.loads((ROOT / "pac-gen/parts_map.json").read_text()),
                                    json.loads((HAL / "pinmux.json").read_text()))


ESDHC_ROW = re.compile(r"pin!\((?:#\[[^\]]*\]\s*)?(\w+), (P[A-K]\d), (PAR_\w+), (\d), (\d), (\w+)\);")

# Table 15-2's SDHC_CLK/CMD/DAT0-3 pads with their PAR fields (pinmux.json)
RM_ESDHC = {
    ("ClkPin", "PD6", "PAR_DSPIOWH", "2", "2", "Alt2"), ("ClkPin", "PG5", "PAR_SDHCL", "0", "2", "Primary"),
    ("CmdPin", "PD5", "PAR_DSPIOWH", "6", "2", "Alt2"), ("CmdPin", "PG6", "PAR_SDHCL", "2", "2", "Primary"),
    ("Dat0Pin", "PD4", "PAR_DSPIOWH", "4", "2", "Alt2"), ("Dat0Pin", "PG7", "PAR_SDHCH", "0", "2", "Primary"),
    ("Dat1Pin", "PD0", "PAR_TIMER", "2", "2", "Alt2"), ("Dat1Pin", "PF0", "PAR_SDHCH", "2", "2", "Primary"),
    ("Dat2Pin", "PD1", "PAR_TIMER", "4", "2", "Alt2"), ("Dat2Pin", "PF1", "PAR_SDHCH", "4", "2", "Primary"),
    ("Dat3Pin", "PD7", "PAR_DSPIOWH", "0", "2", "Alt2"), ("Dat3Pin", "PF2", "PAR_SDHCH", "6", "2", "Primary"),
}


def test_esdhc_pins_output_is_committed():
    assert (HAL / "src/esdhc/pins.rs").read_text() == render_esdhc()


def test_generated_esdhc_pins_are_table_15_2s_sdhc_signals():
    assert set(ESDHC_ROW.findall(render_esdhc())) == RM_ESDHC


def test_the_digitakts_esdhc_pads_are_offered():
    # CLK PD6, CMD PG6, DAT0 PD4, DAT1 PD0, DAT2 PF1, DAT3 PD7 (a proven board's pad setup)
    rows = {(k, p) for k, p, *_ in ESDHC_ROW.findall(render_esdhc())}
    assert {("ClkPin", "PD6"), ("CmdPin", "PG6"), ("Dat0Pin", "PD4"), ("Dat1Pin", "PD0"), ("Dat2Pin", "PF1"),
            ("Dat3Pin", "PD7")} <= rows


def test_esdhc_pins_are_gated_by_package_and_dat4_7_left_out():
    out = render_esdhc()
    assert f"pin!(#[cfg({P256})] CmdPin, PG6," in out
    assert re.search(r"^pin!\(ClkPin, PD6,", out, re.M)
    assert "DAT4" not in out and "PG0" not in out


def test_uart_and_spi_tables_are_unchanged_by_instance_less_signals():
    assert (HAL / "src/uart/pins.rs").read_text() == render_uart()
    assert (HAL / "src/spi/pins.rs").read_text() == render_spi()


def render_ssi():
    return halgen.render_ssi_pins(json.loads((HAL / "pins.json").read_text()),
                                  json.loads((ROOT / "pac-gen/parts_map.json").read_text()),
                                  json.loads((HAL / "pinmux.json").read_text()))


def test_ssi_pins_output_is_committed():
    assert (HAL / "src/ssi/pins.rs").read_text() == render_ssi()


def test_ssi_tokens_and_pins():
    assert "impl_regs!(SSI0, ssi0::Ssi0);" in render()
    assert f"#[cfg({P256})] impl_regs!(SSI1, ssi0::Ssi0);" in render()          # SSI1 isn't on the MCF54410
    s = render_ssi()
    for pin, kind in (("PE6", "BclkPin"), ("PE5", "FsPin"), ("PE3", "TxdPin"), ("PE4", "RxdPin")):
        assert re.search(rf"pin!\((#\[[^\]]*\]\s*)?SSI1, {kind}, {pin}, PAR_UART2, \d, 2, Alt2\);", s), (pin, kind)
    for pin, kind in (("PH3", "BclkPin"), ("PH5", "FsPin"), ("PH6", "TxdPin"), ("PH7", "RxdPin"), ("PH4", "MclkPin")):
        assert re.search(rf"pin!\((#\[[^\]]*\]\s*)?SSI0, {kind}, {pin}, PAR_SSI0[HL], \d, 2, Primary\);", s), (pin, kind)
    assert re.search(r"clkin!\((#\[[^\]]*\]\s*)?PH4, PAR_SSI0H, 0, 2, Alt1\);", s)
