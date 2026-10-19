from verify import verify

SVD = """<?xml version="1.0"?><device><name>X</name><peripherals><peripheral><name>PIT0</name>
<baseAddress>0xFC080000</baseAddress><registers><register><name>PCSR</name><addressOffset>0x0</addressOffset>
<size>16</size></register></registers></peripheral></peripherals></device>"""


def test_verify_flags_missing_addresses_and_wrong_widths(tmp_path):
    svd = tmp_path / "x.svd"
    svd.write_text(SVD)
    o = tmp_path / "o.json"
    o.write_text('{"source": "t", "registers": [{"name": "PIT0_PCSR", "address": 4228382720, "width": 32},'
                 ' {"name": "PIT0_PMR", "address": 4228382722, "width": 16}]}')
    out = verify(svd, [o])
    assert any("width" in x and "PIT0_PCSR" in x for x in out)
    assert any("missing" in x and "PIT0_PMR" in x for x in out)


ARRAYS = """<?xml version="1.0"?><device><name>X</name><peripherals><peripheral><name>EDMA</name>
<baseAddress>0xFC044000</baseAddress><registers>
<register><dim>4</dim><dimIncrement>0x1</dimIncrement><name>DCHPRI%s</name><addressOffset>0x100</addressOffset>
<size>8</size></register>
<cluster><dim>64</dim><dimIncrement>0x20</dimIncrement><name>TCD%s</name><addressOffset>0x1000</addressOffset>
<register><name>CSR</name><addressOffset>0x1E</addressOffset><size>16</size></register></cluster>
</registers></peripheral>
<peripheral derivedFrom="EDMA"><name>EDMA2</name><baseAddress>0xFC144000</baseAddress></peripheral>
</peripherals></device>"""


def test_verify_expands_register_and_cluster_arrays_and_derived_peripherals(tmp_path):
    svd = tmp_path / "x.svd"
    svd.write_text(ARRAYS)
    o = tmp_path / "o.json"
    regs = [{"name": "DCHPRI3", "address": 0xFC044103, "width": 8},
            {"name": "TCD63_CSR", "address": 0xFC045000 + 32 * 63 + 0x1E, "width": 16},
            {"name": "EDMA2_TCD1_CSR", "address": 0xFC145000 + 32 + 0x1E, "width": 16}]
    o.write_text(__import__("json").dumps({"source": "t", "registers": regs}))
    assert verify(svd, [o]) == []


def test_verify_skips_entries_marked_excluded(tmp_path):
    svd = tmp_path / "x.svd"
    svd.write_text(SVD)
    o = tmp_path / "o.json"
    o.write_text('{"source": "t", "registers": [{"name": "PIT0_PCSR", "address": 4228382720, "width": 32, "excluded": true,'
                 ' "note": "excluded: width inferred from spacing"}]}')
    assert verify(svd, [o]) == []


FIELDS = """<?xml version="1.0"?><device><name>X</name><peripherals><peripheral><name>DSPI0</name>
<baseAddress>0xFC05C000</baseAddress><registers>
<register><name>CTAR1</name><addressOffset>0x10</addressOffset><size>32</size><fields>
<field><name>DBR</name><bitOffset>31</bitOffset><bitWidth>1</bitWidth></field>
<field><name>FMSZ</name><bitOffset>27</bitOffset><bitWidth>4</bitWidth></field>
<field><name>BR</name><bitOffset>0</bitOffset><bitWidth>4</bitWidth></field></fields></register>
<register><name>SR</name><addressOffset>0x2C</addressOffset><size>32</size><fields>
<field><name>EOQF</name><bitOffset>28</bitOffset><bitWidth>1</bitWidth></field>
<field><name>TCF</name><bitOffset>31</bitOffset><bitWidth>1</bitWidth></field>
<field><name>PCSn</name><bitOffset>16</bitOffset><bitWidth>8</bitWidth></field></fields></register>
</registers></peripheral></peripherals></device>"""


def bits_oracle(tmp_path, entries):
    o = tmp_path / "o.json"
    o.write_text(__import__("json").dumps({"source": "t", "registers": [], "bits": entries}))
    return o


def test_verify_checks_bits_against_fields(tmp_path):
    svd = tmp_path / "x.svd"
    svd.write_text(FIELDS)
    ok = [{"name": "SR_EOQF", "register": "SR", "address": 0xFC05C02C, "value": 1 << 28,
           "fields": [{"name": "EOQF", "mask": 1 << 28}]},
          {"name": "PUSHR_PCS1", "register": "SR", "address": 0xFC05C02C, "value": 1 << 17,
           "fields": [{"name": "PCS1", "within": True}]},                    # bit 1 of the PCS field
          {"name": "CTAR1_FLASH16", "register": "CTAR1", "address": 0xFC05C010, "value": 0xF800_0001,
           "fields": [{"name": "FMSZ", "value": 15}]},
          {"name": "SHIFT", "register": "CTAR1", "address": 0xFC05C010, "value": 1 << 27,
           "fields": [{"name": "FMSZ", "lsb": 27}]},
          {"name": "SKIPPED", "register": "X", "address": 0, "value": 1, "fields": [], "excluded": True, "note": "n"}]
    assert verify(svd, [bits_oracle(tmp_path, ok)]) == []
    bad = [{"name": "WRONG_BIT", "register": "SR", "address": 0xFC05C02C, "value": 1 << 27,
            "fields": [{"name": "EOQF", "mask": 1 << 27}]},
           {"name": "UNCOVERED", "register": "SR", "address": 0xFC05C02C, "value": 1 << 3, "fields": []},
           {"name": "FMSZ_VALUE", "register": "CTAR1", "address": 0xFC05C010, "value": 0xB800_0000,
            "fields": [{"name": "FMSZ", "value": 15}]},
           {"name": "NO_REG", "register": "X", "address": 0xFC05C000, "value": 1, "fields": []}]
    out = verify(svd, [bits_oracle(tmp_path, bad)])
    assert len(out) == 4 and all(x.startswith("t: bits ") for x in out), out
    assert any("WRONG_BIT" in x and "EOQF" in x for x in out)
    assert any("UNCOVERED" in x and "bit 3" in x for x in out)
    assert any("FMSZ_VALUE" in x and "FMSZ" in x for x in out)
    assert any("NO_REG" in x and "no register" in x for x in out)


def test_verify_catches_a_register_carrying_another_registers_fields(tmp_path):
    # C1: CTAR with SR's fields; digitakt's CTAR1 value names FMSZ
    svd = tmp_path / "x.svd"
    svd.write_text(FIELDS.replace("<name>FMSZ</name>", "<name>RXCTR</name>"))
    entry = [{"name": "dspi::CTAR1_FLASH16", "register": "dspi::DSPI0+CTAR1", "address": 0xFC05C010,
              "value": 0xF800_0001, "fields": [{"name": "FMSZ", "value": 15}]}]
    assert any("FMSZ" in x for x in verify(svd, [bits_oracle(tmp_path, entry)]))


def test_a_width_exclusion_still_checks_the_address(tmp_path):
    # an entry excluded for its width ("excluded": "width") must still find a register at its address
    svd = tmp_path / "x.svd"
    svd.write_text(SVD)
    o = tmp_path / "o.json"
    o.write_text('{"source": "t", "registers": ['
                 '{"name": "PIT0_PCSR", "address": 4228382720, "width": 32, "excluded": "width", "note": "n"},'
                 '{"name": "PIT0_GONE", "address": 4228382724, "width": 32, "excluded": "width", "note": "n"},'
                 '{"name": "NOT_A_REG", "address": 4228382728, "width": null, "excluded": true, "note": "n"}]}')
    assert verify(svd, [o]) == ["t: missing PIT0_GONE at 0xfc080004"]


ALTS = """<?xml version="1.0"?><device><name>X</name><peripherals><peripheral><name>UART0</name>
<baseAddress>0xFC060000</baseAddress><registers>
<register><name>UISR</name><addressOffset>0x14</addressOffset><size>8</size><fields>
<field><name>TXRDY</name><bitOffset>0</bitOffset><bitWidth>1</bitWidth></field></fields></register>
<register><name>UIMR</name><addressOffset>0x14</addressOffset><size>16</size><fields>
<field><name>FFULL</name><bitOffset>1</bitOffset><bitWidth>1</bitWidth></field>
<field><name>PCS2</name><bitOffset>2</bitOffset><bitWidth>1</bitWidth></field></fields></register>
</registers></peripheral></peripherals></device>"""


def test_alternates_at_one_address(tmp_path):
    svd = tmp_path / "x.svd"
    svd.write_text(ALTS)
    o = tmp_path / "o.json"
    # the width agrees with one of the registers there (not only the last one listed)
    o.write_text('{"source": "t", "registers": [{"name": "UART0_UISR", "address": 4228251668, "width": 8}]}')
    assert verify(svd, [o]) == []
    # a constant naming UIMR is checked against UIMR, not against UISR that happens to share the address
    entry = [{"name": "uart::UIMR_TXRDY", "register": "uart::UART0+UIMR", "address": 0xFC060014, "value": 1,
              "fields": [{"name": "TXRDY", "mask": 1}]}]
    out = verify(svd, [bits_oracle(tmp_path, entry)])
    assert len(out) == 1 and "UART0_UIMR" in out[0] and "no field TXRDY" in out[0], out


def test_field_index_digits_must_agree(tmp_path):
    svd = tmp_path / "x.svd"
    svd.write_text(ALTS)
    # PCS1 is not PCS2 (an indexed field PCSn would hold it; see test_verify_checks_bits_against_fields)
    entry = [{"name": "uart::PCS1", "register": "uart::UART0+UIMR", "address": 0xFC060014, "value": 1 << 2,
              "fields": [{"name": "PCS1", "within": True}]}]
    out = verify(svd, [bits_oracle(tmp_path, entry)])
    assert len(out) == 1 and "no field PCS1" in out[0], out


def test_summary_counts_exclusions_and_unplaced_bit_constants(tmp_path):
    from verify import summary
    o = tmp_path / "o.json"
    o.write_text(__import__("json").dumps({"source": "t", "registers": [
        {"name": "A", "address": 1, "width": 8, "excluded": "width", "note": "n"},
        {"name": "B", "address": 2, "width": 8, "excluded": True, "note": "n"}], "bits": [
        {"name": "x::K", "value": 1, "fields": [], "excluded": True, "note": "no register known for this constant"},
        {"name": "x::L", "value": 1, "fields": [], "excluded": True, "note": "a slot number"},
        {"name": "x::M", "register": "R", "address": 1, "value": 1, "fields": []}]}))
    assert summary([o]) == ("oracle entries excluded (see their notes): 4 (registers: 1, widths only: 1; bit "
                            "constants: 2, of which no register known: 1); bit constants checked: 1")


def test_only_a_trailing_index_marker_makes_a_field_indexed():
    from verify import _same_field
    assert _same_field("PCSn", "PCS1") and _same_field("EN_n_", "EN3") and _same_field("PCSn", "PCS12")
    assert not _same_field("PCSn", "PCS")                     # no index to hold
    assert not _same_field("RUN", "RU1")                      # an uppercase N is part of the name
    assert not _same_field("Run", "Ru1")                      # so is a lowercase n after a lowercase letter
    assert not _same_field("n", "1")                          # a marker needs a name before it
