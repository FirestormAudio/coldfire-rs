import xml.etree.ElementTree as ET
from pathlib import Path

from rm2svd.fields import label_is_name

SVD = Path(__file__).resolve().parents[2] / "mcf5441x-pac/svd/mcf5441x.svd"


def test_fields_never_overlap():
    for r in ET.parse(SVD).getroot().iter("register"):
        used = 0
        for f in r.iter("field"):
            m = ((1 << int(f.findtext("bitWidth"))) - 1) << int(f.findtext("bitOffset"))
            assert not used & m, (r.findtext("name"), f.findtext("name"))
            used |= m


def test_no_manual_prose():
    root = ET.parse(SVD).getroot()
    # a description, where there is one, is its own element's name (USBOTG.EPCR3 copied from EPCR2 says EPCR3)
    wrong = [(e.tag, e.findtext("name"), e.findtext("description")) for e in root.iter()
             if e.find("description") is not None and e.findtext("description") != e.findtext("name")]
    assert wrong == []
    # enum labels come from the manual's value descriptions: only name-like ones may stay
    prose = [v.findtext("name") for v in root.iter("enumeratedValue")
             if not label_is_name(v.findtext("name").replace("_", " "))]
    assert prose == []


def test_every_expected_module_is_there():
    names = {p.findtext("name") for p in ET.parse(SVD).getroot().iter("peripheral")}
    for want in [*(f"PIT{i}" for i in range(4)), *(f"UART{i}" for i in range(10)), *(f"DSPI{i}" for i in range(4)),
                 "INTC0", "INTC1", "INTC2", "EDMA", "ESDHC", "MACNET0", "MACNET1"]:
        assert want in names, want


def test_raw_svd_part_groups_are_short_keys():
    # M5: the vendorExtensions name Table 1-1 rows by parts_map.json keys, never by the manual's row labels
    import re
    ext = ET.parse(SVD.with_name("raw.svd")).getroot().find("vendorExtensions")
    groups = {m.get("group") for part in ext for m in part}
    assert groups and all(re.fullmatch(r"[A-Z][A-Z0-9_]*", g) for g in groups), sorted(groups)


def _peripheral(root, name):
    return next(p for p in root.iter("peripheral") if p.findtext("name") == name)


def _register(root, pname, rname):
    return next(r for r in _peripheral(root, pname).iter("register") if r.findtext("name") == rname)


# Registers whose read has a side effect (RM): a FIFO pop, or an interrupt or status flag cleared by the read.
READ_SIDE_EFFECTS = [("UART0", "URB"), ("DSPI0", "POPR"), ("OW", "ISR"), ("CCM", "UOCSR"), ("CCM", "UHCSR"),
                     ("RNG", "RNGOUT"), ("I2C0", "I2DR"), *(("ADC", f"RSLT{n}") for n in range(8)),
                     ("SSI0", "RX0"), ("SSI0", "RX1"), ("SIM", "RBUF0"), ("SIM", "RBUF1"), ("ESDHC", "DATPORT")]


def test_registers_with_read_side_effects_carry_read_action():
    root = ET.parse(SVD).getroot()
    missing = [f"{p}.{r}" for p, r in READ_SIDE_EFFECTS if next(_register(root, p, r).iter("readAction"), None) is None]
    assert missing == []
    # and nothing else claims one
    marked = {(p.findtext("name"), r.findtext("name")) for p in root.iter("peripheral") for r in p.iter("register")
              if next(r.iter("readAction"), None) is not None}
    assert marked == set(READ_SIDE_EFFECTS)


def test_register_names_have_no_module_prefix():
    root = ET.parse(SVD).getroot()
    names = {r.findtext("name") for r in _peripheral(root, "DSPI0").iter("register")}
    assert {"MCR", "SR", "PUSHR", "POPR", "CTAR0"} <= names and not any(n.startswith("DSPI_") for n in names)
    assert {"SERQ", "CR", "INTH"} <= {r.findtext("name") for r in _peripheral(root, "EDMA").iter("register")}


def test_dspi_ctar_has_its_own_fields():
    root = ET.parse(SVD).getroot()
    fs = {f.findtext("name"): (int(f.findtext("bitOffset")), int(f.findtext("bitWidth")))
          for f in _register(root, "DSPI0", "CTAR0").iter("field")}
    assert fs == {"DBR": (31, 1), "FMSZ": (27, 4), "CPOL": (26, 1), "CPHA": (25, 1), "LSBFE": (24, 1),
                  "PCSSCK": (22, 2), "PASC": (20, 2), "PDT": (18, 2), "PBR": (16, 2), "CSSCK": (12, 4),
                  "ASC": (8, 4), "DT": (4, 4), "BR": (0, 4)}


def test_interrupt_names_identify_their_peripheral():
    root = ET.parse(SVD).getroot()
    renamed = {"USBOTG": "USBOTG", "USBHOST": "USBHOST", "RNG": "RNG", "ESDHC": "ESDHC", "PIT0": "PIT0",
               "UART9": "UART9", "DTIM2": "DTIM2", "I2C3": "I2C3"}
    for p, want in renamed.items():
        assert want in [i.findtext("name") for i in _peripheral(root, p).findall("interrupt")], p
    names = [i.findtext("name") for i in root.iter("interrupt")]
    assert not {"USB_STS", "USB_STS_210", "EI"} & set(names)


def _w1c(root, p, r):
    return {f.findtext("name"): f.findtext("modifiedWriteValues") == "oneToClear"
            for f in _register(root, p, r).iter("field")}


def test_known_write_one_to_clear_flags():
    root = ET.parse(SVD).getroot()
    # RM ch. 31 figure 31-2: every ENET EIR event bit (30-15) is w1c; ch. 17's clearing column says "Write 1"
    eir = _w1c(root, "MACNET0", "EIR")
    assert len(eir) == 16 and all(eir.values()), eir
    # RM ch. 33 ERRSTAT: BOFFINT, ERRINT and the warning interrupt flags; the rest is read-only status
    errstat = _w1c(root, "FLEXCAN0", "ERRSTAT")
    assert {n for n, w in errstat.items() if w} == {"ERRINT", "BOFFINT", "RWRN_INT", "TWRN_INT"}
    # RM ch. 35 figure 35-15: SSI ISR's w1c cells are ROE1, ROE0, TUE1, TUE0; the bit row names the rest
    isr = _w1c(root, "SSI0", "ISR")
    assert {n for n, w in isr.items() if w} == {"ROE1", "ROE0", "TUE1", "TUE0"}
    assert {"TFE0", "TFE1", "RFF0", "RFF1", "RLS", "TLS", "RFS", "TFS", "RDR0", "RDR1", "TDE0", "TDE1"} <= set(isr)


def test_registers_and_fields_describe_themselves():
    # patch-added registers, clusters and fields (EDMA TCD SADDR, ESW MAC_LOOKUP LOW, FLEXCAN0 CANMCR, DSPI CTAR
    # DBR, …) carry a description too, and it is their own name
    root = ET.parse(SVD).getroot()
    wrong = [(e.tag, e.findtext("name"), e.findtext("description")) for e in root.iter()
             if e.tag in ("register", "cluster", "field") and e.findtext("description") != e.findtext("name")]
    assert wrong == []


def test_uart_umr2_has_its_own_fields_and_no_cut_off_labels():
    # UMR2 used to take UMR1's figure (digits ignored in figure names); its own table's TXRTS/TXCTS value lines were
    # cut off mid-word in conversion ("The transmitter has no e…"), so those enums are cleared in the patch
    root = ET.parse(SVD).getroot()
    umr2 = {f.findtext("name"): f for f in _register(root, "UART0", "UMR2").iter("field")}
    assert set(umr2) == {"SB", "TXCTS", "TXRTS", "CM"}
    assert umr2["TXRTS"].find("enumeratedValues") is None and umr2["TXCTS"].find("enumeratedValues") is None


# Exceptions where the manual really gives a field more access than its register: none.
ACCESS_EXCEPTIONS: set[tuple[str, str, str]] = set()


def test_no_field_has_more_access_than_its_register():
    # a field of a read-only register is not writable, a field of a write-only register is not readable
    more = {"read-only": {"read-write", "write-only"}, "write-only": {"read-write", "read-only"}}
    bad = [(p.findtext("name"), r.findtext("name"), f.findtext("name"))
           for p in ET.parse(SVD).getroot().iter("peripheral") for r in p.iter("register")
           for f in r.iter("field")
           if f.findtext("access") in more.get(r.findtext("access"), ())
           and (p.findtext("name"), r.findtext("name"), f.findtext("name")) not in ACCESS_EXCEPTIONS]
    assert bad == []


# Enums the label rule dropped from the generated SVD, restored by patch where the manual defines every value:
# (peripheral, register, field) → {name: value}. PLLMULT is not among them: the manual gives four fVCO factors
# for the 2-bit RCON field the 6-bit CCR[PLLMULT] decodes, not the 6-bit field's own values.
RESTORED_ENUMS = {
    **{("FLEXBUS", f"CSCR{n}", "EXTS"): {"ONE_CYCLE": 0, "UNTIL_CS": 1} for n in range(6)},
    **{("FLEXBUS", f"CSCR{n}", "ASET"): {"FIRST_EDGE": 0, "SECOND_EDGE": 1, "THIRD_EDGE": 2, "FOURTH_EDGE": 3}
       for n in range(6)},
    ("UART0", "UOP1", "RTS"): {"NO_EFFECT": 0, "ASSERT": 1},
    ("UART0", "UOP0", "RTS"): {"NO_EFFECT": 0, "NEGATE": 1},
    ("FLEXCAN0", "ERRSTAT", "ERRINT"): {"NO_ERROR": 0, "ERROR": 1},
}


def test_enums_the_label_rule_lost_are_restored_where_every_value_is_defined():
    root = ET.parse(SVD).getroot()
    for (p, r, f), want in RESTORED_ENUMS.items():
        field = next(x for x in _register(root, p, r).iter("field") if x.findtext("name") == f)
        got = {v.findtext("name"): int(v.findtext("value")) for v in field.iter("enumeratedValue")}
        assert got == want, (p, r, f)
    ccr = next(x for x in _register(root, "CCM", "CCR").iter("field") if x.findtext("name") == "PLLMULT")
    assert ccr.find("enumeratedValues") is None


def test_dtim_counter_is_writable():
    # RM ch. 39: DTCN is writable when DTXMR[EN1588] is clear, which outweighs the figure's blank W row
    root = ET.parse(SVD).getroot()
    reg = _register(root, "DTIM0", "DTCN")
    assert reg.findtext("access") == "read-write"
    assert [f.findtext("access") for f in reg.iter("field")] == ["read-write"]
