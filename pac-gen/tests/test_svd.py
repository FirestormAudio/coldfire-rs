import xml.etree.ElementTree as ET

import pytest

from rm2svd.model import Device, Field, Interrupt, Peripheral, Register
from rm2svd.svd import write_svd


def tiny():
    pcsr = Register("PCSR", 0x0, 16, "read-write", 0x0000,
                    [Field("EN", 0, 1, "read-write"), Field("PIF", 2, 1, "read-write", one_to_clear=True),
                     Field("PRE", 8, 4, "read-write", enum={"DIV1": 0, "DIV2": 1})])
    pit0 = Peripheral("PIT0", 0xFC08_0000, "PIT", [pcsr, Register("PMR", 0x2, 16, "read-write", 0xFFFF)])
    return Device("MCF5441x", [pit0, Peripheral("PIT1", 0xFC08_4000, "PIT", derived_from="PIT0")],
                  [Interrupt("PIT0_PIF", 192 + 13, "PIT0")], {"mcf54418": {"PIT": 4}})


def test_svd_has_the_registers_fields_and_interrupts():
    root = ET.fromstring(write_svd(tiny()))
    assert root.tag == "device" and root.findtext("name") == "MCF5441x"
    assert root.findtext("width") == "32" and root.findtext("addressUnitBits") == "8"
    p0, p1 = root.findall("peripherals/peripheral")
    assert p0.findtext("baseAddress") == "0xFC080000" and p1.get("derivedFrom") == "PIT0"
    r = p0.find("registers/register")
    assert (r.findtext("name"), r.findtext("addressOffset"), r.findtext("size"), r.findtext("resetValue")) == \
        ("PCSR", "0x0", "16", "0x0000")
    f = {x.findtext("name"): x for x in r.findall("fields/field")}
    assert f["PRE"].findtext("bitOffset") == "8" and f["PRE"].findtext("bitWidth") == "4"
    assert f["PIF"].findtext("modifiedWriteValues") == "oneToClear"
    assert [v.findtext("name") for v in f["PRE"].findall("enumeratedValues/enumeratedValue")] == ["DIV1", "DIV2"]
    i = p0.find("interrupt")
    assert (i.findtext("name"), i.findtext("value")) == ("PIT0_PIF", "205")


def test_svd_has_no_descriptions_beyond_names():
    # content rule: no manual prose; a description, if present, repeats its own element's name
    root = ET.fromstring(write_svd(tiny()))
    wrong = [(e.tag, e.findtext("name"), e.findtext("description")) for e in root.iter()
             if e.find("description") is not None and e.findtext("description") != e.findtext("name")]
    assert wrong == []


def test_interrupts_for_unknown_peripherals_are_an_error():
    dev = tiny()
    dev.interrupts.append(Interrupt("LOST", 5, "NOPE"))
    with pytest.raises(ValueError, match="LOST"):
        write_svd(dev)


def test_reset_values_are_written_at_the_registers_width():
    dev = tiny()
    p = dev.peripherals[0]
    p.registers += [Register("CNT8", 0x4, 8, "read-only", 0x5), Register("CNT32", 0x8, 32, "read-write", 0x12345),
                    Register("NORST", 0xC, 16, "write-only", None)]
    regs = {r.findtext("name"): r for r in ET.fromstring(write_svd(dev)).iter("register")}
    assert regs["CNT8"].findtext("resetValue") == "0x05" and regs["CNT32"].findtext("resetValue") == "0x00012345"
    assert regs["NORST"].find("resetValue") is None and regs["NORST"].findtext("access") == "write-only"


def test_a_reset_value_wider_than_its_register_is_an_error():
    dev = tiny()
    dev.peripherals[0].registers.append(Register("BIG", 0x4, 8, "read-write", 0x100))
    with pytest.raises(ValueError, match="BIG"):
        write_svd(dev)


def test_a_derived_peripheral_with_registers_of_its_own_is_an_error():
    dev = tiny()
    dev.peripherals[1].registers.append(Register("EXTRA", 0x4, 16, "read-write", 0))
    with pytest.raises(ValueError, match="PIT1"):
        write_svd(dev)


def test_derived_peripherals_keep_their_interrupts_and_empty_ones_get_a_block():
    dev = tiny()
    dev.interrupts.append(Interrupt("PIT1", 192 + 14, "PIT1"))
    dev.peripherals.append(Peripheral("EMPTY", 0xFC0F_0000, "EMPTY"))
    ps = {p.findtext("name"): p for p in ET.fromstring(write_svd(dev)).findall("peripherals/peripheral")}
    assert [(i.findtext("name"), i.findtext("value")) for i in ps["PIT1"].findall("interrupt")] == [("PIT1", "206")]
    assert ps["PIT1"].find("registers") is None and ps["PIT1"].find("addressBlock") is None
    assert ps["EMPTY"].findtext("addressBlock/size") == "0x4" and list(ps["EMPTY"].find("registers")) == []


def test_part_availability_goes_to_vendor_extensions():
    dev = tiny()
    dev.availability = {"mcf54418": {"PIT": 4, "UART": 10}, "mcf54410": {"PIT": 4, "UART": 6}}
    ext = ET.fromstring(write_svd(dev)).find("vendorExtensions")
    assert [(p.get("name"), [(m.get("group"), m.get("instances")) for m in p]) for p in ext] == [
        ("mcf54410", [("PIT", "4"), ("UART", "6")]), ("mcf54418", [("PIT", "4"), ("UART", "10")])]
