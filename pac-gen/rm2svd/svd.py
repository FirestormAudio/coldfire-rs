"""Model → CMSIS-SVD 1.3 text. Names only: descriptions repeat the name (no manual prose)."""
import xml.etree.ElementTree as ET

from .model import Device


def _sub(parent, tag, text):
    e = ET.SubElement(parent, tag)
    e.text = text
    return e


def write_svd(dev: Device) -> str:
    d = ET.Element("device", {"schemaVersion": "1.3"})
    for tag, text in (("vendor", "NXP"), ("name", dev.name), ("version", "1.0"), ("description", dev.name),
                      ("addressUnitBits", "8"), ("width", "32"), ("size", "32"), ("access", "read-write")):
        _sub(d, tag, text)
    ps = ET.SubElement(d, "peripherals")
    known = {p.name for p in dev.peripherals}
    unknown = [i for i in dev.interrupts if i.peripheral not in known]
    if unknown:
        raise ValueError("interrupts name unknown peripherals: " + ", ".join(
            f"{i.name} (value {i.value}, peripheral {i.peripheral})" for i in unknown))
    derived = [p.name for p in dev.peripherals if p.derived_from and p.registers]
    if derived:
        raise ValueError("derived peripherals with registers of their own (not written): " + ", ".join(derived))
    wide = [f"{p.name}.{r.name} (reset {r.reset:#x}, {r.width} bits)" for p in dev.peripherals for r in p.registers
            if r.reset is not None and not 0 <= r.reset < 1 << r.width]
    if wide:
        raise ValueError("reset values that don't fit their register: " + ", ".join(wide))
    irqs = {}
    for i in dev.interrupts:
        irqs.setdefault(i.peripheral, []).append(i)
    for p in dev.peripherals:
        pe = ET.SubElement(ps, "peripheral", {"derivedFrom": p.derived_from} if p.derived_from else {})
        _sub(pe, "name", p.name)
        _sub(pe, "groupName", p.group)
        _sub(pe, "baseAddress", f"0x{p.base:08X}")
        if not p.derived_from:
            span = max((r.offset + r.width // 8 for r in p.registers), default=4)
            ab = ET.SubElement(pe, "addressBlock")
            _sub(ab, "offset", "0x0")
            _sub(ab, "size", f"0x{span:X}")
            _sub(ab, "usage", "registers")
        for i in sorted(irqs.get(p.name, []), key=lambda i: i.value):
            ie = ET.SubElement(pe, "interrupt")
            _sub(ie, "name", i.name)
            _sub(ie, "value", str(i.value))
        if p.derived_from:
            continue
        rs = ET.SubElement(pe, "registers")
        for r in sorted(p.registers, key=lambda r: r.offset):
            re_ = ET.SubElement(rs, "register")
            _sub(re_, "name", r.name)
            _sub(re_, "description", r.name)
            _sub(re_, "addressOffset", f"0x{r.offset:X}")
            _sub(re_, "size", str(r.width))
            _sub(re_, "access", r.access)
            if r.reset is not None:
                _sub(re_, "resetValue", f"0x{r.reset:0{r.width // 4}X}")
            if r.fields:
                fs = ET.SubElement(re_, "fields")
                for f in sorted(r.fields, key=lambda f: f.lsb):
                    fe = ET.SubElement(fs, "field")
                    _sub(fe, "name", f.name)
                    _sub(fe, "description", f.name)
                    _sub(fe, "bitOffset", str(f.lsb))
                    _sub(fe, "bitWidth", str(f.width))
                    _sub(fe, "access", f.access)
                    if f.one_to_clear:
                        _sub(fe, "modifiedWriteValues", "oneToClear")
                    if f.enum:
                        ev = ET.SubElement(fe, "enumeratedValues")
                        for name, value in sorted(f.enum.items(), key=lambda kv: kv[1]):
                            v = ET.SubElement(ev, "enumeratedValue")
                            _sub(v, "name", name)
                            _sub(v, "value", str(value))
    ext = ET.SubElement(d, "vendorExtensions")
    for part, groups in sorted(dev.availability.items()):
        pe = ET.SubElement(ext, "part", {"name": part})
        for g, n in sorted(groups.items()):
            ET.SubElement(pe, "module", {"group": g, "instances": str(n)})
    ET.indent(d)
    return '<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(d, encoding="unicode") + "\n"
