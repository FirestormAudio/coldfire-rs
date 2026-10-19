"""chiptool output → src/lib.rs: drop the Cortex-M interrupt glue, add the part-choice error and the per-part
cfg gates on instance constants and on the Interrupt variants those instances raise.
Usage: postgen.py <chiptool lib.rs> <raw.svd> <parts_map.json> <out lib.rs> <mcf5441x.svd>

The gates are parts_map.json's `absent` lists (RM ch. 2 for the MCF54410, Table 1-1 otherwise). They are checked
against Table 1-1 as raw.svd's vendorExtensions carry it: a dash means every instance of the group is absent; a
group whose row counts instances must leave that many present, unless parts_map.json's `silent` says why not."""
import json
import re
import sys
import xml.etree.ElementTree as ET

PARTS = ["mcf54410", "mcf54415", "mcf54416", "mcf54417", "mcf54418"]


def availability(svd: str) -> dict[str, dict[str, int]]:
    ext = ET.parse(svd).getroot().find("vendorExtensions")
    out = {p.get("name"): {m.get("group"): int(m.get("instances")) for m in p} for p in ext}
    assert sorted(out) == PARTS, sorted(out)
    return out


def interrupt_peripherals(svd: str) -> dict[str, list[str]]:
    """Interrupt name → every peripheral the SVD lists it under."""
    root = ET.parse(svd).getroot()
    out: dict[str, list[str]] = {}
    for p in root.iter("peripheral"):
        for i in p.findall("interrupt"):
            out.setdefault(i.findtext("name"), []).append(p.findtext("name"))
    return out


def cut(src: str, pattern: str, what: str) -> str:
    out, n = re.subn(pattern, "", src, count=1, flags=re.S | re.M)
    assert n == 1, f"chiptool output no longer has {what}"
    return out


def group_of(name: str, groups: dict[str, dict]) -> str | None:
    prefix = re.sub(r"\d+$", "", name)
    return next((g for g, spec in groups.items() if prefix in spec["peripherals"] or name in spec["peripherals"]),
                None)


def check(consts: list[str], pm: dict, avail: dict[str, dict[str, int]]) -> None:
    """The absent lists agree with Table 1-1, or parts_map.json's `silent` says why not."""
    groups, absent, silent = pm["groups"], pm["absent"], pm.get("silent", {})
    if set(groups) != set(avail[PARTS[0]]):
        raise SystemExit(f"parts_map.json groups differ from raw.svd's: {sorted(set(groups) ^ set(avail[PARTS[0]]))}")
    for p in PARTS:
        bad = [n for n in absent[p] if n not in consts]
        if bad:
            raise SystemExit(f"parts_map.json: {p} lists {', '.join(bad)} absent, but no such instance exists")
    for g, spec in groups.items():
        insts = [c for c in consts if group_of(c, groups) == g]
        for p in PARTS:
            n, have = avail[p][g], [c for c in insts if c not in absent[p]]
            why = silent.get(p, {}).get(g)
            if n == 0 and have:
                raise SystemExit(f"parts_map.json: Table 1-1 has no {g} on {p}, but {', '.join(have)} stay present")
            if n and insts and not have:
                raise SystemExit(f"parts_map.json: Table 1-1 has {g} on {p}, but every instance is listed absent")
            if spec.get("instances") and n and len(have) != n and not why:
                raise SystemExit(f"parts_map.json: {p} has {len(have)} {g} instances present, Table 1-1 counts {n}")
            if why and len(have) == n:
                raise SystemExit(f"parts_map.json: silent {p} {g} no longer needed (counts agree)")


_READ = "unsafe { (self.ptr as *mut T).read_volatile() }"
_WRITE = "unsafe { (self.ptr as *mut T).write_volatile(val) }"
_READ_MMIO = """unsafe {
            let a = self.ptr as usize;
            match core::mem::size_of::<T>() {
                4 => core::mem::transmute_copy(&coldfire::mmio::r32(a)),
                2 => core::mem::transmute_copy(&coldfire::mmio::r16(a)),
                1 => core::mem::transmute_copy(&coldfire::mmio::r8(a)),
                _ => panic!("register width"),
            }
        }"""
_WRITE_MMIO = """unsafe {
            let a = self.ptr as usize;
            match core::mem::size_of::<T>() {
                4 => coldfire::mmio::w32(a, core::mem::transmute_copy(&val)),
                2 => coldfire::mmio::w16(a, core::mem::transmute_copy(&val)),
                1 => coldfire::mmio::w8(a, core::mem::transmute_copy(&val)),
                _ => panic!("register width"),
            }
        }"""


def route_mmio(src: str) -> str:
    """Register reads go through coldfire::mmio's width-fixed accesses (asm 16/32-bit moves on ColdFire); writes go
    through coldfire::mmio's volatile writes (not asm). Either way the access width is fixed, so GCC can't narrow a masked read of a 32-bit register into a byte btst of its memory operand."""
    assert src.count(_READ) == 1 and src.count(_WRITE) == 1, "chiptool's Reg::read/write_value changed shape"
    return src.replace(_READ, _READ_MMIO).replace(_WRITE, _WRITE_MMIO)


# `#[interrupt]` through the PAC (feature rt, ColdFire only): `use mcf5441x_pac::interrupt; #[interrupt] fn PIT0() {}`
# finds both the attribute (macro namespace) and the vector, `interrupt::PIT0` (type namespace).
RT_EXPORTS = """
#[cfg(all(feature = "rt", target_arch = "m68k"))]
pub use coldfire_rt::interrupt;
#[cfg(all(feature = "rt", target_arch = "m68k"))]
pub use Interrupt as interrupt;
"""


def main(lib: str, svd: str, pmap: str, out: str, patched: str) -> None:
    src = open(lib).read()
    pm = json.load(open(pmap))
    consts = re.findall(r"^pub const ([A-Z][A-Z0-9_]*): ", src, flags=re.M)
    check(consts, pm, availability(svd))

    src = cut(src, r"^unsafe impl cortex_m::interrupt::InterruptNumber for Interrupt \{.*?^\}\n", "the cortex_m impl")
    src = cut(src, r'^#\[cfg\(feature = "rt"\)\]\nmod _vectors \{.*?^\}\n', "the _vectors module")
    src = cut(src, r'^#\[cfg\(feature = "rt"\)\]\npub use cortex_m_rt::interrupt;\n', "cortex_m_rt::interrupt")
    src = cut(src, r'^#\[cfg\(feature = "rt"\)\]\npub use Interrupt as interrupt;\n', "the interrupt alias")

    def cfg(instances: str | list[str], indent: str = "") -> str:
        """The gate for something that exists where any of its instances does."""
        names = [instances] if isinstance(instances, str) else instances
        have = [p for p in PARTS if any(n not in pm["absent"][p] for n in names)]
        if len(have) == len(PARTS):
            return ""
        return indent + "#[cfg(any(" + ", ".join(f'feature = "{p}"' for p in have) + "))]\n"

    src, n = re.subn(r"^(pub const ([A-Z][A-Z0-9_]*): )", lambda m: cfg(m.group(2)) + m.group(1), src, flags=re.M)
    assert n > 40, n

    # an Interrupt variant is there on the parts that have the peripheral raising it
    owners = interrupt_peripherals(patched)
    enum = re.search(r"^pub enum Interrupt \{\n(.*?)^\}\n", src, flags=re.S | re.M)
    assert enum, "chiptool output no longer has the Interrupt enum"
    variants = re.findall(r"^    ([A-Za-z_][A-Za-z0-9_]*) = \d+,$", enum.group(1), flags=re.M)
    orphans = [v for v in variants if v not in owners]
    if orphans:
        raise SystemExit(f"Interrupt variants no SVD peripheral raises: {', '.join(orphans)}")
    body = re.sub(r"^(    #\[doc = [^\n]*\n)?(    ([A-Za-z_][A-Za-z0-9_]*) = \d+,)$",
                  lambda m: cfg(owners[m.group(3)], "    ") + (m.group(1) or "") + m.group(2), enum.group(1), flags=re.M)
    src = src[:enum.start(1)] + body + src[enum.end(1):]

    err = ", ".join(PARTS)
    anyf = ", ".join(f'feature = "{p}"' for p in PARTS)
    prelude = (f'#[cfg(not(any({anyf})))]\n'
               f'compile_error!("choose the part: one of the features {err}");\n')
    src, n = re.subn(r"^(#!\[no_std\]\n)", lambda m: m.group(1) + prelude, src, count=1, flags=re.M)
    assert n == 1
    src = route_mmio(src)
    src += RT_EXPORTS
    open(out, "w").write(src)


if __name__ == "__main__":
    main(*sys.argv[1:6])
