"""Extract the register oracles from a digitakt checkout (run once; the JSON outputs are committed).

    python pac-gen/oracles/extract.py <digitakt>

- uboot-linux.json: module bases and registers from digitakt's re/peripherals/mcf54415.json (U-Boot and Linux
  headers); its byte `size` becomes `width` in bits.
- digitakt-mcf5441x.json: the address constants of digitakt's register crates (rust/mcf5441x/src/lib.rs and
  rust/mcf5441x-usb/src/regs.rs; the USB crate moved into embassy_mcf5441x::usb on 2026-10-18, so re-run this on a
  digitakt checkout from before that, e.g. its commit 821fd7b), with each register's width taken from the accessor (r8/w8, r16/w16,
  r32/w32) digitakt's drivers use on it, or null when none does; and under "bits" their bit constants (`PIF:
  u16 = 1 << 2`, `pre(p)`, `CTAR1_FLASH16` …), each with the register it belongs to (BIT_REGISTERS) and the
  field checks it supports: a mask named by the constant (`mask`), names listed in its doc comment (`OE | PE |
  FE` → `within`), `NAME = n` in its doc comment (`value`), a `_SHIFT` (`lsb`). A constant that is a field value
  or a command code is still checked for lying in fields, without a name (VALUE_CONSTS).

Entries judged wrong are kept and marked by EXCLUDE below (`"excluded": true` plus a `note`), never dropped; an
entry whose width alone is wrong (EXCLUDE_WIDTH) is `"excluded": "width"`, and verify still checks its address.
"""
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
CRATES = {"rust/mcf5441x/src/lib.rs": None, "rust/mcf5441x-usb/src/regs.rs": "usb"}
# Channel count of an indexed base (`const fn tcd(ch)`): the eDMA has 64 TCDs.
FN_COUNT = {"tcd": 64}
ABSOLUTE = 0x1000_0000

# (source, oracle name) -> reason. The reasons are ours; see the task-7 report for the evidence.
# EXCLUDE_WIDTH: only the width is wrong (`"excluded": "width"`: verify still checks the address);
# EXCLUDE: the entry isn't a register at all (`"excluded": true`: verify skips it).
_SPACING = "the header's width is inferred from address spacing; "
EXCLUDE_WIDTH: dict[tuple[str, str], str] = {
    **{("uboot-linux", f"GPIO::GPIO_{r}_K"): _SPACING + "RM ch. 15 register figure: port K is 8 bits like ports A-J"
       for r in ("PODR", "PDDR", "PPDSDR", "PCLRR")},
    ("uboot-linux", "GPIO::GPIO_PAR_DEBUGl"): _SPACING + "RM ch. 15 memory map: 8 bits",
    ("uboot-linux", "GPIO::GPIO_PAR_FEC"): _SPACING + "RM ch. 15 memory map: 8 bits",
    ("uboot-linux", "EPORT0::EPORT_EPFR"): _SPACING + "RM ch. 18 memory map: 8 bits",
    **{("uboot-linux", f"INTC{n}::INTC{n}_ICR0"): _SPACING + "ICRs are 8 bits (RM ch. 17 memory map; digitakt "
       "writes them with w8 on hardware); digitakt's oracle checks the address" for n in range(3)},
}
EXCLUDE: dict[tuple[str, str], str] = {
    ("uboot-linux", "NFC"): "not a register: 0xFC0FF000 lies in the NFC SRAM buffer (RM ch. 22 buffer map, "
                            "buffer 3); the NFC registers start at 0xFC0FFF00",
}

_CONST = re.compile(r"pub const (\w+): usize = ([^;]+);")
_FN = re.compile(r"pub const fn (\w+)\((\w+): usize\) -> usize \{ ([^}]+) \}")
_ACCESS = re.compile(r"\b([rw])(8|16|32)\(")


def _eval(expr: str, env: dict[str, int]) -> int | None:
    s = re.sub(r"\b[A-Za-z_]\w*\b", lambda m: str(env[m[0]]) if m[0] in env else m[0], expr)
    if re.search(r"\b[A-Za-z_]", s) or not re.fullmatch(r"[0-9a-fA-Fx_+* ()]+", s):
        return None     # an unresolved name, or more than + and * on literals
    return eval(s, {"__builtins__": {}})


def _modules(text: str, top: str | None):
    """Yield (module, const name, expression, kind) with kind 'const' or ('fn', arg) in source order."""
    stack: list[str] = []
    for line in text.splitlines():
        m = re.match(r"\s*pub mod (\w+) \{", line)
        if m:
            stack.append(m[1])
            continue
        if re.match(r"\s*\}\s*$", line) and stack:
            stack.pop()
            continue
        mod = stack[-1] if stack else top
        if mod is None:
            continue
        if (m := _FN.search(line)):
            yield mod, m[1], m[3], ("fn", m[2])
        elif (m := _CONST.search(line)):
            yield mod, m[1], m[2], "const"


def _first_arg(text: str, start: int) -> str:
    depth, i = 0, start
    while i < len(text):
        c = text[i]
        if c == "(":
            depth += 1
        elif c == ")":
            if depth == 0:
                break
            depth -= 1
        elif c == "," and depth == 0:
            break
        i += 1
    return text[start:i]


def digitakt(root: Path) -> dict:
    regs: list[dict] = []
    absolute: dict[tuple[str, str], int] = {}          # (module, name) -> address
    offsets: dict[str, list[tuple[str, int, str]]] = defaultdict(list)   # module -> (name, offset, group)
    bases: dict[str, list[tuple[str, int]]] = defaultdict(list)          # group -> (base name, address)
    for rel, top in CRATES.items():
        env: dict[str, int] = {}
        group: dict[str, str] = {}
        for mod, name, expr, kind in _modules((root / rel).read_text(), top):
            group.setdefault(mod, f"{mod}::consts")
            if kind != "const":
                b0, b1 = _eval(expr.replace(kind[1], "0"), env), _eval(expr.replace(kind[1], "1"), env)
                if b0 is not None and b0 >= ABSOLUTE:
                    group[mod] = f"{mod}::{name}"
                    bases[group[mod]] = [(f"{name.upper()}{n}", b0 + n * (b1 - b0)) for n in range(FN_COUNT[name])]
                continue
            v = _eval(expr, env)
            if v is None or name.endswith("_SOURCE"):
                continue
            env[name] = v
            if v >= ABSOLUTE and v & 0xFFF == 0 and re.fullmatch(r"0x[0-9A-Fa-f_]+", expr.strip()):
                bases[f"{mod}::consts"].append((name, v))
            elif v >= ABSOLUTE:
                absolute[(mod, name)] = v
            else:
                offsets[mod].append((name, v, group[mod]))
    widths = _widths(root, {m for m, _ in absolute} | set(offsets), {**{k: None for k in absolute},
                                                                     **{(m, n): None for m in offsets
                                                                        for n, _, _ in offsets[m]}})
    for (mod, name), a in absolute.items():
        regs.append({"name": f"{mod}::{name}", "address": a, "width": widths.get((mod, name))})
    for mod, offs in offsets.items():
        for name, off, grp in offs:
            for bname, b in bases[grp]:
                regs.append({"name": f"{mod}::{bname}+{name}", "address": b + off, "width": widths.get((mod, name))})
    for grp, bs in bases.items():
        for bname, b in bs:
            if grp.endswith("::consts"):
                regs.append({"name": f"{grp.split('::')[0]}::{bname}", "address": b, "width": None})
    regs.sort(key=lambda r: r["address"])
    return {"source": "digitakt", "registers": regs}


def _widths(root: Path, mods: set[str], known: dict) -> dict:
    """(module, register) -> access width, from every accessor call in digitakt's rust/ tree."""
    seen: dict[tuple[str, str], set[int]] = defaultdict(set)
    for f in sorted((root / "rust").rglob("*.rs")):
        if "/target/" in str(f):
            continue
        text = f.read_text()
        alias = {m: m for m in mods}
        for grp in re.findall(r"use mcf5441x::\{([^}]*)\}", text):
            for a, b in re.findall(r"\b(\w+) as (\w+)\b", grp):
                alias[b] = a
        glob = "usb" if "mcf5441x-usb" in str(f) and re.search(r"use (crate::)?regs::\*", text) else None
        for m in _ACCESS.finditer(text):
            arg, w = _first_arg(text, m.end()), int(m[2])
            for mod, name in re.findall(r"(?:\b(\w+)::)?\b([A-Z][A-Z0-9_]*)\b", arg):
                key = (alias.get(mod), name) if mod else (glob, name)
                if key in known:
                    seen[key].add(w)
    out = {}
    for k, ws in seen.items():
        if len(ws) > 1:
            raise SystemExit(f"{k}: accessed at widths {sorted(ws)}")
        out[k] = ws.pop()
    return out


# Bit constants → the register they belong to: (module, constant-name prefix) → that module's register constant
# ("" = every bit constant of the module). The eSDHC's bits are named without a register; ESDHC_BITS places them.
BIT_REGISTERS = {
    ("pit", ""): "PCSR",
    ("uart", "USR_"): "USR_UCSR", ("uart", "UCR_"): "UCR", ("uart", "UIMR_"): "UIMR",
    ("edma", "CSR_"): "tcd+CSR", ("edma", "ATTR_"): "tcd+ATTR", ("edma", "attr_"): "tcd+ATTR",
    ("dspi", "MCR_"): "MCR", ("dspi", "SR_"): "SR", ("dspi", "PUSHR_"): "PUSHR", ("dspi", "CTAR0_"): "CTAR0",
    ("dspi", "CTAR1_"): "CTAR1",
    ("ssi", "CR_"): "CR", ("ssi", "IER_"): "IER", ("ssi", "TCR_"): "TCR", ("ssi", "CCR_"): "CCR", ("ssi", "FCSR_"): "FCSR",
    ("usb::cmd", ""): "USBCMD", ("usb::sts", ""): "USBSTS", ("usb::portsc", ""): "PORTSC1",
    ("usb::epcr", ""): "EPCR0+4",          # EPCR1: EPCR0 has no RXR/TXR; digitakt uses the bits on EPCR1..3 too
    ("usb", "USBINTR_"): "USBINTR", ("usb", "USBMODE_"): "USBMODE", ("usb", "BURSTSIZE_"): "BURSTSIZE",
    ("usb", "SRCR_FEC_"): "SRCR_FEC",
}
ESDHC_BITS = {"IRQSTAT": "CC TC CTOE CCE CEBE CIE DTOE DCE DEBE AC12E DMAE ERR",
              "PRSSTAT": "CIHB CDIHB DLA BWEN BREN", "SYSCTL": "RSTA RSTC RSTD INITA", "PROCTL": "DTW_4BIT"}
# Constants that are a field's value or a command code, not a field named like the constant: checked for lying in
# fields only. (A constant whose bits aren't one contiguous run is treated so anyway.)
VALUE_CONSTS = {"UCR_RX_ENABLE", "UCR_RESET_ERRORS", "MCR_CLR_FIFOS", "CR_I2S_MASTER", "PUSHR_CTAR1", "DTW_4BIT",
                "PTS_ULPI", "MCR_MASTER", "ERR"}
# Bit constants that aren't register bits, or whose register the SVD can't check: kept, marked, with the reason.
BIT_EXCLUDE = {
    "clocks": "PPMCR slot numbers, not register bits",
    "usb::portsc::PSPD_HIGH": "a PSPD field value (used shifted by PSPD_SHIFT), not a register mask",
    "usb::SLOT_USB_OTG": "a PPMCR slot number, not register bits",
    "usb::SRCR_FEC_ULPI": "GPIO SRCR_FEC has no fields in the SVD (RM ch. 15's table rows aren't read)",
}

_BITCONST = re.compile(r"pub const (\w+): u(?:8|16|32) = ([^;]+);")
_SHIFTFN = re.compile(r"pub const fn (\w+)\(\w+: u(?:8|16|32)\) -> u(?:8|16|32) \{ \(\w+ & (0x[0-9A-Fa-f_]+|\d+)\) << (\d+) \}")
_DOC_LIST = re.compile(r"^([A-Z][A-Z0-9_]*(?:\s*(?:\||,)\s*[A-Z][A-Z0-9_]*)+)\b")
_DOC_VALUE = re.compile(r"\b([A-Z][A-Z0-9_]*) = (0x[0-9A-Fa-f]+|\d+)(?![\w-])")      # not `DTW = 4-bit`


def _eval_bits(expr: str, env: dict[str, int]) -> int | None:
    """`1 << 2`, `0x0010_0000`, `0b10`, `A | B` over earlier constants; None for anything else."""
    e = re.sub(r"\b[A-Za-z_]\w*\b", lambda m: str(env[m[0]]) if m[0] in env else m[0], expr.strip())
    if not re.fullmatch(r"[0-9a-fA-FxXbB_|<() ]+", e) or re.search(r"\b(?!0[xb])[A-Za-z]", e):
        return None
    return eval(e.replace("_", ""), {"__builtins__": {}})


def _run(v: int) -> bool:
    """One contiguous run of set bits."""
    return v > 0 and ((v >> (v & -v).bit_length() - 1) + 1) & (v >> (v & -v).bit_length() - 1) == 0


def _bit_consts(text: str, top: str | None):
    """Yield (module path, name, value, doc) for u8/u16/u32 constants and `(x & M) << S` const fns."""
    stack: list[str] = [top] if top else []
    env: dict[str, int] = {}
    doc: list[str] = []
    for line in text.splitlines():
        m = re.match(r"\s*pub mod (\w+) \{", line)
        if m:
            stack.append(m[1])
            doc = []
            continue
        if re.match(r"\s*\}\s*$", line) and stack and stack != [top]:
            stack.pop()
            continue
        d = re.match(r"\s*///\s?(.*)", line)
        if d:
            doc.append(d[1])
            continue
        mod = "::".join(stack)
        if (m := _BITCONST.search(line)):
            v = _eval_bits(m[2], env)
            if v is not None:
                env[m[1]] = v
                yield mod, m[1], v, " ".join(doc)
        elif (m := _SHIFTFN.search(line)):
            yield mod, m[1], int(m[2].replace("_", ""), 0) << int(m[3]), " ".join(doc)
        doc = []


def bits(root: Path) -> list[dict]:
    """digitakt's bit constants as field checks (see the module doc)."""
    regs = {e["name"]: e["address"] for e in digitakt(root)["registers"]}
    out = []
    for rel, top in CRATES.items():
        consts = list(_bit_consts((root / rel).read_text(), top))
        values = {(m, n): v for m, n, v, _ in consts}
        for mod, name, value, doc in consts:
            full = f"{mod}::{name}"
            entry = {"name": full}
            why = BIT_EXCLUDE.get(full) or BIT_EXCLUDE.get(mod)
            reg = next((r for (m, pre), r in BIT_REGISTERS.items() if m == mod and name.startswith(pre)
                        and (pre or not any(m2 == mod and p2 and name.startswith(p2) for m2, p2 in BIT_REGISTERS))),
                       None)
            if mod == "esdhc":
                reg = next((r for r, ns in ESDHC_BITS.items() if name in ns.split()), None)
            if why or reg is None:
                out.append({**entry, "register": reg, "address": None, "value": value, "fields": [],
                            "excluded": True, "note": why or "no register known for this constant"})
                continue
            address, rname = _bit_address(mod, reg, regs)
            entry.update(register=rname, address=address, value=value, fields=[])
            if name.endswith("_SHIFT"):
                entry["value"] = 1 << value
                entry["fields"] = [{"name": name[:-len("_SHIFT")], "lsb": value}]
            elif _DOC_VALUE.search(doc) and not _DOC_LIST.match(doc):
                entry["fields"] = [{"name": n, "value": int(v, 0)} for n, v in _DOC_VALUE.findall(doc)]
            elif _DOC_LIST.match(doc):
                entry["fields"] = [{"name": n, "within": True} for n in re.split(r"\s*[|,]\s*", _DOC_LIST.match(doc)[1])]
                entry["fields"] += [{"name": n, "value": int(v, 0)} for n, v in _DOC_VALUE.findall(doc)]
            elif name not in VALUE_CONSTS and _run(value):
                field = name.split("_", 1)[1] if (reg != "PCSR" and mod != "esdhc" and "::" not in mod
                                                  and "_" in name) else name
                field = re.sub(r"_MASK$", "", re.sub(r"^attr_", "", field)).upper()
                # `PCS1`: one bit of the PCS field; otherwise the constant is the whole field
                entry["fields"] = [{"name": field, "within": True} if field[-1].isdigit() else
                                   {"name": field, "mask": value}]
            out.append(entry)
    return out


def _bit_address(mod: str, reg: str, regs: dict[str, int]) -> tuple[int, str]:
    """The address of `reg` in module `mod`, from the register oracle's names (`pit::PIT0+PCSR`, `usb::USBCMD`)."""
    top = mod.split("::")[0]
    if reg.startswith("tcd+"):
        name = f"{top}::TCD0+{reg[4:]}"
    elif reg.endswith("+4"):
        name = f"{top}::{reg[:-2]}"
        return regs[name] + 4, name + "+4"
    elif f"{top}::{reg}" in regs:
        name = f"{top}::{reg}"
    else:
        name = next(n for n in regs if n.startswith(f"{top}::") and n.endswith(f"+{reg}"))
    return regs[name], name


def uboot_linux(root: Path) -> dict:
    d = json.loads((root / "re/peripherals/mcf54415.json").read_text())
    regs = []
    for m in d["modules"]:
        b = int(m["base"], 16)
        regs.append({"name": m["name"], "address": b, "width": None})
        for r in m["registers"]:
            regs.append({"name": f"{m['name']}::{r['name']}", "address": b + int(r["offset"], 16),
                         "width": 8 * r["size"] if r.get("size") else None})
    regs.sort(key=lambda r: r["address"])
    return {"source": "uboot-linux", "registers": regs}


def _mark(data: dict) -> dict:
    for r in data["registers"]:
        key = (data["source"], r["name"])
        if key in EXCLUDE:
            r["excluded"], r["note"] = True, EXCLUDE[key]
        elif key in EXCLUDE_WIDTH:
            r["excluded"], r["note"] = "width", EXCLUDE_WIDTH[key]
    return data


def _dump(data: dict, path: Path):
    lines = ",\n".join("  " + json.dumps(r) for r in data["registers"])
    bits = ""
    if "bits" in data:
        bits = ', "bits": [\n' + ",\n".join("  " + json.dumps(b) for b in data["bits"]) + "\n]"
    path.write_text(f'{{"source": {json.dumps(data["source"])}, "registers": [\n{lines}\n]{bits}}}\n')


def check_source(dt: Path) -> None:
    """Refuse a digitakt checkout that lacks one of CRATES, with the reason."""
    missing = [rel for rel in CRATES if not (dt / rel).exists()]
    if missing:
        raise SystemExit(f"{dt}: no {', '.join(missing)}: use a digitakt checkout from before the USB driver moved "
                         "into embassy_mcf5441x::usb (2026-10-18), e.g. its commit 821fd7b")


if __name__ == "__main__":
    dt = Path(sys.argv[1])
    check_source(dt)
    for fn, out in ((uboot_linux, "uboot-linux.json"), (digitakt, "digitakt-mcf5441x.json")):
        data = _mark(fn(dt))
        if out.startswith("digitakt"):
            data["bits"] = bits(dt)
        _dump(data, HERE / out)
        print(f"{out}: {len(data['registers'])} entries, {sum('excluded' in r for r in data['registers'])} excluded"
              + (f"; {len(data['bits'])} bit constants, {sum('excluded' in b for b in data['bits'])} excluded"
                 if "bits" in data else ""))
