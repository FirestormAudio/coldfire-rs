"""The whole manual → Device: peripherals from every chapter's memory map and register figures, interrupts (ch. 17)
mapped onto them, part availability (ch. 1). Everything that couldn't be read is a problem line `<kind>: …`."""
import copy
import json
import re
from dataclasses import dataclass
from pathlib import Path

from .access import settle_access
from .fields import diagram_has_fields, field_table_title, fields_after, title_names
from .interrupts import interrupts_with_problems
from .model import Device, Field, Interrupt, Peripheral, Register
from .parts import availability_with_problems, keyed
from .registers import Figure, MapRow, figures, instance_bases, instance_headers, memory_map_rows_with_problems

PARTS_MAP = Path(__file__).resolve().parents[1] / "parts_map.json"   # Table 1-1 row → short group key
BLOCK = 0x3FFF                                   # the chip's modules sit in 16 KB blocks (RM ch. 1 memory map)
# Chapter titles the naming rule can't use: both USB chapters' first word is "Universal".
NAME_OVERRIDES = {23: "USBHOST", 24: "USBOTG"}
# Ch. 17 module labels that aren't a peripheral name (used only when the flag's register doesn't decide).
LABEL_ALIASES = {"DMA": "EDMA", "SDHC": "ESDHC", "FLASH": "NFC", "ROBUSTRTC": "RTC", "SDRAM": "DDRMC",
                 "SWITCH": "ETHERNET", "L2ETHERNET": "ETHERNET", "PWM": "MCPWM", "MC": "MCPWM",
                 "1WIRE": "OW", "PLL": "CLOCK"}
# The name of a block several chapters share, where the largest chapter's name isn't the block's (POWER and SCM
# tie at 0xFC04_0000; the SCM chapter owns the block).
SHARED_BLOCK_NAMES = {0xFC040000: "SCM"}
_IDENT = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
# Register-name prefixes of modules whose (raw) group name differs from them: DDRMC's registers are DDR_CR00, ….
MODULE_PREFIXES = {"DDRMC": "DDR", "MACNET": "ENET", "MCPWM": "PWM", "ETHERNET": "ESW", "CLOCK": "PLL",
                   "ROBUST": "RTC"}


def strip_prefixes(peripherals: list[Peripheral], problems: list[str]) -> None:
    """Register names without their module prefix (DSPI_MCR → MCR, PWM_SM0CR → SM0CR), so every peripheral's
    registers read alike (DSPI0.mcr(), PIT0.pcsr()). The prefix is the group name, or MODULE_PREFIXES's, with an
    instance digit or not (ENET1_RACC). A name the stripped one would collide with keeps its prefix (reported)."""
    for p in peripherals:
        prefixes = {p.group, MODULE_PREFIXES.get(p.group, p.group)}
        new = {}
        for r in p.registers:
            m = re.match(r"([A-Za-z]+)\d*_(.+)$", r.name)
            new[r.name] = m.group(2) if m and m.group(1) in prefixes and _IDENT.fullmatch(m.group(2)) \
                and not m.group(2)[0].isdigit() else r.name
        taken = {r.name for r in p.registers if new[r.name] == r.name}
        for r in p.registers:
            n = new[r.name]
            if n == r.name:
                continue
            if n in taken:
                problems.append(f"prefix kept: {p.name}.{r.name} ({n} exists)")
                continue
            taken.add(n)
            r.name = n


def chapter_name(title: str, num: int) -> str | None:
    """`(eDMA)` → EDMA, `(PIT0–PIT3)` → PIT; no acronym → the first word; None if that isn't an identifier."""
    if num in NAME_OVERRIDES:
        return NAME_OVERRIDES[num]
    t = re.sub(r"^#\s*Chapter\s+\d+:\s*", "", title.strip())
    m = re.search(r"\(([^()]*)\)\s*$", t)
    if m:
        name = re.sub(r"\d+$", "", re.split(r"[–,\s-]", m.group(1).strip())[0])
    else:
        name = re.sub(r"[^A-Za-z0-9]+", "_", t.split()[0]).strip("_") if t.split() else ""
    name = name.upper()
    return name if _IDENT.fullmatch(name) else None


def _index(name: str) -> str | None:
    m = re.search(r"(\d+)$", name)
    return m.group(1) if m else None


def _reg_name(row: MapRow, digit: str | None) -> str:
    """Register name: instance markers (`{x}`, and `{n}` outside arrays) removed; for a row listing one instance
    of several (`IPRH1` in INTC1), that instance's digit removed too."""
    t = row.template.replace("{x}", "").replace("{n}", "")
    if digit is not None:
        idx = row.index or ""
        if idx and t.endswith(digit + idx) and len(t) > len(digit + idx):
            t = t[:-len(digit + idx)] + idx
        elif t.endswith(digit) and len(t) > len(digit):
            t = t[:-len(digit)]
    return re.sub(r"_+", "_", t).strip("_")


def _same_name(a: str, b: str, digit: str | None = None) -> bool:
    """Whether a figure's name `a` and a register name `b` name one register: equal without underscores and
    case, or equal once the instance's own number `digit` is left out of the figure's name (`DAC0_CR` as DAC_CR
    in DAC0, `UMR10` as UMR1 in UART0). Other digits count: UMR1 and UMR2 differ."""
    ka, kb = a.replace("_", "").upper(), b.replace("_", "").upper()
    if ka == kb:
        return True
    return digit is not None and any(ka[:i] + ka[i + len(digit):] == kb
                                     for i in range(len(ka)) if ka.startswith(digit, i))


@dataclass
class _Reg:
    reg: Register
    address: int
    chapter: int
    section: int | None
    stem: str = ""


def _blocks(name: str, absrows: list[MapRow], bases: list[tuple[str, int]], text: str) -> dict[int, str]:
    """A chapter's instances: 16 KB block → peripheral name, from its base-address table, else its memory map's
    instance header cells (or `name0`, `name1`, … for a map with several addresses per row), and `name` (or
    numbered names) for the blocks left over."""
    multi = max(len(r.addresses) for r in absrows)
    blocks: dict[int, str] = {}
    if bases:
        for n, b in bases:
            blocks.setdefault(b & ~BLOCK, n)
    elif multi > 1:
        headers = instance_headers(text)
        full = [r for r in absrows if len(r.addresses) == multi]
        names = headers if len(headers) == multi else [f"{name}{k}" for k in range(multi)]
        for k in range(multi):
            blocks.setdefault(min(r.addresses[k] for r in full) & ~BLOCK, names[k])
    leftovers = sorted({a & ~BLOCK for r in absrows for a in r.addresses} - set(blocks))
    for k, b in enumerate(leftovers):
        blocks[b] = name if len(leftovers) == 1 and name not in blocks.values() else f"{name}{k}"
    return blocks


def _chapter(stem: str, num: int, text: str, problems: list[str]) -> list[tuple[Peripheral, list[_Reg]]]:
    rows, ps = memory_map_rows_with_problems(text)
    problems += [f"{p.split(':')[0]}: {stem} {p.split(':', 1)[1].strip()}" for p in ps]
    figs = figures(text)
    name = chapter_name(text.splitlines()[0] if text else "", num)
    bases = instance_bases(text)
    absrows = [r for r in rows if not r.relative]
    relrows = [r for r in rows if r.relative]
    if relrows and not absrows:
        if not bases:
            problems.append(f"not memory-mapped: {stem} {len(relrows)} rows with offsets/BDM codes and no base address")
            return []
        # offset rows repeat for every base address
        absrows = [MapRow(r.abbrev, [b + r.addresses[0] for _, b in bases], r.width, r.access, r.reset,
                          r.template, False, r.array, r.section, r.index) for r in rows]
    if not absrows:
        if not relrows and re.search(r"\*\*Table [\d-]+\..*Memory Map\*\*", text):
            problems.append(f"not memory-mapped: {stem} memory map has no absolute addresses")
        return []
    if name is None:
        prefixes = {re.split(r"[_{]", r.template)[0] for r in absrows}
        name = prefixes.pop().upper() if len(prefixes) == 1 else re.sub(r"\W", "", stem.split("-", 1)[1]).upper()
        problems.append(f"name: {stem} title gives no identifier; named {name} from its registers")

    blocks = _blocks(name, absrows, bases, text)
    several = len(blocks) > 1
    periph = {b: Peripheral(n, b, re.sub(r"\d+$", "", n) if several or _index(n) and n != name else n)
              for b, n in sorted(blocks.items(), key=lambda kv: kv[1])}
    regs: dict[int, list[_Reg]] = {b: [] for b in periph}

    by_addr: dict[int, list[tuple[Figure, str]]] = {}
    for f in figs:
        for a, n in f.addresses:
            by_addr.setdefault(a, []).append((f, n))

    def inst(a: int) -> str | None:
        """The instance number of the peripheral holding address a (None for a single-instance chapter)."""
        p = periph.get(a & ~BLOCK)
        return _index(p.name) if p is not None and several else None

    def candidates(addrs: list[int], rname: str) -> tuple[list[Figure], list[Figure]]:
        """Figures of this register (its name, at any of its addresses), and figures at its own address under
        another name (a misspelling, or an alternate). A figure can be listed twice (a plain `Address:` line
        repeated above the picture): every one is a candidate."""
        same, other = [], []
        for x in addrs:
            for f, n in by_addr.get(x, []):
                if _same_name(n, rname, inst(x)) and f not in same:
                    same.append(f)
        for f, n in by_addr.get(addrs[0], []):
            if not _same_name(n, rname, inst(addrs[0])) and f not in same and f not in other:
                other.append(f)
        return same, other

    def table_fields(fig: Figure, width: int, access: str, rname: str, report: bool) -> list[Field]:
        """The fields of a figure's table; [] (reported) when its title names another register and the figure's
        diagram doesn't show them."""
        unread: list[str] = []
        out = fields_after(fig.text_after, width, fig.access or access, unread)
        if report:
            problems.extend(f"{u.split(': ', 1)[0]}: {stem} {rname} {u.split(': ', 1)[1]}" for u in unread)
        title = field_table_title(fig.text_after) if out else None
        if title is not None and not title_names(title, rname):
            diagram = re.split(r"\*\*Figure [\d-]+|Field Descriptions?", fig.text_after)[0]
            kept = diagram_has_fields(diagram, [f.name for f in out])
            if report:
                problems.append(f"field table names another register: {stem} {rname} {fig.addresses[0][0]:#010x} "
                                f"table {title!r}; " + ("kept, the figure shows its fields" if kept else "left out"))
            return out if kept else []
        return out

    def with_fields(reg: Register, fig: Figure, fs: list[Field], pname: str) -> None:
        fields, seen = [], 0
        for f in fs:
            mask = ((1 << f.width) - 1) << f.lsb
            if mask & seen:
                problems.append(f"overlap: {stem} {pname}.{reg.name}.{f.name} bits {f.lsb}+{f.width}")
                continue
            seen |= mask
            f.one_to_clear = f.one_to_clear or bool(mask & sum(1 << b for b in fig.one_to_clear_bits))
            fields.append(f)
        reg.fields = sorted(fields, key=lambda f: f.lsb)

    by_way: dict[int, dict[str, set[str]]] = {}              # address → access → register names there
    for r in absrows:
        for a in r.addresses:
            by_way.setdefault(a, {}).setdefault(r.access, set()).add(r.template)
    paired = {a for a, w in by_way.items() if w.get("read-only") and w.get("write-only")
              and w["read-only"] != w["write-only"]}     # a read and a write register sharing an address (UISR/UIMR)
    claimed: dict[int, list[str]] = {}                      # address → register names there
    taken: set[int] = set()                                 # figures whose fields went to a differently named row
    for r in absrows:
        for a in r.addresses:
            b = a & ~BLOCK
            p = periph[b]
            digit = _index(p.name) if several and len(r.addresses) == 1 else None
            rname = _reg_name(r, digit)
            if not _IDENT.fullmatch(rname or "-"):
                problems.append(f"name: {stem} register {r.template!r} at {a:#010x} is not an identifier")
                continue
            first = a == r.addresses[0]
            same, other = candidates([a, *r.addresses], rname)
            fig = same[0] if same else None
            width = r.width or next((f.width for f in same if f.width), None)
            if width is None:
                width = 32
                problems.append(f"no width: {stem} {rname} {a:#010x} (32 assumed)")
            if width not in (8, 16, 32):
                if first:
                    problems.append(f"width: {stem} {rname} {a:#010x} is {width} bits; left out")
                continue
            reg = Register(rname, a - b, width, r.access, r.reset)
            fs: list[Field] = []
            for f in same:                                  # the first of its figures that has a field table
                fs = table_fields(f, width, r.access, rname, first)
                if fs:
                    fig = f
                    break
            if not fs:
                for f in other:                             # a figure misspelling the name, its table titled for us
                    t = field_table_title(f.text_after)
                    got = fields_after(f.text_after, width, f.access or r.access)
                    if got and t is not None and title_names(t, rname):
                        fig, fs = f, got
                        taken.add(id(f))
                        break
            if fig is not None:
                with_fields(reg, fig, fs, p.name)
                note = settle_access(reg, fig.rows, fig.access, a in paired)
                if note and first:
                    problems.append(f"access: {stem} {rname} {a:#010x} {note}")
                if not reg.fields and first:
                    problems.append(f"no fields: {stem} {rname} {a:#010x}")
            elif r.section not in (None, num):
                pass                                        # described in chapter r.section: fields come from there
            elif a == r.addresses[0]:
                problems.append(f"no figure: {stem} {rname} {a:#010x}")
            regs[b].append(_Reg(reg, a, num, r.section, stem))
            claimed.setdefault(a, []).append(rname)

    for f in figs:                                          # figures with no memory-map row
        for a, n in f.addresses:
            if any(_same_name(n, c, inst(a)) for c in claimed.get(a, [])):
                continue                                    # the figure of a mapped register
            if a in claimed:                                # an alternate, or the figure misspells the map's name
                problems.append(f"figure name differs: {stem} {n} {a:#010x} mapped as {', '.join(claimed[a])}"
                                + ("; its field table names the register, fields taken" if id(f) in taken else ""))
                claimed[a].append(n)
                continue
            claimed.setdefault(a, []).append(n)
            b = a & ~BLOCK
            if b not in periph:
                problems.append(f"no map row: {stem} {n} {a:#010x} outside the chapter's peripherals; left out")
                continue
            problems.append(f"no map row: {stem} {n} {a:#010x}")
            digit = _index(periph[b].name) if several else None
            rname = n[:-len(digit)] if digit and n.endswith(digit) and len(n) > len(digit) else n
            if f.width is None:
                problems.append(f"no width: {stem} {n} {a:#010x} figure has no bit numbering (32 assumed)")
            reg = Register(rname, a - b, f.width or 32, f.access or "read-write", None)
            with_fields(reg, f, table_fields(f, reg.width, reg.access, rname, True), periph[b].name)
            if not reg.fields:
                problems.append(f"no fields: {stem} {n} {a:#010x}")
            regs[b].append(_Reg(reg, a, num, None))
    return [(periph[b], regs[b]) for b in periph]


def _resolve(irq: Interrupt, periphs: list[Peripheral]) -> tuple[str | None, str]:
    """The peripheral an interrupt belongs to: by its flag's register (`UISR3` → the UART3 that has UISR), else
    by the ch. 17 module label (`MAC-NET0` → MACNET0, aliases for the rest)."""
    n = irq.name
    variants = [(n, None)] + [(n[:k] + n[k + 1:], n[k]) for k in range(len(n)) if n[k].isdigit()]
    best: list[tuple[int, str]] = []
    for v, d in variants:
        for p in periphs:
            idx = _index(p.name) if p.name != p.group else None
            if d is not None and idx is not None and idx != d:
                continue
            score = max((len(r.name) for r in p.registers if v == r.name or v.startswith(r.name + "_")), default=0)
            if not score:
                prefix = {r.name.split("_")[0] for r in p.registers if "_" in r.name}
                if len(prefix) == 1 and v.split("_")[0] == prefix.pop():
                    score = 1
            if score:
                best.append((score + (2 if d is not None and idx == d else 0), p.name))
    if best:
        top = max(s for s, _ in best)
        names = sorted({pn for s, pn in best if s == top})
        if len(names) == 1:
            return names[0], "register"
    label = re.sub(r"[^A-Za-z0-9]", "", re.sub(r"<[^>]+>", "", irq.peripheral)).upper()
    known = {p.name for p in periphs}
    for cand in (label, LABEL_ALIASES.get(label), LABEL_ALIASES.get(re.sub(r"\d+$", "", label))):
        if cand in known:
            return cand, "label"
    if best:
        top = max(s for s, _ in best)
        return None, "ambiguous between " + ", ".join(sorted({pn for s, pn in best if s == top}))
    return None, "no peripheral"


def build_device(rm_dir: Path, parts_map: Path = PARTS_MAP) -> tuple[Device, list[str]]:
    rm_dir = Path(rm_dir)
    problems: list[str] = []
    found: list[tuple[Peripheral, list[_Reg]]] = []
    ch01 = ch17 = ""
    for path in sorted(rm_dir.glob("ch*.md")):
        num = int(re.match(r"ch(\d+)", path.name).group(1))
        text = path.read_text()
        if num == 1:
            ch01 = text
            continue                                        # the overview's maps list modules, not registers
        if num == 17:
            ch17 = text
        found += _chapter(path.stem, num, text, problems)

    # rows whose Section/Page points at another chapter (USB host → the OTG chapter) take that chapter's fields
    described = {(r.chapter, r.reg.name): r.reg for _, rs in found for r in rs if r.reg.fields}
    for _, rs in found:
        for r in rs:
            if r.section not in (None, r.chapter) and not r.reg.fields:
                src = described.get((r.section, r.reg.name))
                if src is not None:
                    r.reg.fields = copy.deepcopy(src.fields)
                elif r.address == min(x.address for x in rs if x.reg.name == r.reg.name):
                    problems.append(f"no figure: {r.stem} {r.reg.name} {r.address:#010x} (described in ch{r.section:02})")

    # a register listed in two chapters (same address and name) stays where its Section/Page column points
    owners: dict[tuple[int, str], list[_Reg]] = {}
    for _, rs in found:
        for r in rs:
            owners.setdefault((r.address, r.reg.name), []).append(r)
    drop = set()
    for (addr, rname), rs in owners.items():
        if len(rs) > 1 and len({r.chapter for r in rs}) == 1:
            # one chapter listing a register twice: a write map and a read map (RGPIO)
            if {r.reg.access for r in rs} >= {"read-only", "write-only"}:
                rs[0].reg.access = "read-write"
            drop.update(id(r) for r in rs[1:])
        elif len(rs) > 1:
            keep = next((r for r in rs if r.section == r.chapter), rs[0])
            for r in rs:
                if r is not keep:
                    drop.add(id(r))
                    problems.append(f"duplicate: {rname} {addr:#010x} in ch{r.chapter:02} (kept in ch{keep.chapter:02})")

    peripherals, names = [], set()
    for p, rs in found:
        seen: dict[str, Register] = {}
        offsets: dict[int, str] = {}
        for r in rs:
            if id(r) in drop:
                continue
            reg = r.reg
            if reg.name in seen:
                problems.append(f"duplicate: {p.name}.{reg.name} at {reg.offset:#x} (kept {seen[reg.name].offset:#x})")
                continue
            if reg.offset in offsets:
                problems.append(f"alternate: {p.name}.{reg.name} shares offset {reg.offset:#x} with {offsets[reg.offset]}")
            offsets.setdefault(reg.offset, reg.name)
            seen[reg.name] = reg
        p.registers = sorted(seen.values(), key=lambda r: (r.offset, r.name))
        if not p.registers:
            problems.append(f"empty peripheral: {p.name} {p.base:#010x} has no registers; left out")
            continue
        if p.name in names:
            problems.append(f"name: peripheral {p.name} defined twice; second at {p.base:#010x} dropped")
            continue
        names.add(p.name)
        peripherals.append(p)
    peripherals.sort(key=lambda p: (p.group, int(_index(p.name) or -1) if p.name != p.group else -1, p.name))
    # chapters whose registers share one block (reset, CCM, SBF and LPCR at 0xEC09_0000) are one peripheral
    by_base: dict[int, list[Peripheral]] = {}
    for p in peripherals:
        by_base.setdefault(p.base, []).append(p)
    for base, ps in sorted(by_base.items()):
        if len(ps) > 1:
            name = SHARED_BLOCK_NAMES.get(base) or max(ps, key=lambda p: (len(p.registers), [-ord(c) for c in p.name])).name
            keep = Peripheral(name, base, name, sorted((r for p in ps for r in p.registers), key=lambda r: (r.offset, r.name)))
            gone = {id(p) for p in ps[1:]}
            peripherals = [keep if p is ps[0] else p for p in peripherals if id(p) not in gone]
            problems.append(f"shared block: {base:#010x} {', '.join(p.name for p in ps)} merged into {name}")

    irqs, iprobs = interrupts_with_problems(ch17)
    problems += [f"interrupt: {p}" for p in iprobs]
    interrupts = []
    for i in irqs:
        pname, how = _resolve(i, peripherals)
        if pname is None:
            problems.append(f"interrupt: {i.name} (vector {i.value}, label {i.peripheral!r}): {how}; left out")
            continue
        interrupts.append(Interrupt(i.name, i.value, pname))
    strip_prefixes(peripherals, problems)                   # after _resolve, which matches the prefixed names
    table, tprobs = availability_with_problems(ch01)
    avail, aprobs = keyed(table, json.loads(Path(parts_map).read_text())["groups"])
    problems += tprobs + aprobs
    return Device("MCF5441x", peripherals, interrupts, avail), problems
