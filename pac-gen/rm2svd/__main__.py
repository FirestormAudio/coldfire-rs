import argparse
from pathlib import Path

from .build import build_device
from .svd import write_svd

ap = argparse.ArgumentParser(prog="rm2svd")
ap.add_argument("rm", type=Path)
ap.add_argument("-o", "--out", type=Path, required=True)
ap.add_argument("--coverage", type=Path)
a = ap.parse_args()
dev, problems = build_device(a.rm)
a.out.write_text(write_svd(dev))
if a.coverage:
    lines = [f"{p.name:12} {p.base:#010x} registers={len(p.registers):3} fields={sum(len(r.fields) for r in p.registers):4}"
             for p in dev.peripherals]
    a.coverage.write_text("\n".join(lines + ["", f"problems: {len(problems)}"] + problems) + "\n")
print(f"{len(dev.peripherals)} peripherals, {len(dev.interrupts)} interrupts, {len(problems)} problems")
