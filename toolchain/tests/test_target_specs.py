import json
from pathlib import Path

T = Path(__file__).resolve().parents[1]


def test_the_editor_target_differs_from_the_build_target_only_in_data_layout():
    # rust-analyzer's `cargo check` runs through stock rustc, whose LLVM rejects GCC's ColdFire data layout;
    # the check-only spec uses LLVM's m68k default and must otherwise match what builds use
    build = json.loads((T / "m68k-coldfire-none-elf.json").read_text())
    check = json.loads((T / "m68k-coldfire-none-elf-check.json").read_text())
    for spec in (build, check):
        spec.pop("data-layout")
        spec["metadata"].pop("description")
    assert build == check
    assert json.loads((T / "m68k-coldfire-none-elf-check.json").read_text())["data-layout"] == \
        "E-m:e-p:32:16:32-i8:8:8-i16:16:16-i32:16:32-n8:16:32-a:0:16-S16"
