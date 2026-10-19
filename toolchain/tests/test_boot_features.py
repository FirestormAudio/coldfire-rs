"""embassy-mcf5441x needs exactly one boot feature: cold boot from one of the three sources, or a bootloader."""
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FOUR = ("boot-serial", "boot-nor", "boot-nand", "bootloader")


def check(features):
    return subprocess.run(["cargo", "check", "-q", "-p", "embassy-mcf5441x", "--features", ",".join(["mcf54418", *features])],
                          cwd=ROOT, capture_output=True, text=True)


def test_no_boot_feature_is_an_error_naming_all_four():
    r = check([])
    assert r.returncode != 0
    assert all(f in r.stderr for f in FOUR), r.stderr[-2000:]


def test_two_boot_features_are_an_error():
    r = check(["bootloader", "boot-nor"])
    assert r.returncode != 0
    assert "enable exactly one of the features" in r.stderr, r.stderr[-2000:]


def test_each_boot_feature_alone_checks():
    for f in FOUR:
        r = check([f])
        assert r.returncode == 0, (f, r.stderr[-2000:])
