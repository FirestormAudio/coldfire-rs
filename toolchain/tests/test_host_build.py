import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


# embassy-mcf5441x allows at most one time-driver-pitN feature, so the --all-features build excludes it; the
# part-feature check below covers it with a single part.
def test_the_workspace_builds_on_the_host_without_warnings():
    # the ColdFire-only parts are cfg'd out on a host; their dependencies must be too (no unused-dependency noise)
    r = subprocess.run(["cargo", "build", "--workspace", "--all-features", "--exclude", "embassy-mcf5441x"], cwd=ROOT, capture_output=True, text=True)
    assert r.returncode == 0 and "warning" not in r.stderr, r.stderr


def test_plain_cargo_build_skips_the_pac_and_is_warning_free():
    # the PAC needs a part feature; default-members keeps flagless commands working
    r = subprocess.run(["cargo", "build"], cwd=ROOT, capture_output=True, text=True)
    assert r.returncode == 0 and "warning" not in r.stderr, r.stderr


def test_workspace_check_with_a_part_feature_succeeds():
    r = subprocess.run(["cargo", "check", "--workspace", "--features", "mcf5441x-pac/mcf54418,embassy-mcf5441x/mcf54418"], cwd=ROOT,
                       capture_output=True, text=True)
    assert r.returncode == 0 and "warning" not in r.stderr, r.stderr


def test_hal_check_on_mcf54410_succeeds():
    r = subprocess.run(["cargo", "check", "-p", "embassy-mcf5441x", "--features", "mcf54410"], cwd=ROOT, capture_output=True, text=True)
    assert r.returncode == 0 and "warning" not in r.stderr, r.stderr
