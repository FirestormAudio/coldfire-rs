"""The generated PAC's part gates, by compiling probes (mcf5441x-pac/check-parts.sh)."""
import shutil
import subprocess
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / "mcf5441x-pac/check-parts.sh"


@pytest.mark.skipif(shutil.which("cargo") is None, reason="needs cargo")
def test_mcf54410_lacks_uart2_and_keeps_uart7():
    r = subprocess.run([str(SCRIPT)], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
