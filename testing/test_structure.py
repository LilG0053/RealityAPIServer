"""Fails if an import crosses the layers described in AGENTS.md.

The rules live in .importlinter at the repo root. Needs `pip install import-linter`;
the test is skipped if it isn't installed.
"""

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent


@pytest.mark.skipif(shutil.which("lint-imports") is None, reason="pip install import-linter")
def test_import_layers():
    result = subprocess.run(["lint-imports"], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stdout[result.stdout.find("Broken contracts"):]
