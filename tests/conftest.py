"""Test-path setup for the pure iha_pkg helper modules."""

import sys
from pathlib import Path


IHA_PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "src" / "iha_pkg"
sys.path.insert(0, str(IHA_PACKAGE_ROOT))
