"""Shared fixtures for the odgen tests."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
ODGEN_DIR = HERE.parent
REPO = ODGEN_DIR.parent.parent
sys.path.insert(0, str(ODGEN_DIR))
sys.path.insert(0, str(HERE))

DEFAULT_CANOPENNODE = REPO / "hal" / "hf-hal-vortex-v1" / "third_party" / "CANopenNode"


def canopennode_dir() -> Path | None:
    d = Path(os.environ.get("VORTEX_CANOPENNODE_DIR", DEFAULT_CANOPENNODE))
    return d if (d / "301" / "CO_ODinterface.h").is_file() else None


@pytest.fixture(scope="session")
def repo() -> Path:
    return REPO


@pytest.fixture(scope="session")
def canopennode() -> Path:
    d = canopennode_dir()
    if d is None:
        msg = ("CANopenNode sources not found; run `git submodule update --init hal/hf-hal-vortex-v1` and "
               "`git -C hal/hf-hal-vortex-v1 submodule update --init third_party/CANopenNode`, "
               "or set VORTEX_CANOPENNODE_DIR")
        if os.environ.get("VORTEX_REQUIRE_CANOPENNODE") == "1":
            pytest.fail(msg)
        pytest.skip(msg)
    return d


@pytest.fixture(scope="session")
def model(repo):
    from odgen import model as m

    return m.load(repo / "od" / "vortex_od.yaml")
