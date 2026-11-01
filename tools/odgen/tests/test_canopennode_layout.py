"""OD.c / OD.h layout oracle: the CANopenNode example through odgen.

The CANopenNode example ships DS301_profile.eds together with the OD.c /
OD.h that CANopenEditor generated from it. Feeding that EDS through the
odgen model and emitter must reproduce those files exactly (after the file
banner), so the emitter follows the v4 layout the stack is built against.
"""

from __future__ import annotations

import yaml

from eds_import import eds_to_yaml_dict
from odgen import emit_canopennode, model


def _body(text: str) -> str:
    """Drop the leading file comment."""
    end = text.index("*/") + 2
    return text[end:]


def test_example_od_is_reproduced(canopennode, tmp_path):
    ex = canopennode / "example"
    y = tmp_path / "example.yaml"
    y.write_text(yaml.safe_dump(eds_to_yaml_dict(ex / "DS301_profile.eds"), sort_keys=False))
    m = model.load(y)
    got_h = emit_canopennode.emit_h(m, "/* x */")
    got_c = emit_canopennode.emit_c(m, "/* x */")
    assert _body(got_h) == _body((ex / "OD.h").read_text())
    assert _body(got_c) == _body((ex / "OD.c").read_text())
