"""The generated OD.c builds with CANopenNode v4 and the stack accepts it.

Builds tests/c_build (OD.c + the whole CANopenNode stack + the upstream blank
driver) on the host, twice: single OD and CO_MULTIPLE_OD (OD_INIT_CONFIG).
The program reads every entry back through the OD interface and runs
CO_CANopenInit / CO_CANopenInitPDO, which validate the counts, the SDO
server 2 record and every default PDO mapping.
"""

from __future__ import annotations

import shutil
import struct
import subprocess
from pathlib import Path

import pytest

from conftest import REPO
from odgen import cli
from odgen.model import FLOATS, SIGNED, TYPES, load

ODA_SDO_R, ODA_SDO_W, ODA_TPDO, ODA_RPDO, ODA_MB, ODA_STR = 0x01, 0x02, 0x04, 0x08, 0x40, 0x80


def attribute(e) -> int:
    a = {"ro": ODA_SDO_R, "const": ODA_SDO_R, "wo": ODA_SDO_W, "rw": ODA_SDO_R | ODA_SDO_W}[e.access]
    a |= ODA_TPDO if "t" in e.pdo else 0
    a |= ODA_RPDO if "r" in e.pdo else 0
    size = TYPES[e.data_type][1]
    a |= ODA_MB if size is not None and size > 1 else 0
    a |= ODA_STR if e.data_type == "VS" else 0
    return a


def default_bytes(e) -> bytes:
    t, v = e.data_type, e.default
    if t == "VS":
        return v.encode().ljust(e.size, b"\0")
    if t == "OS":
        return bytes(v)
    if t in FLOATS:
        return struct.pack("<f" if t == "R32" else "<d", v)
    return int(v).to_bytes(TYPES[t][1], "little", signed=t in SIGNED)


def expect_header(model) -> str:
    rows = []
    for o in model.objects:
        for e in o.entries:
            has = e.storage and e.default is not None and e.data_type != "DOM"
            data = default_bytes(e) if has else b""
            n = len(data) if len(data) <= 64 else 0
            body = ", ".join(f"0x{b:02X}" for b in data[:64]) or "0"
            rows.append(f"    {{0x{o.index:04X}, 0x{e.sub:02X}, 0x{attribute(e):02X}, {e.size}, {int(has)}, {n}, {{{body}}}}},")
    pdos = []
    for o in model.objects:
        if 0x1600 <= o.index <= 0x17FF or 0x1A00 <= o.index <= 0x1BFF:
            tx = o.index >= 0x1A00
            count = o.entries[0].default
            bits = sum(e.default & 0xFF for e in o.entries[1:count + 1])
            comm = model.find(o.index - 0x200)
            valid = (comm.entries[1].default & 0x80000000) == 0
            pdos.append(f"    {{{int(tx)}, {o.index - (0x1A00 if tx else 0x1600) + 1}, {count}, {bits // 8}, {int(valid)}}},")
    return "\n".join([
        "#include <stdint.h>",
        "typedef struct { uint8_t tx; uint16_t n; uint8_t count; unsigned bytes; uint8_t valid; } pdo_expect_t;",
        "static const pdo_expect_t kPdoExpect[] = {", *(pdos or ["    {0, 0, 0, 0, 0},"]), "};",
        "typedef struct { uint16_t index; uint8_t sub; uint8_t attribute; uint32_t length; uint8_t has_data;",
        "                 uint8_t compare_len; uint8_t bytes[64]; } od_expect_t;",
        "static const od_expect_t kExpect[] = {", *rows, "};", ""])


FIXTURE = Path(__file__).parent / "fixtures" / "all_types.yaml"


@pytest.mark.parametrize("multiple_od", [False, True], ids=["single_od", "multiple_od"])
@pytest.mark.parametrize("source", ["vortex", "all_types"])
def test_od_builds_and_stack_accepts_it(canopennode, model, tmp_path, multiple_od, source):
    if shutil.which("cmake") is None:
        pytest.skip("cmake not installed")
    if source == "vortex":
        od_dir = REPO / "od" / "canopennode"  # the checked-in files
    else:
        model = load(FIXTURE)
        od_dir = tmp_path / "od"
        od_dir.mkdir()
        out = cli.render(model)
        (od_dir / "OD.c").write_text(out[cli.OUTPUTS["od_c"]])
        (od_dir / "OD.h").write_text(out[cli.OUTPUTS["od_h"]])
    (tmp_path / "od_expect.h").write_text(expect_header(model))
    build = tmp_path / "build"
    src = REPO / "tools" / "odgen" / "tests" / "c_build"
    subprocess.run(["cmake", "-S", str(src), "-B", str(build), f"-DCANOPENNODE_DIR={canopennode}",
                    f"-DOD_DIR={od_dir}", f"-DEXPECT_DIR={tmp_path}",
                    f"-DMULTIPLE_OD={'ON' if multiple_od else 'OFF'}",
                    f"-DENTRIES_ONLY={'ON' if source == 'all_types' else 'OFF'}"], check=True, capture_output=True)
    r = subprocess.run(["cmake", "--build", str(build)], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout[-4000:] + r.stderr[-4000:]
    run = subprocess.run([str(build / "od_stack_init")], capture_output=True, text=True)
    assert run.returncode == 0, run.stdout[-4000:]
    assert "0 failures" in run.stdout
