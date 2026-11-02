"""odgen command line: regenerate every artefact, or check them for drift.

    python3 tools/odgen            # write the artefacts and record the OD hash
    python3 tools/odgen --check    # CI: fail if any artefact or the revision lock is stale
"""

from __future__ import annotations

import argparse
import difflib
import json
import sys
from pathlib import Path

from . import emit_canopennode, emit_eds, emit_ids, emit_md
from .model import Model, OdError, load

YAML = Path("od/vortex_od.yaml")
LOCK = Path("od/vortex_od.lock.json")
OUTPUTS = {
    "od_h": Path("od/canopennode/OD.h"),
    "od_c": Path("od/canopennode/OD.c"),
    "eds": Path("od/vortex_v1.eds"),
    "eds_pkg": Path("tools/vortex_canopen/vortex_canopen/data/vortex_v1.eds"),
    "hpp": Path("contracts/vortex_od_ids.hpp"),
    "py": Path("tools/vortex_canopen/vortex_canopen/od_ids.py"),
    "md": Path("docs/od-reference.md"),
}


def banner(model: Model) -> list[str]:
    return [f"Source: od/vortex_od.yaml, revision {model.revision_str}, OD hash 0x{model.od_hash:08X}.",
            "Do not edit: change the YAML and run `python3 tools/odgen`."]


def render(model: Model) -> dict[Path, str]:
    b = banner(model)
    c_banner = "/" + "*" * 79 + "\n" + "\n".join((f"    {x}" if x else "") for x in ["Vortex V1 object dictionary for CANopenNode V4", ""] + b) + \
        "\n" + "*" * 79 + "/"
    eds = emit_eds.emit(model, b)
    return {
        OUTPUTS["od_h"]: emit_canopennode.emit_h(model, c_banner),
        OUTPUTS["od_c"]: emit_canopennode.emit_c(model, c_banner),
        OUTPUTS["eds"]: eds,
        OUTPUTS["eds_pkg"]: eds,
        OUTPUTS["hpp"]: emit_ids.emit_cpp(model, b),
        OUTPUTS["py"]: emit_ids.emit_py(model, ["", *b]),
        OUTPUTS["md"]: emit_md.emit(model, b),
    }


def _read_lock(root: Path) -> dict:
    p = root / LOCK
    if not p.is_file():
        return {"revisions": {}}
    return json.loads(p.read_text())


def _lock_text(lock: dict) -> str:
    return json.dumps(lock, indent=2, sort_keys=True) + "\n"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="odgen", description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[3], help="repository root")
    ap.add_argument("--yaml", type=Path, default=None, help=f"OD source (default {YAML})")
    ap.add_argument("--check", action="store_true", help="fail on any stale artefact; write nothing")
    ap.add_argument("--amend-revision", action="store_true",
                    help="re-record the hash of the current (unreleased) revision instead of failing")
    args = ap.parse_args(argv)
    root = args.root.resolve()
    src = args.yaml if args.yaml is not None else root / YAML
    try:
        model = load(src)
    except (OdError, OSError) as exc:
        print(f"odgen: {exc}", file=sys.stderr)
        return 2
    outputs = render(model)
    rev, digest = model.revision_str, f"0x{model.od_hash:08X}"
    lock = _read_lock(root)
    revisions = dict(lock.get("revisions", {}))
    problems: list[str] = []

    recorded = revisions.get(rev)
    if recorded is not None and recorded != digest and not args.amend_revision:
        problems.append(f"the dictionary changed (OD hash {recorded} -> {digest}) but device.revision is still "
                        f"{rev}: bump device.revision (0x1018:03), or pass --amend-revision for an unreleased revision")
    if recorded is None and args.check:
        problems.append(f"revision {rev} is not recorded in {LOCK}: run odgen")

    if args.check:
        for rel, text in outputs.items():
            p = root / rel
            have = p.read_text() if p.is_file() else None
            if have is None:
                problems.append(f"{rel} is missing")
            elif have != text:
                diff = "".join(list(difflib.unified_diff(have.splitlines(True), text.splitlines(True),
                                                         f"{rel} (checked in)", f"{rel} (from YAML)", n=1))[:40])
                problems.append(f"{rel} is stale:\n{diff}")
        for p in problems:
            print(f"odgen --check: {p}", file=sys.stderr)
        if not problems:
            print(f"odgen --check: {len(outputs)} artefacts current, revision {rev}, OD hash {digest}")
        return 1 if problems else 0

    if problems:
        for p in problems:
            print(f"odgen: {p}", file=sys.stderr)
        return 1
    revisions[rev] = digest
    lock = {"comment": "OD hash recorded per device.revision; odgen --check fails when the dictionary changes "
                       "without a revision bump.", "revisions": revisions}
    outputs[LOCK] = _lock_text(lock)
    changed = 0
    for rel, text in outputs.items():
        p = root / rel
        if p.is_file() and p.read_text() == text:
            continue
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text)
        changed += 1
        print(f"odgen: wrote {rel}")
    print(f"odgen: revision {rev}, OD hash {digest}, {changed} file(s) updated")
    return 0
