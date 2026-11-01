"""Turn a CANopenEditor-written EDS into odgen YAML (test oracle helper).

Only what the CANopenNode example EDS uses: VAR / ARRAY / RECORD, the
`;StorageLocation=` comments, $NODEID defaults and empty defaults (no OD
variable, served by an OD extension).
"""

from __future__ import annotations

import configparser
import re
from pathlib import Path

CODES = {0x01: "BOOL", 0x02: "I8", 0x03: "I16", 0x04: "I32", 0x05: "U8", 0x06: "U16", 0x07: "U32",
         0x08: "R32", 0x09: "VS", 0x0A: "OS", 0x0F: "DOM", 0x11: "R64", 0x15: "I64", 0x1B: "U64"}
GROUPS = {"PERSIST_COMM": "comm", "RAM": "none", "PERSIST_APP": "app", "PERSIST_CAL": "cal"}


def _storage(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    section = None
    for line in path.read_text().splitlines():
        m = re.match(r"^\[(.+)\]$", line.strip())
        if m:
            section = m.group(1)
        elif line.startswith(";StorageLocation=") and section:
            out[section] = line.split("=", 1)[1].strip()
    return out


def _pdo(access: str, mapping: str) -> str | bool:
    if int(mapping or "0", 0) == 0:
        return False
    return {"ro": "t", "const": "t", "wo": "r", "rw": "tr"}[access]


def _default(raw: str):
    raw = raw.strip()
    if raw == "":
        return None
    return raw if raw.startswith("$NODEID") else int(raw, 0)


def eds_to_yaml_dict(path: Path) -> dict:
    eds = configparser.RawConfigParser()
    eds.optionxform = str
    eds.read(path)
    storage = _storage(path)
    objects = []
    for section in eds.sections():
        if not re.fullmatch(r"[0-9A-Fa-f]{4}", section):
            continue
        index = int(section, 16)
        name = eds.get(section, "ParameterName")
        otype = int(eds.get(section, "ObjectType", fallback="0x7"), 0)
        obj: dict = {"index": index, "id": f"x{index:04x}", "name": name,
                     "persist": GROUPS[storage.get(section, "RAM")]}
        if otype == 0x7:
            dt = CODES[int(eds.get(section, "DataType"), 0)]
            acc = eds.get(section, "AccessType")
            obj.update({"object": "var", "data_type": dt, "access": acc,
                        "pdo": _pdo(acc, eds.get(section, "PDOMapping", fallback="0")),
                        "default": _default(eds.get(section, "DefaultValue", fallback=""))})
            objects.append(obj)
            continue
        n = int(eds.get(section, "SubNumber"), 0)
        subs = []
        s = 0
        while len(subs) < n:
            sec = f"{section}sub{s:X}"
            if eds.has_section(sec):
                subs.append((s, sec))
            s += 1
        s0sec = subs[0][1]
        sub0 = {"name": eds.get(s0sec, "ParameterName"), "access": eds.get(s0sec, "AccessType")}
        d0 = _default(eds.get(s0sec, "DefaultValue", fallback=""))
        if d0 is not None:
            sub0["default"] = d0
        obj["sub0"] = sub0
        rest = subs[1:]
        if otype == 0x8:
            first = rest[0][1]
            acc = eds.get(first, "AccessType")
            obj.update({"object": "array", "data_type": CODES[int(eds.get(first, "DataType"), 0)], "access": acc,
                        "pdo": _pdo(acc, eds.get(first, "PDOMapping", fallback="0"))})
            defaults = [_default(eds.get(sec, "DefaultValue", fallback="")) for _, sec in rest]
            if all(d is None for d in defaults):
                obj["storage"] = False
                obj["od_extension"] = "stack"
                obj["sub0"].pop("default", None)
            else:
                obj["default"] = defaults
            obj["elements"] = [{"name": eds.get(sec, "ParameterName"), "id": f"e{i}"}
                               for i, (_, sec) in enumerate(rest, start=1)]
        else:
            obj["object"] = "record"
            obj["subs"] = []
            for sub, sec in rest:
                acc = eds.get(sec, "AccessType")
                default = _default(eds.get(sec, "DefaultValue", fallback=""))
                if (index, sub) == (0x1018, 3):
                    assert default == 0, "the oracle device has revision 0.0"
                    default = "$REVISION"
                obj["subs"].append({"sub": sub, "id": f"s{sub}", "name": eds.get(sec, "ParameterName"),
                                    "data_type": CODES[int(eds.get(sec, "DataType"), 0)], "access": acc,
                                    "pdo": _pdo(acc, eds.get(sec, "PDOMapping", fallback="0")),
                                    "default": default})
        objects.append(obj)
    return {
        "schema": 1,
        "device": {"vendor_name": "x", "vendor_id": 0, "product_name": "x", "product_code": 0,
                   "revision": {"major": 0, "minor": 0}},
        "objects": objects,
    }
