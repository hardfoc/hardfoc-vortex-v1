"""Object dictionary model: load the YAML source, validate it, expand it.

The YAML (od/vortex_od.yaml, schema 1) mirrors the CiA 306 attributes and
adds the Vortex ones (design §4.2):

    objects:
      - index: 0x1017                # ObjectType VAR / ARRAY / RECORD via `object`
        id: producer_heartbeat_time  # stable identifier for C++ / Python constants
        name: Producer heartbeat time  # ParameterName
        object: var                  # var | array | record
        data_type: U16               # DataType (CiA 301 short names, see TYPES)
        access: rw                   # AccessType: ro | rw | wo | const
        pdo: false                   # PDOMapping: false | t | r | tr
        default: 100                 # DefaultValue ($NODEID+..., $REVISION, $OD_HASH, $OD_VERSION)
        low: 0                       # LowLimit
        high: 65535                  # HighLimit
        persist: comm                # comm | app | cal | none
        units: ms
        scale: 1.0                   # SI value = raw * scale (tools only)
        od_extension: Handler        # served by a C++ OD extension (snapshots)
        storage: true                # false: no OD variable, the extension serves it
        fd_only: false               # compiled out of classic CAN builds
        tags: [telemetry]            # free-form grouping for tools and docs
        docs: text

Extension points: `includes:` pulls `objects:` from more files (another
workstream adds a file instead of editing this one), `areas:` declares index
ranges and their owners, and any key starting with `x_` is carried through
unvalidated.
"""

from __future__ import annotations

import json
import re
import zlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

SCHEMA_VERSION = 1

# name: (CiA 301 code, byte size or None for variable, C type)
TYPES: dict[str, tuple[int, int | None, str | None]] = {
    "BOOL": (0x01, 1, "bool_t"),
    "I8": (0x02, 1, "int8_t"),
    "I16": (0x03, 2, "int16_t"),
    "I32": (0x04, 4, "int32_t"),
    "U8": (0x05, 1, "uint8_t"),
    "U16": (0x06, 2, "uint16_t"),
    "U32": (0x07, 4, "uint32_t"),
    "R32": (0x08, 4, "float32_t"),
    "VS": (0x09, None, "char"),
    "OS": (0x0A, None, "uint8_t"),
    "DOM": (0x0F, None, None),
    "R64": (0x11, 8, "float64_t"),
    "I64": (0x15, 8, "int64_t"),
    "U64": (0x1B, 8, "uint64_t"),
}
SIGNED = {"I8", "I16", "I32", "I64"}
UNSIGNED = {"BOOL", "U8", "U16", "U32", "U64"}
FLOATS = {"R32", "R64"}
VARLEN = {"VS", "OS", "DOM"}
ACCESS = {"ro", "rw", "wo", "const"}
PDO = {"t", "r", "tr"}
PERSIST = {"comm": "PERSIST_COMM", "app": "PERSIST_APP", "cal": "PERSIST_CAL", "none": "RAM"}
PERSIST_GROUP_ID = {"none": 0, "comm": 1, "app": 2, "cal": 3}
KINDS = {"var": 0x7, "array": 0x8, "record": 0x9}
TOKENS = ("$REVISION", "$OD_HASH", "$OD_VERSION")

# CANopenNode CO_countLabel per index (doc/objectDictionary.md table).
COUNT_LABELS = [
    (0x1000, 0x1000, "NMT"),
    (0x1001, 0x1001, "EM"),
    (0x1005, 0x1005, "SYNC"),
    (0x1006, 0x1006, "SYNC_PROD"),
    (0x1010, 0x1010, "STORAGE"),
    (0x1012, 0x1012, "TIME"),
    (0x1014, 0x1014, "EM_PROD"),
    (0x1016, 0x1016, "HB_CONS"),
    (0x1017, 0x1017, "HB_PROD"),
    (0x1200, 0x127F, "SDO_SRV"),
    (0x1280, 0x12FF, "SDO_CLI"),
    (0x1400, 0x15FF, "RPDO"),
    (0x1800, 0x19FF, "TPDO"),
]

OBJECT_KEYS = {
    "index", "id", "name", "object", "data_type", "access", "pdo", "default", "low", "high", "persist",
    "units", "scale", "od_extension", "storage", "fd_only", "tags", "docs", "length", "subs", "elements",
    "sub0", "count_label",
}
SUB_KEYS = {
    "sub", "id", "name", "data_type", "access", "pdo", "default", "low", "high", "units", "scale", "docs",
    "length", "persist", "fd_only", "tags",
}
ELEMENT_KEYS = {"id", "name", "default", "docs"}
TOP_KEYS = {"schema", "device", "areas", "objects", "includes", "pdo_defaults", "migrations", "eds"}
DEVICE_KEYS = {
    "vendor_name", "vendor_id", "product_name", "product_code", "revision", "order_code", "baud_rates",
    "lss_supported", "granularity", "description", "created_by",
}


class OdError(Exception):
    """Validation error with a YAML path."""


def _fail(path: str, msg: str) -> None:
    raise OdError(f"{path}: {msg}")


def _check_keys(d: dict, allowed: set[str], path: str) -> None:
    for k in d:
        if not isinstance(k, str):
            _fail(path, f"key {k!r} is not a string")
        if k not in allowed and not k.startswith("x_"):
            _fail(path, f"unknown key '{k}' (allowed: {', '.join(sorted(allowed))}; extensions use 'x_')")


def make_cname(name: str) -> str:
    """CANopenEditor-style C identifier from a ParameterName.

    "COB-ID SYNC message" -> COB_ID_SYNCMessage, "Highest sub-index supported"
    -> highestSub_indexSupported (hyphens become '_', acronyms after an
    uppercase character are separated by '_', first letter lowercased unless
    the second is uppercase).
    """
    words = [w for w in re.split(r"[^0-9A-Za-z\-]+", name) if w.strip("-")]
    out = ""
    for w in words:
        parts = [p for p in w.split("-") if p]
        if not parts:
            continue
        first = parts[0]
        token = first[0].upper() + first[1:]
        if len(parts) > 1:
            token += "_" + "_".join(parts[1:])
        if out and out[-1].isupper() and len(token) > 1 and token[0].isupper() and token[1].isupper():
            out += "_"
        out += token
    if len(out) > 1 and not out[1].isupper():
        out = out[0].lower() + out[1:]
    elif len(out) == 1:
        out = out.lower()
    if out and out[0].isdigit():
        out = "_" + out
    return out


def snake(name: str) -> str:
    s = re.sub(r"[^0-9A-Za-z]+", "_", name).strip("_")
    s = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s).lower()
    return re.sub(r"_+", "_", s) or "x"


@dataclass
class Entry:
    """One OD variable (a VAR object or one sub-index)."""

    index: int
    sub: int
    name: str
    id: str
    data_type: str
    access: str
    pdo: str = ""  # "", "t", "r", "tr"
    default: Any = None  # int / float / str / bytes / None (no storage)
    default_expr: str = ""  # canonical text of the YAML default
    node_relative: bool = False
    token: str = ""  # $REVISION / $OD_HASH / $OD_VERSION
    low: Any = None
    high: Any = None
    length: int = 0  # bytes for VS / OS capacity; DOM 0
    persist: str = "none"
    units: str = ""
    scale: float | None = None
    docs: str = ""
    fd_only: bool = False
    storage: bool = True
    tags: list[str] = field(default_factory=list)
    extra: dict[str, Any] = field(default_factory=dict)

    @property
    def code(self) -> int:
        return TYPES[self.data_type][0]

    @property
    def size(self) -> int:
        """Byte length of the OD variable (strings: capacity without the terminator)."""
        fixed = TYPES[self.data_type][1]
        return fixed if fixed is not None else self.length

    @property
    def bit_length(self) -> int:
        return 8 * self.size


@dataclass
class Obj:
    index: int
    id: str
    name: str
    kind: str  # var | array | record
    entries: list[Entry]
    persist: str = "none"
    count_label: str = ""
    od_extension: str = ""
    storage: bool = True
    fd_only: bool = False
    docs: str = ""
    tags: list[str] = field(default_factory=list)
    source: str = ""
    extra: dict[str, Any] = field(default_factory=dict)

    @property
    def cname(self) -> str:
        return make_cname(self.name)

    @property
    def array_len(self) -> int:
        return len(self.entries) - 1


@dataclass
class Area:
    first: int
    last: int
    name: str
    owner: str


@dataclass
class Migration:
    from_index: int
    from_sub: int
    to_index: int
    to_sub: int


@dataclass
class Model:
    device: dict[str, Any]
    objects: list[Obj]
    areas: list[Area]
    migrations: list[Migration]
    eds: dict[str, Any]
    sources: list[Path]
    od_hash: int = 0

    def find(self, index: int) -> Obj | None:
        for o in self.objects:
            if o.index == index:
                return o
        return None

    def entry(self, index: int, sub: int) -> Entry | None:
        o = self.find(index)
        if o is None:
            return None
        for e in o.entries:
            if e.sub == sub:
                return e
        return None

    def all_entries(self) -> list[Entry]:
        return [e for o in self.objects for e in o.entries]

    @property
    def revision(self) -> tuple[int, int]:
        r = self.device["revision"]
        return int(r["major"]), int(r["minor"])

    @property
    def revision_str(self) -> str:
        major, minor = self.revision
        return f"{major}.{minor}"

    @property
    def revision_number(self) -> int:
        major, minor = self.revision
        return (major << 16) | minor

    @property
    def od_version(self) -> int:
        major, minor = self.revision
        return (major << 8) | minor


# ---------------------------------------------------------------------------
# value parsing
# ---------------------------------------------------------------------------


def _int(v: Any, path: str) -> int:
    if isinstance(v, bool):
        return int(v)
    if isinstance(v, int):
        return v
    if isinstance(v, str):
        try:
            return int(v.replace("_", ""), 0)
        except ValueError:
            pass
    _fail(path, f"expected an integer, got {v!r}")
    return 0


def _range(t: str) -> tuple[float, float]:
    if t == "BOOL":
        return 0, 1
    if t in UNSIGNED:
        return 0, (1 << (8 * TYPES[t][1])) - 1
    if t in SIGNED:
        bits = 8 * TYPES[t][1]
        return -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    return float("-inf"), float("inf")


def _parse_number(t: str, v: Any, path: str) -> int | float:
    if t in FLOATS:
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            _fail(path, f"{t} value must be a number, got {v!r}")
        return float(v)
    n = _int(v, path)
    lo, hi = _range(t)
    if not lo <= n <= hi:
        _fail(path, f"value {v!r} out of range for {t} [{lo}, {hi}]")
    return n


def _parse_default(e: Entry, v: Any, path: str) -> None:
    t = e.data_type
    if v is None:
        e.default = None
        return
    if isinstance(v, str) and v.strip() in TOKENS:
        if t not in ("U16", "U32"):
            _fail(path, f"{v} needs data_type U16 or U32")
        e.token = v.strip()
        e.default = 0
        e.default_expr = e.token
        return
    if isinstance(v, str) and "$NODEID" in v:
        if t not in UNSIGNED or t == "BOOL":
            _fail(path, "$NODEID defaults need an unsigned integer type")
        m = re.fullmatch(r"\s*\$NODEID\s*\+\s*(0[xX][0-9A-Fa-f_]+|\d+)\s*", v)
        if not m:
            _fail(path, f"default {v!r}: write $NODEID+<number>")
        e.node_relative = True
        e.default = _parse_number(t, m.group(1), path)
        e.default_expr = f"$NODEID+0x{e.default:X}"
        return
    if t == "VS":
        if not isinstance(v, str):
            _fail(path, "VS default must be a string")
        if ";" in v or "\n" in v:
            _fail(path, "VS default may not contain ';' or newlines (EDS comment / line syntax)")
        try:
            v.encode("ascii")
        except UnicodeEncodeError:
            _fail(path, "VS default must be ASCII")
        e.default = v
        e.default_expr = json.dumps(v)
        return
    if t == "OS":
        if isinstance(v, str):
            try:
                b = bytes.fromhex(v)
            except ValueError:
                _fail(path, "OS default string must be hex bytes")
        elif isinstance(v, list):
            b = bytes(_parse_number("U8", x, f"{path}[{i}]") for i, x in enumerate(v))
        else:
            _fail(path, "OS default must be hex text or a list of bytes")
        e.default = b
        e.default_expr = b.hex()
        return
    if t == "DOM":
        _fail(path, "DOM objects have no default")
    if t == "BOOL" and isinstance(v, bool):
        v = int(v)
    e.default = _parse_number(t, v, path)
    e.default_expr = repr(e.default) if t in FLOATS else str(e.default)


# ---------------------------------------------------------------------------
# loading
# ---------------------------------------------------------------------------


def _load_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise OdError(f"{path}: top level must be a mapping")
    return data


def _entry(raw: dict, index: int, sub: int, defaults: dict, path: str) -> Entry:
    if not isinstance(raw, dict):
        _fail(path, "must be a mapping")
    name = raw.get("name", defaults.get("name"))
    if not name or not isinstance(name, str):
        _fail(path, "name is required")
    if ";" in name or "\n" in name or "[" in name or "]" in name:
        _fail(path, "names may not contain ';', '[', ']' or newlines (EDS syntax)")
    dt = raw.get("data_type", defaults.get("data_type"))
    if dt not in TYPES:
        _fail(path, f"data_type {dt!r} unknown (one of {', '.join(TYPES)})")
    access = raw.get("access", defaults.get("access"))
    if access not in ACCESS:
        _fail(path, f"access {access!r} must be one of {', '.join(sorted(ACCESS))}")
    pdo = raw.get("pdo", defaults.get("pdo", False))
    if pdo is False or pdo is None:
        pdo = ""
    if pdo not in PDO and pdo != "":
        _fail(path, f"pdo {pdo!r} must be false, t, r or tr")
    if pdo and dt in VARLEN:
        _fail(path, "variable-length types are not PDO mappable")
    if pdo in ("r", "tr") and access in ("ro", "const"):
        _fail(path, "an RPDO-mappable entry must be writable")
    if pdo in ("t", "tr") and access == "wo":
        _fail(path, "a TPDO-mappable entry must be readable")
    eid = raw.get("id", defaults.get("id")) or snake(name)
    if not re.fullmatch(r"[a-z][a-z0-9_]*", str(eid)):
        _fail(path, f"id {eid!r} must be snake_case")
    e = Entry(index=index, sub=sub, name=name, id=str(eid), data_type=dt, access=access, pdo=pdo)
    e.units = str(raw.get("units", defaults.get("units", "")) or "")
    sc = raw.get("scale", defaults.get("scale"))
    if sc is not None:
        if isinstance(sc, bool) or not isinstance(sc, (int, float)):
            _fail(path, "scale must be a number")
        e.scale = float(sc)
    e.docs = str(raw.get("docs", "") or "").strip()
    e.persist = raw.get("persist", defaults.get("persist", "none"))
    if e.persist not in PERSIST:
        _fail(path, f"persist {e.persist!r} must be one of {', '.join(PERSIST)}")
    e.fd_only = bool(raw.get("fd_only", defaults.get("fd_only", False)))
    e.tags = list(raw.get("tags", defaults.get("tags", [])) or [])
    e.extra = {k: v for k, v in raw.items() if isinstance(k, str) and k.startswith("x_")}
    e.storage = bool(defaults.get("storage", True))
    length = raw.get("length", defaults.get("length"))
    if dt in ("VS", "OS"):
        e.length = _int(length, path + ".length") if length is not None else 0
    elif length is not None:
        _fail(path, "length applies to VS / OS only")
    if "default" in raw:
        dv = raw["default"]
    else:
        dv = defaults.get("default", None if (dt in VARLEN or not e.storage) else 0)
    if dt == "DOM":
        if raw.get("default") is not None:
            _fail(path, "DOM objects have no default")
        e.default = None
        e.storage = False
    elif not e.storage:
        if "default" in raw and raw["default"] is not None:
            _fail(path, "storage: false objects have no default (the OD extension serves them)")
        e.default = None
    else:
        _parse_default(e, dv, path + ".default")
        if dt in ("VS", "OS"):
            cur = len(e.default) if e.default is not None else 0
            e.length = max(e.length, cur)
            if e.default is None:
                e.default = "" if dt == "VS" else b""
                e.default_expr = '""' if dt == "VS" else ""
            if e.length == 0:
                _fail(path, f"{dt} with storage needs a default or a length")
            if dt == "OS" and len(e.default) not in (0, e.length):
                _fail(path, "OS default must fill the whole length")
    for key in ("low", "high"):
        v = raw.get(key, defaults.get(key))
        if v is None:
            continue
        if dt in VARLEN:
            _fail(path, f"{key} does not apply to {dt}")
        setattr(e, key, _parse_number(dt, v, f"{path}.{key}"))
    if e.low is not None and e.high is not None and e.low > e.high:
        _fail(path, "low > high")
    if isinstance(e.default, (int, float)) and not isinstance(e.default, bool) and not e.token:
        if e.low is not None and e.default < e.low:
            _fail(path, "default below low")
        if e.high is not None and e.default > e.high:
            _fail(path, "default above high")
    return e


def _object(raw: dict, path: str, source: str) -> Obj:
    if not isinstance(raw, dict):
        _fail(path, "object must be a mapping")
    _check_keys(raw, OBJECT_KEYS, path)
    index = _int(raw.get("index"), path + ".index")
    if not 0x1000 <= index <= 0xFFFF:
        _fail(path, f"index 0x{index:04X} outside 0x1000..0xFFFF")
    path = f"{path}[0x{index:04X}]"
    name = raw.get("name")
    if not name:
        _fail(path, "name is required")
    oid = raw.get("id")
    if not oid or not re.fullmatch(r"[a-z][a-z0-9_]*", str(oid)):
        _fail(path, "id is required and must be snake_case")
    kind = raw.get("object", "var")
    if kind not in KINDS:
        _fail(path, f"object {kind!r} must be var, array or record")
    persist = raw.get("persist", "none")
    if persist not in PERSIST:
        _fail(path, f"persist {persist!r} must be one of {', '.join(PERSIST)}")
    storage = bool(raw.get("storage", True))
    obj = Obj(index=index, id=str(oid), name=str(name), kind=kind, entries=[], persist=persist, storage=storage,
              source=source)
    obj.od_extension = str(raw.get("od_extension", "") or "")
    if not storage and not obj.od_extension:
        _fail(path, "storage: false needs od_extension (something must serve the data)")
    obj.fd_only = bool(raw.get("fd_only", False))
    obj.docs = str(raw.get("docs", "") or "").strip()
    obj.tags = list(raw.get("tags", []) or [])
    obj.count_label = str(raw.get("count_label", "") or "")
    obj.extra = {k: v for k, v in raw.items() if isinstance(k, str) and k.startswith("x_")}
    inherit = {"persist": persist, "storage": storage, "fd_only": obj.fd_only, "tags": obj.tags}

    if kind == "var":
        for k in ("subs", "elements", "sub0"):
            if k in raw:
                _fail(path, f"'{k}' is for arrays / records")
        d = dict(inherit)
        d["id"] = obj.id
        d["name"] = obj.name
        sub_raw = {k: v for k, v in raw.items() if k in SUB_KEYS - {"sub", "persist", "fd_only", "tags"}}
        sub_raw.update({k: v for k, v in raw.items() if isinstance(k, str) and k.startswith("x_")})
        obj.entries.append(_entry(sub_raw, index, 0, d, path))
        if obj.entries[0].data_type == "DOM" and not obj.od_extension:
            _fail(path, "DOM objects need od_extension (CANopenNode keeps no data for them)")
        return obj

    sub0_raw = raw.get("sub0", {}) or {}
    if not isinstance(sub0_raw, dict):
        _fail(path, "sub0 must be a mapping")
    _check_keys(sub0_raw, {"name", "access", "default", "id", "docs"}, path + ".sub0")

    if kind == "array":
        if "subs" in raw:
            _fail(path, "arrays use 'elements' (records use 'subs')")
        dt = raw.get("data_type")
        if dt not in TYPES or dt in VARLEN:
            _fail(path, "arrays need a fixed-size data_type")
        elements = raw.get("elements")
        if not isinstance(elements, list) or not elements:
            _fail(path, "arrays need a non-empty 'elements' list (or element count via length-free list)")
        common = dict(inherit)
        for k in ("data_type", "access", "pdo", "low", "high", "units", "scale"):
            if k in raw:
                common[k] = raw[k]
        if "default" in raw:
            common["default"] = raw["default"]
        n = len(elements)
        s0 = Entry(index=index, sub=0, name=sub0_raw.get("name", "Highest sub-index supported"),
                   id=str(sub0_raw.get("id", "highest_sub_index")), data_type="U8",
                   access=sub0_raw.get("access", "ro"), persist=persist, storage=storage)
        if s0.access not in ACCESS:
            _fail(path + ".sub0", "bad access")
        _parse_default(s0, sub0_raw.get("default", n) if storage else None, path + ".sub0.default")
        s0.docs = str(sub0_raw.get("docs", "") or "")
        obj.entries.append(s0)
        list_default = common.get("default") if isinstance(common.get("default"), list) else None
        if list_default is not None and len(list_default) != n:
            _fail(path, f"default list has {len(list_default)} values for {n} elements")
        for i, el in enumerate(elements, start=1):
            if isinstance(el, str):
                el = {"name": el}
            if not isinstance(el, dict):
                _fail(f"{path}.elements[{i}]", "element must be a name or a mapping")
            _check_keys(el, ELEMENT_KEYS, f"{path}.elements[{i}]")
            d = dict(common)
            if list_default is not None:
                d["default"] = list_default[i - 1]
            if "id" not in el:
                d["id"] = f"{snake(el['name'])}" if "name" in el else f"element_{i}"
            obj.entries.append(_entry(el, index, i, d, f"{path}.elements[{i}]"))
        ids = [e.id for e in obj.entries[1:]]
        if len(set(ids)) != len(ids):
            for i, e in enumerate(obj.entries[1:], start=1):
                e.id = f"{e.id}_{i}"
        return obj

    # record
    if "elements" in raw:
        _fail(path, "records use 'subs' (arrays use 'elements')")
    for k in ("data_type", "access", "pdo", "default", "low", "high", "length", "units", "scale"):
        if k in raw:
            _fail(path, f"'{k}' belongs on each sub of a record")
    subs = raw.get("subs")
    if not isinstance(subs, list) or not subs:
        _fail(path, "records need a non-empty 'subs' list")
    parsed: list[Entry] = []
    for i, sr in enumerate(subs):
        p = f"{path}.subs[{i}]"
        if not isinstance(sr, dict):
            _fail(p, "sub must be a mapping")
        _check_keys(sr, SUB_KEYS, p)
        sub = _int(sr.get("sub"), p + ".sub")
        if not 1 <= sub <= 0xFE:
            _fail(p, "sub must be 1..254 (sub 0 is generated; override it with 'sub0')")
        parsed.append(_entry(sr, index, sub, dict(inherit), f"{p}[sub {sub}]"))
    subs_seen = [e.sub for e in parsed]
    if subs_seen != sorted(set(subs_seen)):
        _fail(path, "record subs must be unique and ascending")
    s0 = Entry(index=index, sub=0, name=sub0_raw.get("name", "Highest sub-index supported"),
               id=str(sub0_raw.get("id", "highest_sub_index")), data_type="U8",
               access=sub0_raw.get("access", "const"), persist=persist, storage=storage)
    if s0.access not in ACCESS:
        _fail(path + ".sub0", "bad access")
    _parse_default(s0, sub0_raw.get("default", max(subs_seen)) if storage else None, path + ".sub0.default")
    s0.docs = str(sub0_raw.get("docs", "") or "")
    obj.entries = [s0] + parsed
    if any(e.data_type == "DOM" for e in parsed) and not obj.od_extension:
        _fail(path, "DOM subs need od_extension (CANopenNode keeps no data for them)")
    ids = [e.id for e in obj.entries]
    if len(set(ids)) != len(ids):
        _fail(path, f"duplicate sub ids: {sorted(x for x in ids if ids.count(x) > 1)}")
    return obj


# ---------------------------------------------------------------------------
# PDO defaults (design §5.4) → 0x1400 / 0x1600 / 0x1800 / 0x1A00 records
# ---------------------------------------------------------------------------

RPDO_COMM_SUBS = [(1, "COB-ID used by RPDO", "cob_id", "U32"), (2, "Transmission type", "transmission_type", "U8"),
                  (5, "Event timer", "event_timer", "U16")]
TPDO_COMM_SUBS = [(1, "COB-ID used by TPDO", "cob_id", "U32"), (2, "Transmission type", "transmission_type", "U8"),
                  (3, "Inhibit time", "inhibit_time", "U16"), (5, "Event timer", "event_timer", "U16"),
                  (6, "SYNC start value", "sync_start", "U8")]
PDO_KEYS = {"n", "cob_id", "transmission_type", "event_timer", "inhibit_time", "sync_start", "map", "docs", "use"}
MAX_MAP_ENTRIES = 8
CLASSIC_PDO_BITS = 64


def _parse_map_ref(ref: Any, path: str) -> tuple[int, int]:
    """'0x6040' or '0x2300:01' (sub-index in hex, as in CANopen tools). A bare integer is index:00."""
    if isinstance(ref, int) and not isinstance(ref, bool) and 0 <= ref <= 0xFFFF:
        return ref, 0
    if not isinstance(ref, str):
        _fail(path, f"entry {ref!r} must be 'index' or 'index:sub' text")
    m = re.fullmatch(r"\s*0[xX]([0-9A-Fa-f]{4})(?::(?:0[xX])?([0-9A-Fa-f]{1,2}))?\s*", ref)
    if not m:
        _fail(path, f"entry {ref!r} must look like 0x6040 or 0x2300:01")
    return int(m.group(1), 16), int(m.group(2), 16) if m.group(2) else 0


def _expand_pdos(raw: dict, objects: list[Obj]) -> list[tuple[Obj, list[tuple[int, int]]]]:
    """Build the PDO objects. Mapping targets are resolved after all objects load."""
    out: list[tuple[Obj, list[tuple[int, int]]]] = []
    if not raw:
        return out
    if not isinstance(raw, dict):
        _fail("pdo_defaults", "must be a mapping with rpdo / tpdo lists")
    _check_keys(raw, {"rpdo", "tpdo"}, "pdo_defaults")
    for kind, comm_base, map_base, comm_subs in (("rpdo", 0x1400, 0x1600, RPDO_COMM_SUBS),
                                                 ("tpdo", 0x1800, 0x1A00, TPDO_COMM_SUBS)):
        items = raw.get(kind, []) or []
        numbers = []
        for i, p in enumerate(items):
            path = f"pdo_defaults.{kind}[{i}]"
            if not isinstance(p, dict):
                _fail(path, "must be a mapping")
            _check_keys(p, PDO_KEYS, path)
            n = _int(p.get("n"), path + ".n")
            if not 1 <= n <= 512:
                _fail(path, "n must be 1..512")
            numbers.append(n)
            label = kind.upper()
            subs = []
            for sub, name, key, dt in comm_subs:
                d = {"sub": sub, "name": name, "data_type": dt, "access": "rw",
                     "default": p.get(key, 0 if key != "transmission_type" else 254)}
                if key == "cob_id":
                    d["default"] = p.get("cob_id")
                    if d["default"] is None:
                        _fail(path, "cob_id is required")
                subs.append(d)
            comm = _object({"index": comm_base + n - 1, "id": f"{kind}{n}_communication",
                            "name": f"{label} communication parameter", "object": "record", "persist": "comm",
                            "subs": subs, "docs": str(p.get("docs", "") or p.get("use", "") or ""),
                            "count_label": label},
                           f"{path} comm", "pdo_defaults")
            comm.entries[0].default = 5 if kind == "rpdo" else 6
            comm.entries[0].default_expr = str(comm.entries[0].default)
            comm.entries[0].access = "const"
            maps = p.get("map", []) or []
            if len(maps) > MAX_MAP_ENTRIES:
                _fail(path, f"at most {MAX_MAP_ENTRIES} mapping entries")
            refs = [_parse_map_ref(m, f"{path}.map[{j}]") for j, m in enumerate(maps)]
            msubs = [{"sub": j, "name": f"Application object {j}", "id": f"application_object_{j}",
                      "data_type": "U32", "access": "rw", "default": 0} for j in range(1, MAX_MAP_ENTRIES + 1)]
            mobj = _object({"index": map_base + n - 1, "id": f"{kind}{n}_mapping",
                            "name": f"{label} mapping parameter", "object": "record", "persist": "comm",
                            "sub0": {"name": "Number of mapped application objects in PDO", "access": "rw",
                                     "default": len(refs), "id": "number_of_mapped_objects"},
                            "subs": msubs},
                           f"{path} map", "pdo_defaults")
            objects.extend([comm, mobj])
            out.append((mobj, refs))
        if sorted(numbers) != list(range(1, len(numbers) + 1)):
            _fail(f"pdo_defaults.{kind}", f"PDO numbers must be 1..N without gaps, got {sorted(numbers)}")
    return out


def _resolve_pdo_maps(model: Model, maps: list[tuple[Obj, list[tuple[int, int]]]], fd: bool) -> None:
    for mobj, refs in maps:
        is_rpdo = mobj.index < 0x1A00
        bits = 0
        for j, (idx, sub) in enumerate(refs, start=1):
            e = model.entry(idx, sub)
            where = f"0x{mobj.index:04X} entry {j} (0x{idx:04X}:{sub:02X})"
            if e is None:
                raise OdError(f"{where}: mapped object does not exist")
            want = "r" if is_rpdo else "t"
            if want not in e.pdo:
                raise OdError(f"{where}: '{e.name}' is not {'R' if is_rpdo else 'T'}PDO mappable")
            if e.fd_only and not fd:
                raise OdError(f"{where}: fd_only object in a classic PDO")
            bits += e.bit_length
            value = (idx << 16) | (sub << 8) | e.bit_length
            m = mobj.entries[j]
            m.default = value
            m.default_expr = str(value)
        if bits > CLASSIC_PDO_BITS and not fd:
            raise OdError(f"0x{mobj.index:04X}: mapping is {bits} bits, classic CAN carries {CLASSIC_PDO_BITS}")


# ---------------------------------------------------------------------------


def load(path: Path, fd: bool = False) -> Model:
    """Load, validate and expand the YAML. `fd` keeps fd_only objects (FD build)."""
    path = Path(path)
    data = _load_yaml(path)
    _check_keys(data, TOP_KEYS, str(path))
    if data.get("schema") != SCHEMA_VERSION:
        raise OdError(f"{path}: schema must be {SCHEMA_VERSION}")
    device = data.get("device")
    if not isinstance(device, dict):
        raise OdError(f"{path}: device section is required")
    _check_keys(device, DEVICE_KEYS, "device")
    for k in ("vendor_name", "product_name", "vendor_id", "product_code", "revision"):
        if k not in device:
            raise OdError(f"device.{k} is required")
    rev = device["revision"]
    if not isinstance(rev, dict) or set(rev) != {"major", "minor"}:
        raise OdError("device.revision must be {major, minor}")
    for k in ("major", "minor"):
        v = _int(rev[k], f"device.revision.{k}")
        if not 0 <= v <= 0xFF:
            raise OdError(f"device.revision.{k} must be 0..255 (0x5FFF packs major.minor in a U16)")
    device = dict(device)
    device["vendor_id"] = _int(device["vendor_id"], "device.vendor_id")
    device["product_code"] = _int(device["product_code"], "device.product_code")

    sources = [path]
    raw_objects: list[tuple[dict, str, str]] = []
    for i, o in enumerate(data.get("objects", []) or []):
        raw_objects.append((o, f"objects[{i}]", path.name))
    for inc in data.get("includes", []) or []:
        ip = (path.parent / str(inc)).resolve()
        if not ip.is_file():
            raise OdError(f"includes: {inc} not found")
        sources.append(ip)
        idata = _load_yaml(ip)
        _check_keys(idata, {"schema", "objects", "areas"}, str(ip))
        if idata.get("schema") != SCHEMA_VERSION:
            raise OdError(f"{ip}: schema must be {SCHEMA_VERSION}")
        for i, o in enumerate(idata.get("objects", []) or []):
            raw_objects.append((o, f"{ip.name}:objects[{i}]", ip.name))
        data.setdefault("areas", [])
        data["areas"] = list(data["areas"] or []) + list(idata.get("areas", []) or [])

    objects = [_object(o, p, s) for o, p, s in raw_objects]
    pdo_maps = _expand_pdos(data.get("pdo_defaults", {}) or {}, objects)

    if not fd:
        objects = [o for o in objects if not o.fd_only]
        for o in objects:
            if o.kind == "record":
                kept = [o.entries[0]] + [e for e in o.entries[1:] if not e.fd_only]
                if len(kept) != len(o.entries):
                    o.entries = kept
                    if o.storage:
                        o.entries[0].default = max(e.sub for e in kept[1:]) if len(kept) > 1 else 0
                        o.entries[0].default_expr = str(o.entries[0].default)
    objects.sort(key=lambda o: o.index)
    seen: dict[int, Obj] = {}
    ids: dict[str, Obj] = {}
    for o in objects:
        if o.index in seen:
            raise OdError(f"0x{o.index:04X} defined twice ({seen[o.index].source}, {o.source})")
        seen[o.index] = o
        if o.id in ids:
            raise OdError(f"id '{o.id}' used by 0x{ids[o.id].index:04X} and 0x{o.index:04X}")
        ids[o.id] = o
        cnames = [make_cname(e.name) for e in o.entries]
        if o.kind == "record" and len(set(cnames)) != len(cnames):
            raise OdError(f"0x{o.index:04X}: sub names give duplicate C names {cnames}")
        for lo, hi, label in COUNT_LABELS:
            if lo <= o.index <= hi:
                if o.count_label and o.count_label != label:
                    raise OdError(f"0x{o.index:04X}: count_label must be {label}")
                o.count_label = label

    areas = []
    for i, a in enumerate(data.get("areas", []) or []):
        p = f"areas[{i}]"
        if not isinstance(a, dict):
            raise OdError(f"{p}: must be a mapping")
        _check_keys(a, {"first", "last", "name", "owner", "docs"}, p)
        ar = Area(_int(a.get("first"), p + ".first"), _int(a.get("last"), p + ".last"), str(a.get("name", "")),
                  str(a.get("owner", "")))
        if ar.first > ar.last:
            raise OdError(f"{p}: first > last")
        areas.append(ar)
    areas.sort(key=lambda a: a.first)
    for a, b in zip(areas, areas[1:]):
        if b.first <= a.last:
            raise OdError(f"areas '{a.name}' and '{b.name}' overlap")
    if areas:
        for o in objects:
            if not any(a.first <= o.index <= a.last for a in areas):
                raise OdError(f"0x{o.index:04X} ({o.name}) lies outside every declared area")

    migrations = []
    for i, m in enumerate(data.get("migrations", []) or []):
        p = f"migrations[{i}]"
        if not isinstance(m, dict) or set(m) - {"from", "to", "docs"}:
            raise OdError(f"{p}: needs from / to")
        fi, fs = _parse_map_ref(m.get("from"), p + ".from")
        ti, ts = _parse_map_ref(m.get("to"), p + ".to")
        migrations.append(Migration(fi, fs, ti, ts))

    model = Model(device=device, objects=objects, areas=areas, migrations=migrations,
                  eds=dict(data.get("eds", {}) or {}), sources=sources)
    for m in migrations:
        if model.entry(m.to_index, m.to_sub) is None:
            raise OdError(f"migration target 0x{m.to_index:04X}:{m.to_sub:02X} does not exist")
        if model.entry(m.from_index, m.from_sub) is not None:
            raise OdError(f"migration source 0x{m.from_index:04X}:{m.from_sub:02X} still exists")
    _resolve_pdo_maps(model, pdo_maps, fd)
    for (idx, sub), token in (((0x1018, 3), "$REVISION"), ((0x5FFE, 0), "$OD_HASH"), ((0x5FFF, 0), "$OD_VERSION")):
        e = model.entry(idx, sub)
        if e is not None and e.token != token:
            raise OdError(f"0x{idx:04X}:{sub:02X} must default to {token}")
    for idx in (0x1000, 0x1001, 0x1018):
        if model.find(idx) is None:
            raise OdError(f"mandatory object 0x{idx:04X} missing")
    model.od_hash = compute_hash(model)
    _apply_tokens(model)
    return model


def _hash_view(model: Model) -> dict:
    """Everything that defines the dictionary, minus docs and the self-referential values."""
    skip = {(0x1018, 3), (0x5FFE, 0), (0x5FFF, 0)}
    objs = []
    for o in model.objects:
        ents = []
        for e in o.entries:
            ents.append([e.sub, e.name, e.data_type, e.access, e.pdo,
                         "" if (o.index, e.sub) in skip else (e.token or e.default_expr),
                         None if e.low is None else repr(e.low), None if e.high is None else repr(e.high),
                         e.length, e.persist, e.storage, e.fd_only])
        objs.append([o.index, o.name, o.kind, o.persist, o.storage, o.od_extension, ents])
    dev = {k: model.device[k] for k in ("vendor_id", "product_code", "vendor_name", "product_name")}
    return {"schema": SCHEMA_VERSION, "device": dev, "objects": objs,
            "migrations": [[m.from_index, m.from_sub, m.to_index, m.to_sub] for m in model.migrations]}


def compute_hash(model: Model) -> int:
    blob = json.dumps(_hash_view(model), sort_keys=True, separators=(",", ":")).encode()
    return zlib.crc32(blob) & 0xFFFFFFFF


def _apply_tokens(model: Model) -> None:
    for e in model.all_entries():
        if e.token == "$REVISION":
            e.default = model.revision_number
        elif e.token == "$OD_HASH":
            e.default = model.od_hash
        elif e.token == "$OD_VERSION":
            e.default = model.od_version
        else:
            continue
        lo, hi = _range(e.data_type)
        if not lo <= e.default <= hi:
            raise OdError(f"0x{e.index:04X}:{e.sub:02X}: {e.token} does not fit {e.data_type}")
        e.default_expr = str(e.default)
