"""C++ (contracts/vortex_od_ids.hpp) and Python (vortex_canopen/od_ids.py) identifier emitters."""

from __future__ import annotations

from .model import FLOATS, PERSIST_GROUP_ID, SIGNED, TYPES, Entry, Model, Obj


def pascal(s: str) -> str:
    return "".join(p[:1].upper() + p[1:] for p in s.split("_") if p)


def entry_key(o: Obj, e: Entry) -> str:
    """Stable identifier of one entry: object id, plus the sub id for arrays / records."""
    return o.id if o.kind == "var" else f"{o.id}_{e.id}"


def _persisted(o: Obj, e: Entry) -> bool:
    return e.persist != "none" and e.storage and e.default is not None and e.access in ("rw", "wo")


def _cpp_int(v: int, t: str) -> str:
    if t in SIGNED:
        bits = 8 * TYPES[t][1]
        if v == -(1 << (bits - 1)):
            return f"({v + 1} - 1)"
        return str(v)
    return f"0x{v:X}"


def _cpp_str(s: str) -> str:
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


def emit_cpp(model: Model, banner: list[str]) -> str:
    major, minor = model.revision
    L = ["#pragma once", "/**", " * @file vortex_od_ids.hpp", " * @brief Object dictionary identifiers and tables."]
    L += [" *" + (" " + b if b else "") for b in [""] + banner]
    L += [" */", "#include <cstddef>", "#include <cstdint>", "", '#include "contracts/od_store_codec.hpp"', "",
          "namespace vortex::od {", "", "namespace odstore = ::vortex::contracts::odstore;", "",
          f"inline constexpr uint32_t kOdHash = 0x{model.od_hash:08X}U;  ///< also 0x5FFE",
          f"inline constexpr uint8_t kRevisionMajor = {major}U;",
          f"inline constexpr uint8_t kRevisionMinor = {minor}U;",
          f"inline constexpr uint32_t kRevisionNumber = 0x{model.revision_number:08X}U;  ///< 0x1018:03",
          f"inline constexpr uint16_t kOdVersion = 0x{model.od_version:04X}U;  ///< 0x5FFF", "",
          "/** @brief One OD address. */", "struct Address {", "  uint16_t index;", "  uint8_t sub;", "};",
          "constexpr bool operator==(Address a, Address b) noexcept { return a.index == b.index && a.sub == b.sub; }",
          "", "/** @brief Object indices. */", "namespace idx {"]
    for o in model.objects:
        L.append(f"inline constexpr uint16_t k{pascal(o.id)} = 0x{o.index:04X}U;  ///< {o.name}")
    keys = [entry_key(o, e) for o in model.objects for e in o.entries]
    dup = sorted({k for k in keys if keys.count(k) > 1})
    if dup:
        raise ValueError(f"entry keys collide: {dup}")
    L += ["}  // namespace idx", "", "/** @brief Entry addresses (object, or object + sub). */", "namespace addr {"]
    for o in model.objects:
        for e in o.entries:
            L.append(f"inline constexpr Address k{pascal(entry_key(o, e))}{{0x{o.index:04X}U, 0x{e.sub:02X}U}};")
    L += ["}  // namespace addr", "",
          "enum class Access : uint8_t { Ro = 0, Rw, Wo, Const };",
          "namespace pdo {", "inline constexpr uint8_t kNone = 0U;", "inline constexpr uint8_t kTpdo = 1U;",
          "inline constexpr uint8_t kRpdo = 2U;", "}  // namespace pdo", "",
          "/** @brief Description of one entry. Integers hold the default in `default_int` (the base value for",
          " *         $NODEID-relative defaults), reals in `default_real`, strings in `default_str`. */",
          "struct EntryInfo {", "  Address address;", "  uint8_t type;  ///< odstore::dtype code",
          "  Access access;", "  uint8_t pdo;   ///< pdo bits", "  odstore::Group persist;",
          "  uint8_t length;  ///< bytes (string capacity for VS / OS)", "  bool has_storage;",
          "  bool node_relative;", "  int64_t default_int;", "  double default_real;", "  const char* default_str;",
          "  const char* name;", "};", "", "inline constexpr EntryInfo kEntries[] = {"]
    acc = {"ro": "Access::Ro", "rw": "Access::Rw", "wo": "Access::Wo", "const": "Access::Const"}
    grp = {"none": "odstore::Group::None", "comm": "odstore::Group::Comm", "app": "odstore::Group::App",
           "cal": "odstore::Group::Cal"}
    for o in model.objects:
        for e in o.entries:
            pdo = " | ".join(x for x, f in (("pdo::kTpdo", "t"), ("pdo::kRpdo", "r")) if f in e.pdo) or "pdo::kNone"
            has = e.storage and e.default is not None
            dint = _cpp_int(int(e.default), e.data_type) if has and e.data_type not in FLOATS | {"VS", "OS"} else "0"
            dreal = repr(float(e.default)) if has and e.data_type in FLOATS else "0.0"
            dstr = _cpp_str(e.default) if has and e.data_type == "VS" else "nullptr"
            L.append(f"    {{{{0x{o.index:04X}U, 0x{e.sub:02X}U}}, 0x{e.code:02X}U, {acc[e.access]}, "
                     f"static_cast<uint8_t>({pdo}), {grp[e.persist]}, {e.size}U, {str(has).lower()}, "
                     f"{str(e.node_relative).lower()}, {dint}, {dreal}, {dstr}, {_cpp_str(e.name)}}},")
    L += ["};", "inline constexpr size_t kEntryCount = sizeof(kEntries) / sizeof(kEntries[0]);", ""]

    L += ["/** @brief Persistable entries for the TLV store (od_store_codec.hpp). */",
          "inline constexpr odstore::ObjectSpec kPersistSpec[] = {"]
    for o in model.objects:
        for e in o.entries:
            if not _persisted(o, e):
                continue
            lim = (e.low is not None or e.high is not None) and e.data_type not in FLOATS
            if lim:
                lo_d, hi_d = _range_of(e.data_type)
                lo = e.low if e.low is not None else lo_d
                hi = e.high if e.high is not None else hi_d
                lims = f"true, {_cpp_i64(lo)}, {_cpp_i64(hi)}"
            else:
                lims = "false, 0, 0"
            L.append(f"    {{0x{o.index:04X}U, 0x{e.sub:02X}U, 0x{e.code:02X}U, {grp[e.persist]}, {e.size}U, {lims}}},")
    L += ["};", "inline constexpr size_t kPersistSpecCount = sizeof(kPersistSpec) / sizeof(kPersistSpec[0]);", ""]

    L += ["/** @brief Renamed addresses (YAML `migrations:`). */"]
    if model.migrations:
        L.append("inline constexpr odstore::Migration kMigrations[] = {")
        L += [f"    {{0x{m.from_index:04X}U, 0x{m.from_sub:02X}U, 0x{m.to_index:04X}U, 0x{m.to_sub:02X}U}},"
              for m in model.migrations]
        L += ["};", f"inline constexpr size_t kMigrationCount = {len(model.migrations)}U;", ""]
    else:
        L += ["inline constexpr odstore::Migration kMigrations[1] = {{0U, 0U, 0U, 0U}};",
              "inline constexpr size_t kMigrationCount = 0U;", ""]

    rel = [(o, e) for o in model.objects for e in o.entries if e.node_relative]
    L += ["/** @brief Defaults that are $NODEID + base. CANopenNode adds the node-ID itself only for 0x1014,",
          " *         0x1200 and PDOs 1-4; CoNode applies this table before CO_CANopenInit. */",
          "struct NodeIdDefault {", "  Address address;", "  uint32_t base;", "};",
          "inline constexpr NodeIdDefault kNodeIdRelative[] = {"]
    L += [f"    {{{{0x{o.index:04X}U, 0x{e.sub:02X}U}}, 0x{e.default:08X}U}}," for o, e in rel]
    L += ["};", "inline constexpr size_t kNodeIdRelativeCount = sizeof(kNodeIdRelative) / sizeof(kNodeIdRelative[0]);",
          ""]

    ext = [o for o in model.objects if o.od_extension]
    L += ["/** @brief Objects served by an OD extension (YAML `od_extension:`). */",
          "struct Extension {", "  uint16_t index;", "  const char* handler;", "};",
          "inline constexpr Extension kExtensions[] = {"]
    L += [f"    {{0x{o.index:04X}U, {_cpp_str(o.od_extension)}}}," for o in ext]
    L += ["};", "inline constexpr size_t kExtensionCount = sizeof(kExtensions) / sizeof(kExtensions[0]);", "",
          "}  // namespace vortex::od", ""]
    return "\n".join(L)


def _range_of(t: str) -> tuple[int, int]:
    from .model import _range

    lo, hi = _range(t)
    return int(lo), int(hi)


def _cpp_i64(v) -> str:
    v = int(v)
    if v == -(1 << 63):
        return "(-9223372036854775807LL - 1)"
    if v > (1 << 63) - 1:
        return f"static_cast<int64_t>(0x{v:X}ULL)"
    return f"{v}LL" if abs(v) > 0x7FFFFFFF else str(v)


def emit_py(model: Model, banner: list[str]) -> str:
    major, minor = model.revision
    L = ['"""Object dictionary identifiers for the Vortex V1 CANopen device.', ""] + banner + ['"""', "",
         "from __future__ import annotations", "", "from typing import NamedTuple", "",
         f"OD_HASH = 0x{model.od_hash:08X}", f"REVISION = ({major}, {minor})",
         f"REVISION_NUMBER = 0x{model.revision_number:08X}", f"OD_VERSION = 0x{model.od_version:04X}", "", "",
         "class Entry(NamedTuple):", "    index: int", "    sub: int", "    key: str", "    name: str",
         "    data_type: str", "    access: str", "    pdo: str", "    default: object",
         "    node_relative: bool", "    low: object", "    high: object", "    persist: str", "    units: str",
         "    scale: float | None", "", ""]
    for o in model.objects:
        L.append(f"IDX_{o.id.upper()} = 0x{o.index:04X}")
    L += ["", "ENTRIES: tuple[Entry, ...] = ("]
    for o in model.objects:
        for e in o.entries:
            dflt = e.default if not isinstance(e.default, bytes) else e.default
            L.append(f"    Entry(0x{o.index:04X}, 0x{e.sub:02X}, {entry_key(o, e)!r}, {e.name!r}, {e.data_type!r}, "
                     f"{e.access!r}, {e.pdo!r}, {dflt!r}, {e.node_relative!r}, {e.low!r}, {e.high!r}, "
                     f"{e.persist!r}, {e.units!r}, {e.scale!r}),")
    L += [")", "", "BY_KEY: dict[str, Entry] = {e.key: e for e in ENTRIES}",
          "BY_ADDRESS: dict[tuple[int, int], Entry] = {(e.index, e.sub): e for e in ENTRIES}", ""]
    return "\n".join(L)


__all__ = ["emit_cpp", "emit_py", "entry_key", "pascal", "PERSIST_GROUP_ID"]
