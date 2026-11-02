"""CANopenNode v4 OD.h / OD.c emitter.

Matches the layout CANopenEditor writes (CANopenNode `example/OD.c`): one
struct per storage group, OD_obj_var_t / OD_obj_array_t / OD_obj_record_t
descriptors, the sorted ODList and the OD_INIT_CONFIG block. A test feeds
the CANopenNode example EDS through this emitter and compares the result
with the example OD.c / OD.h.
"""

from __future__ import annotations

from .model import FLOATS, PERSIST, SIGNED, TYPES, Entry, Model, Obj, make_cname

BANNER = "/" + "*" * 79
END = "*" * 79 + "/"


def _group(o: Obj) -> str:
    return PERSIST[o.persist] if o.persist in PERSIST else o.persist


def _c_value(e: Entry) -> str:
    t = e.data_type
    v = e.default
    if t == "VS":
        s = v.replace("\\", "\\\\").replace('"', '\\"')
        return f'"{s}"'
    if t == "OS":
        return "{" + ", ".join(f"0x{b:02X}" for b in v) + "}" if v else "{0}"
    if t in FLOATS:
        return repr(float(v)) + ("F" if t == "R32" else "")
    if t == "BOOL":
        return "true" if v else "false"
    if t in SIGNED:
        bits = 8 * TYPES[t][1]
        if v == -(1 << (bits - 1)):
            return f"({v + 1} - 1)"
        return str(v)
    width = 2 * TYPES[t][1]
    return f"0x{v:0{width}X}"


def _attr(e: Entry, sub0_of_array: bool = False) -> str:
    acc = {"ro": "ODA_SDO_R", "const": "ODA_SDO_R", "wo": "ODA_SDO_W", "rw": "ODA_SDO_RW"}[e.access]
    parts = [acc]
    if e.pdo == "t":
        parts.append("ODA_TPDO")
    elif e.pdo == "r":
        parts.append("ODA_RPDO")
    elif e.pdo == "tr":
        parts.append("ODA_TRPDO")
    size = TYPES[e.data_type][1]
    if size is not None and size > 1:
        parts.append("ODA_MB")
    if e.data_type == "VS":
        parts.append("ODA_STR")
    del sub0_of_array
    return " | ".join(parts)


def _has_data(e: Entry) -> bool:
    return e.storage and e.default is not None and e.data_type != "DOM"


def _member_decl(e: Entry, name: str) -> str:
    t = e.data_type
    if t == "VS":
        return f"char {name}[{e.size + 1}];"
    if t == "OS":
        return f"uint8_t {name}[{e.size}];"
    return f"{TYPES[t][2]} {name};"


def _var_name(o: Obj) -> str:
    return f"x{o.index:04X}_{o.cname}"


def _obj_name(o: Obj) -> str:
    return f"o_{o.index:04X}_{o.cname}"


def _groups(model: Model) -> list[str]:
    out: list[str] = []
    for o in model.objects:
        g = _group(o)
        if g not in out and any(_has_data(e) for e in o.entries):
            out.append(g)
    return out


def _count_labels(model: Model) -> list[tuple[str, int]]:
    out: dict[str, int] = {}
    for o in model.objects:
        if o.count_label:
            out[o.count_label] = out.get(o.count_label, 0) + 1
    return list(out.items())


def emit_h(model: Model, header_comment: str) -> str:
    L: list[str] = [header_comment, "", "#ifndef OD_H", "#define OD_H"]
    L += [BANNER, "    Counters of OD objects", END]
    for label, n in _count_labels(model):
        L.append(f"#define OD_CNT_{label} {n}")
    L += ["", "", BANNER, "    Sizes of OD arrays", END]
    for o in model.objects:
        if o.kind == "array":
            L.append(f"#define OD_CNT_ARR_{o.index:04X} {o.array_len}")
    L += ["", "", BANNER, "    OD data declaration of all groups", END]
    for g in _groups(model):
        L.append("typedef struct {")
        for o in model.objects:
            if _group(o) != g:
                continue
            vn = _var_name(o)
            if o.kind == "var":
                e = o.entries[0]
                if _has_data(e):
                    L.append("    " + _member_decl(e, vn))
            elif o.kind == "array":
                s0, el = o.entries[0], o.entries[1]
                if _has_data(s0):
                    L.append(f"    uint8_t {vn}_sub0;")
                if _has_data(el):
                    L.append(f"    {TYPES[el.data_type][2]} {vn}[OD_CNT_ARR_{o.index:04X}];")
            else:
                members = [e for e in o.entries if _has_data(e)]
                if members:
                    L.append("    struct {")
                    for e in members:
                        L.append("        " + _member_decl(e, make_cname(e.name)))
                    L.append(f"    }} {vn};")
        L.append(f"}} OD_{g}_t;")
        L.append("")
    for g in _groups(model):
        L += [f"#ifndef OD_ATTR_{g}", f"#define OD_ATTR_{g}", "#endif", f"extern OD_ATTR_{g} OD_{g}_t OD_{g};", ""]
    L += ["#ifndef OD_ATTR_OD", "#define OD_ATTR_OD", "#endif", "extern OD_ATTR_OD OD_t *OD;", "", ""]
    L += [BANNER, "    Object dictionary entries - shortcuts", END]
    for i, o in enumerate(model.objects):
        L.append(f"#define OD_ENTRY_H{o.index:04X} &OD->list[{i}]")
    L += ["", "", BANNER, "    Object dictionary entries - shortcuts with names", END]
    for i, o in enumerate(model.objects):
        L.append(f"#define OD_ENTRY_H{o.index:04X}_{o.cname} &OD->list[{i}]")
    L += ["", "", BANNER, "    OD config structure", END]
    L += _init_config(model)
    L += ["", "#endif /* OD_H */", ""]
    return "\n".join(L)


def _init_config(model: Model) -> list[str]:
    cnt = dict(_count_labels(model))

    def has(idx: int) -> bool:
        return model.find(idx) is not None

    def entry(idx: int) -> str:
        return f"OD_ENTRY_H{idx:04X}" if has(idx) else "NULL"

    def count(label: str) -> str:
        return f"OD_CNT_{label}" if label in cnt else "0"

    def arr(idx: int) -> str:
        return f"OD_CNT_ARR_{idx:04X}" if has(idx) and model.find(idx).kind == "array" else "0"

    fields = [
        ("CNT_NMT", count("NMT")), ("ENTRY_H1017", entry(0x1017)),
        ("CNT_HB_CONS", count("HB_CONS")), ("CNT_ARR_1016", arr(0x1016)), ("ENTRY_H1016", entry(0x1016)),
        ("CNT_EM", count("EM")), ("ENTRY_H1001", entry(0x1001)), ("ENTRY_H1014", entry(0x1014)),
        ("ENTRY_H1015", entry(0x1015)), ("CNT_ARR_1003", arr(0x1003)), ("ENTRY_H1003", entry(0x1003)),
        ("CNT_SDO_SRV", count("SDO_SRV")), ("ENTRY_H1200", entry(0x1200)),
        ("CNT_SDO_CLI", count("SDO_CLI")), ("ENTRY_H1280", entry(0x1280)),
        ("CNT_TIME", count("TIME")), ("ENTRY_H1012", entry(0x1012)),
        ("CNT_SYNC", count("SYNC")), ("ENTRY_H1005", entry(0x1005)), ("ENTRY_H1006", entry(0x1006)),
        ("ENTRY_H1007", entry(0x1007)), ("ENTRY_H1019", entry(0x1019)),
        ("CNT_RPDO", count("RPDO")), ("ENTRY_H1400", entry(0x1400)), ("ENTRY_H1600", entry(0x1600)),
        ("CNT_TPDO", count("TPDO")), ("ENTRY_H1800", entry(0x1800)), ("ENTRY_H1A00", entry(0x1A00)),
        ("CNT_LEDS", "0"), ("CNT_GFC", "0"), ("ENTRY_H1300", "NULL"), ("CNT_SRDO", "0"),
        ("ENTRY_H1301", "NULL"), ("ENTRY_H1381", "NULL"), ("ENTRY_H13FE", "NULL"), ("ENTRY_H13FF", "NULL"),
        ("CNT_LSS_SLV", "0"), ("CNT_LSS_MST", "0"), ("CNT_GTWA", "0"), ("CNT_TRACE", "0"),
    ]
    out = ["#ifdef CO_MULTIPLE_OD", "#define OD_INIT_CONFIG(config) {\\"]
    out += [f"    (config).{k} = {v};\\" for k, v in fields]
    out += ["}", "#endif"]
    return out


def _init_lines(o: Obj) -> list[str]:
    """Initialiser lines of one object inside its group (no trailing comma handling)."""
    vn = _var_name(o)
    if o.kind == "var":
        e = o.entries[0]
        return [f"    .{vn} = {_c_value(e)}"] if _has_data(e) else []
    if o.kind == "array":
        s0, els = o.entries[0], o.entries[1:]
        out = []
        if _has_data(s0):
            out.append(f"    .{vn}_sub0 = {_c_value(s0)}")
        if _has_data(els[0]):
            out.append(f"    .{vn} = {{" + ", ".join(_c_value(e) for e in els) + "}")
        return [",\n".join(out)] if out else []
    members = [e for e in o.entries if _has_data(e)]
    if not members:
        return []
    inner = [f"        .{make_cname(e.name)} = {_c_value(e)}" for e in members]
    return [f"    .{vn} = {{", ",\n".join(inner), "    }"]


def _join_init(blocks: list[list[str]]) -> list[str]:
    out: list[str] = []
    blocks = [b for b in blocks if b]
    for i, b in enumerate(blocks):
        text = "\n".join(b)
        out.append(text + ("," if i < len(blocks) - 1 else ""))
    return out


def _descriptor(o: Obj, model: Model) -> list[str]:
    g = _group(o)
    vn = _var_name(o)
    on = _obj_name(o)
    if o.kind == "var":
        e = o.entries[0]
        if not _has_data(e):
            ptr = "NULL"
        elif e.data_type in ("VS", "OS"):
            ptr = f"&OD_{g}.{vn}[0]"
        else:
            ptr = f"&OD_{g}.{vn}"
        return [f"    .{on} = {{", f"        .dataOrig = {ptr},", f"        .attribute = {_attr(e)},",
                f"        .dataLength = {e.size}", "    }"]
    if o.kind == "array":
        s0, el = o.entries[0], o.entries[1]
        p0 = f"&OD_{g}.{vn}_sub0" if _has_data(s0) else "NULL"
        p = f"&OD_{g}.{vn}[0]" if _has_data(el) else "NULL"
        return [f"    .{on} = {{", f"        .dataOrig0 = {p0},", f"        .dataOrig = {p},",
                f"        .attribute0 = {_attr(s0, True)},", f"        .attribute = {_attr(el)},",
                f"        .dataElementLength = {el.size},",
                f"        .dataElementSizeof = sizeof({TYPES[el.data_type][2]})", "    }"]
    subs = []
    for e in o.entries:
        if not _has_data(e):
            ptr = "NULL"
        elif e.data_type in ("VS", "OS"):
            ptr = f"&OD_{g}.{vn}.{make_cname(e.name)}[0]"
        else:
            ptr = f"&OD_{g}.{vn}.{make_cname(e.name)}"
        subs.append("\n".join([
            "        {", f"            .dataOrig = {ptr},", f"            .subIndex = {e.sub},",
            f"            .attribute = {_attr(e)},", f"            .dataLength = {e.size}", "        }"]))
    del model
    return [f"    .{on} = {{", ",\n".join(subs), "    }"]


def _odt(o: Obj) -> str:
    return {"var": "ODT_VAR", "array": "ODT_ARR", "record": "ODT_REC"}[o.kind]


def emit_c(model: Model, header_comment: str) -> str:
    L: list[str] = [header_comment, "", "#define OD_DEFINITION", '#include "301/CO_ODinterface.h"', '#include "OD.h"',
                    "", "#if CO_VERSION_MAJOR < 4",
                    "#error This Object dictionary is compatible with CANopenNode V4.0 and above!", "#endif", ""]
    L += [BANNER, "    OD data initialization of all groups", END]
    for g in _groups(model):
        L.append(f"OD_ATTR_{g} OD_{g}_t OD_{g} = {{")
        L += _join_init([_init_lines(o) for o in model.objects if _group(o) == g])
        L += ["};", ""]
    L += ["", "", BANNER, "    All OD objects (constant definitions)", END, "typedef struct {"]
    for o in model.objects:
        if o.kind == "var":
            L.append(f"    OD_obj_var_t {_obj_name(o)};")
        elif o.kind == "array":
            L.append(f"    OD_obj_array_t {_obj_name(o)};")
        else:
            L.append(f"    OD_obj_record_t {_obj_name(o)}[{len(o.entries)}];")
    L += ["} ODObjs_t;", "", "static CO_PROGMEM ODObjs_t ODObjs = {"]
    L += _join_init([_descriptor(o, model) for o in model.objects])
    L += ["};", "", "", BANNER, "    Object dictionary", END, "static OD_ATTR_OD OD_entry_t ODList[] = {"]
    for o in model.objects:
        n = 1 if o.kind == "var" else len(o.entries)
        L.append(f"    {{0x{o.index:04X}, 0x{n:02X}, {_odt(o)}, &ODObjs.{_obj_name(o)}, NULL}},")
    L += ["    {0x0000, 0x00, 0, NULL, NULL}", "};", "", "static OD_t _OD = {",
          "    (sizeof(ODList) / sizeof(ODList[0])) - 1,", "    &ODList[0]", "};", "", "OD_t *OD = &_OD;", ""]
    return "\n".join(L)
