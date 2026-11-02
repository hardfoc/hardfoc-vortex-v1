"""docs/od-reference.md emitter."""

from __future__ import annotations

from .emit_eds import eds_value
from .model import Model


def _cell(s: str) -> str:
    return str(s).replace("|", "\\|").replace("\n", " ").strip()


def _default(e) -> str:
    if not e.storage or e.default is None:
        return "(extension)" if e.data_type != "DOM" else "—"
    if e.token:
        return f"`{e.token}` = `{eds_value(e)}`"
    if e.data_type == "VS":
        return f'"{e.default}"'
    return f"`{eds_value(e)}`"


def _range(e) -> str:
    if e.low is None and e.high is None:
        return ""
    lo = eds_value(e, e.low) if e.low is not None else ""
    hi = eds_value(e, e.high) if e.high is not None else ""
    return f"{lo} … {hi}"


def emit(model: Model, banner: list[str]) -> str:
    dev = model.device
    L = ["# Vortex V1 object dictionary", ""] + [f"<!-- {b} -->" for b in banner if b] + [""]
    L += ["| | |", "|---|---|", f"| Device | {dev['product_name']} |", f"| Vendor ID | `0x{dev['vendor_id']:08X}` |",
          f"| Product code | `0x{dev['product_code']:08X}` |",
          f"| Revision (0x1018:03) | {model.revision_str} = `0x{model.revision_number:08X}` |",
          f"| OD hash (0x5FFE) | `0x{model.od_hash:08X}` |", f"| Objects | {len(model.objects)} |", ""]
    L += ["Access: `ro`, `rw`, `wo`, `const`. PDO: `t` TPDO-mappable, `r` RPDO-mappable, `tr` both. "
          "Persist: `comm` (0x1010:02), `app` (0x1010:03), `cal` (0x1010:04). "
          "`$NODEID+x` defaults add the node-ID at boot. *(extension)*: served by an OD extension.", ""]
    if model.areas:
        L += ["## Areas", "", "| First | Last | Area | Owner |", "|---|---|---|---|"]
        L += [f"| `0x{a.first:04X}` | `0x{a.last:04X}` | {_cell(a.name)} | {_cell(a.owner)} |" for a in model.areas]
        L.append("")

    def area_of(index: int) -> str:
        for a in model.areas:
            if a.first <= index <= a.last:
                return a.name
        return "Objects"

    current = None
    for o in model.objects:
        a = area_of(o.index)
        if a != current:
            current = a
            L += [f"## {a}", ""]
        L += [f"### 0x{o.index:04X} {o.name}", ""]
        meta = [f"{o.kind}", f"persist `{o.persist}`"]
        if o.od_extension:
            meta.append(f"extension `{o.od_extension}`")
        if o.tags:
            meta.append("tags " + ", ".join(f"`{t}`" for t in o.tags))
        L += ["*" + ", ".join(meta) + "*", ""]
        if o.docs:
            L += [o.docs, ""]
        L += ["| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |",
              "|---|---|---|---|---|---|---|---|---|"]
        for e in o.entries:
            L.append(f"| {e.sub:02X} | {_cell(e.name)} | {e.data_type}{f'[{e.size}]' if e.data_type in ('VS', 'OS') else ''}"
                     f" | {e.access} | {e.pdo or ''} | {_cell(_default(e))} | {_cell(_range(e))} | {_cell(e.units)} "
                     f"| {_cell(e.docs)} |")
        L.append("")
    if model.migrations:
        L += ["## Migrations", "", "| From | To |", "|---|---|"]
        L += [f"| `0x{m.from_index:04X}:{m.from_sub:02X}` | `0x{m.to_index:04X}:{m.to_sub:02X}` |"
              for m in model.migrations]
        L.append("")
    return "\n".join(L)
