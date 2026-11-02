"""CiA 306 EDS emitter (EDSVersion 4.0)."""

from __future__ import annotations

from .model import FLOATS, KINDS, PERSIST, SIGNED, TYPES, Entry, Model

BAUD_RATES = (10, 20, 50, 125, 250, 500, 800, 1000)


def eds_value(e: Entry, v=None) -> str:
    """DefaultValue / limit text for @p e (@p v overrides the default)."""
    if v is None:
        v = e.default
        if e.node_relative:
            return f"$NODEID+0x{v:X}"
    t = e.data_type
    if v is None:
        return ""
    if t == "VS":
        return v
    if t in ("OS", "DOM"):
        return bytes(v).hex().upper()
    if t in FLOATS:
        return repr(float(v))
    if t in SIGNED:
        return str(int(v))
    return f"0x{int(v):0{2 * TYPES[t][1]}X}"


def _var_lines(e: Entry) -> list[str]:
    out = [f"ParameterName={e.name}", "ObjectType=0x7", f"DataType=0x{e.code:04X}", f"AccessType={e.access}"]
    if e.low is not None:
        out.append(f"LowLimit={eds_value(e, e.low)}")
    if e.high is not None:
        out.append(f"HighLimit={eds_value(e, e.high)}")
    out.append(f"DefaultValue={eds_value(e)}")
    out.append(f"PDOMapping={1 if e.pdo else 0}")
    return out


def emit(model: Model, comment_lines: list[str]) -> str:
    dev = model.device
    eds = model.eds
    major, minor = model.revision
    nr_rx = sum(1 for o in model.objects if 0x1400 <= o.index <= 0x15FF)
    nr_tx = sum(1 for o in model.objects if 0x1800 <= o.index <= 0x19FF)
    rates = set(dev.get("baud_rates", BAUD_RATES))
    L = ["[FileInfo]", "FileName=vortex_v1.eds", f"FileVersion={major}", f"FileRevision={minor}", "LastEDS=",
         "EDSVersion=4.0", f"Description={dev.get('description', '')}",
         f"CreationTime={eds.get('creation_time', '12:00AM')}",
         f"CreationDate={eds.get('creation_date', '01-01-2026')}", f"CreatedBy={dev.get('created_by', '')}",
         f"ModificationTime={eds.get('modification_time', eds.get('creation_time', '12:00AM'))}",
         f"ModificationDate={eds.get('modification_date', eds.get('creation_date', '01-01-2026'))}",
         f"ModifiedBy={dev.get('created_by', '')}", "",
         "[DeviceInfo]", f"VendorName={dev['vendor_name']}", f"VendorNumber=0x{dev['vendor_id']:08X}",
         f"ProductName={dev['product_name']}", f"ProductNumber=0x{dev['product_code']:08X}",
         f"RevisionNumber=0x{model.revision_number:08X}", f"OrderCode={dev.get('order_code', '')}"]
    L += [f"BaudRate_{r}={1 if r in rates else 0}" for r in BAUD_RATES]
    L += ["SimpleBootUpMaster=0", "SimpleBootUpSlave=0", f"Granularity={dev.get('granularity', 8)}",
          "DynamicChannelsSupported=0", "CompactPDO=0", "GroupMessaging=0", f"NrOfRXPDO={nr_rx}",
          f"NrOfTXPDO={nr_tx}", f"LSS_Supported={1 if dev.get('lss_supported', True) else 0}", "",
          "[DummyUsage]", "Dummy0001=0"] + [f"Dummy000{i}=1" for i in range(2, 8)] + [""]
    L += ["[Comments]", f"Lines={len(comment_lines)}"] + [f"Line{i}={c}" for i, c in enumerate(comment_lines, 1)]
    L.append("")

    mandatory = [o for o in model.objects if o.index in (0x1000, 0x1001, 0x1018)]
    optional = [o for o in model.objects if o not in mandatory and not 0x2000 <= o.index <= 0x5FFF]
    manufacturer = [o for o in model.objects if 0x2000 <= o.index <= 0x5FFF]
    for title, objs in (("MandatoryObjects", mandatory), ("OptionalObjects", optional),
                        ("ManufacturerObjects", manufacturer)):
        L += [f"[{title}]", f"SupportedObjects={len(objs)}"]
        L += [f"{i}=0x{o.index:04X}" for i, o in enumerate(objs, 1)]
        L.append("")
        for o in objs:
            sec = f"{o.index:04X}"
            group = PERSIST[o.persist]
            if o.kind == "var":
                e = o.entries[0]
                lines = _var_lines(e)
                L += [f"[{sec}]", lines[0], lines[1], f";StorageLocation={group}"] + lines[2:] + [""]
                continue
            L += [f"[{sec}]", f"ParameterName={o.name}", f"ObjectType=0x{KINDS[o.kind]:X}",
                  f";StorageLocation={group}", f"SubNumber=0x{len(o.entries):X}", ""]
            for e in o.entries:
                L += [f"[{sec}sub{e.sub:X}]"] + _var_lines(e) + [""]
    return "\n".join(L)
