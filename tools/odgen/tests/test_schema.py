"""Schema validation: every rule the loader enforces has a failing example."""

from __future__ import annotations

import copy

import pytest
import yaml

from odgen import model as m

BASE = {
    "schema": 1,
    "device": {"vendor_name": "v", "vendor_id": 0, "product_name": "p", "product_code": 1,
               "revision": {"major": 1, "minor": 2}},
    "objects": [
        {"index": 0x1000, "id": "device_type", "name": "Device type", "data_type": "U32", "access": "const",
         "default": 0x192},
        {"index": 0x1001, "id": "error_register", "name": "Error register", "data_type": "U8", "access": "ro",
         "pdo": "t"},
        {"index": 0x1018, "id": "identity", "name": "Identity", "object": "record", "subs": [
            {"sub": 1, "name": "Vendor-ID", "data_type": "U32", "access": "const"},
            {"sub": 3, "name": "Revision number", "data_type": "U32", "access": "const", "default": "$REVISION"}]},
        {"index": 0x6040, "id": "controlword", "name": "Controlword", "data_type": "U16", "access": "rw", "pdo": "r"},
        {"index": 0x6041, "id": "statusword", "name": "Statusword", "data_type": "U16", "access": "ro", "pdo": "t"},
    ],
}


def load(tmp_path, data, name="od.yaml", fd=False):
    p = tmp_path / name
    p.write_text(yaml.safe_dump(data, sort_keys=False))
    return m.load(p, fd=fd)


def with_obj(**obj):
    d = copy.deepcopy(BASE)
    d["objects"].append(obj)
    return d


def var(index=0x2000, **kw):
    o = {"index": index, "id": f"x{index:04x}", "name": f"Object {index:04X}", "data_type": "U16", "access": "rw"}
    o.update(kw)
    return o


def test_base_loads(tmp_path):
    md = load(tmp_path, BASE)
    assert [o.index for o in md.objects] == [0x1000, 0x1001, 0x1018, 0x6040, 0x6041]
    assert md.entry(0x1018, 3).default == (1 << 16) | 2
    assert md.find(0x1018).entries[0].default == 3  # highest sub-index generated
    assert md.find(0x1000).count_label == "NMT"


@pytest.mark.parametrize("bad,msg", [
    (var(data_type="U12"), "data_type"),
    (var(access="rx"), "access"),
    (var(pdo="x"), "pdo"),
    (var(pdo="r", access="ro"), "RPDO-mappable entry must be writable"),
    (var(pdo="t", access="wo"), "TPDO-mappable entry must be readable"),
    (var(data_type="VS", pdo="t", default="a"), "not PDO mappable"),
    (var(default=70000), "out of range"),
    (var(data_type="I8", default=-129), "out of range"),
    (var(low=10, high=5), "low > high"),
    (var(default=3, low=5), "default below low"),
    (var(default=9, high=5), "default above high"),
    (var(data_type="I16", default="$NODEID+0x80"), "unsigned"),
    (var(data_type="U32", default="$NODEID-1"), "\\$NODEID\\+"),
    (var(data_type="U8", default="$OD_HASH"), "U16 or U32"),
    (var(data_type="VS", default="a;b"), "';'"),
    (var(data_type="VS"), "needs a default or a length"),
    (var(data_type="OS", length=4, default="0102"), "whole length"),
    (var(data_type="DOM", default=1), "no default"),
    (var(data_type="DOM"), "need od_extension"),
    (var(storage=False), "needs od_extension"),
    (var(storage=False, od_extension="H", default=1), "no default"),
    (var(length=4), "VS / OS only"),
    (var(bogus=1), "unknown key 'bogus'"),
    (var(name="a;b"), "names may not contain"),
    (var(id="BadId"), "snake_case"),
    (var(index=0x0FFF), "outside"),
    (var(object="struct"), "var, array or record"),
    (var(persist="flash"), "persist"),
    (var(subs=[]), "'subs' is for arrays / records"),
    ({"index": 0x2000, "id": "r", "name": "R", "object": "record", "subs": []}, "non-empty 'subs'"),
    ({"index": 0x2000, "id": "r", "name": "R", "object": "record", "data_type": "U8",
      "subs": [{"sub": 1, "name": "a", "data_type": "U8", "access": "ro"}]}, "belongs on each sub"),
    ({"index": 0x2000, "id": "r", "name": "R", "object": "record",
      "subs": [{"sub": 2, "name": "a", "data_type": "U8", "access": "ro"},
               {"sub": 1, "name": "b", "data_type": "U8", "access": "ro"}]}, "unique and ascending"),
    ({"index": 0x2000, "id": "r", "name": "R", "object": "record",
      "subs": [{"sub": 0, "name": "a", "data_type": "U8", "access": "ro"}]}, "sub must be 1..254"),
    ({"index": 0x2000, "id": "r", "name": "R", "object": "record",
      "subs": [{"sub": 1, "name": "Same", "data_type": "U8", "access": "ro"},
               {"sub": 2, "name": "Same", "data_type": "U8", "access": "ro"}]}, "duplicate sub ids"),
    ({"index": 0x2000, "id": "r", "name": "R", "object": "record",
      "subs": [{"sub": 1, "id": "a", "name": "Same", "data_type": "U8", "access": "ro"},
               {"sub": 2, "id": "b", "name": "Same", "data_type": "U8", "access": "ro"}]}, "duplicate C names"),
    ({"index": 0x2000, "id": "a", "name": "A", "object": "array", "data_type": "VS", "access": "ro",
      "elements": ["x"]}, "fixed-size"),
    ({"index": 0x2000, "id": "a", "name": "A", "object": "array", "data_type": "U8", "access": "ro",
      "elements": []}, "non-empty 'elements'"),
    ({"index": 0x2000, "id": "a", "name": "A", "object": "array", "data_type": "U8", "access": "ro",
      "default": [1, 2], "elements": ["x"]}, "default list"),
    ({"index": 0x2000, "id": "a", "name": "A", "object": "array", "data_type": "U8", "access": "ro",
      "subs": []}, "arrays use 'elements'"),
])
def test_rejects(tmp_path, bad, msg):
    with pytest.raises(m.OdError, match=msg):
        load(tmp_path, with_obj(**bad))


def test_extension_keys_pass_through(tmp_path):
    md = load(tmp_path, with_obj(**var(x_tmc_param="ACTUAL_VELOCITY", tags=["telemetry"])))
    o = md.find(0x2000)
    assert o.extra == {"x_tmc_param": "ACTUAL_VELOCITY"}
    assert o.tags == ["telemetry"]


@pytest.mark.parametrize("change,msg", [
    (lambda d: d.update(schema=2), "schema must be 1"),
    (lambda d: d.update(colour=1), "unknown key 'colour'"),
    (lambda d: d["device"].pop("vendor_id"), "vendor_id is required"),
    (lambda d: d["device"].update(revision={"major": 256, "minor": 0}), "0..255"),
    (lambda d: d["device"].update(revision={"major": 1}), "major, minor"),
    (lambda d: d["objects"].pop(1), "mandatory object 0x1001"),
    (lambda d: d["objects"].append(dict(d["objects"][0])), "defined twice"),
    (lambda d: d["objects"].append(var(id="device_type")), "id 'device_type' used"),
    (lambda d: d["objects"][2]["subs"][1].update(default=5), "must default to \\$REVISION"),
    (lambda d: d["objects"].append(var(index=0x5FFE, data_type="U32", access="const", default=1)),
     "must default to \\$OD_HASH"),
    (lambda d: d.update(areas=[{"first": 0x1000, "last": 0x1FFF, "name": "a"},
                               {"first": 0x1FFF, "last": 0x2FFF, "name": "b"}]), "overlap"),
    (lambda d: d.update(areas=[{"first": 0x1000, "last": 0x1FFF, "name": "comm"}]), "outside every declared area"),
    (lambda d: d.update(migrations=[{"from": "0x2000:01", "to": "0x2001:01"}]), "does not exist"),
    (lambda d: d.update(migrations=[{"from": "0x6040", "to": "0x6041"}]), "still exists"),
    (lambda d: d.update(includes=["missing.yaml"]), "not found"),
    (lambda d: d["objects"].__setitem__(0, dict(d["objects"][0], count_label="EM")), "count_label must be NMT"),
])
def test_rejects_top_level(tmp_path, change, msg):
    d = copy.deepcopy(BASE)
    change(d)
    with pytest.raises(m.OdError, match=msg):
        load(tmp_path, d)


def pdo_data(**tpdo):
    d = copy.deepcopy(BASE)
    entry = {"n": 1, "cob_id": "$NODEID+0x180", "map": ["0x6041"]}
    entry.update(tpdo)
    d["pdo_defaults"] = {"tpdo": [entry], "rpdo": [{"n": 1, "cob_id": "$NODEID+0x200", "map": [0x6040]}]}
    return d


def test_pdo_expansion(tmp_path):
    md = load(tmp_path, pdo_data(map=["0x6041", "0x1001"], event_timer=100))
    comm, mp = md.find(0x1800), md.find(0x1A00)
    assert comm.count_label == "TPDO" and comm.persist == "comm"
    assert [e.sub for e in comm.entries] == [0, 1, 2, 3, 5, 6]
    assert comm.entries[1].node_relative and comm.entries[1].default == 0x180
    assert comm.entries[4].default == 100
    assert mp.entries[0].default == 2 and mp.entries[0].access == "rw"
    assert mp.entries[1].default == 0x60410010
    assert mp.entries[2].default == 0x10010008
    assert md.find(0x1600).entries[1].default == 0x60400010
    assert [e.sub for e in md.find(0x1400).entries] == [0, 1, 2, 5]


@pytest.mark.parametrize("tpdo,msg", [
    ({"map": ["0x6040"]}, "not TPDO mappable"),
    ({"map": ["0x2222"]}, "does not exist"),
    ({"map": ["0x6041"] * 9}, "at most 8"),
    ({"map": ["0x1000"]}, "not TPDO mappable"),
    ({"map": ["6041"]}, "must look like"),
    ({"n": 2}, "without gaps"),
    ({"cob_id": None}, "cob_id is required"),
    ({"colour": 1}, "unknown key"),
])
def test_pdo_rejects(tmp_path, tpdo, msg):
    with pytest.raises(m.OdError, match=msg):
        load(tmp_path, pdo_data(**tpdo))


def test_pdo_length_limit(tmp_path):
    d = pdo_data(map=["0x2000"] * 5)
    d["objects"].append(var(index=0x2000, data_type="U16", access="ro", pdo="t"))
    with pytest.raises(m.OdError, match="80 bits"):
        load(tmp_path, d)


def test_fd_only_objects(tmp_path):
    d = copy.deepcopy(BASE)
    d["objects"].append(var(index=0x2000, fd_only=True))
    d["objects"].append({"index": 0x2001, "id": "r", "name": "R", "object": "record", "subs": [
        {"sub": 1, "name": "a", "data_type": "U8", "access": "ro"},
        {"sub": 2, "name": "b", "data_type": "U8", "access": "ro", "fd_only": True}]})
    classic = load(tmp_path, d)
    fd = load(tmp_path, d, fd=True)
    assert classic.find(0x2000) is None and fd.find(0x2000) is not None
    assert [e.sub for e in classic.find(0x2001).entries] == [0, 1]
    assert classic.find(0x2001).entries[0].default == 1
    assert [e.sub for e in fd.find(0x2001).entries] == [0, 1, 2]
    assert classic.od_hash != fd.od_hash


def test_includes_and_areas(tmp_path):
    (tmp_path / "telemetry.yaml").write_text(yaml.safe_dump({
        "schema": 1,
        "areas": [{"first": 0x3030, "last": 0x303F, "name": "TMC9660 telemetry", "owner": "telemetry"}],
        "objects": [var(index=0x3030, access="ro", pdo="t", tags=["telemetry"])]}))
    d = copy.deepcopy(BASE)
    d["includes"] = ["telemetry.yaml"]
    d["areas"] = [{"first": 0x1000, "last": 0x1FFF, "name": "comm"}, {"first": 0x6000, "last": 0x67FF, "name": "402"}]
    md = load(tmp_path, d)
    assert md.find(0x3030).source == "telemetry.yaml"
    assert [a.name for a in md.areas] == ["comm", "TMC9660 telemetry", "402"]
    assert len(md.sources) == 2


def test_hash_ignores_docs_but_not_content(tmp_path):
    a = load(tmp_path, with_obj(**var(docs="one")), "a.yaml")
    b = load(tmp_path, with_obj(**var(docs="two", units="ms")), "b.yaml")
    c = load(tmp_path, with_obj(**var(default=5)), "c.yaml")
    d = copy.deepcopy(with_obj(**var()))
    d["device"]["revision"] = {"major": 9, "minor": 9}
    e = load(tmp_path, d, "e.yaml")
    assert a.od_hash == b.od_hash
    assert a.od_hash != c.od_hash
    assert a.od_hash == e.od_hash  # the revision bump itself is not a dictionary change


def test_make_cname_matches_canopeneditor():
    cases = {
        "COB-ID SYNC message": "COB_ID_SYNCMessage",
        "Highest sub-index supported": "highestSub_indexSupported",
        "COB-ID used by RPDO": "COB_IDUsedByRPDO",
        "COB-ID client to server (tx)": "COB_IDClientToServerTx",
        "Node-ID of the SDO server": "node_IDOfTheSDOServer",
        "SYNC start value": "SYNCStartValue",
        "Pre-defined error field": "pre_definedErrorField",
        "Inhibit time EMCY": "inhibitTimeEMCY",
        "COB-ID EMCY": "COB_ID_EMCY",
        "Application object 1": "applicationObject1",
        "Vendor-ID": "vendor_ID",
        "1st value": "_1stValue",
    }
    for name, want in cases.items():
        assert m.make_cname(name) == want, name
