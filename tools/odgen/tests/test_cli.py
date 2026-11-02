"""odgen CLI: --check passes on the checked-in tree, fails on any drift, and
enforces the revision bump through od/vortex_od.lock.json."""

from __future__ import annotations

import importlib.util
import shutil
import subprocess
import sys

import pytest
import yaml

from conftest import REPO
from odgen import cli


def test_checked_in_artefacts_are_current():
    assert cli.main(["--root", str(REPO), "--check"]) == 0


def test_cli_runs_as_a_directory():
    r = subprocess.run([sys.executable, str(REPO / "tools" / "odgen"), "--check"], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert "artefacts current" in r.stdout


@pytest.fixture
def tree(tmp_path):
    """A scratch repository with only the YAML; odgen writes everything else."""
    (tmp_path / "od").mkdir()
    shutil.copy(REPO / "od" / "vortex_od.yaml", tmp_path / "od" / "vortex_od.yaml")
    assert cli.main(["--root", str(tmp_path)]) == 0
    return tmp_path


def edit_yaml(root, fn):
    p = root / "od" / "vortex_od.yaml"
    data = yaml.safe_load(p.read_text())
    fn(data)
    p.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True))


def test_generate_then_check(tree):
    assert cli.main(["--root", str(tree), "--check"]) == 0
    for rel in cli.OUTPUTS.values():
        assert (tree / rel).is_file(), rel
    assert (tree / cli.LOCK).is_file()
    # the scratch output equals the checked-in output
    for rel in cli.OUTPUTS.values():
        assert (tree / rel).read_text() == (REPO / rel).read_text(), rel


@pytest.mark.parametrize("rel", list(cli.OUTPUTS.values()), ids=[k for k in cli.OUTPUTS])
def test_check_fails_on_edited_artefact(tree, rel, capsys):
    p = tree / rel
    p.write_text(p.read_text() + "\n// hand edit\n")
    assert cli.main(["--root", str(tree), "--check"]) == 1
    assert f"{rel} is stale" in capsys.readouterr().err


def test_check_fails_on_missing_artefact(tree, capsys):
    (tree / cli.OUTPUTS["od_c"]).unlink()
    assert cli.main(["--root", str(tree), "--check"]) == 1
    assert "OD.c is missing" in capsys.readouterr().err


def test_od_change_needs_a_revision_bump(tree, capsys):
    edit_yaml(tree, lambda d: d["objects"].append(
        {"index": 0x2400, "id": "spare_gpio_inputs", "name": "Spare GPIO inputs", "data_type": "U32",
         "access": "ro", "pdo": "t"}))
    # stale artefacts and the same revision: both reported
    assert cli.main(["--root", str(tree), "--check"]) == 1
    err = capsys.readouterr().err
    assert "bump device.revision" in err and "is stale" in err
    # regenerating without a bump is refused and writes nothing
    before = (tree / cli.OUTPUTS["od_c"]).read_text()
    assert cli.main(["--root", str(tree)]) == 1
    assert (tree / cli.OUTPUTS["od_c"]).read_text() == before
    # bump: generation records the new revision next to the old one
    edit_yaml(tree, lambda d: d["device"].update(revision={"major": 1, "minor": 1}))
    assert cli.main(["--root", str(tree)]) == 0
    assert cli.main(["--root", str(tree), "--check"]) == 0
    lock = yaml.safe_load((tree / cli.LOCK).read_text())
    assert set(lock["revisions"]) == {"1.0", "1.1"}
    assert "REVISION = (1, 1)" in (tree / cli.OUTPUTS["py"]).read_text()


def test_docs_change_needs_no_bump(tree):
    edit_yaml(tree, lambda d: d["objects"][0].update(docs="reworded"))
    assert cli.main(["--root", str(tree)]) == 0
    assert cli.main(["--root", str(tree), "--check"]) == 0


def test_amend_revision(tree):
    edit_yaml(tree, lambda d: d["objects"][0].update(default=0x00020193))
    assert cli.main(["--root", str(tree)]) == 1
    assert cli.main(["--root", str(tree), "--amend-revision"]) == 0
    assert cli.main(["--root", str(tree), "--check"]) == 0


def test_unrecorded_revision_fails_check(tree, capsys):
    (tree / cli.LOCK).unlink()
    assert cli.main(["--root", str(tree), "--check"]) == 1
    assert "not recorded" in capsys.readouterr().err


def test_invalid_yaml_exit_code(tree, capsys):
    edit_yaml(tree, lambda d: d["objects"][0].update(data_type="U99"))
    assert cli.main(["--root", str(tree), "--check"]) == 2
    assert "data_type" in capsys.readouterr().err


def test_python_ids_module(repo, model):
    path = repo / cli.OUTPUTS["py"]
    spec = importlib.util.spec_from_file_location("od_ids", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert mod.OD_HASH == model.od_hash
    assert mod.REVISION == model.revision
    assert len(mod.ENTRIES) == len(model.all_entries())
    assert mod.IDX_PRODUCER_HEARTBEAT_TIME == 0x1017
    e = mod.BY_KEY["identity_revision_number"]
    assert (e.index, e.sub, e.default) == (0x1018, 3, model.revision_number)
    assert mod.BY_ADDRESS[(0x2300, 1)].scale == 0.1
    assert mod.BY_ADDRESS[(0x1014, 0)].node_relative


def test_reference_doc_lists_every_object(repo, model):
    md = (repo / cli.OUTPUTS["md"]).read_text()
    for o in model.objects:
        assert f"### 0x{o.index:04X} {o.name}" in md
    assert f"0x{model.od_hash:08X}" in md
