import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from simulate_urdf_workcell import DEFAULT_URDF, SCENE_ASSUMPTIONS, run


def report():
    stage = {"name": "reorient", "joint_deg": [0,-68,92,-22,0], "accepted": True}
    return {"schema": "body_side_grasp_v1", "motion_command_emitted": False,
            "urdf_sha256": hashlib.sha256(DEFAULT_URDF.read_bytes()).hexdigest(),
            "scene": SCENE_ASSUMPTIONS, "sides": {
                "left": {"stages": [dict(stage)]}, "right": {"stages": [dict(stage)]}}}


def invoke(tmp_path, data):
    source = tmp_path / "plan.json"
    source.write_text(json.dumps(data))
    run(SimpleNamespace(plan=None,state=[],body_plan=source,urdf=DEFAULT_URDF,
                        output_dir=tmp_path/"output",check_only=True))


def test_valid_body_review_is_offline(tmp_path, capsys):
    invoke(tmp_path,report())
    result=json.loads(capsys.readouterr().out)
    assert result["hardware_accessed"] is False
    assert len(result["replay"]["poses"]) == 2
    assert not (tmp_path/"output").exists()


@pytest.mark.parametrize("issue", ["hash", "boolean", "nan", "shape", "stages", "scene"])
def test_malformed_body_reports_are_rejected(tmp_path, issue):
    data=report()
    stage=data["sides"]["left"]["stages"][0]
    if issue == "hash": data["urdf_sha256"] = "old"
    if issue == "boolean": stage["accepted"] = "false"
    if issue == "nan": stage["joint_deg"] = [float("nan")]*5
    if issue == "shape": stage["joint_deg"] = [0]*4
    if issue == "stages": data["sides"]["left"]["stages"] = {}
    if issue == "scene": data["scene"] = {**SCENE_ASSUMPTIONS, "cup_height_m": -1}
    with pytest.raises(ValueError): invoke(tmp_path,data)
