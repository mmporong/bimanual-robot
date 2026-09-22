import json

import pytest

from tray_pose_probe import load_cases


def test_probe_rejects_missing_nonfinite_or_duplicate_cases(tmp_path):
    path = tmp_path/"cases.json"
    valid = {"id": "center", "left_joint_deg": [0.]*5, "right_joint_deg": [0.]*5}
    path.write_text(json.dumps({"cases": [valid]}))
    assert load_cases(path) == [valid]
    for cases in ([], [valid, valid], [{**valid, "left_joint_deg": [0.]*4}],
                  [{**valid, "right_joint_deg": [0., 0., 0., 0., float("nan")]}]):
        path.write_text(json.dumps({"cases": cases}))
        with pytest.raises(ValueError):
            load_cases(path)
