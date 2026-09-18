import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml


MODULE_PATH = Path(__file__).with_name("preview_finray_tcp.py")
SPEC = importlib.util.spec_from_file_location("preview_finray_tcp", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def binary_stl(vertices):
    header = b"test".ljust(80, b"\0")
    triangle = struct.pack("<12fH", 0, 0, 1, *np.asarray(vertices).reshape(-1), 0)
    return header + struct.pack("<I", 1) + triangle


def test_inspect_stl_verifies_hash_and_envelope(tmp_path):
    path = tmp_path / "finger.stl"
    path.write_bytes(binary_stl([[0, 0, 0], [80, 0, 0], [0, 24.75, 24.09]]))
    digest = hashlib.sha256(path.read_bytes()).hexdigest()

    report = MODULE.inspect_stl(path, digest, [80.0, 24.75, 24.09])

    assert report["sha256_matches_selection"] is True
    assert report["envelope_matches_selection"] is True
    assert report["inspection_only"] is True


def test_normalize_candidates_defaults_to_zero_and_rejects_nonfinite():
    assert MODULE.normalize_candidates([])[0].offset_mm == (0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="유한"):
        MODULE.normalize_candidates([(0.0, float("nan"), 0.0)])
    for invalid in [(0.0, True, 0.0), (0.0, "1", 0.0)]:
        with pytest.raises(ValueError, match="실수"):
            MODULE.normalize_candidates([invalid])


def test_preview_urdf_has_only_fixed_marker_and_preserves_source(tmp_path):
    source = tmp_path / "active.urdf"
    source.write_text(
        """<?xml version="1.0"?><robot name="test">
        <link name="base_footprint"/><link name="left_tool0"/><link name="left_cup_tcp"/>
        <joint name="tool" type="fixed"><parent link="base_footprint"/><child link="left_tool0"/><origin xyz="1 2 3" rpy="0 0 0"/></joint>
        <joint name="tcp" type="fixed"><parent link="left_tool0"/><child link="left_cup_tcp"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>
        </robot>""",
        encoding="utf-8",
    )
    original = source.read_bytes()
    output = tmp_path / "preview.urdf"
    candidate = MODULE.Candidate("left_finray_tcp_candidate_001", (10.0, -20.0, 30.0))

    MODULE.write_preview_urdf(source, output, [candidate])

    assert source.read_bytes() == original
    root = ET.parse(output).getroot()
    marker = next(link for link in root.findall("link") if link.attrib["name"] == candidate.name)
    assert list(marker) == []
    joint = next(joint for joint in root.findall("joint") if joint.attrib["name"] == f"{candidate.name}_joint")
    assert joint.attrib["type"] == "fixed"
    assert joint.find("parent").attrib["link"] == "left_tool0"
    assert joint.find("origin").attrib["xyz"] == "0.01 -0.02 0.03"


def test_preview_refuses_to_overwrite_active_urdf(tmp_path):
    source = tmp_path / "active.urdf"
    source.write_text("<robot name='test'/>", encoding="utf-8")
    with pytest.raises(ValueError, match="입력 또는 활성"):
        MODULE.write_preview_urdf(
            source,
            source,
            [MODULE.Candidate("left_finray_tcp_candidate_001", (0.0, 0.0, 0.0))],
        )


def test_preview_refuses_default_active_urdf_even_with_other_input(tmp_path):
    source = tmp_path / "copy.urdf"
    source.write_text("<robot name='test'/>", encoding="utf-8")
    with pytest.raises(ValueError, match="입력 또는 활성"):
        MODULE.write_preview_urdf(
            source,
            MODULE.DEFAULT_URDF,
            [MODULE.Candidate("left_finray_tcp_candidate_001", (0.0, 0.0, 0.0))],
        )


def test_preview_refuses_existing_output(tmp_path):
    output = tmp_path / "preview.urdf"
    output.write_text("keep", encoding="utf-8")
    with pytest.raises(FileExistsError, match="덮어쓰지"):
        MODULE.write_preview_urdf(
            MODULE.DEFAULT_URDF,
            output,
            [MODULE.Candidate("left_finray_tcp_candidate_001", (0.0, 0.0, 0.0))],
        )
    assert output.read_text(encoding="utf-8") == "keep"


def test_fk_comparison_uses_tool_local_offset(tmp_path):
    output = tmp_path / "preview.urdf"
    candidate = MODULE.Candidate("left_finray_tcp_candidate_001", (12.0, -7.0, 25.0))
    MODULE.write_preview_urdf(MODULE.DEFAULT_URDF, output, [candidate])

    report = MODULE.compare_fk(output, [candidate], [10.0, -20.0, 30.0, 15.0, 5.0])

    delta = np.asarray(report["candidates"][0]["delta_from_active_left_cup_tcp_mm"])
    assert np.linalg.norm(delta) == pytest.approx(np.linalg.norm(candidate.offset_mm), abs=1e-5)
    assert report["candidates"][0]["assumed_tool0_offset_mm"] == list(candidate.offset_mm)


@pytest.mark.parametrize("collision", ["selection", "stl", "input_urdf", "output_urdf"])
def test_output_json_must_not_collide_with_inputs_or_preview(tmp_path, collision):
    paths = {
        "selection": tmp_path / "selection.yaml",
        "stl": tmp_path / "finger.stl",
        "input_urdf": tmp_path / "input.urdf",
        "output_urdf": tmp_path / "preview.urdf",
    }
    with pytest.raises(ValueError, match="출력|입력 또는 활성"):
        MODULE.validate_output_paths(
            paths["selection"], paths["input_urdf"], paths["stl"],
            paths["output_urdf"], paths[collision],
        )


def test_invalid_joint_or_candidate_is_rejected_before_output(tmp_path):
    output = tmp_path / "preview.urdf"
    with pytest.raises(ValueError, match="실수"):
        MODULE.normalize_joint_deg([0.0, 0.0, True, 0.0, 0.0])
    assert not output.exists()
    with pytest.raises(ValueError, match="실수"):
        MODULE.write_preview_urdf(
            MODULE.DEFAULT_URDF,
            output,
            [MODULE.Candidate("left_finray_tcp_candidate_001", (0.0, "8", 0.0))],
        )
    assert not output.exists()


def test_main_writes_offline_result_with_explicit_semantics(tmp_path, capsys):
    output_urdf = tmp_path / "preview.urdf"
    output_json = tmp_path / "preview.json"
    stl = tmp_path / "finger.stl"
    stl.write_bytes(binary_stl([[0, 0, 0], [80, 0, 0], [0, 24.75, 24.09]]))
    selection = tmp_path / "selection.yaml"
    selection.write_text(yaml.safe_dump({
        "status": "physical_validation_pending",
        "selected_finger": {
            "source_filename": stl.name,
            "source_sha256": hashlib.sha256(stl.read_bytes()).hexdigest(),
            "measured_stl_envelope_mm": [80.0, 24.75, 24.09],
            "license_status": "not_verified",
        },
    }), encoding="utf-8")

    assert MODULE.main([
        "--selection", str(selection),
        "--stl", str(stl),
        "--output-urdf", str(output_urdf),
        "--output-json", str(output_json),
        "--offset-mm", "0", "0", "80",
    ]) == 0

    result = json.loads(output_json.read_text(encoding="utf-8"))
    assert result["mode"] == "offline_finray_tcp_assumption_preview"
    assert result["motion_command_emitted"] is False
    assert result["active_urdf_modified"] is False
    assert result["license_status"] == "not_verified"
    assert result["fk_comparison"]["candidates"][0]["assumed_tool0_offset_mm"] == [0.0, 0.0, 80.0]
    assert "실측 보정" in result["semantics"][-1]
    assert "새 preview 프레임에만" in result["semantics"][-2]
    assert json.loads(capsys.readouterr().out) == result
