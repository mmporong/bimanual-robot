import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest


MODULE_PATH = Path(__file__).with_name("workcell_preview_inputs.py")
SPEC = importlib.util.spec_from_file_location("workcell_preview_inputs", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value), encoding="utf-8")


def test_load_replay_validates_sources_and_records_hashes(tmp_path):
    plan_path = tmp_path / "plan.json"
    state_path = tmp_path / "captured.json"
    pregrasp = [0, -25, 31, 49, 0]
    grasp = [0, -2, 30, 27, 1]
    _write_json(plan_path, {"stages": {
        "pregrasp": {"joint_deg": pregrasp},
        "grasp": {"joint_deg": grasp},
    }})
    _write_json(state_path, {
        "side": "left",
        "joint_order": MODULE.ARM_JOINTS,
        "joint_degrees": [1, 2, 3, 4, 5],
    })

    result = MODULE.load_replay(plan_path, [state_path])

    assert [pose["name"] for pose in result["poses"]] == [
        "planned_pregrasp", "planned_grasp", "captured",
    ]
    assert result["poses"][2]["source_kind"] == "saved_state"
    for source in result["source_files"]:
        raw = Path(source["path"]).read_bytes()
        assert source["sha256"] == hashlib.sha256(raw).hexdigest()


@pytest.mark.parametrize("bad", [True, "1", float("nan"), float("inf")])
def test_load_replay_rejects_non_finite_and_bool(bad, tmp_path):
    plan_path = tmp_path / "plan.json"
    _write_json(plan_path, {"stages": {
        "pregrasp": {"joint_deg": [bad, 0, 0, 0, 0]},
        "grasp": {"joint_deg": [0, 0, 0, 0, 0]},
    }})
    with pytest.raises(ValueError):
        MODULE.load_replay(plan_path, [])


def test_load_replay_rejects_wrong_side_and_order(tmp_path):
    state_path = tmp_path / "state.json"
    state = {"side": "right", "joint_order": MODULE.ARM_JOINTS, "joint_degrees": [0] * 5}
    _write_json(state_path, state)
    with pytest.raises(ValueError):
        MODULE.load_replay(None, [state_path])
    state["side"] = "left"
    state["joint_order"] = list(reversed(MODULE.ARM_JOINTS))
    _write_json(state_path, state)
    with pytest.raises(ValueError):
        MODULE.load_replay(None, [state_path])


def test_default_pose_is_explicitly_assumed():
    result = MODULE.load_replay(None, [])
    assert result["poses"] == [{
        "name": "nominal_preview_assumed",
        "left_joint_deg": MODULE.NOMINAL_PREVIEW_DEG,
        "source_kind": "ASSUMED",
        "source_path": None,
    }]


def test_sample_pose_uses_looping_common_cubic_scalar():
    poses = [
        {"name": "a", "left_joint_deg": [0, 0, 0, 0, 0]},
        {"name": "b", "left_joint_deg": [10, 20, 30, 40, 50]},
    ]
    name, midpoint = MODULE.sample_pose(poses, 1.0, 2.0)
    assert name == "a->b"
    assert midpoint == [5, 10, 15, 20, 25]
    name, loop_midpoint = MODULE.sample_pose(poses, 3.0, 2.0)
    assert name == "b->a"
    assert loop_midpoint == [5, 10, 15, 20, 25]
    assert MODULE.sample_pose(poses, 4.0, 2.0)[1] == poses[0]["left_joint_deg"]


@pytest.mark.parametrize("elapsed,seconds", [(-1, 1), (True, 1), (0, True), (0, 0), (0, float("nan"))])
def test_sample_pose_rejects_bad_time(elapsed, seconds):
    with pytest.raises(ValueError):
        MODULE.sample_pose([{"name": "a", "left_joint_deg": [0] * 5}], elapsed, seconds)


def _package_fixture(tmp_path: Path, mesh_exists: bool = True) -> tuple[Path, Path, Path]:
    package = tmp_path / "hold_flow_description"
    urdf_dir = package / "urdf"
    mesh = package / "meshes" / "part.stl"
    urdf_dir.mkdir(parents=True)
    mesh.parent.mkdir(parents=True)
    (package / "package.xml").write_text("<package/>", encoding="utf-8")
    if mesh_exists:
        mesh.write_text("solid part\nendsolid part\n", encoding="utf-8")
    source = urdf_dir / "robot.urdf"
    source.write_text(
        '<robot name="x"><link name="x"><visual><geometry>'
        '<mesh filename="package://hold_flow_description/meshes/part.stl"/>'
        '</geometry></visual></link></robot>',
        encoding="utf-8",
    )
    return source, tmp_path / "preview" / "robot.urdf", mesh


def test_materialize_resolves_mesh_and_preserves_source(tmp_path):
    source, output, mesh = _package_fixture(tmp_path)
    before = source.read_bytes()
    result = MODULE.materialize_urdf(source, output)
    assert source.read_bytes() == before
    assert result["source_preserved"] is True
    assert result["source_sha256"] == hashlib.sha256(before).hexdigest()
    assert str(mesh.resolve()) in output.read_text(encoding="utf-8")
    assert result["resolved_mesh_files"] == [str(mesh.resolve())]
    with pytest.raises(FileExistsError):
        MODULE.materialize_urdf(source, output)


def test_materialize_rejects_missing_mesh_without_creating_output(tmp_path):
    source, output, _ = _package_fixture(tmp_path, mesh_exists=False)
    with pytest.raises(FileNotFoundError):
        MODULE.materialize_urdf(source, output)
    assert not output.exists()


def _joint_origins(root: ET.Element) -> dict[str, str | None]:
    return {
        joint.get("name"): joint.find("origin").get("xyz") if joint.find("origin") is not None else None
        for joint in root.findall("./joint")
    }


def test_base_contract_overlay_updates_preview_base_only(tmp_path):
    source = Path(__file__).parents[1] / "src/hold_flow_description/urdf/hold_flow.urdf"
    contract_source = Path(__file__).parents[1] / "config/navigation/jdamr_migration.json"
    contract = json.loads(contract_source.read_text(encoding="utf-8"))
    contract_path = tmp_path / "jdamr_migration.json"
    _write_json(contract_path, contract)
    output = tmp_path / "hold_flow.preview.urdf"
    source_before = source.read_bytes()
    before_root = ET.fromstring(source_before)
    before_origins = _joint_origins(before_root)
    assert [float(value) for value in before_origins["left_wheel_joint"].split()] == [0.0, 0.160, 0.0]
    assert [float(value) for value in before_origins["right_wheel_joint"].split()] == [0.0, -0.160, 0.0]
    assert [float(value) for value in before_origins["front_caster_joint"].split()] == [0.12, 0.0, -0.0204]
    assert [float(value) for value in before_origins["rear_caster_joint"].split()] == [-0.12, 0.0, -0.0204]

    result = MODULE.materialize_urdf(source, output, base_contract=contract_path)
    after_root = ET.parse(output).getroot()

    assert source.read_bytes() == source_before
    overlay = result["base_overlay"]
    assert overlay["applied"] is True
    assert overlay["contract_sha256"] == hashlib.sha256(contract_path.read_bytes()).hexdigest()
    assert overlay["scene_frame_basis"]["axle_x_in_scene_m"] == pytest.approx(0.105)
    assert overlay["untouched_arm_frames"] is True
    assert overlay["inertias_updated"] is False
    assert overlay["dynamics_validated"] is False
    assert overlay["contact_stability_validated"] is False

    after_origins = _joint_origins(after_root)
    changed = {
        name for name in before_origins
        if before_origins[name] != after_origins[name]
    }
    assert changed == {
        "left_wheel_joint", "right_wheel_joint", "front_caster_joint", "rear_caster_joint",
    }
    assert after_origins["base_footprint_joint"] == before_origins["base_footprint_joint"]
    assert after_origins["left_mount_joint"] == before_origins["left_mount_joint"]
    assert after_origins["right_mount_joint"] == before_origins["right_mount_joint"]

    left_xyz = [float(value) for value in after_origins["left_wheel_joint"].split()]
    right_xyz = [float(value) for value in after_origins["right_wheel_joint"].split()]
    assert left_xyz == pytest.approx([0.105, 0.255, 0.0])
    assert right_xyz == pytest.approx([0.105, -0.255, 0.0])
    assert left_xyz[1] - right_xyz[1] == pytest.approx(0.510)
    measured = contract["hardware"]["measured_base_footprint"]
    expected_axle_x = -(measured["front_m"] + measured["rear_m"]) / 2.0
    assert left_xyz[0] == pytest.approx(expected_axle_x)

    for link_name in ("left_wheel_link", "right_wheel_link"):
        link = after_root.find(f"./link[@name='{link_name}']")
        cylinders = link.findall("./visual/geometry/cylinder") + link.findall("./collision/geometry/cylinder")
        assert [float(item.get("radius")) for item in cylinders] == pytest.approx([0.0329, 0.0329])
        assert [float(item.get("length")) for item in cylinders] == pytest.approx([0.030, 0.030])

    front_caster = [float(value) for value in after_origins["front_caster_joint"].split()]
    rear_caster = [float(value) for value in after_origins["rear_caster_joint"].split()]
    assert front_caster == pytest.approx([-0.110, 0.215, -0.0204])
    assert rear_caster == pytest.approx([-0.110, -0.215, -0.0204])
    assert all(
        caster["basis"] == "proxy_inference_not_measured"
        for caster in overlay["changes"]["casters"].values()
    )


def test_without_base_contract_keeps_original_base_model(tmp_path):
    source = Path(__file__).parents[1] / "src/hold_flow_description/urdf/hold_flow.urdf"
    output = tmp_path / "hold_flow.preview.urdf"
    before_root = ET.parse(source).getroot()
    result = MODULE.materialize_urdf(source, output)
    after_root = ET.parse(output).getroot()
    assert result["base_overlay"] == {"applied": False}
    assert _joint_origins(after_root) == _joint_origins(before_root)
