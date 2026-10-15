import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest


MODULE_PATH = Path(__file__).resolve().parents[2] / 'simulation/workcell_preview_inputs.py'
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


def _four_wheel_fixture(tmp_path: Path) -> tuple[Path, Path, dict]:
    package = tmp_path / "hold_flow_description"
    urdf_dir = package / "urdf"
    urdf_dir.mkdir(parents=True)
    (package / "package.xml").write_text("<package/>", encoding="utf-8")
    wheel_parts = []
    for joint_name, link_name, x_m, y_m in (
        ("left_wheel_joint", "left_wheel_link", 0.105, 0.160),
        ("right_wheel_joint", "right_wheel_link", 0.105, -0.160),
        ("rear_left_wheel_joint", "rear_left_wheel_link", -0.105, 0.160),
        ("rear_right_wheel_joint", "rear_right_wheel_link", -0.105, -0.160),
    ):
        wheel_parts.append(
            f'<link name="{link_name}">'
            '<visual><geometry><cylinder radius="0.030" length="0.020"/></geometry></visual>'
            '<collision><geometry><cylinder radius="0.030" length="0.020"/></geometry></collision>'
            '</link>'
            f'<joint name="{joint_name}" type="continuous">'
            f'<parent link="base_link"/><child link="{link_name}"/>'
            f'<origin xyz="{x_m} {y_m} -0.0029"/></joint>'
        )
    source = urdf_dir / "robot.urdf"
    source.write_text(
        '<robot name="x"><link name="base_footprint"/><link name="base_link"/>'
        '<joint name="base_footprint_joint" type="fixed">'
        '<parent link="base_footprint"/><child link="base_link"/><origin xyz="0 0 0.0329"/>'
        '</joint><link name="left_arm_mount"/><link name="right_arm_mount"/>'
        '<joint name="left_mount_joint" type="fixed">'
        '<parent link="base_link"/><child link="left_arm_mount"/><origin xyz="0.01 0.02 0.03"/>'
        '</joint><joint name="right_mount_joint" type="fixed">'
        '<parent link="base_link"/><child link="right_arm_mount"/><origin xyz="0.01 -0.02 0.03"/>'
        '</joint>' + "".join(wheel_parts) + '</robot>',
        encoding="utf-8",
    )
    contract = {
        "target_design": {
            "drive_type": "skid_steer_four_wheel_drive",
            "model_base_frame_definition": "chassis_center_on_floor",
            "runtime_base_frame_definition": "front_axle_projection_on_floor",
            "model_from_runtime_translation_m": [0.105, 0.0, 0.0],
            "wheelbase_design_m": 0.210,
            "wheelbase_measured_m": None,
            "rear_axle_geometry_status": "design_assumption_not_measured",
            "wheel_radius_initial_m": 0.0329,
            "wheel_separation_geometric_m": 0.510,
        },
        "hardware": {"measured_base_footprint": {"wheel_outer_width_m": 0.540}},
    }
    return source, tmp_path / "preview" / "robot.urdf", contract


def test_base_contract_overlay_updates_all_four_wheels_and_preserves_design_axles(tmp_path):
    source, output, contract = _four_wheel_fixture(tmp_path)
    contract_path = tmp_path / "jdamr_migration.json"
    _write_json(contract_path, contract)
    source_before = source.read_bytes()
    before_origins = _joint_origins(ET.fromstring(source_before))

    result = MODULE.materialize_urdf(source, output, base_contract=contract_path)
    after_root = ET.parse(output).getroot()
    after_origins = _joint_origins(after_root)

    assert source.read_bytes() == source_before
    overlay = result["base_overlay"]
    assert overlay["applied"] is True
    assert overlay["contract_sha256"] == hashlib.sha256(contract_path.read_bytes()).hexdigest()
    assert overlay["scene_frame_basis"] == {
        "base_footprint": "chassis_center_on_floor",
        "runtime_base_frame": "front_axle_projection_on_floor",
        "model_from_runtime_translation_m": [0.105, 0.0, 0.0],
    }
    assert overlay["model_geometry"]["drive_type"] == "skid_steer_four_wheel_drive"
    assert overlay["model_geometry"]["wheel_count"] == 4
    assert overlay["model_geometry"]["caster_count"] == 0
    assert overlay["model_geometry"]["rear_axle_geometry_status"] == "design_assumption_not_measured"
    assert overlay["changes"]["casters"] == {}
    assert overlay["untouched_arm_frames"] is True
    assert overlay["inertias_updated"] is False
    assert overlay["dynamics_validated"] is False
    assert overlay["contact_stability_validated"] is False

    wheel_origins = {
        "left_wheel_joint": [0.105, 0.255, 0.0],
        "right_wheel_joint": [0.105, -0.255, 0.0],
        "rear_left_wheel_joint": [-0.105, 0.255, 0.0],
        "rear_right_wheel_joint": [-0.105, -0.255, 0.0],
    }
    for joint_name, expected_xyz in wheel_origins.items():
        xyz = [float(value) for value in after_origins[joint_name].split()]
        assert xyz == pytest.approx(expected_xyz)
        assert xyz[0] == pytest.approx(float(before_origins[joint_name].split()[0]))
        change = overlay["changes"]["wheels"][joint_name]
        assert change["source_design_x_preserved"] is True

    assert after_origins["base_footprint_joint"] == before_origins["base_footprint_joint"]
    assert after_origins["left_mount_joint"] == before_origins["left_mount_joint"]
    assert after_origins["right_mount_joint"] == before_origins["right_mount_joint"]
    assert not any("caster" in (element.get("name") or "") for element in after_root.iter())
    for link_name in (
        "left_wheel_link", "right_wheel_link", "rear_left_wheel_link", "rear_right_wheel_link",
    ):
        link = after_root.find(f"./link[@name='{link_name}']")
        cylinders = link.findall("./visual/geometry/cylinder") + link.findall("./collision/geometry/cylinder")
        assert [float(item.get("radius")) for item in cylinders] == pytest.approx([0.0329, 0.0329])
        assert [float(item.get("length")) for item in cylinders] == pytest.approx([0.030, 0.030])


@pytest.mark.parametrize("bad", [True, "0.105", float("nan"), float("inf")])
def test_base_contract_overlay_rejects_bad_frame_translation_numbers(tmp_path, bad):
    source, output, contract = _four_wheel_fixture(tmp_path)
    contract["target_design"]["model_from_runtime_translation_m"][0] = bad
    contract_path = tmp_path / "jdamr_migration.json"
    _write_json(contract_path, contract)

    with pytest.raises(ValueError):
        MODULE.materialize_urdf(source, output, base_contract=contract_path)
    assert not output.exists()


def test_base_contract_overlay_rejects_caster_or_missing_rear_wheel(tmp_path):
    source, output, contract = _four_wheel_fixture(tmp_path)
    contract_path = tmp_path / "jdamr_migration.json"
    _write_json(contract_path, contract)
    text = source.read_text(encoding="utf-8")
    source.write_text(text.replace(
        '</robot>', '<link name="rear_caster_link"/><joint name="rear_caster_joint" type="fixed">'
        '<parent link="base_link"/><child link="rear_caster_link"/><origin xyz="0 0 0"/></joint></robot>',
    ), encoding="utf-8")
    with pytest.raises(ValueError, match="caster"):
        MODULE.materialize_urdf(source, output, base_contract=contract_path)

    source, output, contract = _four_wheel_fixture(tmp_path / "missing")
    contract_path = tmp_path / "missing" / "jdamr_migration.json"
    _write_json(contract_path, contract)
    source.write_text(source.read_text(encoding="utf-8").replace(
        'rear_right_wheel_joint', 'missing_rear_right_joint',
    ), encoding="utf-8")
    with pytest.raises(ValueError, match="각각 4개"):
        MODULE.materialize_urdf(source, output, base_contract=contract_path)


def test_repository_four_wheel_urdf_and_contract_materialize_without_geometry_regression(tmp_path):
    repository = Path(__file__).resolve().parents[2]
    source = repository / "src/description/hold_flow_description/urdf/hold_flow.urdf"
    contract = repository / "config/navigation/jdamr_migration.json"
    output = tmp_path / "hold_flow.preview.urdf"
    source_before = source.read_bytes()
    before_root = ET.fromstring(source_before)

    result = MODULE.materialize_urdf(source, output, base_contract=contract)
    after_root = ET.parse(output).getroot()
    overlay = result["base_overlay"]

    assert source.read_bytes() == source_before
    assert overlay["model_geometry"]["wheel_count"] == 4
    assert overlay["model_geometry"]["caster_count"] == 0
    assert overlay["changes"]["casters"] == {}
    for joint_name in overlay["model_geometry"]["wheel_joint_names"]:
        before = [float(value) for value in _joint_origins(before_root)[joint_name].split()]
        after = [float(value) for value in _joint_origins(after_root)[joint_name].split()]
        assert after == pytest.approx(before)
        assert overlay["changes"]["wheels"][joint_name]["source_design_x_preserved"] is True
    assert not any(
        "caster" in (element.get("name") or "").lower()
        for tag in ("joint", "link") for element in after_root.findall(f"./{tag}")
    )


def test_without_base_contract_keeps_original_base_model(tmp_path):
    source = Path(__file__).resolve().parents[2] / 'src/description/hold_flow_description/urdf/hold_flow.urdf'
    output = tmp_path / "hold_flow.preview.urdf"
    before_root = ET.parse(source).getroot()
    result = MODULE.materialize_urdf(source, output)
    after_root = ET.parse(output).getroot()
    assert result["base_overlay"] == {"applied": False}
    assert _joint_origins(after_root) == _joint_origins(before_root)
