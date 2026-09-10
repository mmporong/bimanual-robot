from __future__ import annotations

import hashlib
import importlib.util
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
SPEC_PATH = REPO_ROOT / "design/mechanical/hold_flow_mechanical_v0_3.yaml"
CAD_MANIFEST = REPO_ROOT / "design/cad/exports/manifest.json"
CUP_MANIFEST = REPO_ROOT / "design/gripper/exports/cup_overcaps/manifest.json"
IMPORT_SCRIPT = REPO_ROOT / "src/hold_flow_description/scripts/import_isaac_sim.py"
URDF_PATH = REPO_ROOT / "src/hold_flow_description/urdf/hold_flow.urdf"
DESCRIPTION_VALIDATOR = REPO_ROOT / "src/hold_flow_description/scripts/validate_description.py"
QUALITY_AUDITOR = REPO_ROOT / "src/hold_flow_description/scripts/audit_urdf_quality.py"
SO101_LICENSE = REPO_ROOT / "src/hold_flow_description/third_party/so_arm_101/LICENSE"


class MechanicalV03Test(unittest.TestCase):
    def test_design_spec_has_requested_geometry(self) -> None:
        spec = yaml.safe_load(SPEC_PATH.read_text(encoding="utf-8"))
        self.assertEqual(spec["chassis"]["tabletop_footprint"], [340.0, 450.0])
        self.assertEqual(len(spec["chassis"]["plates"]["tabletop"]["quadrant_names"]), 4)
        self.assertEqual(spec["chassis"]["profile_rings"]["y_rail_center_x"], 160.0)
        self.assertEqual(spec["chassis"]["plates"]["tabletop"]["z_top"], 720.0)
        self.assertEqual(spec["arm_mounts"]["left"]["xyz"], [20.0, 75.0, 726.0])
        self.assertEqual(spec["arm_mounts"]["right"]["xyz"], [20.0, -75.0, 726.0])
        self.assertEqual(spec["camera"]["height_above_tabletop"], 250.0)
        self.assertEqual(spec["camera"]["mount"]["mast_z_bottom"], 728.0)
        self.assertEqual(
            spec["camera"]["mount"]["mast_z_bottom"],
            spec["chassis"]["plates"]["tabletop"]["z_top"]
            + spec["camera"]["mount"]["backing_thickness"],
        )
        self.assertEqual(len(spec["navigation"]["ball_casters"]["centers_xyz"]), 2)
        self.assertEqual(spec["navigation"]["reuse_source"]["left_servo_id"], 2)
        self.assertEqual(spec["navigation"]["reuse_source"]["right_servo_id"], 1)
        self.assertEqual(
            spec["navigation"]["wheel"]["initial_host_parameters"]["wheel_separation_m"],
            0.320,
        )
        self.assertEqual(spec["compute_plan"]["later_raspberry_pi_5"]["priority"], "deferred")
        self.assertEqual(
            spec["provenance"]["so101"]["checked_commit"],
            "eecbe3e0a9ebb23e25ad7b2759b03884c6660903",
        )

    def test_calculation_and_isaac_static_contract_pass(self) -> None:
        commands = (
            ["python3", "design/mechanical/calculate_full_size_tabletop_design.py"],
            ["python3", str(DESCRIPTION_VALIDATOR.relative_to(REPO_ROOT))],
            ["python3", str(QUALITY_AUDITOR.relative_to(REPO_ROOT)), "--strict"],
            ["python3", "src/hold_flow_description/scripts/validate_isaac_contract.py"],
            ["python3", "src/hold_flow_description/scripts/import_isaac_sim.py", "--check-only"],
        )
        for command in commands:
            with self.subTest(command=command):
                subprocess.run(command, cwd=REPO_ROOT, check=True, capture_output=True, text=True)

    def test_generated_cad_and_cup_holder_manifests(self) -> None:
        cad = json.loads(CAD_MANIFEST.read_text(encoding="utf-8"))
        cup = json.loads(CUP_MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(cad["source_spec"], "design/mechanical/hold_flow_mechanical_v0_3.yaml")
        self.assertTrue(cad["all_breps_valid"])
        parts = {part["name"]: part for part in cad["parts"]}
        panel_names = {
            "tabletop_front_left", "tabletop_front_right",
            "tabletop_rear_left", "tabletop_rear_right",
        }
        self.assertTrue(panel_names <= parts.keys())
        self.assertTrue(all(parts[name]["fdm_part"] for name in panel_names))
        self.assertTrue(all(parts[name]["k1_max_safe_fit"] for name in panel_names))
        self.assertEqual(parts["frame_column"]["quantity"], 4)
        self.assertTrue(parts["battery_mount_plate"]["fdm_part"])
        self.assertTrue(parts["lidar_mount_plate"]["fdm_part"])
        placements = cad["assembly_placements_mm"]
        self.assertEqual(placements["camera_backing"], [-120.0, 0.0, 720.0])
        self.assertEqual(placements["camera_mast_segment"], [-120.0, 0.0, 728.0])
        self.assertEqual(
            placements["arm_adapters"],
            [[20.0, 75.0, 720.0], [20.0, -75.0, 720.0]],
        )
        self.assertEqual(len(placements["frame_columns"]), 4)
        self.assertEqual(len(placements["frame_rail_x"]), 5)
        self.assertEqual(len(placements["frame_rail_y"]), 5)
        self.assertTrue(cup["valid_brep"])
        self.assertEqual(cup["quantity"], 2)
        self.assertTrue(cup["support_expected"])
        self.assertTrue((REPO_ROOT / cup["stl"]).is_file())

    def test_urdf_mass_matches_provisional_mass_model(self) -> None:
        calculation = subprocess.run(
            ["python3", "design/mechanical/calculate_full_size_tabletop_design.py"],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        mass_model = json.loads(calculation.stdout)["provisional_mass_model"]
        expected_total = mass_model["exact_total_kg"]
        root = ET.parse(URDF_PATH).getroot()
        actual_total = sum(
            float(mass.attrib["value"])
            for mass in root.findall("link/inertial/mass")
        )
        self.assertAlmostEqual(actual_total, expected_total, places=6)
        self.assertAlmostEqual(sum(mass_model["groups_kg"].values()), mass_model["total_kg"], places=3)
        self.assertEqual(mass_model["task_mass_cases_kg"]["empty_robot"], 7.379)

    def test_isaac_yaml_is_the_importer_source(self) -> None:
        spec = importlib.util.spec_from_file_location("hold_flow_isaac_import", IMPORT_SCRIPT)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        settings, robot_type, source_urdf, output_usd = module.load_import_settings(
            REPO_ROOT / "src/hold_flow_description/config/isaac_sim_6.yaml"
        )
        self.assertEqual(robot_type, "Mobile Manipulators")
        self.assertEqual(source_urdf, URDF_PATH)
        self.assertEqual(output_usd, REPO_ROOT / "build/isaac/hold_flow/hold_flow.usd")
        self.assertEqual(
            settings,
            {
                "collision_from_visuals": False,
                "merge_mesh": False,
                "merge_fixed_joints": False,
                "allow_self_collision": False,
                "fix_base": False,
            },
        )

        class FakeImporterConfig:
            def __init__(self, **kwargs) -> None:
                self.kwargs = kwargs

        configured = module.create_import_config(
            FakeImporterConfig,
            source_urdf,
            output_usd.parent,
            settings,
            robot_type,
        )
        self.assertEqual(
            configured.kwargs,
            {
                "urdf_path": str(source_urdf),
                "usd_path": str(output_usd.parent),
                **settings,
                "robot_type": "Mobile Manipulators",
            },
        )

    def test_isaac_materialized_urdf_has_no_package_uri(self) -> None:
        spec = importlib.util.spec_from_file_location("hold_flow_isaac_import", IMPORT_SCRIPT)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory(prefix="hold_flow_isaac_test_") as directory:
            output = module.materialize_isaac_urdf(URDF_PATH, Path(directory))
            text = output.read_text(encoding="utf-8")
        self.assertNotIn("package://hold_flow_description/", text)
        self.assertIn(str(REPO_ROOT / "src/hold_flow_description/meshes"), text)

    def test_so101_upstream_license_copy_is_pinned(self) -> None:
        self.assertEqual(
            hashlib.sha256(SO101_LICENSE.read_bytes()).hexdigest(),
            "c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4",
        )


if __name__ == "__main__":
    unittest.main()
