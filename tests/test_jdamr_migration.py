"""Pinned export must exclude dirty source files and reject overwrite."""

import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest


spec = importlib.util.spec_from_file_location(
    'migration', Path(__file__).resolve().parents[1]
    / 'tools/prepare_jdamr_migration.py')
migration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(migration)


class MigrationTest(unittest.TestCase):
    def test_user_hardware_plan_does_not_enable_unmeasured_motion(self):
        contract = json.loads(
            (migration.ROOT / 'config/navigation/jdamr_migration.json').read_text())
        self.assertEqual(contract['schema_version'], 2)
        hardware = contract['hardware']
        self.assertNotIn('motor_and_controller_reuse_confirmed', hardware)
        self.assertTrue(hardware['target_motor_same_model_confirmed'])
        self.assertEqual(hardware['target_motor_origin'], 'different_chassis')
        self.assertTrue(hardware['controller_reuse_confirmed'])
        self.assertTrue(hardware['pi_reuse_confirmed'])
        self.assertEqual(hardware['planned_lidar'], 'YDLIDAR G4')
        self.assertEqual(hardware['target_left_id'], 2)
        self.assertEqual(hardware['target_right_id'], 1)
        self.assertEqual(hardware['installed_lidar'], 'YDLIDAR G4')
        lidar_tf = hardware['measured_lidar_tf']
        self.assertEqual(lidar_tf['parent_frame'], 'base_footprint')
        self.assertEqual(lidar_tf['child_frame'], 'laser_link')
        self.assertEqual(lidar_tf['translation_m'], [-0.010, 0.000, 0.150])
        self.assertEqual(lidar_tf['rpy_rad'], [0.000, 0.000, 3.141592653589793])
        base = hardware['measured_base_footprint']
        self.assertEqual(base['base_frame'], 'base_footprint')
        self.assertEqual(base['scope'], 'assembled_lower_frame_and_wheels_only')
        self.assertEqual(base['front_m'] - base['rear_m'], base['frame_length_m'])
        self.assertEqual(base['left_m'] - base['right_m'], base['wheel_outer_width_m'])
        self.assertAlmostEqual(base['wheel_outer_width_m'] - base['frame_width_m'], 0.090)
        self.assertEqual(base['front_m'], 0.065)
        self.assertEqual(base['measurement_date'], '2026-09-15')
        self.assertIsNone(hardware['measured_transport_footprint'])
        self.assertEqual(
            contract['target_design']['wheel_separation_geometric_m'], 0.510)
        self.assertIsNone(contract['target_design']['wheel_separation_effective_m'])
        self.assertTrue(
            contract['physical_verification']['motor_side_mapping_passed'])
        self.assertEqual(contract['software']['discovery_range'], 'SUBNET')
        self.assertFalse(contract['software']['physical_motion_enabled'])
        self.assertFalse(contract['software']['synthetic_traction_guard_for_physical_robot'])

    def test_pinned_commit_excludes_dirty_files_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / 'source'
            source.mkdir()

            def git(*args):
                return subprocess.check_output(
                    ['git', '-C', str(source), *args], text=True).strip()

            git('init', '-q')
            (source / 'baseline.txt').write_text('committed')
            git('add', 'baseline.txt')
            git('-c', 'user.name=Test', '-c', 'user.email=test@example.test',
                '-c', 'commit.gpgsign=false', 'commit', '-qm', 'baseline')
            revision = git('rev-parse', 'HEAD')
            (source / 'baseline.txt').write_text('uncommitted')
            (source / 'private.txt').write_text('untracked')
            contract = {'source': {'commit': revision}, 'target_design': {}}
            output = root / 'output'
            report = migration.prepare(source, output, contract)
            self.assertEqual(
                (output / 'baseline/baseline.txt').read_text(), 'committed')
            self.assertFalse((output / 'baseline/private.txt').exists())
            self.assertFalse(report['physical_motion_enabled'])
            self.assertEqual((source / 'baseline.txt').read_text(), 'uncommitted')
            with self.assertRaises(ValueError):
                migration.prepare(source, output, contract)


if __name__ == '__main__':
    unittest.main()
