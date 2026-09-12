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
        for field in ('target_left_id', 'target_right_id', 'installed_lidar',
                      'measured_lidar_tf', 'measured_transport_footprint'):
            self.assertIsNone(hardware[field], field)
        self.assertIsNone(contract['target_design']['wheel_separation_effective_m'])
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
