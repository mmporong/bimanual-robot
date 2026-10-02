"""New-base candidate must not inherit a too-small JD-AMR collision envelope."""

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'candidate', ROOT / 'tools/build_base_nav2_candidate.py')
candidate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(candidate)


class BaseNav2CandidateTest(unittest.TestCase):
    def test_zone_must_contain_base(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            baseline = root / 'source.yaml'
            baseline.write_text(yaml.safe_dump({}))
            contract = json.loads(candidate.CONTRACT.read_text(encoding='utf-8'))
            contract['base_nav2_candidate_policy']['stop_zone']['half_width_m'] = 0.25
            policy_file = root / 'contract.json'
            policy_file.write_text(json.dumps(contract))
            with self.assertRaisesRegex(ValueError, 'StopZone does not contain'):
                candidate.build(baseline, policy_file)

    def test_envelopes_and_speed(self):
        with tempfile.TemporaryDirectory() as temporary:
            baseline = Path(temporary) / 'source.yaml'
            source = {
                'amcl': {'ros__parameters': {'set_initial_pose': True}},
                'bt_navigator': {'ros__parameters': {'robot_base_frame': 'base_link'}},
                'behavior_server': {'ros__parameters': {'robot_base_frame': 'base_link'}},
                'docking_server': {'ros__parameters': {'base_frame': 'base_link'}},
                'controller_server': {'ros__parameters': {'FollowPath': {}}},
                'velocity_smoother': {'ros__parameters': {}},
                'collision_monitor': {'ros__parameters': {
                    'StopZone': {}, 'SlowdownZone': {},
                }},
            }
            for key in ('local_costmap', 'global_costmap'):
                source[key] = {key: {'ros__parameters': {'footprint': 'old'}}}
            baseline.write_text(yaml.safe_dump(source))
            result = candidate.build(baseline)

        contract = json.loads(candidate.CONTRACT.read_text(encoding='utf-8'))
        base = contract['hardware']['measured_base_footprint']
        local = result['local_costmap']['local_costmap']['ros__parameters']
        global_ = result['global_costmap']['global_costmap']['ros__parameters']
        self.assertEqual(local['footprint'], global_['footprint'])
        self.assertEqual(local['robot_base_frame'], 'base_footprint')
        self.assertEqual(global_['robot_base_frame'], 'base_footprint')
        self.assertEqual(result['bt_navigator']['ros__parameters']['robot_base_frame'],
                         'base_footprint')
        self.assertEqual(result['behavior_server']['ros__parameters']['robot_base_frame'],
                         'base_footprint')
        self.assertEqual(result['docking_server']['ros__parameters']['base_frame'],
                         'base_footprint')
        footprint = json.loads(local['footprint'])
        self.assertGreater(footprint[0][0], base['front_m'])
        self.assertLess(footprint[2][0], base['rear_m'])
        self.assertGreater(footprint[0][1], base['left_m'])

        monitor = result['collision_monitor']['ros__parameters']
        stop = json.loads(monitor['StopZone']['points'])
        slow = json.loads(monitor['SlowdownZone']['points'])
        for polygon in (stop, slow):
            self.assertGreater(polygon[0][0], footprint[0][0])
            self.assertLess(polygon[2][0], footprint[2][0])
            self.assertGreater(polygon[0][1], footprint[0][1])
        self.assertGreater(slow[0][0], stop[0][0])
        self.assertGreater(slow[0][1], stop[0][1])
        self.assertEqual(result['velocity_smoother']['ros__parameters']['max_velocity'][0],
                         contract['base_nav2_candidate_policy']['maximum_forward_speed_mps'])
        self.assertFalse(result['amcl']['ros__parameters']['set_initial_pose'])


if __name__ == '__main__':
    unittest.main()
