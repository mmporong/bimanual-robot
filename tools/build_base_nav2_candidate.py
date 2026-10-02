"""Build a review-only Nav2 YAML from the JD-AMR baseline and measured base envelope."""

import argparse
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / 'config/navigation/jdamr_migration.json'


def _polygon(front, rear, half_width):
    return json.dumps([
        [front, half_width], [front, -half_width],
        [rear, -half_width], [rear, half_width],
    ])


def _contains(outer, inner):
    return (outer['front_m'] > inner['front_m']
            and outer['rear_m'] < inner['rear_m']
            and outer['half_width_m'] > inner['half_width_m'])


def build(source_path, contract_path=CONTRACT):
    source = yaml.safe_load(Path(source_path).read_text(encoding='utf-8'))
    contract = json.loads(Path(contract_path).read_text(encoding='utf-8'))
    base = contract['hardware']['measured_base_footprint']
    policy = contract['base_nav2_candidate_policy']
    if base['base_frame'] != 'base_footprint' or contract['software']['physical_motion_enabled']:
        raise ValueError('expected measured base_footprint and physical motion disabled')
    if base['front_m'] <= 0 or base['rear_m'] >= 0:
        raise ValueError('invalid front/rear envelope')
    if base['left_m'] != -base['right_m']:
        raise ValueError('asymmetric base needs a separately reviewed polygon')
    if policy['claim_scope'] != 'offline_base_only_review_not_physical_navigation_approval':
        raise ValueError('only the offline review policy may be exported')

    # Candidate padding is a planning allowance, not a measured clearance.
    padding_m = policy['footprint_padding_m']
    if padding_m <= 0 or policy['maximum_forward_speed_mps'] <= 0:
        raise ValueError('candidate padding and speed must be positive')
    footprint_bounds = {
        'front_m': base['front_m'] + padding_m,
        'rear_m': base['rear_m'] - padding_m,
        'half_width_m': base['left_m'] + padding_m,
    }
    if not _contains(policy['stop_zone'], footprint_bounds):
        raise ValueError('StopZone does not contain padded base footprint')
    if not _contains(policy['slowdown_zone'], policy['stop_zone']):
        raise ValueError('SlowdownZone does not contain StopZone')
    footprint = _polygon(
        round(footprint_bounds['front_m'], 3),
        round(footprint_bounds['rear_m'], 3),
        round(footprint_bounds['half_width_m'], 3),
    )
    for costmap in ('local_costmap', 'global_costmap'):
        params = source[costmap][costmap]['ros__parameters']
        params['footprint'] = footprint
        params['robot_base_frame'] = 'base_footprint'
    for node in ('bt_navigator', 'behavior_server'):
        source[node]['ros__parameters']['robot_base_frame'] = 'base_footprint'
    source['docking_server']['ros__parameters']['base_frame'] = 'base_footprint'

    # This is deliberately slower than the source robot; it is not a calibrated limit.
    source['amcl']['ros__parameters']['set_initial_pose'] = False
    follow = source['controller_server']['ros__parameters']['FollowPath']
    follow['desired_linear_vel'] = policy['maximum_forward_speed_mps']
    follow['rotate_to_heading_angular_vel'] = policy['maximum_angular_speed_radps']
    smoother = source['velocity_smoother']['ros__parameters']
    smoother['max_velocity'] = [policy['maximum_forward_speed_mps'], 0.0,
                                policy['maximum_angular_speed_radps']]
    smoother['min_velocity'] = [0.0, 0.0,
                                -policy['maximum_angular_speed_radps']]

    monitor = source['collision_monitor']['ros__parameters']
    for name, key in (('StopZone', 'stop_zone'), ('SlowdownZone', 'slowdown_zone')):
        zone = policy[key]
        monitor[name]['points'] = _polygon(
            zone['front_m'], zone['rear_m'], zone['half_width_m'])
    monitor['source_timeout'] = policy['scan_source_timeout_s']
    return source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True,
                        help='JD-AMR nav2_params.yaml baseline')
    parser.add_argument('--output', type=Path, required=True,
                        help='new review-only YAML; existing files are refused')
    args = parser.parse_args()
    if args.output.exists():
        parser.error(f'output already exists: {args.output}')
    result = build(args.source)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        stream.write('# REVIEW ONLY: not approved for physical autonomous navigation.\n')
        yaml.safe_dump(result, stream, sort_keys=False, allow_unicode=True)
    print(args.output)


if __name__ == '__main__':
    main()
