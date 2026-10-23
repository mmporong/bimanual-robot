"""Replay provenance, time, calibration and split-leakage regressions."""

import csv
from dataclasses import asdict
import json
from pathlib import Path

from jdamr_cube_navigation.pointcloud_replay import (
    file_sha256, load_depth_capture, load_manifest, replay, validate_split_groups,
)
import numpy as np
import pytest
import yaml
from test_table_edge_detection import _render_table, config, INTRINSICS


def capture_manifest(tmp_path, *, stamped=False, embedded_frame=False,
                     transform_contract=False, schema_version=2):
    configured = config()
    raw = tmp_path / 'capture.npz'
    image = _render_table(configured, background_depth_m=1.8)[0]
    fields = {'depth': image[None], 'intrinsics': np.array([
        INTRINSICS.focal_x_px, INTRINSICS.focal_y_px,
        INTRINSICS.center_x_px, INTRINSICS.center_y_px])}
    if stamped:
        fields['stamps'] = np.array([10.])
        fields['odom_from_optical'] = np.eye(4)[None]
    if embedded_frame:
        fields['frame_id'] = 'camera_color_optical_frame'
    np.savez(raw, **fields)
    provenance = tmp_path / 'calibration.json'
    provenance.write_text(json.dumps({'kind': 'synthetic', 'configuration': asdict(configured)}))
    manifest = {
        'schema_version': schema_version, 'capture_group': 'session-table-mount',
        'split': 'development', 'frame_id': 'camera_color_optical_frame',
        'input': {'path': raw.name, 'sha256': file_sha256(raw), 'encoding': '16UC1_mm'},
        'calibration': {'id': 'synthetic', 'status': 'derived', 'source': provenance.name,
                        'source_sha256': file_sha256(provenance),
                        'configuration_map': {
                            'camera_height_m': 'configuration.camera_height_m',
                            'camera_roll_rad': 'configuration.camera_roll_rad',
                            'camera_pitch_rad': 'configuration.camera_pitch_rad',
                        }},
        'table_config': asdict(configured),
    }
    if transform_contract:
        manifest['odom_transform'] = {
            'parent_frame': 'odom', 'child_frame': 'camera_color_optical_frame'}
    path = tmp_path / 'manifest.json'
    path.write_text(json.dumps(manifest))
    return path, raw, manifest


def test_real_missing_time_is_not_replaced_by_a_synthetic_clock(tmp_path):
    path, _, _ = capture_manifest(tmp_path)
    summary = replay(path, tmp_path / 'result')
    assert summary['frame_count'] == 1
    assert summary['baseline_detected_frames'] == 1
    assert summary['component_detected_frames'] == 1
    assert summary['accuracy_evaluated'] is False
    assert summary['availability']['timestamp_available'] is False
    assert summary['availability']['synchronized_odom_available'] is False
    assert summary['availability']['capture_frame_id_available'] is False
    csv = (tmp_path / 'result/frames.csv').read_text()
    assert 'unavailable_missing_stamped_odom' in csv


def test_exact_timestamped_transforms_enable_tracking(tmp_path):
    path, _, _ = capture_manifest(
        tmp_path, stamped=True, embedded_frame=True, transform_contract=True)
    summary = replay(path, tmp_path / 'result')
    assert summary['availability']['synchronized_odom_available'] is True
    assert summary['availability']['tracking_available'] is True
    assert summary['calibration_binding_verified'] is True
    assert summary['odom_transform_provenance'] == (
        'manifest_declared_not_independently_verified')
    assert ',observed,' in (tmp_path / 'result/frames.csv').read_text()
    with (tmp_path / 'result/frames.csv').open() as stream:
        row = next(csv.DictReader(stream))
    assert row['measurement_stamp_s'] == '10.0' and row['track_id'] == '1'
    assert json.loads(row['tracked_edge_odom_m']) == json.loads(
        row['candidate_edges_optical_m'])[0]


def test_raw_change_invalidates_the_sealed_input(tmp_path):
    path, raw, _ = capture_manifest(tmp_path)
    with raw.open('ab') as stream:
        stream.write(b'changed')
    with pytest.raises(ValueError, match='SHA-256'):
        load_manifest(path)


def test_calibration_source_change_invalidates_the_replay(tmp_path):
    path, _, _ = capture_manifest(tmp_path)
    (tmp_path / 'calibration.json').write_text('{}')
    with pytest.raises(ValueError, match='calibration source SHA-256'):
        load_manifest(path)


def test_embedded_capture_frame_must_match_the_manifest(tmp_path):
    path, _, manifest = capture_manifest(tmp_path, embedded_frame=True)
    manifest['frame_id'] = 'different_optical_frame'
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='embedded frame_id'):
        replay(path, tmp_path / 'result')


def test_stamped_transform_without_frame_contract_does_not_enable_tracking(tmp_path):
    path, _, _ = capture_manifest(tmp_path, stamped=True, embedded_frame=True)
    summary = replay(path, tmp_path / 'result')
    assert summary['availability']['synchronized_odom_available'] is True
    assert summary['availability']['tracking_available'] is False
    assert summary['availability']['tracking_unavailable_reason'] == (
        'missing_odom_transform_frame_contract')
    assert 'unavailable_missing_odom_transform_frame_contract' in (
        tmp_path / 'result/frames.csv').read_text()


def test_stamped_transform_without_embedded_frame_does_not_enable_tracking(tmp_path):
    path, _, _ = capture_manifest(tmp_path, stamped=True, transform_contract=True)
    summary = replay(path, tmp_path / 'result')
    assert summary['availability']['tracking_available'] is False
    assert summary['availability']['tracking_unavailable_reason'] == (
        'missing_verified_capture_frame')
    assert summary['availability']['frame_id_provenance'] == (
        'missing_embedded_frame_id_manifest_only')


def test_source_hash_without_configuration_mapping_is_not_calibration_proof(tmp_path):
    path, _, manifest = capture_manifest(tmp_path)
    del manifest['calibration']['configuration_map']
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='configuration_map'):
        load_manifest(path)


def test_v1_without_configuration_mapping_replays_detection_but_blocks_tracking(tmp_path):
    path, _, manifest = capture_manifest(
        tmp_path, stamped=True, embedded_frame=True, transform_contract=True,
        schema_version=1)
    del manifest['calibration']['configuration_map']
    path.write_text(json.dumps(manifest))
    summary = replay(path, tmp_path / 'result')
    assert summary['baseline_detected_frames'] == 1
    assert summary['component_detected_frames'] == 1
    assert summary['calibration_binding_verified'] is False
    assert summary['availability']['tracking_unavailable_reason'] == (
        'missing_verified_calibration_binding')


def test_v1_with_explicit_configuration_mapping_is_verified(tmp_path):
    path, _, _ = capture_manifest(tmp_path, schema_version=1)
    summary = replay(path, tmp_path / 'result')
    assert summary['calibration_binding_verified'] is True


def test_v1_still_checks_embedded_capture_frame(tmp_path):
    path, _, manifest = capture_manifest(
        tmp_path, embedded_frame=True, schema_version=1)
    del manifest['calibration']['configuration_map']
    manifest['frame_id'] = 'different_optical_frame'
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='embedded frame_id'):
        replay(path, tmp_path / 'result')


def test_calibration_mapping_rejects_config_not_present_in_the_source(tmp_path):
    path, _, manifest = capture_manifest(tmp_path)
    manifest['table_config']['camera_pitch_rad'] += .1
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='calibration source value'):
        load_manifest(path)


def test_yaml_camera_mount_mapping_supports_explicit_degrees(tmp_path):
    path, _, manifest = capture_manifest(tmp_path)
    provenance = tmp_path / 'camera_mount.yaml'
    provenance.write_text(yaml.safe_dump({'camera_mount': {'transform': {
        'height_m': manifest['table_config']['camera_height_m'],
        'roll_deg': np.degrees(manifest['table_config']['camera_roll_rad']).item(),
        'pitch_deg': np.degrees(manifest['table_config']['camera_pitch_rad']).item(),
    }}}))
    manifest['calibration'].update({
        'source': provenance.name,
        'source_sha256': file_sha256(provenance),
        'configuration_map': {
            'camera_height_m': {'path': 'camera_mount.transform.height_m', 'unit': 'm'},
            'camera_roll_rad': {'path': 'camera_mount.transform.roll_deg', 'unit': 'deg'},
            'camera_pitch_rad': {'path': 'camera_mount.transform.pitch_deg', 'unit': 'deg'},
        },
    })
    path.write_text(json.dumps(manifest))
    _, _, configured = load_manifest(path)
    assert configured.camera_pitch_rad == pytest.approx(
        manifest['table_config']['camera_pitch_rad'])


def test_repository_camera_mount_maps_height_roll_and_pitch(tmp_path):
    path, _, manifest = capture_manifest(tmp_path)
    source = (Path(__file__).parents[1] / 'config/camera_mount_current.yaml').resolve()
    mount = yaml.safe_load(source.read_text())['camera_mount']['transform']
    manifest['table_config'].update({
        'camera_height_m': mount['z_m'],
        'camera_roll_rad': mount['roll_rad'],
        'camera_pitch_rad': mount['pitch_rad'],
    })
    manifest['calibration'].update({
        'source': str(source),
        'source_sha256': file_sha256(source),
        'configuration_map': {
            'camera_height_m': 'camera_mount.transform.z_m',
            'camera_roll_rad': 'camera_mount.transform.roll_rad',
            'camera_pitch_rad': 'camera_mount.transform.pitch_rad',
        },
    })
    path.write_text(json.dumps(manifest))
    _, _, configured = load_manifest(path)
    assert configured.camera_height_m == mount['z_m']
    assert configured.camera_roll_rad == mount['roll_rad']
    assert configured.camera_pitch_rad == mount['pitch_rad']


@pytest.mark.parametrize(('key', 'value'), [
    ('scene_config', {'distance_m': .04}),
    ('track_config', {'prediction_horizon_s': .1}),
    ('target_region_odom', {'center_xy_m': [0., .8], 'radius_m': .3}),
])
def test_every_effective_processing_configuration_is_in_the_receipt(tmp_path, key, value):
    path, _, manifest = capture_manifest(tmp_path)
    first = replay(path, tmp_path / 'before')
    manifest[key] = value
    path.write_text(json.dumps(manifest))
    second = replay(path, tmp_path / 'after')
    assert first['config_sha256'] != second['config_sha256']
    assert first['implementation_sha256'] == second['implementation_sha256']


def test_replay_applies_the_declared_target_region(tmp_path):
    path, _, manifest = capture_manifest(
        tmp_path, stamped=True, embedded_frame=True, transform_contract=True)
    manifest['target_region_odom'] = {'center_xy_m': [100., 100.], 'radius_m': .1}
    path.write_text(json.dumps(manifest))
    replay(path, tmp_path / 'result')
    assert ',unobservable,' in (tmp_path / 'result/frames.csv').read_text()


def test_projection_p_is_used_when_k_is_nan(tmp_path):
    raw = tmp_path / 'raw.npz'
    np.savez(raw, depth=np.ones((2, 3), dtype=np.uint16) * 1000,
             k=np.full(9, np.nan), p=np.array([100., 0., 1., 0.,
                                               0., 100., 1., 0., 0., 0., 1., 0.]),
             encoding='16UC1')
    _, camera, stamps, transforms, provenance = load_depth_capture(raw)
    assert camera.focal_x_px == 100.
    assert stamps is None and transforms is None
    assert provenance['intrinsics_source'] == 'rectified_projection_p'


@pytest.mark.parametrize('fields', [
    {'depth': np.ones((1, 2, 3), dtype=np.uint16), 'stamps': [0.]},
    {'depth': np.ones((2, 2, 3), dtype=np.uint16), 'stamps': [2., 1.]},
    {'depth': np.ones((1, 2, 3), dtype=np.uint16), 'odom_from_optical': np.eye(4)[None]},
    {'depth': np.ones((1, 2, 3), dtype=float)},
])
def test_invalid_units_clocks_and_unsynchronized_transforms_are_rejected(tmp_path, fields):
    raw = tmp_path / 'bad.npz'
    np.savez(raw, intrinsics=[100., 100., 1., 1.], **fields)
    with pytest.raises(ValueError):
        load_depth_capture(raw)


@pytest.mark.parametrize('same', ['group', 'raw'])
def test_splitting_adjacent_frames_or_copying_raw_does_not_create_a_holdout(same):
    first = {'capture_group': 'a', 'split': 'calibration', 'input': {'sha256': 'hash-a'}}
    second = {'capture_group': 'a' if same == 'group' else 'b', 'split': 'evaluation',
              'input': {'sha256': 'hash-a' if same == 'raw' else 'hash-b'}}
    with pytest.raises(ValueError, match='leak'):
        validate_split_groups([first, second])
