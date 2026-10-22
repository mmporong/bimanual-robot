"""Replay provenance, time, calibration and split-leakage regressions."""

import csv
from dataclasses import asdict
import json

from jdamr_cube_navigation.pointcloud_replay import (
    file_sha256, load_depth_capture, load_manifest, replay, validate_split_groups,
)
import numpy as np
import pytest
from test_table_edge_detection import _render_table, config, INTRINSICS


def capture_manifest(tmp_path, *, stamped=False):
    configured = config()
    raw = tmp_path / 'capture.npz'
    image = _render_table(configured, background_depth_m=1.8)[0]
    fields = {'depth': image[None], 'intrinsics': np.array([
        INTRINSICS.focal_x_px, INTRINSICS.focal_y_px,
        INTRINSICS.center_x_px, INTRINSICS.center_y_px])}
    if stamped:
        fields['stamps'] = np.array([10.])
        fields['odom_from_optical'] = np.eye(4)[None]
    np.savez(raw, **fields)
    provenance = tmp_path / 'calibration.json'
    provenance.write_text(json.dumps({'kind': 'synthetic', 'configuration': asdict(configured)}))
    manifest = {
        'schema_version': 1, 'capture_group': 'session-table-mount',
        'split': 'development', 'frame_id': 'camera_color_optical_frame',
        'input': {'path': raw.name, 'sha256': file_sha256(raw), 'encoding': '16UC1_mm'},
        'calibration': {'id': 'synthetic', 'status': 'derived', 'source': provenance.name,
                        'source_sha256': file_sha256(provenance)},
        'table_config': asdict(configured),
    }
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
    csv = (tmp_path / 'result/frames.csv').read_text()
    assert 'unavailable_missing_stamped_odom' in csv


def test_exact_timestamped_transforms_enable_tracking(tmp_path):
    path, _, _ = capture_manifest(tmp_path, stamped=True)
    summary = replay(path, tmp_path / 'result')
    assert summary['availability']['synchronized_odom_available'] is True
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
    path, _, manifest = capture_manifest(tmp_path, stamped=True)
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
