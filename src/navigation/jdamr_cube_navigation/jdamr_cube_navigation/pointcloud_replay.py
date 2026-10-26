"""Replay sealed depth captures without ROS, sensor startup or motion commands."""

import argparse
import csv
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import yaml

from .box_top_detection import CameraIntrinsics
from .table_edge_detection import detect_table_edge, TableEdgeConfig
from .table_surface_model import detect_table_scene, SceneConfig
from .table_surface_tracking import rigid_transform, TableSurfaceTracker, TargetRegion, TrackConfig


def file_sha256(path):
    """Hash raw bytes so changing a capture invalidates its manifest."""
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _scalar_text(value, field):
    """Return a nonempty text scalar stored in an NPZ field."""
    value = np.asarray(value)
    if value.shape != ():
        raise ValueError(f'{field} must be a scalar string')
    item = value.item()
    if isinstance(item, bytes):
        item = item.decode('utf-8')
    if not isinstance(item, str) or not item:
        raise ValueError(f'{field} must be a nonempty string')
    return item


def load_depth_capture(path):
    """Read actual capture fields; never invent time or substitute NaN K for P."""
    with np.load(path, allow_pickle=False) as capture:
        depth = capture['depth'].copy()
        if depth.ndim == 2:
            depth = depth[None]
        if depth.ndim != 3 or depth.dtype != np.uint16 or not len(depth):
            raise ValueError('capture must contain nonempty uint16 depth in millimetres')
        count, height, width = depth.shape
        for key, actual in (('height', height), ('width', width)):
            if key in capture and int(capture[key].item()) != actual:
                raise ValueError('capture dimensions disagree with depth pixels')
        encoding = str(capture['encoding'].item()) if 'encoding' in capture else None
        if encoding not in (None, '16UC1', 'mono16'):
            raise ValueError('capture encoding disagrees with millimetre depth')
        if 'p' in capture:
            projection = np.asarray(capture['p'], dtype=float).reshape(-1)
            if projection.shape != (12,):
                raise ValueError('rectified projection must contain 12 entries')
            if not np.all(np.isfinite(projection)) or not np.allclose(
                    projection[[1, 3, 4, 7, 8, 9, 11]], 0., atol=1e-8):
                raise ValueError('unsupported rectified projection')
            if not np.isclose(projection[10], 1.):
                raise ValueError('invalid projective scale')
            fx, fy, cx, cy = projection[[0, 5, 2, 6]]
            intrinsics_source = 'rectified_projection_p'
        elif 'intrinsics' in capture:
            values = np.asarray(capture['intrinsics'], dtype=float)
            if values.shape != (4,):
                raise ValueError('intrinsics must contain fx, fy, cx, cy')
            fx, fy, cx, cy = values
            intrinsics_source = 'embedded_intrinsics'
        else:
            raise ValueError('missing valid projection/intrinsics; K is not assumed valid')
        if not np.all(np.isfinite([fx, fy, cx, cy])) or fx <= 0 or fy <= 0:
            raise ValueError('camera intrinsics must be finite with positive focal lengths')
        stamps = np.asarray(capture['stamps'], dtype=float) if 'stamps' in capture else None
        if stamps is not None and (stamps.shape != (count,)
                                   or not np.all(np.isfinite(stamps))
                                   or np.any(stamps <= 0.) or np.any(np.diff(stamps) <= 0.)):
            raise ValueError('capture stamps must be positive and strictly increasing')
        transforms = (capture['odom_from_optical'].copy()
                      if 'odom_from_optical' in capture else None)
        if transforms is not None:
            if stamps is None or transforms.shape != (count, 4, 4):
                raise ValueError('odom transforms need one exact timestamped transform per image')
            transforms = np.asarray([rigid_transform(item) for item in transforms])
        capture_frame_id = (_scalar_text(capture['frame_id'], 'frame_id')
                            if 'frame_id' in capture else None)
    return depth, CameraIntrinsics(width, height, fx, fy, cx, cy), stamps, transforms, {
        'intrinsics_source': intrinsics_source,
        'encoding_source': 'embedded' if encoding else 'manifest_uint16_millimetres',
        'timestamp_available': stamps is not None,
        'synchronized_odom_available': transforms is not None,
        'capture_frame_id_available': capture_frame_id is not None,
        'capture_frame_id': capture_frame_id,
    }


def _load_structured_source(path):
    """Load a JSON or YAML calibration record without format-specific assumptions."""
    text = path.read_text()
    if path.suffix.lower() == '.json':
        document = json.loads(text)
    else:
        document = yaml.safe_load(text)
    if not isinstance(document, dict):
        raise ValueError('calibration source must contain a JSON/YAML object')
    return document


def _source_value(document, mapping, field):
    """Resolve and normalize one explicitly mapped calibration value."""
    if isinstance(mapping, str):
        path, unit = mapping, None
    elif isinstance(mapping, dict):
        path, unit = mapping.get('path'), mapping.get('unit')
    else:
        raise ValueError(f'calibration mapping for {field} must be a path or object')
    if not isinstance(path, str) or not path:
        raise ValueError(f'calibration mapping path for {field} is required')
    value = document
    for part in path.split('.'):
        if not isinstance(value, dict) or part not in value:
            raise ValueError(f'calibration source path for {field} does not exist')
        value = value[part]
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        raise ValueError(f'calibration source value for {field} must be finite numeric')
    if unit is None:
        return float(value)
    expected = 'rad' if field.endswith('_rad') else 'm' if field.endswith('_m') else None
    if unit == expected:
        return float(value)
    if unit == 'deg' and expected == 'rad':
        return math.radians(value)
    raise ValueError(f'unsupported calibration unit {unit!r} for {field}')


def _validate_calibration_configuration(calibration, source, configured, *, required):
    """Prove that mount values used by the detector came from the sealed source."""
    mapping = calibration.get('configuration_map')
    required_fields = {'camera_height_m', 'camera_roll_rad', 'camera_pitch_rad'}
    if mapping is None and not required:
        return False
    if not isinstance(mapping, dict) or not required_fields.issubset(mapping):
        raise ValueError('calibration configuration_map must map height, roll and pitch')
    effective = asdict(configured)
    document = _load_structured_source(source)
    for field, source_mapping in mapping.items():
        if field not in effective:
            raise ValueError(f'calibration mapping targets unknown table_config field {field}')
        source_value = _source_value(document, source_mapping, field)
        configured_value = effective[field]
        if (not isinstance(configured_value, (int, float))
                or not math.isclose(source_value, configured_value, rel_tol=1e-12, abs_tol=1e-12)):
            raise ValueError(f'calibration source value disagrees with table_config.{field}')
    return True


def load_manifest(path):
    """Require declared split, capture group and calibration provenance."""
    path = Path(path).resolve()
    manifest = json.loads(path.read_text())
    schema_version = manifest.get('schema_version')
    if schema_version not in {1, 2}:
        raise ValueError('unsupported manifest schema_version')
    if manifest.get('split') not in {'calibration', 'development', 'evaluation'}:
        raise ValueError('declare calibration/development/evaluation split')
    for key in ('capture_group', 'frame_id'):
        if not isinstance(manifest.get(key), str) or not manifest[key]:
            raise ValueError(f'{key} is required')
    calibration = manifest.get('calibration', {})
    if (not calibration.get('id') or calibration.get('status') not in {'derived', 'measured'}
            or not calibration.get('source')):
        raise ValueError('calibration id, derived/measured status and source are required')
    calibration_source = Path(calibration['source']).expanduser()
    if not calibration_source.is_absolute():
        calibration_source = path.parent / calibration_source
    if file_sha256(calibration_source) != calibration.get('source_sha256'):
        raise ValueError('calibration source SHA-256 mismatch')
    source = Path(manifest['input']['path']).expanduser()
    if not source.is_absolute():
        source = path.parent / source
    if manifest['input'].get('encoding') != '16UC1_mm':
        raise ValueError('raw depth unit must be declared as 16UC1_mm')
    if file_sha256(source) != manifest['input'].get('sha256'):
        raise ValueError('capture SHA-256 mismatch')
    configured = TableEdgeConfig(**manifest['table_config'])
    _validate_calibration_configuration(
        calibration, calibration_source, configured, required=schema_version >= 2)
    return manifest, source.resolve(), configured


def validate_split_groups(manifests):
    """Keep a capture group and identical raw bytes out of different splits."""
    groups, hashes = {}, {}
    for manifest in manifests:
        for seen, key in ((groups, manifest['capture_group']),
                          (hashes, manifest['input']['sha256'])):
            if key in seen and seen[key] != manifest['split']:
                raise ValueError('capture group or raw bytes leak between splits')
            seen[key] = manifest['split']


def replay(manifest_path, output_directory):
    """Compare existing detection and component detection on the same frames."""
    manifest, source, configured = load_manifest(manifest_path)
    depth, intrinsics, stamps, transforms, availability = load_depth_capture(source)
    calibration_binding_verified = isinstance(
        manifest['calibration'].get('configuration_map'), dict)
    availability['calibration_binding_verified'] = calibration_binding_verified
    capture_frame_id = availability['capture_frame_id']
    if capture_frame_id is not None and capture_frame_id != manifest['frame_id']:
        raise ValueError('embedded frame_id disagrees with manifest frame_id')
    availability['frame_id_verified'] = capture_frame_id is not None
    availability['frame_id_provenance'] = (
        'embedded_and_manifest_match' if capture_frame_id is not None
        else 'missing_embedded_frame_id_manifest_only')
    transform_contract = manifest.get('odom_transform')
    if transform_contract is not None:
        if not isinstance(transform_contract, dict):
            raise ValueError('odom_transform frame contract must be an object')
        if transform_contract.get('parent_frame') != 'odom':
            raise ValueError('odom_transform parent_frame must be odom')
        if transform_contract.get('child_frame') != manifest['frame_id']:
            raise ValueError('odom_transform child_frame must match manifest frame_id')
    odom_transform_provenance = (
        'manifest_declared_not_independently_verified'
        if transform_contract is not None else 'missing')
    availability['odom_transform_provenance'] = odom_transform_provenance
    if transforms is None:
        tracking_reason = 'missing_stamped_odom'
    elif capture_frame_id is None:
        tracking_reason = 'missing_verified_capture_frame'
    elif transform_contract is None:
        tracking_reason = 'missing_odom_transform_frame_contract'
    elif not calibration_binding_verified:
        tracking_reason = 'missing_verified_calibration_binding'
    else:
        tracking_reason = None
    availability['tracking_available'] = tracking_reason is None
    availability['tracking_unavailable_reason'] = tracking_reason
    scene_config = SceneConfig(**manifest.get('scene_config', {}))
    track_config = TrackConfig(**manifest.get('track_config', {}))
    region_config = manifest.get('target_region_odom')
    region = TargetRegion(**region_config) if region_config is not None else None
    tracker = TableSurfaceTracker(track_config)
    rows = []
    effective_configuration = {
        'table': asdict(configured), 'scene': asdict(scene_config), 'track': asdict(track_config),
        'target_region_odom': asdict(region) if region is not None else None,
    }
    config_hash = hashlib.sha256(json.dumps(
        effective_configuration, sort_keys=True).encode()).hexdigest()
    for index, image in enumerate(depth):
        started = time.perf_counter()
        baseline = detect_table_edge(image, intrinsics, configured)
        baseline_duration = time.perf_counter() - started
        started = time.perf_counter()
        priority = {}
        if tracking_reason is None and region is not None:
            priority = {'target_region_odom': region, 'odom_from_optical': transforms[index]}
        scene = detect_table_scene(
            image, intrinsics, configured, **asdict(scene_config), **priority)
        scene_duration = time.perf_counter() - started
        track = None
        if tracking_reason is None:
            track = tracker.update(scene.candidates, float(stamps[index]), transforms[index],
                                   manifest['calibration']['id'], region)
        rows.append({
            'frame_index': index,
            'stamp_s': None if stamps is None else float(stamps[index]),
            'baseline_detected': baseline is not None,
            'baseline_edge_optical_m': json.dumps(
                baseline.edge_endpoints_optical_m if baseline is not None else None),
            'component_count': len(scene.component_sizes),
            'evaluated_components': scene.evaluated_components,
            'region_prioritized_components': scene.region_prioritized_components,
            'candidate_count': len(scene.candidates),
            'candidate_edges_optical_m': json.dumps([
                item.edge_endpoints_optical_m for item in scene.candidates]),
            'input_point_count': scene.input_point_count,
            'height_point_count': scene.height_point_count,
            'track_state': track.state if track else f'unavailable_{tracking_reason}',
            'track_id': track.track_id if track else None,
            'measurement_stamp_s': track.measurement_stamp_s if track else None,
            'tracked_edge_odom_m': json.dumps(track.endpoints_odom_m if track else None),
            'baseline_processing_ms': baseline_duration * 1000.,
            'scene_processing_ms': scene_duration * 1000.,
        })
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    with (output / 'frames.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        'schema_version': 1, 'mode': 'offline_replay_no_motion',
        'source_path': str(source), 'source_sha256': manifest['input']['sha256'],
        'manifest_sha256': file_sha256(manifest_path), 'config_sha256': config_hash,
        'effective_configuration': effective_configuration,
        'implementation_sha256': {name: file_sha256(Path(__file__).with_name(name)) for name in (
            'pointcloud_replay.py', 'pointcloud_components.py', 'table_edge_detection.py',
            'table_surface_model.py', 'table_surface_tracking.py', 'box_top_detection.py')},
        'capture_group': manifest['capture_group'], 'split': manifest['split'],
        'calibration': manifest['calibration'], 'availability': availability,
        'calibration_binding_verified': calibration_binding_verified,
        'odom_transform_provenance': odom_transform_provenance,
        'frame_count': len(rows),
        'baseline_detected_frames': sum(row['baseline_detected'] for row in rows),
        'component_detected_frames': sum(row['candidate_count'] > 0 for row in rows),
        'multi_candidate_frames': sum(row['candidate_count'] > 1 for row in rows),
        'scene_processing_ms_p50': float(np.percentile(
            [row['scene_processing_ms'] for row in rows], 50)),
        'scene_processing_ms_p95': float(np.percentile(
            [row['scene_processing_ms'] for row in rows], 95)),
        'accuracy_evaluated': False,
        'accuracy_reason': 'independent_object_and_distance_ground_truth_not_evaluated',
        'performance_scope': 'this_host_offline_processing_not_pi_or_end_to_end_latency',
    }
    (output / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    return summary


def main(args=None):
    """Run a manifest-verified capture with no ROS or hardware side effects."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    parsed = parser.parse_args(args)
    try:
        manifests = [load_manifest(path)[0] for path in parsed.manifest]
        validate_split_groups(manifests)
        summaries = [replay(path, parsed.output if len(manifests) == 1
                            else parsed.output / f'{index:03d}')
                     for index, path in enumerate(parsed.manifest)]
    except (ValueError, KeyError, OSError, TypeError) as error:
        parser.exit(2, f'pointcloud replay input error: {error}\n')
    print(json.dumps(summaries, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
