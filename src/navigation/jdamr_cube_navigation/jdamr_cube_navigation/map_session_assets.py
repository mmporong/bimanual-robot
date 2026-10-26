"""Prepare immutable, map-bound runtime assets for one taught map session."""

from copy import deepcopy
from datetime import date, datetime
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

import yaml

from jdamr_cube_navigation.keepout_mask import _read_pgm
from jdamr_cube_navigation.service_destinations import (
    add_pose,
    load_registry,
    map_identity,
    new_registry,
    set_home_pose,
    taught_pose,
)


DEFAULT_POINTER = Path.home() / 'jdamr_data/maps/current_manual_mapping.json'
DEFAULT_GEOMETRY = (
    Path(__file__).resolve().parents[2]
    / 'jdamr_cube_description/config/new_base_geometry.yaml')
TARGET_GAP_M = 0.01
LATERAL_TOLERANCE_M = 0.005
APPROACH_OFFSET_M = 0.5


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _path(value, parent):
    if not isinstance(value, str) or not value.strip():
        raise ValueError('asset path must be a non-empty string')
    candidate = Path(os.path.expandvars(os.path.expanduser(value)))
    return (candidate if candidate.is_absolute() else parent / candidate).resolve()


def _mapping(path, label):
    path = Path(path)
    if not path.is_file():
        raise ValueError(f'{label} does not exist: {path}')
    if path.suffix.lower() == '.json':
        value = json.loads(path.read_text(encoding='utf-8'))
    else:
        value = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError(f'{label} must be a mapping')
    return value


def _finite_pose(document, label):
    if document.get('frame_id') != 'map' or document.get('robot_base_frame') != 'base_link':
        raise ValueError(f'{label} must use map and base_link frames')
    values = []
    for key in ('x_m', 'y_m', 'yaw_rad'):
        value = document.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{label} {key} must be finite')
        values.append(float(value))
    return tuple(values)


def _verify_bound_map(document, source_identity, label):
    bound = document.get('map')
    if not isinstance(bound, dict):
        raise ValueError(f'{label} map identity is missing')
    for key in ('yaml_sha256', 'image_sha256'):
        if bound.get(key) != source_identity[key]:
            raise ValueError(f'{label} has stale map {key}')


def _atomic_write(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f'.{path.name}.', dir=path.parent)
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _yaml_bytes(value):
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode('utf-8')


def _json_bytes(value):
    def encode_special(item):
        if isinstance(item, (date, datetime)):
            return item.isoformat()
        raise TypeError(f'unsupported JSON value: {type(item).__name__}')

    return (json.dumps(
        value, ensure_ascii=False, indent=2, sort_keys=True,
        default=encode_special) + '\n').encode('utf-8')


def _keepout_assets(source_yaml, source_image):
    metadata = _mapping(source_yaml, 'source map YAML')
    width, height, pixels = _read_pgm(source_image)
    unknown = bytes(0 if pixel == 205 else 254 for pixel in pixels)
    pgm = f'P5\n{width} {height}\n255\n'.encode('ascii') + unknown
    keepout = {
        'image': 'keepout.pgm',
        'mode': 'trinary',
        'resolution': float(metadata['resolution']),
        'origin': [float(value) for value in metadata['origin']],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'generation': {
            'scope': 'unknown-only',
            'source_map_yaml_sha256': _sha256(source_yaml),
            'source_map_image_sha256': _sha256(source_image),
            'unknown_source_pixel': 205,
            'no_table_keepout_inferred': True,
        },
    }
    return _yaml_bytes(keepout), pgm, unknown.count(0)


def _front_extent(geometry_path):
    geometry = _mapping(geometry_path, 'base geometry')
    field = geometry.get('front_to_wheel_axis')
    value = field.get('value') if isinstance(field, dict) else None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('geometry front_to_wheel_axis.value must be numeric')
    value = float(value)
    if not math.isfinite(value) or not 0.0 < value < 0.5:
        raise ValueError('geometry front extent is invalid')
    return value, deepcopy(field)


def _source_observation(document, source_path):
    return {
        'source_file': str(source_path),
        'source_record': deepcopy(document),
        'physical_accuracy': document.get('physical_accuracy', 'NOT_EXTERNALLY_MEASURED'),
    }


def _finite_triplet(value, label):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f'{label} must be a finite map pose')
    result = []
    for item in value:
        if (isinstance(item, bool) or not isinstance(item, (int, float))
                or not math.isfinite(item)):
            raise ValueError(f'{label} must be a finite map pose')
        result.append(float(item))
    return tuple(result)


def _same_pose(left, right):
    return (math.isclose(left[0], right[0], abs_tol=1e-9, rel_tol=0.)
            and math.isclose(left[1], right[1], abs_tol=1e-9, rel_tol=0.)
            and math.isclose(
                math.atan2(math.sin(left[2] - right[2]),
                           math.cos(left[2] - right[2])),
                0., abs_tol=1e-9, rel_tol=0.))


def _dock_scan_evidence(
        pointer, pointer_path, source_identity, home, home_path, home_xyz):
    reference_path = _path(
        pointer.get('dock_scan_reference'), pointer_path.parent)
    reference = _mapping(reference_path, 'dock scan reference')
    identity = reference.get('map_identity')
    expected_identity = [
        source_identity['yaml_sha256'], source_identity['image_sha256']]
    if identity != expected_identity:
        raise ValueError('dock scan reference has stale map identity')
    pose_map = _finite_triplet(reference.get('pose_map'), 'dock scan pose_map')
    home_pose = _finite_triplet(reference.get('home_pose'), 'dock scan home_pose')
    if not _same_pose(pose_map, home_xyz) or not _same_pose(home_pose, home_xyz):
        raise ValueError('dock scan reference does not match home pose')
    scan_path = _path(reference.get('scan_path'), reference_path.parent)
    home_scan_path = _path(home.get('scan_path'), home_path.parent)
    if scan_path != home_scan_path:
        raise ValueError('dock scan reference scan_path does not match home scan_path')
    scan = _mapping(scan_path, 'dock scan')
    required = ('scans', 'odom', 'battery', 'tf_static')
    if any(not isinstance(scan.get(key), list) or not scan[key] for key in required):
        raise ValueError('dock scan schema requires non-empty sensor arrays')
    first_scan = scan['scans'][0]
    first_odom = scan['odom'][0]
    if (not isinstance(first_scan, dict)
            or not {'stamp', 'frame', 'angle_min', 'angle_increment',
                    'range_min', 'range_max', 'ranges'} <= first_scan.keys()
            or not isinstance(first_scan.get('ranges'), list)
            or not first_scan['ranges']
            or not isinstance(first_odom, dict)
            or not {'stamp', 'x', 'y', 'qz', 'qw', 'v', 'w'} <= first_odom.keys()
            or not isinstance(scan.get('map_to_base_before_finish'), dict)
            or not isinstance(scan.get('base_to_laser'), dict)):
        raise ValueError('dock scan schema is invalid')
    return {
        'reference_path': str(reference_path),
        'reference_sha256': _sha256(reference_path),
        'scan_path': str(scan_path),
        'scan_sha256': _sha256(scan_path),
        'map_identity': {
            'yaml_sha256': identity[0], 'image_sha256': identity[1]},
        'pose_map': list(pose_map),
        'home_pose': list(home_pose),
        'schema': {
            'scan_count': len(scan['scans']),
            'odom_count': len(scan['odom']),
            'battery_count': len(scan['battery']),
            'static_tf_count': len(scan['tf_static']),
            'has_map_to_base_before_finish': True,
            'has_base_to_laser': True,
        },
    }


def prepare_map_session(pointer_path=DEFAULT_POINTER, output_dir=None,
                        geometry_path=DEFAULT_GEOMETRY):
    """Build one runtime directory without modifying the source capture.

    The taught map pose remains the destination.  ``face_xy`` is only an
    expected search-region face derived from chassis geometry, not a measured
    surface position.
    """
    pointer_path = Path(pointer_path).expanduser().resolve()
    pointer = _mapping(pointer_path, 'manual mapping pointer')
    source_dir = _path(pointer.get('directory'), pointer_path.parent)
    map_yaml = _path(pointer.get('map_yaml'), pointer_path.parent)
    home_path = _path(pointer.get('home_dock'), pointer_path.parent)
    station_path = source_dir / 'water_station.yaml'
    geometry_path = Path(geometry_path).expanduser().resolve()
    if not source_dir.is_dir() or not station_path.is_file():
        raise ValueError(f'required water_station is missing: {station_path}')
    if map_yaml.parent != source_dir:
        raise ValueError('pointer map must belong to its declared map directory')

    source_map = map_identity(map_yaml)
    map_metadata = _mapping(map_yaml, 'source map YAML')
    map_image = _path(map_metadata.get('image'), map_yaml.parent)
    home = _mapping(home_path, 'home dock')
    station = _mapping(station_path, 'water station')
    _verify_bound_map(home, source_map, 'home dock')
    _verify_bound_map(station, source_map, 'water station')
    home_xyz = _finite_pose(home, 'home dock')
    station_xyz = _finite_pose(station, 'water station')
    if home.get('id') != 'home_dock' or home.get('parking_direction') != 'reverse':
        raise ValueError('home dock must be the reverse-parking home_dock pose')
    dock_scan = _dock_scan_evidence(
        pointer, pointer_path, source_map, home, home_path, home_xyz)
    station_id = station.get('id')
    if not isinstance(station_id, str) or station_id != 'water_station':
        raise ValueError('required dynamic station ID water_station is missing')
    if not math.isclose(float(station.get('target_front_gap_m', math.nan)), TARGET_GAP_M,
                        abs_tol=1e-9):
        raise ValueError('water_station target front gap must be 0.01 m')
    if not math.isclose(float(station.get('lateral_tolerance_m', math.nan)),
                        LATERAL_TOLERANCE_M, abs_tol=1e-9):
        raise ValueError('water_station lateral tolerance must be 0.005 m')

    runtime = (Path(output_dir).expanduser().resolve()
               if output_dir is not None else source_dir / 'runtime')
    if runtime == source_dir or runtime in (map_yaml.parent, home_path.parent):
        raise ValueError('runtime output must not overwrite the source map directory')
    source_paths = {path.resolve() for path in (
        map_yaml, map_image, home_path, station_path,
        Path(dock_scan['reference_path']), Path(dock_scan['scan_path']))}
    target_paths = {
        runtime / 'map.yaml', runtime / 'map.pgm', runtime / map_image.name,
        runtime / 'keepout.yaml', runtime / 'keepout.pgm',
        runtime / 'service_destinations.yaml', runtime / 'destinations.json',
        runtime / 'station_profiles.json',
    }
    if any(path.resolve() in source_paths for path in target_paths):
        raise ValueError('runtime output would overwrite an original capture')

    map_yaml_bytes = map_yaml.read_bytes()
    map_image_bytes = map_image.read_bytes()
    keepout_yaml_bytes, keepout_pgm_bytes, keepout_cells = _keepout_assets(
        map_yaml, map_image)
    front_extent_m, front_extent_provenance = _front_extent(geometry_path)

    runtime.mkdir(parents=True, exist_ok=True)
    _atomic_write(runtime / 'map.yaml', map_yaml_bytes)
    _atomic_write(runtime / map_image.name, map_image_bytes)
    _atomic_write(runtime / 'map.pgm', map_image_bytes)
    _atomic_write(runtime / 'keepout.yaml', keepout_yaml_bytes)
    _atomic_write(runtime / 'keepout.pgm', keepout_pgm_bytes)

    registry = new_registry(runtime / 'map.yaml', runtime / 'keepout.yaml')
    home_observation = _source_observation(home, home_path)
    home_observation['dock_scan'] = deepcopy(dock_scan)
    home_pose = taught_pose(
        'home_dock', home_xyz, home_observation,
        approach_offset_m=APPROACH_OFFSET_M)
    home_pose['teaching']['captured_at'] = home.get('captured_at')
    home_pose.update({
        'parking_direction': 'reverse',
        'rear_parking_reference': 'lidar_box',
        'target_rear_gap_m': TARGET_GAP_M,
        'source': home.get('source'),
        'scan_path': home.get('scan_path'),
        'target_status': 'NOT_VALIDATED',
    })
    registry = set_home_pose(registry, home_pose)
    station_pose = taught_pose(
        station_id, station_xyz, _source_observation(station, station_path),
        approach_offset_m=APPROACH_OFFSET_M,
        target_front_gap_m=TARGET_GAP_M)
    station_pose['teaching']['captured_at'] = station.get('captured_at')
    station_pose.update({
        'lateral_tolerance_m': LATERAL_TOLERANCE_M,
        'lateral_precision_status': 'NOT_VALIDATED',
        'target_gap_status': 'NOT_VALIDATED',
        'source': station.get('source'),
    })
    registry = add_pose(registry, station_id, station_pose)
    service_path = runtime / 'service_destinations.yaml'
    _atomic_write(service_path, _yaml_bytes(registry))
    # Exercise the same reader used by the runtime before publishing companions.
    loaded = load_registry(service_path)

    heading = station_xyz[2]
    face_distance = front_extent_m + TARGET_GAP_M
    expected_face = [
        station_xyz[0] + face_distance * math.cos(heading),
        station_xyz[1] + face_distance * math.sin(heading),
    ]
    destinations = {
        'schema_version': 1,
        'frame': 'map',
        'face_xy': 'expected search-region face; derived, not measured',
        'normal_deg': 'expected outward face normal; robot approaches from this side',
        'destinations': {
            station_id: {
                'face_xy': expected_face,
                'normal_deg': (math.degrees(heading) + 180.0) % 360.0,
                'source': 'taught map pose plus physical front extent and requested gap',
                'provenance': {
                    'status': 'DERIVED_NOT_MEASURED',
                    'taught_pose_xy_yaw': list(station_xyz),
                    'front_extent_m': front_extent_m,
                    'requested_gap_m': TARGET_GAP_M,
                    'geometry_path': str(geometry_path),
                    'front_extent': front_extent_provenance,
                },
            },
        },
    }
    profiles = {
        'schema_version': 1,
        'map': loaded['map'],
        'keepout': {
            **loaded['keepout'],
            'scope': 'unknown-only',
            'keepout_cells': keepout_cells,
            'table_keepout_inferred': False,
        },
        'home': {
            'id': 'home_dock',
            'source_record': str(home_path),
            'scan_path': home.get('scan_path'),
            'dock_scan': deepcopy(dock_scan),
            'parking_direction': 'reverse',
            'target_rear_gap_m': TARGET_GAP_M,
            'target_status': 'NOT_VALIDATED',
        },
        'stations': {
            station_id: {
                'source_record': str(station_path),
                'capture_folder': station.get('capture_folder'),
                'reference_capture': station.get('reference_capture'),
                'rgbd_reference': station.get('rgbd_reference'),
                'camera_mount_calibration': station.get('camera_mount_calibration'),
                'target_priority': 'operator_taught_map_pose_and_heading',
                'object_reference_role': station.get('object_reference_role'),
                'target_front_gap_m': TARGET_GAP_M,
                'target_gap_status': 'NOT_VALIDATED',
                'lateral_tolerance_m': LATERAL_TOLERANCE_M,
                'lateral_precision_status': 'NOT_VALIDATED',
                'approach_offset_m': APPROACH_OFFSET_M,
                'approach_offset_role': 'route_observation_only_not_final_parking_gap',
                'actual_measured_gap_m': None,
            },
        },
    }
    destinations_path = runtime / 'destinations.json'
    profiles_path = runtime / 'station_profiles.json'
    _atomic_write(destinations_path, _json_bytes(destinations))
    _atomic_write(profiles_path, _json_bytes(profiles))

    return {
        'session_directory': str(source_dir),
        'runtime_directory': str(runtime),
        'output_dir': str(runtime),
        'assets': {
            'map_yaml': str(runtime / 'map.yaml'),
            'map_image': str(runtime / map_image.name),
            'map_pgm_compatibility_copy': str(runtime / 'map.pgm'),
            'keepout_yaml': str(runtime / 'keepout.yaml'),
            'keepout_image': str(runtime / 'keepout.pgm'),
            'service_destinations': str(service_path),
            'destinations': str(destinations_path),
            'station_profiles': str(profiles_path),
        },
        'station_ids': [station_id],
        'map_hashes': {
            'source': source_map,
            'runtime': loaded['map'],
            'keepout': loaded['keepout'],
        },
    }
