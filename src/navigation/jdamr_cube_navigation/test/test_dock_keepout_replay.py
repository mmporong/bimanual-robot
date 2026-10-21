"""Replay the stopped rear observation against the bundled dock mask."""

import base64
import hashlib
import json
import math
import zlib
from pathlib import Path

from jdamr_cube_navigation.keepout_mask import _map_metadata, _point_in_polygon, _write_pgm
from jdamr_cube_navigation.reverse_parking import CachedStaticCorridorValidator
from jdamr_cube_navigation.surface_approach import face_approach_waypoints
import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
RUNTIME = Path(__file__).resolve().parents[4] / 'config/navigation/jdamr_runtime'
EVIDENCE = PACKAGE / 'test/fixtures/rear_dock_keepout_20261020.json'


def _nominal_full_approach(evidence):
    observed, transform = evidence['observation'], evidence['odom_to_map']
    angle = 2. * math.atan2(transform['rotation'][2], transform['rotation'][3])
    cosine, sine = math.cos(angle), math.sin(angle)

    def to_map(pose):
        x, y, yaw = pose
        return (transform['translation'][0] + cosine * x - sine * y,
                transform['translation'][1] + sine * x + cosine * y, yaw + angle)

    x, y, yaw = observed['base_pose_odom']
    preview = face_approach_waypoints(
        (x, y, yaw + math.pi), observed['face_odom'], observed['normal_odom'],
        evidence['rear_extent_m'], evidence['half_width_m'], evidence['target_gap_m'],
        command_heading_rad=yaw + math.pi)
    poses = [to_map((x, y, yaw - math.pi)) for x, y, yaw in preview]
    face = to_map((*observed['face_odom'], 0.))[:2]
    nx, ny = observed['normal_odom']
    normal = cosine * nx - sine * ny, sine * nx + cosine * ny
    # The steering preview ends at the alignment band. Include the remaining
    # straight segment so the mask also permits the unchanged final target.
    start = poses[-1]
    remaining = (sum((start[i] - face[i]) * normal[i] for i in (0, 1))
                 - evidence['rear_extent_m'] - evidence['target_gap_m'])
    for step in range(1, 10):
        poses.append((start[0] - normal[0] * remaining * step / 9,
                      start[1] - normal[1] * remaining * step / 9,
                      math.atan2(normal[1], normal[0])))
    return poses


def test_retreat_clears_recorded_rear_approach_through_one_centimetre(tmp_path):
    evidence = json.loads(EVIDENCE.read_text())
    params = yaml.safe_load((PACKAGE / 'config/new_base_nav2_params.yaml').read_text())
    footprint = yaml.safe_load(
        params['global_costmap']['global_costmap']['ros__parameters']['footprint'])
    poses = _nominal_full_approach(evidence)
    old_pixels = zlib.decompress(base64.b64decode(evidence['old_mask_pixels_zlib_base64']))
    meta = _map_metadata(RUNTIME / 'keepout.yaml')
    _write_pgm(tmp_path / 'before.pgm', meta['width'], meta['height'], old_pixels)
    assert hashlib.sha256((tmp_path / 'before.pgm').read_bytes()).hexdigest() == (
        evidence['old_mask_image_sha256'])
    document = yaml.safe_load((RUNTIME / 'keepout.yaml').read_text())
    document['image'] = 'before.pgm'
    (tmp_path / 'before.yaml').write_text(yaml.safe_dump(document))
    before = CachedStaticCorridorValidator(
        RUNTIME / 'map.yaml', tmp_path / 'before.yaml', footprint)
    assert not before(poses)
    assert before.last_blockage['grid'] == 'keepout'
    after = CachedStaticCorridorValidator(
        RUNTIME / 'map.yaml', RUNTIME / 'keepout.yaml', footprint)
    assert after(poses)
    assert after.last_blockage is None
    assert evidence['motion'] == {'linear_mps': 0., 'angular_radps': 0.}

    old, new = _map_metadata(tmp_path / 'before.yaml'), _map_metadata(RUNTIME / 'keepout.yaml')
    removed = 0
    for index, (prior, current) in enumerate(zip(old['pixels'], new['pixels'])):
        if prior == current:
            continue
        assert prior == 0 and current == 254
        column, row = index % old['width'], index // old['width']
        center = (old['origin'][0] + (column + .5) * old['resolution'],
                  old['origin'][1] + (old['height'] - row - .5) * old['resolution'])
        assert _point_in_polygon(*center, evidence['old_dock_polygon'])
        removed += 1
    assert removed == evidence['cleared_mask_cells'] > 0


def test_dock_mask_identity_and_source_geometry_remain_consistent():
    config = yaml.safe_load((PACKAGE / 'config/table_keepout_current.yaml').read_text())
    assert config == yaml.safe_load((RUNTIME / 'table_keepout_current.yaml').read_text())
    registry = yaml.safe_load((RUNTIME / 'service_destinations.yaml').read_text())
    digest = hashlib.sha256((RUNTIME / 'keepout.pgm').read_bytes()).hexdigest()
    assert registry['keepout']['image_sha256'] == digest
    assert registry['map']['image_sha256'] == hashlib.sha256(
        (RUNTIME / 'map.pgm').read_bytes()).hexdigest()
    for route in RUNTIME.glob('*route.yaml'):
        assert yaml.safe_load(route.read_text())['expected_mask_sha256'] == digest
    dock = registry['home']
    assert (dock['x_m'], dock['y_m'], dock['yaw_rad']) == pytest.approx(
        json.loads(EVIDENCE.read_text())['expected_home_pose_map'])
