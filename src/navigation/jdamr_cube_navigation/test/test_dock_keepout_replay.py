"""Replay the stopped rear observation against the bundled dock mask."""

import base64
import hashlib
import json
import math
import zlib
from pathlib import Path

from jdamr_cube_navigation.keepout_mask import (
    _enabled_polygons, _map_metadata, _point_in_polygon, _write_pgm)
from jdamr_cube_navigation.reverse_parking import CachedStaticCorridorValidator
from jdamr_cube_navigation.surface_approach import face_approach_waypoints

import pytest

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
RUNTIME = (
    Path(__file__).resolve().parents[4] / 'config/navigation/jdamr_runtime')
EVIDENCE = PACKAGE / 'test/fixtures/rear_dock_keepout_20261020.json'
LATEST_EVIDENCE = (
    PACKAGE / 'test/fixtures/rear_dock_keepout_20261020_171136.json')
REMOVAL_EVIDENCE = PACKAGE / 'test/fixtures/rear_dock_keepout_20261021.json'


def _mask_before_dock_removal(tmp_path):
    """Reconstruct the historical mask without rewriting recorded evidence."""
    evidence = json.loads(REMOVAL_EVIDENCE.read_text())
    current = _map_metadata(RUNTIME / 'keepout.yaml')
    if hashlib.sha256((RUNTIME / 'keepout.pgm').read_bytes()).hexdigest() == (
            evidence['previous_mask_image_sha256']):
        return current
    pixels = bytearray(current['pixels'])
    for column, row in evidence['removed_cells_column_image_row']:
        assert pixels[row * current['width'] + column] == 254
        pixels[row * current['width'] + column] = 0
    image = tmp_path / 'before_removal.pgm'
    _write_pgm(image, current['width'], current['height'], bytes(pixels))
    assert hashlib.sha256(image.read_bytes()).hexdigest() == (
        evidence['previous_mask_image_sha256'])
    return {**current, 'pixels': bytes(pixels), 'image': image}


def _nominal_full_approach(evidence):
    observed, transform = evidence['observation'], evidence['odom_to_map']
    angle = transform.get('angle_rad')
    if angle is None:
        angle = 2. * math.atan2(
            transform['rotation'][2], transform['rotation'][3])
    cosine, sine = math.cos(angle), math.sin(angle)

    def to_map(pose):
        x, y, yaw = pose
        return (transform['translation'][0] + cosine * x - sine * y,
                transform['translation'][1] + sine * x + cosine * y,
                yaw + angle)

    x, y, yaw = observed['base_pose_odom']
    preview = face_approach_waypoints(
        (x, y, yaw + math.pi), observed['face_odom'], observed['normal_odom'],
        evidence['rear_extent_m'], evidence['half_width_m'],
        evidence['target_gap_m'],
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


def test_retreat_clears_recorded_rear_approach_through_one_centimetre(
        tmp_path):
    """The earlier stopped approach remains clear after the second retreat."""
    evidence = json.loads(EVIDENCE.read_text())
    params = yaml.safe_load(
        (PACKAGE / 'config/new_base_nav2_params.yaml').read_text())
    footprint = yaml.safe_load(
        params['global_costmap']['global_costmap']['ros__parameters'][
            'footprint'])
    poses = _nominal_full_approach(evidence)
    old_pixels = zlib.decompress(base64.b64decode(
        evidence['old_mask_pixels_zlib_base64']))
    meta = _map_metadata(RUNTIME / 'keepout.yaml')
    _write_pgm(
        tmp_path / 'before.pgm', meta['width'], meta['height'], old_pixels)
    before_digest = hashlib.sha256(
        (tmp_path / 'before.pgm').read_bytes()).hexdigest()
    assert before_digest == evidence['old_mask_image_sha256']
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

    old = _map_metadata(tmp_path / 'before.yaml')
    new = _mask_before_dock_removal(tmp_path)
    removed = 0
    pairs = zip(old['pixels'], new['pixels'])
    for index, (prior, current) in enumerate(pairs):
        if prior == current:
            continue
        assert prior == 0 and current == 254
        column, row = index % old['width'], index // old['width']
        center = (
            old['origin'][0] + (column + .5) * old['resolution'],
            old['origin'][1]
            + (old['height'] - row - .5) * old['resolution'])
        assert _point_in_polygon(*center, evidence['old_dock_polygon'])
        removed += 1
    latest = json.loads(LATEST_EVIDENCE.read_text())
    assert removed == (
        evidence['cleared_mask_cells']
        + len(latest['restored_cells_column_image_row'])) > 0


def test_latest_rear_preview_and_one_centimetre_endpoint_clear_dock_mask(
        tmp_path):
    """The failed latest preview clears its nominal one-centimetre end."""
    evidence = json.loads(LATEST_EVIDENCE.read_text())
    params = yaml.safe_load(
        (PACKAGE / 'config/new_base_nav2_params.yaml').read_text())
    footprint = yaml.safe_load(
        params['global_costmap']['global_costmap']['ros__parameters'][
            'footprint'])
    poses = _nominal_full_approach(evidence)
    assert poses[0] == pytest.approx(evidence['logged_preview_start_map'])
    # map<-odom moved by 2.7 mm while production built the preview. This
    # transform is anchored to the start and still reproduces its logged end.
    assert poses[-10] == pytest.approx(
        evidence['logged_preview_end_map'], abs=.003)
    assert poses[-1] == pytest.approx(
        evidence['derived_nominal_one_centimetre_endpoint_map'])

    current = _mask_before_dock_removal(tmp_path)
    prior_pixels = bytearray(current['pixels'])
    for column, row in evidence['restored_cells_column_image_row']:
        index = row * current['width'] + column
        assert prior_pixels[index] == 254
        prior_pixels[index] = 0
    _write_pgm(
        tmp_path / 'before.pgm', current['width'], current['height'],
        bytes(prior_pixels))
    before_digest = hashlib.sha256(
        (tmp_path / 'before.pgm').read_bytes()).hexdigest()
    assert before_digest == evidence['previous_mask_image_sha256']
    document = yaml.safe_load((RUNTIME / 'keepout.yaml').read_text())
    document['image'] = 'before.pgm'
    (tmp_path / 'before.yaml').write_text(yaml.safe_dump(document))

    before = CachedStaticCorridorValidator(
        RUNTIME / 'map.yaml', tmp_path / 'before.yaml', footprint)
    assert not before(poses)
    assert before.last_blockage['grid'] == 'keepout'
    restored = [
        tuple(cell) for cell in evidence['restored_cells_column_image_row']]
    assert (
        before.last_blockage['column'],
        before.last_blockage['image_row']) in restored
    assert (
        evidence['blocked_cell']['column'],
        evidence['blocked_cell']['image_row']) in restored

    dock = next(
        zone for zone in yaml.safe_load(
            (RUNTIME / 'table_keepout_current.yaml').read_text())['zones']
        if zone['id'] == 'dock_table')
    previous_polygon = [list(point) for point in dock['polygon']]
    previous_polygon[1][0] += evidence['additional_retreat_map_x_m']
    previous_polygon[2][0] += evidence['additional_retreat_map_x_m']
    insufficient_polygon = [list(point) for point in previous_polygon]
    insufficient_polygon[1][0] -= .11
    insufficient_polygon[2][0] -= .11
    insufficient_pixels = bytearray(prior_pixels)
    for column, row in restored:
        center = (
            current['origin'][0] + (column + .5) * current['resolution'],
            current['origin'][1]
            + (current['height'] - row - .5) * current['resolution'])
        if not _point_in_polygon(*center, insufficient_polygon):
            insufficient_pixels[row * current['width'] + column] = 254
    _write_pgm(
        tmp_path / 'eleven.pgm', current['width'], current['height'],
        bytes(insufficient_pixels))
    document['image'] = 'eleven.pgm'
    (tmp_path / 'eleven.yaml').write_text(yaml.safe_dump(document))
    insufficient = CachedStaticCorridorValidator(
        RUNTIME / 'map.yaml', tmp_path / 'eleven.yaml', footprint)
    assert not insufficient(poses)
    assert insufficient.last_blockage['grid'] == 'keepout'

    after = CachedStaticCorridorValidator(
        RUNTIME / 'map.yaml', RUNTIME / 'keepout.yaml', footprint)
    assert after(poses)
    assert after.last_blockage is None
    _write_pgm(tmp_path / 'retreated.pgm', current['width'], current['height'],
               current['pixels'])
    current_digest = hashlib.sha256((tmp_path / 'retreated.pgm').read_bytes()).hexdigest()
    assert current_digest == evidence['new_mask_image_sha256']


def test_disabled_dock_keepout_clears_latest_blocked_path_only(tmp_path):
    evidence = json.loads(REMOVAL_EVIDENCE.read_text())
    config = yaml.safe_load((RUNTIME / 'table_keepout_current.yaml').read_text())
    assert 'dock_table' not in {zone_id for zone_id, _ in _enabled_polygons(config)}
    current = _map_metadata(RUNTIME / 'keepout.yaml')
    previous = _mask_before_dock_removal(tmp_path)
    base = _map_metadata(RUNTIME / 'keepout_base_unknown.yaml')
    changed = []
    for index, (before, after) in enumerate(zip(previous['pixels'], current['pixels'])):
        column, row = index % current['width'], index // current['width']
        center = (
            current['origin'][0] + (column + .5) * current['resolution'],
            current['origin'][1] + (current['height'] - row - .5) * current['resolution'])
        if _point_in_polygon(*center, evidence['disabled_dock_polygon']):
            assert after == base['pixels'][index]
        if before == after:
            continue
        assert before == 0 and after == 254
        assert _point_in_polygon(*center, evidence['disabled_dock_polygon'])
        changed.append([column, row])
    assert changed == evidence['removed_cells_column_image_row']
    assert [evidence['blocked_cell']['column'], evidence['blocked_cell']['image_row']] in (
        changed)

    params = yaml.safe_load((PACKAGE / 'config/new_base_nav2_params.yaml').read_text())
    footprint = yaml.safe_load(
        params['global_costmap']['global_costmap']['ros__parameters']['footprint'])
    poses = _nominal_full_approach(evidence)
    assert poses[0] == pytest.approx(evidence['logged_preview_start_map'])
    assert poses[-10] == pytest.approx(evidence['logged_preview_end_map'], abs=.003)
    document = yaml.safe_load((RUNTIME / 'keepout.yaml').read_text())
    document['image'] = str(previous['image'])
    prior_yaml = tmp_path / 'prior.yaml'
    prior_yaml.write_text(yaml.safe_dump(document))
    before = CachedStaticCorridorValidator(RUNTIME / 'map.yaml', prior_yaml, footprint)
    assert not before(poses)
    assert before.last_blockage['grid'] == 'keepout'
    # Isolate the requested mask change from occupied cells in the SLAM map.
    mask_only = CachedStaticCorridorValidator(
        RUNTIME / 'keepout_base_unknown.yaml', RUNTIME / 'keepout.yaml', footprint)
    assert mask_only(poses)
    assert mask_only.last_blockage is None
    after = CachedStaticCorridorValidator(
        RUNTIME / 'map.yaml', RUNTIME / 'keepout.yaml', footprint)
    assert not after(poses)
    assert after.last_blockage['reason'] == 'map_not_free'
    assert after.last_blockage['grid'] == 'map'


def test_dock_mask_identity_and_source_geometry_remain_consistent():
    """The package and runtime configs name the same checked mask image."""
    config = yaml.safe_load(
        (PACKAGE / 'config/table_keepout_current.yaml').read_text())
    assert config == yaml.safe_load(
        (RUNTIME / 'table_keepout_current.yaml').read_text())
    registry = yaml.safe_load(
        (RUNTIME / 'service_destinations.yaml').read_text())
    digest = hashlib.sha256((RUNTIME / 'keepout.pgm').read_bytes()).hexdigest()
    assert registry['keepout']['image_sha256'] == digest
    assert registry['map']['image_sha256'] == hashlib.sha256(
        (RUNTIME / 'map.pgm').read_bytes()).hexdigest()
    for route in RUNTIME.glob('*route.yaml'):
        assert yaml.safe_load(
            route.read_text())['expected_mask_sha256'] == digest
    dock = registry['home']
    previous = json.loads(EVIDENCE.read_text())['expected_home_pose_map']
    reframed = dock['laser_extrinsic_reframe']
    assert reframed['previous_pose_map'] == pytest.approx(previous)
    assert reframed['physical_home_changed'] is False

    def world_laser(base, laser):
        return (base[0] + math.cos(base[2])*laser[0] - math.sin(base[2])*laser[1],
                base[1] + math.sin(base[2])*laser[0] + math.cos(base[2])*laser[1],
                base[2] + laser[2])

    current = (dock['x_m'], dock['y_m'], dock['yaw_rad'])
    assert world_laser(current, reframed['new_laser']) == pytest.approx(
        world_laser(previous, reframed['previous_laser']), abs=1e-12)
    geometry = yaml.safe_load((
        PACKAGE.parent / 'jdamr_cube_description/config/new_base_geometry.yaml').read_text())
    assert reframed['new_laser'] == pytest.approx([
        *geometry['laser_translation']['value'][:2], geometry['laser_yaw']['value']])
