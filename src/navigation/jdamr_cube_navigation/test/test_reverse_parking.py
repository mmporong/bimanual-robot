"""Tests for pure reverse-parking path and controller helpers."""

import json
import math
from copy import deepcopy  # noqa: I100
from pathlib import Path

from jdamr_cube_navigation.reverse_parking import (  # noqa: I101
    CachedStaticCorridorValidator, RPP_PLUGIN, reverse_controller_overrides,
    reverse_curve_waypoints, reverse_waypoints, static_corridor_clear,
)
from jdamr_cube_navigation.surface_approach import face_approach_waypoints

import pytest

import yaml


def _controller(desired_linear_vel=0.12):
    return {
        'controller_plugins': ['FollowPath', 'Parking'],
        'FollowPath': {'plugin': RPP_PLUGIN},
        'Parking': {
            'plugin': RPP_PLUGIN,
            'desired_linear_vel': desired_linear_vel,
            'use_rotate_to_heading': True,
            'allow_reversing': False,
            'use_collision_detection': True,
            'lookahead_dist': 0.4,
        },
    }


def _write_grid(
        root: Path, name: str, pixels: list[int], *, width: int = 7,
        height: int = 5, resolution: float = 0.1,
        origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
        negate: int = 0) -> Path:
    """Write a small binary PGM map fixture and its YAML metadata."""
    image = root / f'{name}.pgm'
    image.write_bytes(
        f'P5\n{width} {height}\n255\n'.encode('ascii') + bytes(pixels))
    metadata = root / f'{name}.yaml'
    metadata.write_text(yaml.safe_dump({
        'image': image.name,
        'resolution': resolution,
        'origin': list(origin),
        'negate': negate,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }), encoding='utf-8')
    return metadata


@pytest.mark.parametrize('case,expected', [
    ('target_alias', True), ('source_obstacle', False), ('other_keepout', False),
    ('outside_span', False), ('crosses_face', False), ('unknown_keepout', False),
])
def test_observed_table_only_resolves_its_own_keepout_grid_alias(tmp_path, case, expected):
    width, height = 20, 20
    source_pixels, mask_pixels = [254] * 400, [254] * 400
    for row in range(7, 13):
        mask_pixels[row * width + 9] = 0
    if case == 'source_obstacle':
        source_pixels[10 * width + 9] = 0
    elif case == 'other_keepout':
        mask_pixels[10 * width + 6] = 0
    elif case == 'unknown_keepout':
        mask_pixels[10 * width + 9] = 100
    source = _write_grid(tmp_path, 'source', source_pixels, width=width,
                         height=height, resolution=.05, origin=(-.5, -.5, 0.))
    mask = _write_grid(tmp_path, 'mask', mask_pixels, width=width,
                       height=height, resolution=.05, origin=(-.5, -.5, 0.))
    footprint = ((.05, .05), (-.05, .05), (-.05, -.05), (.05, -.05))
    path = [(-.25, 0., 0.), (-.02 if case == 'crosses_face' else -.08, 0., 0.)]
    surface = {'point_xy_m': (0., 0.), 'normal_xy': (-1., 0.),
               'half_span_m': .02 if case == 'outside_span' else .3}
    assert not static_corridor_clear(source, mask, path, footprint)
    check = CachedStaticCorridorValidator(source, mask, footprint)
    assert check(path, target_surface=surface) is expected


@pytest.mark.parametrize('case_index', [0, 1])
def test_recorded_water_near_turn_and_reapproach_clear_target_alias(tmp_path, case_index):
    fixture = json.loads((Path(__file__).parent / 'fixtures'
                          / 'water_parking_failure_20261019.json').read_text())
    grid, case = fixture['map_fixture'], fixture['cases'][case_index]
    width, height = grid['width'], grid['height']
    source_pixels, mask_pixels = [254] * (width * height), [254] * (width * height)
    for row in range(height):
        for column in range(width):
            if grid['origin'][0] + column * grid['resolution'] < 0.:
                mask_pixels[row * width + column] = 0
    kwargs = {key: grid[key] for key in ('width', 'height', 'resolution', 'origin')}
    source = _write_grid(tmp_path, 'source', source_pixels, **kwargs)
    mask = _write_grid(tmp_path, 'mask', mask_pixels, **kwargs)
    poses = face_approach_waypoints(case['pose'], case['face'], case['normal'], .065, .225,
                                    allow_near_alignment=True)
    check = CachedStaticCorridorValidator(source, mask, fixture['footprint'])
    assert not check(poses)
    surface = {'point_xy_m': case['face'], 'normal_xy': case['normal'],
               'half_span_m': case['span_m'] / 2.}
    assert check(poses, target_surface=surface)


@pytest.mark.parametrize('yaw', [0.0, math.pi / 2.0, math.pi])
def test_waypoints_reverse_along_cardinal_headings(yaw):
    """Generate dense rearward segments and retain both exact endpoints."""
    target = (1.0, -2.0, yaw)
    start = (
        target[0] + 0.2 * math.cos(yaw),
        target[1] + 0.2 * math.sin(yaw),
        yaw,
    )

    path = reverse_waypoints(start, target)

    assert path[0] == start
    assert path[-1] == target
    for current, following in zip(path, path[1:]):
        dx = following[0] - current[0]
        dy = following[1] - current[1]
        assert math.hypot(dx, dy) <= 0.025 + 1e-12
        assert dx * math.cos(current[2]) + dy * math.sin(current[2]) < 0.0


def test_waypoints_interpolate_shortest_yaw_across_pi_boundary():
    """Interpolate measured yaw without taking the long rotation."""
    start_yaw = math.radians(179.0)
    target_yaw = math.radians(-179.0)
    target = (0.0, 0.0, target_yaw)
    start = (
        0.2 * math.cos(target_yaw),
        0.2 * math.sin(target_yaw),
        start_yaw,
    )

    path = reverse_waypoints(start, target)

    assert path[0][2] == start_yaw
    assert path[-1][2] == target_yaw
    yaw_steps = [
        math.atan2(
            math.sin(following[2] - current[2]),
            math.cos(following[2] - current[2]))
        for current, following in zip(path, path[1:])
    ]
    assert all(0.0 < step < math.radians(1.0) for step in yaw_steps)


@pytest.mark.parametrize('start,target,kwargs,match', [
    ((-0.2, 0.0, 0.0), (0.0, 0.0, 0.0), {}, 'in front'),
    ((0.2, 0.051, 0.0), (0.0, 0.0, 0.0), {}, 'lateral'),
    ((0.2, 0.0, math.radians(3.1)), (0.0, 0.0, 0.0), {}, 'yaw'),
    ((0.05, 0.0, 0.0), (0.0, 0.0, 0.0), {}, 'already at goal'),
    ((2.01, 0.0, 0.0), (0.0, 0.0, 0.0), {}, 'max_distance'),
    ((0.001, 0.05, math.radians(-3.0)), (0.0, 0.0, 0.0), {},
     'not behind'),
])
def test_waypoints_reject_unsafe_geometry(start, target, kwargs, match):
    """Reject paths that are not short, aligned, and reverse-only."""
    with pytest.raises(ValueError, match=match):
        reverse_waypoints(start, target, **kwargs)


@pytest.mark.parametrize('start,target,kwargs', [
    ((math.nan, 0.0, 0.0), (0.0, 0.0, 0.0), {}),
    ((True, 0.0, 0.0), (0.0, 0.0, 0.0), {}),
    ((0.2, 0.0, 0.0), (0.0, math.inf, 0.0), {}),
    ((0.2, 0.0, 0.0), (0.0, 0.0, 0.0), {'spacing_m': False}),
    ((0.2, 0.0, 0.0), (0.0, 0.0, 0.0), {'spacing_m': 0.0}),
    ((0.2, 0.0, 0.0), (0.0, 0.0, 0.0), {'spacing_m': 0.026}),
    ((0.2, 0.0, 0.0), (0.0, 0.0, 0.0), {'xy_tolerance_m': 0.051}),
    ((0.2, 0.0, 0.0), (0.0, 0.0, 0.0), {'max_distance_m': 2.01}),
    ((0.2, 0.0, 0.0), (0.0, 0.0, 0.0),
     {'yaw_tolerance_rad': math.inf}),
])
def test_waypoints_reject_nonfinite_boolean_and_invalid_bounds(
        start, target, kwargs):
    """Reject malformed poses and attempts to relax accepted bounds."""
    with pytest.raises(ValueError):
        reverse_waypoints(start, target, **kwargs)


def test_controller_override_is_isolated_and_safety_limited():
    """Clone Parking without mutating or aliasing the caller's mappings."""
    original = _controller()
    before = deepcopy(original)

    configured = reverse_controller_overrides(original)

    assert original == before
    assert configured['controller_plugins'] == [
        'FollowPath', 'Parking', 'ParkingReverse']
    assert configured['Parking'] == original['Parking']
    reverse = configured['ParkingReverse']
    assert reverse['plugin'] == RPP_PLUGIN
    assert reverse['desired_linear_vel'] == 0.08
    assert reverse['use_rotate_to_heading'] is False
    assert reverse['allow_reversing'] is True
    assert reverse['use_collision_detection'] is True
    assert reverse['lookahead_dist'] == 0.4

    reverse['lookahead_dist'] = 99.0
    assert configured['Parking']['lookahead_dist'] == 0.4
    assert original['Parking']['lookahead_dist'] == 0.4


def test_controller_override_keeps_stricter_parking_velocity():
    """Avoid raising a source controller velocity already below the cap."""
    configured = reverse_controller_overrides(_controller(0.05))
    assert configured['ParkingReverse']['desired_linear_vel'] == 0.05


@pytest.mark.parametrize('mutate', [
    lambda value: value['controller_plugins'].remove('Parking'),
    lambda value: value['controller_plugins'].append('Parking'),
    lambda value: value.pop('Parking'),
    lambda value: value['Parking'].__setitem__('plugin', 'not-rpp'),
    lambda value: value['Parking'].__setitem__('desired_linear_vel', True),
    lambda value: value['controller_plugins'].append('ParkingReverse'),
])
def test_controller_override_rejects_missing_duplicate_or_unsafe_source(
        mutate):
    """Require one RPP Parking source and a free ID."""
    controller = _controller()
    mutate(controller)
    with pytest.raises(ValueError):
        reverse_controller_overrides(controller)


def test_static_corridor_accepts_only_clear_map_and_keepout_cells(tmp_path):
    """Accept a footprint corridor when both aligned grids are free."""
    free = [254] * (9 * 7)
    source = _write_grid(tmp_path, 'map', free, width=9, height=7)
    keepout = _write_grid(tmp_path, 'keepout', free, width=9, height=7)

    assert static_corridor_clear(
        source, keepout, [(0.45, 0.35, math.pi / 2.0)],
        [(-0.04, -0.02), (0.04, -0.02), (0.04, 0.02),
         (-0.04, 0.02)])


@pytest.mark.parametrize('case,allowed', [
    ('rear', True), ('front', False), ('unknown', False), ('map', False),
    ('turn', False), ('reverse', False), ('new_cell', False), ('still_overlapping', False),
])
def test_dock_escape_leaves_only_existing_rear_black_keepout_cells(tmp_path, case, allowed):
    width, height = 15, 9
    source_pixels = [254] * (width * height)
    mask_pixels = list(source_pixels)
    rear_index = (height - 1 - 4) * width + 3
    mask_pixels[rear_index] = 0
    if case == 'front':
        mask_pixels[rear_index] = 254
        mask_pixels[(height - 1 - 4) * width + 6] = 0
    elif case == 'unknown':
        mask_pixels[rear_index] = 205
    elif case == 'map':
        source_pixels[rear_index] = 0
    elif case == 'new_cell':
        mask_pixels[(height - 1 - 4) * width + 8] = 0
    source = _write_grid(tmp_path, 'map', source_pixels, width=width, height=height)
    keepout = _write_grid(tmp_path, 'keepout', mask_pixels, width=width, height=height)
    end = (1.0, 0.45, 0.0)
    if case == 'turn':
        end = (1.0, 0.45, 0.1)
    elif case == 'reverse':
        end = (0.5, 0.45, 0.0)
    elif case == 'still_overlapping':
        end = (0.61, 0.45, 0.0)
    footprint = [(-0.28, -0.04), (0.07, -0.04), (0.07, 0.04), (-0.28, 0.04)]
    poses = [(0.6, 0.45, 0.0), end]
    assert not static_corridor_clear(source, keepout, poses, footprint)
    assert static_corridor_clear(
        source, keepout, poses, footprint, allow_rear_keepout_escape=True) is allowed


@pytest.mark.parametrize('grid_name,pixel', [
    ('map', 205),
    ('map', 0),
    ('keepout', 205),
    ('keepout', 0),
])
def test_static_corridor_rejects_unknown_occupied_and_keepout_pixels(
        tmp_path, grid_name, pixel):
    """Treat every pixel that is not clearly free as blocked."""
    grids = {
        'map': [254] * (9 * 7),
        'keepout': [254] * (9 * 7),
    }
    center_index = (7 - 1 - 3) * 9 + 4
    grids[grid_name][center_index] = pixel
    source = _write_grid(tmp_path, 'map', grids['map'], width=9, height=7)
    keepout = _write_grid(
        tmp_path, 'keepout', grids['keepout'], width=9, height=7)

    assert not static_corridor_clear(
        source, keepout, [(0.45, 0.35, 0.0)],
        [(-0.03, -0.03), (0.03, -0.03), (0.03, 0.03),
         (-0.03, 0.03)])


def test_static_corridor_applies_negate_and_free_threshold(tmp_path):
    """Honor map-server occupancy conversion for negated images."""
    free_when_negated = [0] * (9 * 7)
    source = _write_grid(
        tmp_path, 'map', free_when_negated, width=9, height=7, negate=1)
    keepout = _write_grid(
        tmp_path, 'keepout', free_when_negated,
        width=9, height=7, negate=1)
    footprint = [
        (-0.03, -0.03), (0.03, -0.03), (0.03, 0.03), (-0.03, 0.03)]

    assert static_corridor_clear(
        source, keepout, [(0.45, 0.35, 0.0)], footprint)

    occupied = list(free_when_negated)
    occupied[(7 - 1 - 3) * 9 + 4] = 255
    blocked_source = _write_grid(
        tmp_path, 'blocked', occupied, width=9, height=7, negate=1)
    assert not static_corridor_clear(
        blocked_source, keepout, [(0.45, 0.35, 0.0)], footprint)


def test_static_corridor_expands_between_pose_samples(tmp_path):
    """Cover translation gaps between poses with a conservative margin."""
    source_pixels = [254] * (15 * 7)
    source_pixels[(7 - 1 - 3) * 15 + 6] = 0
    source = _write_grid(
        tmp_path, 'map', source_pixels, width=15, height=7)
    keepout = _write_grid(
        tmp_path, 'keepout', [254] * (15 * 7), width=15, height=7)

    assert not static_corridor_clear(
        source, keepout, [(0.45, 0.35, 0.0), (0.75, 0.35, 0.0)],
        [(-0.02, -0.02), (0.02, -0.02), (0.02, 0.02),
         (-0.02, 0.02)])


def test_static_corridor_does_not_expand_into_neighboring_cells(tmp_path):
    """Avoid rejecting a free swept polygon for a nearby blocked cell."""
    source_pixels = [254] * (15 * 7)
    source_pixels[(7 - 1 - 2) * 15 + 6] = 0
    source = _write_grid(
        tmp_path, 'map', source_pixels, width=15, height=7)
    keepout = _write_grid(
        tmp_path, 'keepout', [254] * (15 * 7), width=15, height=7)

    assert static_corridor_clear(
        source, keepout, [(0.45, 0.35, 0.0), (0.75, 0.35, 0.0)],
        [(-0.02, -0.02), (0.02, -0.02), (0.02, 0.02),
         (-0.02, 0.02)])


def test_static_corridor_preserves_step_recess_until_motion_sweeps_it(tmp_path):
    """A blocked cell between wheel steps is free only while the body stays clear."""
    width = height = 60
    source_pixels = [254] * (width * height)
    source_pixels[(height - 1 - 25) * width + 31] = 0
    source = _write_grid(
        tmp_path, 'map', source_pixels, width=width, height=height,
        resolution=.02)
    keepout = _write_grid(
        tmp_path, 'keepout', [254] * (width * height), width=width,
        height=height, resolution=.02)
    footprint = [
        (-.20, -.20), (.20, -.20), (.20, -.10), (.05, -.10),
        (.05, .10), (.20, .10), (.20, .20), (-.20, .20),
        (-.20, .10), (-.05, .10), (-.05, -.10), (-.20, -.10),
    ]

    assert static_corridor_clear(source, keepout, [(.50, .50, 0.)], footprint)
    assert not static_corridor_clear(
        source, keepout, [(.50, .50, 0.), (.58, .50, 0.)], footprint)


def test_static_corridor_rejects_self_intersecting_footprint(tmp_path):
    free = [254] * (9 * 7)
    source = _write_grid(tmp_path, 'map', free, width=9, height=7)
    keepout = _write_grid(tmp_path, 'keepout', free, width=9, height=7)

    with pytest.raises(ValueError, match='simple polygon'):
        static_corridor_clear(
            source, keepout, [(.45, .35, 0.)],
            [(-.05, -.05), (.05, .05), (-.05, .05), (.05, -.05)])


def test_static_corridor_rejects_map_boundary_and_grid_mismatch(tmp_path):
    """Fail closed outside either grid and reject differing geometry."""
    free = [254] * (9 * 7)
    source = _write_grid(tmp_path, 'map', free, width=9, height=7)
    keepout = _write_grid(tmp_path, 'keepout', free, width=9, height=7)
    footprint = [
        (-0.04, -0.04), (0.04, -0.04), (0.04, 0.04), (-0.04, 0.04)]

    assert not static_corridor_clear(
        source, keepout, [(0.04, 0.35, 0.0)], footprint)

    mismatch = _write_grid(
        tmp_path, 'mismatch', free, width=9, height=7, resolution=0.11)
    with pytest.raises(ValueError, match='identical geometry'):
        static_corridor_clear(
            source, mismatch, [(0.45, 0.35, 0.0)], footprint)


def test_static_corridor_rejects_rotated_grid_origins(tmp_path):
    """Fail closed even when both grids share one unsupported rotation."""
    free = [254] * (9 * 7)
    rotated_origin = (0.0, 0.0, 0.1)
    source = _write_grid(
        tmp_path, 'map', free, width=9, height=7, origin=rotated_origin)
    keepout = _write_grid(
        tmp_path, 'keepout', free, width=9, height=7,
        origin=rotated_origin)

    with pytest.raises(ValueError, match='rotated map origins'):
        static_corridor_clear(
            source, keepout, [(0.45, 0.35, 0.0)],
            [(-0.04, -0.04), (0.04, -0.04), (0.04, 0.04),
             (-0.04, 0.04)])


def test_cached_static_corridor_loads_grids_once(tmp_path, monkeypatch):
    """Cache map pixels and the verified footprint across control-loop calls."""
    free = [254] * (15 * 9)
    source = _write_grid(tmp_path, 'map', free, width=15, height=9)
    keepout = _write_grid(tmp_path, 'keepout', free, width=15, height=9)
    footprint = [
        (-0.04, -0.03), (0.04, -0.03), (0.04, 0.03), (-0.04, 0.03)]
    from jdamr_cube_navigation import reverse_parking
    original = reverse_parking._map_metadata
    loaded = []

    def record(path):
        loaded.append(path)
        return original(path)

    monkeypatch.setattr(reverse_parking, '_map_metadata', record)
    validate = CachedStaticCorridorValidator(source, keepout, footprint)

    assert validate([(0.45, 0.45, 0.0), (0.65, 0.45, 0.0)])
    assert validate([(0.45, 0.45, 0.0), (0.55, 0.45, 0.0)])
    assert len(loaded) == 2


def test_cached_static_corridor_bounds_swept_polygon_axes(tmp_path, monkeypatch):
    """Keep SAT work bounded when a curved preview contains hundreds of poses."""
    free = [254] * (25 * 17)
    source = _write_grid(tmp_path, 'map', free, width=25, height=17)
    keepout = _write_grid(tmp_path, 'keepout', free, width=25, height=17)
    validate = CachedStaticCorridorValidator(
        source, keepout,
        [(-0.04, -0.03), (0.04, -0.03), (0.04, 0.03), (-0.04, 0.03)])
    from jdamr_cube_navigation import reverse_parking
    original = reverse_parking._corridor_blockage
    vertex_counts = []

    def record(source_metadata, keepout_metadata, pieces, *args, **kwargs):
        vertex_counts.extend(len(polygon) for polygon, _padding in pieces)
        return original(
            source_metadata, keepout_metadata, pieces, *args, **kwargs)

    monkeypatch.setattr(reverse_parking, '_corridor_blockage', record)
    poses = [
        (0.65 + 0.45 * index / 399,
         0.75 + 0.08 * math.sin(math.pi * index / 399),
         math.radians(30.0) * math.sin(math.pi * index / 399))
        for index in range(400)]

    assert validate(poses)
    assert vertex_counts and max(vertex_counts) <= 16


def test_cached_static_corridor_reuses_nonfree_cell_index(tmp_path, monkeypatch):
    """Avoid rescanning clearly free map cells after cache construction."""
    width = height = 20
    source_pixels = [254] * (width * height)
    keepout_pixels = list(source_pixels)
    keepout_pixels[10 * width + 12] = 0
    source = _write_grid(
        tmp_path, 'map', source_pixels, width=width, height=height,
        resolution=0.05)
    keepout = _write_grid(
        tmp_path, 'keepout', keepout_pixels, width=width, height=height,
        resolution=0.05)
    validate = CachedStaticCorridorValidator(
        source, keepout,
        [(-0.04, -0.03), (0.04, -0.03), (0.04, 0.03), (-0.04, 0.03)])
    from jdamr_cube_navigation import reverse_parking

    def unexpected_lookup(*_args, **_kwargs):
        raise AssertionError('cached corridor validation rescanned a free cell')

    monkeypatch.setattr(reverse_parking, '_clearly_free', unexpected_lookup)

    assert validate([(0.2, 0.2, 0.0), (0.4, 0.2, 0.0)])


@pytest.mark.parametrize('negate', [0, 1])
def test_cached_nonfree_index_matches_full_scan_first_blockage(tmp_path, negate):
    """Keep exact first-block ordering across boundaries and ignored cells."""
    width, height = 13, 9
    pixel_values = (0, 100, 200, 205, 254)
    source_pixels = [pixel_values[(row * 3 + column * 2) % len(pixel_values)]
                     for row in range(height) for column in range(width)]
    keepout_pixels = [pixel_values[(row * 2 + column * 3 + 1) % len(pixel_values)]
                      for row in range(height) for column in range(width)]
    source = _write_grid(
        tmp_path, 'map', source_pixels, width=width, height=height,
        resolution=0.05, origin=(-0.1, -0.15, 0.0), negate=negate)
    keepout = _write_grid(
        tmp_path, 'keepout', keepout_pixels, width=width, height=height,
        resolution=0.05, origin=(-0.1, -0.15, 0.0), negate=negate)
    validate = CachedStaticCorridorValidator(
        source, keepout,
        [(-0.04, -0.03), (0.04, -0.03), (0.04, 0.03), (-0.04, 0.03)])
    from jdamr_cube_navigation import reverse_parking

    ignored_keepout = {(2, 2), (7, 5)}
    ignored_unknown = {(3, 1), (8, 6)}
    for index in range(60):
        left = -0.16 + (index % 15) * 0.055
        bottom = -0.21 + (index % 11) * 0.047
        polygon = [(left, bottom), (left + 0.09, bottom),
                   (left + 0.08, bottom + 0.07),
                   (left - 0.01, bottom + 0.06)]
        pieces = [(polygon, (index % 3) * 0.004)]
        arguments = (validate._source, validate._keepout, pieces,
                     ignored_keepout)
        keywords = {
            'skip_clearly_free_cells': True,
            'ignored_unknown_cells': ignored_unknown,
        }
        full_scan = reverse_parking._corridor_blockage(
            *arguments, **keywords)
        indexed = reverse_parking._corridor_blockage(
            *arguments, **keywords,
            nonfree_columns_by_row=validate._nonfree_columns_by_row)

        assert indexed == full_scan


def test_cached_static_corridor_uses_body_edge_axes_at_rotated_pose(tmp_path):
    """Avoid a fixed-world support corner touching keepout beside the body."""
    width = height = 80
    source_pixels = [254] * (width * height)
    keepout_pixels = list(source_pixels)
    keepout_pixels[(height - 1 - 23) * width + 31] = 0
    source = _write_grid(
        tmp_path, 'map', source_pixels, width=width, height=height,
        resolution=0.02)
    keepout = _write_grid(
        tmp_path, 'keepout', keepout_pixels, width=width, height=height,
        resolution=0.02)
    footprint = [
        (-0.28, -0.275), (0.07, -0.275),
        (0.07, 0.275), (-0.28, 0.275)]
    pose = (0.8, 0.8, math.radians(11.25))
    from jdamr_cube_navigation import reverse_parking
    metadata = reverse_parking._map_metadata(source)
    mask = reverse_parking._map_metadata(keepout)
    fixed_world_support = reverse_parking._circumscribed_support_polygon(
        reverse_parking._world_footprint(pose, footprint))

    assert reverse_parking._corridor_blockage(
        metadata, mask, [(fixed_world_support, 0.0)],
        skip_clearly_free_cells=True) is not None
    assert static_corridor_clear(source, keepout, [pose], footprint)
    assert CachedStaticCorridorValidator(
        source, keepout, footprint)([pose])


@pytest.mark.parametrize('poses', [
    [],
    [(math.nan, 0.45, 0.0)],
    [(0.45, True, 0.0)],
    [(0.45, 0.45)],
])
def test_cached_static_corridor_rejects_invalid_poses(tmp_path, poses):
    free = [254] * (15 * 9)
    source = _write_grid(tmp_path, 'map', free, width=15, height=9)
    keepout = _write_grid(tmp_path, 'keepout', free, width=15, height=9)
    validate = CachedStaticCorridorValidator(
        source, keepout,
        [(-0.04, -0.03), (0.04, -0.03), (0.04, 0.03), (-0.04, 0.03)])

    with pytest.raises(ValueError):
        validate(poses)


def test_cached_static_corridor_rejects_invalid_footprint(tmp_path):
    free = [254] * (15 * 9)
    source = _write_grid(tmp_path, 'map', free, width=15, height=9)
    keepout = _write_grid(tmp_path, 'keepout', free, width=15, height=9)

    with pytest.raises(ValueError, match='footprint'):
        CachedStaticCorridorValidator(
            source, keepout, [(0.0, 0.0), (math.inf, 0.0), (0.0, 0.1)])


@pytest.mark.parametrize('grid_name,pixel', [
    ('map', 205), ('map', 0), ('keepout', 205), ('keepout', 0),
])
def test_cached_static_corridor_rejects_unknown_and_blocked_cells(
        tmp_path, grid_name, pixel):
    width, height = 15, 9
    grids = {
        'map': [254] * (width * height),
        'keepout': [254] * (width * height),
    }
    grids[grid_name][(height - 1 - 4) * width + 6] = pixel
    source = _write_grid(
        tmp_path, 'map', grids['map'], width=width, height=height)
    keepout = _write_grid(
        tmp_path, 'keepout', grids['keepout'], width=width, height=height)
    validate = CachedStaticCorridorValidator(
        source, keepout,
        [(-0.04, -0.03), (0.04, -0.03), (0.04, 0.03), (-0.04, 0.03)])

    assert not validate([(0.65, 0.45, 0.0)])


def test_cached_static_corridor_rejects_outside_grid(tmp_path):
    free = [254] * (15 * 9)
    source = _write_grid(tmp_path, 'map', free, width=15, height=9)
    keepout = _write_grid(tmp_path, 'keepout', free, width=15, height=9)
    validate = CachedStaticCorridorValidator(
        source, keepout,
        [(-0.04, -0.03), (0.04, -0.03), (0.04, 0.03), (-0.04, 0.03)])

    assert not validate([(0.02, 0.45, 0.0)])


def test_cached_static_corridor_rejects_curved_path_collision(tmp_path):
    """Detect a blocked cell swept by a curved preview between its endpoints."""
    width, height = 19, 15
    source_pixels = [254] * (width * height)
    source_pixels[(height - 1 - 7) * width + 9] = 0
    source = _write_grid(
        tmp_path, 'map', source_pixels, width=width, height=height)
    keepout = _write_grid(
        tmp_path, 'keepout', [254] * (width * height),
        width=width, height=height)
    validate = CachedStaticCorridorValidator(
        source, keepout,
        [(-0.04, -0.03), (0.04, -0.03), (0.04, 0.03), (-0.04, 0.03)])
    poses = [
        (0.55, 0.55, 0.0),
        (0.75, 0.65, math.radians(25.0)),
        (1.05, 0.75, math.radians(40.0)),
    ]

    assert not static_corridor_clear(source, keepout, poses, [
        (-0.04, -0.03), (0.04, -0.03), (0.04, 0.03), (-0.04, 0.03)])
    assert not validate(poses)


@pytest.mark.parametrize('blocked_column,blocked_map_row', [
    (5, 4), (7, 4), (9, 5), (11, 6), (13, 6), (15, 7),
])
def test_cached_static_corridor_never_frees_an_exact_blockage(
        tmp_path, blocked_column, blocked_map_row):
    """The aggregate hull may reject more cells, but never omit an exact piece."""
    width, height = 21, 13
    source_pixels = [254] * (width * height)
    source_pixels[
        (height - 1 - blocked_map_row) * width + blocked_column] = 0
    source = _write_grid(
        tmp_path, 'map', source_pixels, width=width, height=height)
    keepout = _write_grid(
        tmp_path, 'keepout', [254] * (width * height),
        width=width, height=height)
    footprint = [
        (-0.06, -0.04), (0.06, -0.04), (0.06, 0.04), (-0.06, 0.04)]
    poses = [
        (0.55, 0.45, 0.0),
        (0.75, 0.45, math.radians(8.0)),
        (0.95, 0.55, math.radians(18.0)),
        (1.15, 0.65, math.radians(25.0)),
        (1.55, 0.75, math.radians(30.0)),
    ]
    exact = static_corridor_clear(source, keepout, poses, footprint)
    cached = CachedStaticCorridorValidator(source, keepout, footprint)(poses)

    assert exact or not cached


def test_reverse_curve_takes_out_a_staging_offset_while_backing_up():
    """A 10 cm lateral, 1.2 deg offset ends exactly on the dock pose, always backing."""
    target = (-0.204, 0.184, 0.0)
    start = (0.496, 0.284, math.radians(-1.2))
    path = reverse_curve_waypoints(start, target)
    assert path[0] == pytest.approx(start)
    assert path[-1] == pytest.approx(target)
    for (x0, y0, yaw0), (x1, y1, _yaw1) in zip(path, path[1:]):
        assert (x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0) < 0.0


def test_reverse_curve_rejects_a_turn_tighter_than_the_bound():
    with pytest.raises(ValueError, match='tighter'):
        reverse_curve_waypoints((0.2, 0.4, 0.0), (0.0, 0.0, 0.0))
