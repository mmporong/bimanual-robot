"""Ideal approach rollout and static corridor regressions without hardware."""

import math

from jdamr_cube_navigation.reverse_parking import static_corridor_clear
from jdamr_cube_navigation.surface_approach import (
    face_approach_command, face_approach_waypoints, filtered_face_heading,
)
import pytest
import yaml


@pytest.mark.parametrize('angle_deg', [-30.0, -12.0, 12.0, 30.0])
@pytest.mark.parametrize('heading', [0.0, math.pi - 0.01])
def test_preview_aligns_before_precision_band(angle_deg, heading):
    face = (0.5 * math.cos(heading), 0.5 * math.sin(heading))
    normal = (-math.cos(heading), -math.sin(heading))
    poses = face_approach_waypoints(
        (0.0, 0.0, heading + math.radians(angle_deg)), face, normal, 0.065, 0.225)
    assert len(poses) > 1
    final = poses[-1]
    residual = math.atan2(math.sin(heading - final[2]), math.cos(heading - final[2]))
    gap = (sum((final[i] - face[i]) * normal[i] for i in (0, 1))
           - 0.065 * math.cos(residual) - 0.225 * abs(math.sin(residual)))
    assert abs(residual) <= math.radians(0.3)
    assert 0.049 <= gap <= 0.0505
    assert math.dist(poses[0][:2], final[:2]) <= 0.6


def test_close_aligned_face_keeps_existing_precision_band():
    assert face_approach_waypoints(
        (0.4, 0.0, 0.0), (0.5, 0.0), (-1.0, 0.0), 0.065, 0.225) == [(0.4, 0.0, 0.0)]
    with pytest.raises(ValueError, match='more clearance'):
        face_approach_waypoints(
            (0.4, 0.0, 0.1), (0.5, 0.0), (-1.0, 0.0), 0.065, 0.225)


def test_close_table_alignment_turns_toward_face_without_backing_out():
    error = .014061414934757988
    nearest = .033676181083231255
    face = (.065 * math.cos(error) + .225 * math.sin(error) + nearest, 0.)
    poses = face_approach_waypoints(
        (0., 0., error), face, (-1., 0.), .065, .225,
        allow_near_alignment=True)
    assert len(poses) > 1
    assert all(pose[:2] == (0., 0.) for pose in poses)
    assert abs(poses[-1][2]) <= math.radians(.3)
    gaps = [face[0] - .065 * math.cos(p[2]) - .225 * abs(math.sin(p[2])) for p in poses]
    assert min(gaps) >= nearest - 1e-9
    assert all(a <= b + 1e-9 for a, b in zip(gaps, gaps[1:]))
    linear, angular = face_approach_command(.02368, -error, allow_near_alignment=True)
    assert linear == 0.
    assert angular < 0.


@pytest.mark.parametrize('angle', [-.23, 0., .23])
def test_rear_coarse_preview_stops_at_intermediate_gap(angle):
    normal = math.cos(angle), math.sin(angle)
    offset = -1.45 - .275 * normal[0] - .225 * abs(normal[1])
    face = offset / normal[0], 0.
    target_gap = 1.45 - .5 * math.cos(angle)
    poses = face_approach_waypoints(
        (0., 0., math.pi), face, normal, .275, .225, target_gap,
        command_heading_rad=angle + math.pi)
    final = poses[-1]
    error = math.atan2(math.sin(angle + math.pi - final[2]),
                       math.cos(angle + math.pi - final[2]))
    gap = (sum((final[i] - face[i]) * normal[i] for i in (0, 1))
           - .275 * math.cos(error) - .225 * abs(math.sin(error)))
    assert target_gap - .001 <= gap <= target_gap + .0005
    assert math.dist(poses[0][:2], final[:2]) <= .6


@pytest.mark.parametrize('remaining,error', [(math.nan, 0.0), (0.1, math.inf)])
def test_command_rejects_nonfinite_inputs(remaining, error):
    with pytest.raises(ValueError):
        face_approach_command(remaining, error)


def test_command_preserves_straight_speed_and_requires_early_alignment():
    linear, angular = face_approach_command(0.3, math.radians(12.0))
    assert 0.0 < linear < 0.05
    assert angular == 0.02
    with pytest.raises(ValueError, match='more clearance'):
        face_approach_command(0.02, math.radians(12.0))
    assert face_approach_command(0.3, 0.0) == (0.05, 0.0)


def test_command_tapers_only_inside_the_final_approach_band():
    assert face_approach_command(0.026, 0.0) == (0.05, 0.0)
    assert face_approach_command(0.020, 0.0) == pytest.approx((0.04, 0.0))
    assert face_approach_command(0.005, 0.0) == (0.01, 0.0)


def test_heading_filter_crosses_pi_along_shortest_arc():
    filtered = filtered_face_heading(math.pi - 0.01, -math.pi + 0.01, 0.35)
    assert filtered == pytest.approx(math.pi - 0.01 + (1.0 - math.exp(-1.0)) * 0.02)


def test_preview_uses_current_filtered_heading_then_converges_to_raw_normal():
    start = (0.0, 0.0, math.radians(12.0))
    filtered = math.radians(-5.0)
    poses = face_approach_waypoints(
        start, (0.5, 0.0), (-1.0, 0.0), 0.065, 0.225,
        command_heading_rad=filtered)
    remaining = (0.5 - 0.065 * math.cos(start[2])
                 - 0.225 * abs(math.sin(start[2])) - 0.01) / math.cos(start[2])
    linear, angular = face_approach_command(remaining, filtered - start[2])
    assert poses[1] == pytest.approx((linear * math.cos(start[2]) * 0.05,
                                     linear * math.sin(start[2]) * 0.05,
                                     start[2] + angular * 0.05))
    assert abs(poses[-1][2]) <= math.radians(0.3)


def test_curved_corridor_rejects_keepout_missed_by_straight_plan(tmp_path):
    poses = face_approach_waypoints(
        (0.0, 0.0, math.radians(30.0)), (0.5, 0.0), (-1.0, 0.0), 0.065, 0.225)
    point = max(poses, key=lambda pose: pose[1])
    resolution, width, height = 0.01, 80, 80
    origin = (-0.1, -0.4, 0.0)
    pixels = bytearray([254] * (width * height))
    column = math.floor((point[0] - origin[0]) / resolution)
    map_row = math.floor((point[1] - origin[1]) / resolution)

    def write(name, data):
        image = tmp_path / (name + '.pgm')
        image.write_bytes(f'P5\n{width} {height}\n255\n'.encode() + bytes(data))
        path = tmp_path / (name + '.yaml')
        path.write_text(yaml.safe_dump({
            'image': image.name, 'resolution': resolution, 'origin': list(origin),
            'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}))
        return path

    source = write('map', pixels)
    pixels[(height - 1 - map_row) * width + column] = 0
    mask = write('mask', pixels)
    footprint = [(-0.003, -0.003), (0.003, -0.003), (0.003, 0.003), (-0.003, 0.003)]
    assert static_corridor_clear(source, mask, [(0.0, 0.0, 0.0), (0.4, 0.0, 0.0)], footprint)
    assert not static_corridor_clear(source, mask, poses, footprint)
