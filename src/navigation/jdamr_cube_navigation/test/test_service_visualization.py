"""Ensure display-only assets cannot masquerade as taught table poses."""

from copy import deepcopy
import math

from jdamr_cube_navigation.service_visualization import (
    build_markers,
    load_saved_station_poses,
)
import pytest
from visualization_msgs.msg import Marker


@pytest.fixture
def assets():
    """Provide synthetic provenance and one measured home pose."""
    return ({
        'frame_id': 'map', 'map': {'image_sha256': 'abc'}, 'tables': [],
        'home': {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0,
                 'parking_direction': 'reverse'},
    }, {'source_map_sha256': 'abc', 'markers': {
        'table_1': {'approximate_map_xy_m': [1.0, 2.0]},
        'table_2': {'approximate_map_xy_m': [3.0, 4.0]},
    }})


def test_approximate_regions_have_no_docking_arrows(assets):
    registry, annotation = assets
    original = deepcopy(registry)
    markers = build_markers(registry, annotation).markers
    assert markers[0].action == Marker.DELETEALL
    assert registry == original
    assert [m.id for m in markers if m.type == Marker.ARROW] == [100]
    areas = [m for m in markers if m.type == Marker.CYLINDER]
    assert [(m.pose.position.x, m.pose.position.y) for m in areas] == [(1, 2), (3, 4)]
    assert any('TABLE_1_SELECTED' in m.text and 'AREA_ONLY' in m.text for m in markers)
    assert all(m.header.frame_id == 'map' for m in markers)


def test_wrong_map_annotation_rejected(assets):
    registry, annotation = assets
    annotation['source_map_sha256'] = 'other'
    with pytest.raises(ValueError, match='different map'):
        build_markers(registry, annotation)


def test_single_taught_water_station_does_not_invent_old_table_regions(assets):
    registry, _ = assets
    registry['tables'] = [{'table_id': 'water_station', 'service_poses': [
        {'x_m': 0.1, 'y_m': -0.2, 'yaw_rad': 0.3}]}]
    markers = build_markers(
        registry, {'source_map_sha256': 'abc', 'markers': {}},
        selected='water_station').markers
    assert not any(item.type == Marker.CYLINDER for item in markers)
    station = next(item for item in markers if item.id == 200)
    assert station.type == Marker.ARROW
    assert station.pose.position.x == 0.1
    assert [item.id for item in markers if item.type == Marker.ARROW] == [200, 100]
    assert len({item.id for item in markers}) == len(markers)
    assert any('WATER_STATION_SELECTED' in item.text for item in markers)


def test_taught_pose_gains_heading_arrow(assets):
    registry, annotation = assets
    registry['tables'] = [{'table_id': 'table_01', 'service_poses': [
        {'x_m': 1.2, 'y_m': 2.2, 'yaw_rad': 1.5707963267948966}]}]
    markers = build_markers(registry, annotation).markers
    arrow = next(m for m in markers if m.id == 12)
    assert arrow.pose.orientation.z == pytest.approx(2**-0.5)
    assert arrow.pose.position.x == 1.2
    assert any('DOCK_POSE_TAUGHT' in m.text for m in markers)


def test_republish_removes_old_taught_pose(assets):
    registry, annotation = assets
    registry['tables'] = [{'table_id': 'table_01', 'service_poses': [
        {'x_m': 1.2, 'y_m': 2.2, 'yaw_rad': 0.0}]}]
    cache = {}
    for message in (build_markers(registry, annotation), build_markers(
            {**registry, 'tables': [], 'home': None}, annotation)):
        for item in message.markers:
            if item.action == Marker.DELETEALL:
                cache.clear()
            else:
                cache[(item.ns, item.id)] = item
    assert not any(m.type == Marker.ARROW for m in cache.values())


def test_t39_water_station_marker_is_drawn_when_annotated(assets):
    """The water stop appears as its own area; tables and home are unchanged."""
    registry, annotation = assets
    before = build_markers(registry, annotation).markers
    annotation = deepcopy(annotation)
    annotation['markers']['water_station'] = {'approximate_map_xy_m': [0.06, -1.21]}
    markers = build_markers(registry, annotation).markers
    texts = [item.text for item in markers if item.type == Marker.TEXT_VIEW_FACING]
    assert any(text.startswith('WATER_STATION') for text in texts)
    area = next(item for item in markers if item.type == Marker.CYLINDER
                and (item.pose.position.x, item.pose.position.y) == (0.06, -1.21))
    assert area.action == Marker.ADD
    assert len(markers) == len(before) + 2


def _write_saved_pose(path, *, station_id='table_01', frame='map',
                      base_frame='base_link', yaml_sha='yaml-sha', image_sha='abc'):
    path.write_text(
        f'id: {station_id}\n'
        f'frame_id: {frame}\n'
        f'robot_base_frame: {base_frame}\n'
        'x_m: -0.734910483363684\n'
        'y_m: -1.6068820918282494\n'
        'yaw_rad: -0.03841021709958648\n'
        'map:\n'
        f'  yaml_sha256: {yaml_sha}\n'
        f'  image_sha256: {image_sha}\n')


def test_saved_table_pose_draws_exact_arrow_and_table_label(tmp_path, assets):
    registry, annotation = assets
    registry['map']['yaml_sha256'] = 'yaml-sha'
    _write_saved_pose(tmp_path / 'table_01.yaml')
    saved = load_saved_station_poses(registry, tmp_path)
    markers = build_markers(registry, annotation, saved_station_poses=saved).markers
    arrow = next(item for item in markers if item.id == 10000)
    assert arrow.type == Marker.ARROW
    assert arrow.pose.position.x == pytest.approx(-0.734910483363684)
    assert arrow.pose.position.y == pytest.approx(-1.6068820918282494)
    assert arrow.pose.orientation.z == pytest.approx(math.sin(-0.03841021709958648 / 2))
    assert arrow.pose.orientation.w == pytest.approx(math.cos(-0.03841021709958648 / 2))
    assert any(item.text == 'TABLE 1\nSAVED POSE' for item in markers)
    assert not any(item.type == Marker.CYLINDER
                   and item.pose.position.x == pytest.approx(-0.734910483363684)
                   for item in markers)


@pytest.mark.parametrize('case', [
    'missing', 'image_mismatch', 'yaml_mismatch', 'frame_mismatch',
    'base_frame_mismatch', 'filename_mismatch',
])
def test_missing_or_stale_saved_pose_is_ignored(tmp_path, assets, case):
    registry, annotation = assets
    registry['map']['yaml_sha256'] = 'yaml-sha'
    directory = tmp_path / 'stations'
    if case != 'missing':
        directory.mkdir()
        _write_saved_pose(
            directory / ('table_02.yaml' if case == 'filename_mismatch'
                         else 'table_01.yaml'),
            frame='odom' if case == 'frame_mismatch' else 'map',
            base_frame=('base_footprint' if case == 'base_frame_mismatch'
                        else 'base_link'),
            yaml_sha='old-yaml' if case == 'yaml_mismatch' else 'yaml-sha',
            image_sha='old-image' if case == 'image_mismatch' else 'abc')
    saved = load_saved_station_poses(registry, directory)
    markers = build_markers(registry, annotation, saved_station_poses=saved).markers
    assert saved == []
    assert [item.id for item in markers if item.type == Marker.ARROW] == [100]


def test_registered_table_is_not_duplicated_by_saved_pose(tmp_path, assets):
    registry, annotation = assets
    registry['map']['yaml_sha256'] = 'yaml-sha'
    registry['tables'] = [{'table_id': 'table_01', 'service_poses': [
        {'x_m': 1.2, 'y_m': 2.2, 'yaw_rad': 0.0}]}]
    _write_saved_pose(tmp_path / 'table_01.yaml')
    saved = load_saved_station_poses(registry, tmp_path)
    markers = build_markers(registry, annotation, saved_station_poses=saved).markers
    assert saved == []
    assert [item.id for item in markers if item.type == Marker.ARROW] == [12, 100]
    assert len({item.id for item in markers}) == len(markers)
