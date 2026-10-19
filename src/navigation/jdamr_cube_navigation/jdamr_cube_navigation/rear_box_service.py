"""Rear LiDAR face feedback on the existing guarded precision motion path."""

import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist
from jdamr_cube_navigation.box_lidar_witness import (
    MAX_CANDIDATE_PLANE_DISTANCE_M, MAXIMUM_NORMAL_YAW_DIFFERENCE_RAD,
)
from jdamr_cube_navigation.corridor_route import load_route, spin_node
from jdamr_cube_navigation.parking import (
    MAXIMUM_FRONT_GAP_M, MINIMUM_FRONT_GAP_M,
)
from jdamr_cube_navigation.rear_box_docking import (
    estimate_rear_box_face, rear_corridor_clearance, RearBoxFaceTracker,
)
from jdamr_cube_navigation.rear_collision_profile import rear_collision_profile
from jdamr_cube_navigation.restaurant_service import (
    LIVE_APPROACH_MAX_M, STRAIGHT_ESCAPE_TOLERANCE_M,
)
from jdamr_cube_navigation.reverse_parking import (
    CachedStaticCorridorValidator, MAX_REVERSE_DISTANCE_M, reverse_waypoints,
)
from jdamr_cube_navigation.surface_approach import (
    FACE_APPROACH_TIME_S, face_approach_waypoints, FACE_HEADING_FILTER_S, filtered_face_heading,
)
from nav_msgs.msg import Path as RosPath
import rclpy
from tf2_ros import TransformException


REAR_COARSE_HANDOFF_M = 0.5
REAR_COARSE_SEGMENT_M = 0.5


def _angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _same_target(reference, observed):
    normal, old_normal = observed['normal_odom'], reference['normal_odom']
    change = abs(_angle(math.atan2(normal[1], normal[0])
                        - math.atan2(old_normal[1], old_normal[0])))
    displacement = abs(sum(
        (observed['face_odom'][i] - reference['face_odom'][i]) * old_normal[i]
        for i in (0, 1)))
    if (change > MAXIMUM_NORMAL_YAW_DIFFERENCE_RAD
            or displacement > MAX_CANDIDATE_PLANE_DISTANCE_M):
        raise RuntimeError('rear box target changed')


def _stationary_face(observations, extent, half_width):
    """Use the stopped scan window rather than its last noisy line fit."""
    result = dict(observations[-1])
    for normal_key, face_key in (
            ('normal_base_xy', 'face_center_base_xy_m'),
            ('normal_odom', 'face_odom')):
        normals = [item[normal_key] for item in observations]
        angle = math.atan2(sum(n[1] for n in normals), sum(n[0] for n in normals))
        normal = math.cos(angle), math.sin(angle)
        offset = sum(sum(item[normal_key][i] * item[face_key][i] for i in (0, 1))
                     for item in observations) / len(observations)
        result[normal_key], result[face_key] = normal, tuple(offset*n for n in normal)
    nx, ny = result['normal_base_xy']
    point = result['face_center_base_xy_m']
    offset = sum(point[i] * result['normal_base_xy'][i] for i in (0, 1))
    result.update(
        face_center_base_xy_m=(offset / nx, 0.),
        line_anchor_base_xy_m=point, line_offset_m=offset,
        tangent_base_xy=(-ny, nx), normal_yaw_error_rad=math.atan2(ny, nx),
        center_gap_m=-offset / nx - extent,
        gap_m=-offset - extent * nx - half_width * abs(ny),
        averaged_frames=len(observations))
    return result


class RearBoxParkingMixin:
    """Observe a rear box without starting the depth camera or a transit task."""

    def _rear_box_scan(self):
        scan = self.last_scan
        received = self.samples.get('scan')
        if scan is None or received is None or time.monotonic() - received > 0.5:
            raise RuntimeError('rear box scan unavailable')
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if not 0.0 <= now_s - stamp <= 0.5:
            raise RuntimeError('rear box scan stale or future-dated')
        if scan.header.frame_id != 'laser_link':
            raise RuntimeError('rear box scan frame mismatch')
        return scan, stamp

    @staticmethod
    def _rear_scan_values(scan):
        return {
            'ranges': list(scan.ranges), 'angle_min': scan.angle_min,
            'angle_increment': scan.angle_increment,
            'range_min': scan.range_min, 'range_max': scan.range_max,
        }

    def _rear_box_observation(self, geometry, extent, half_width):
        scan, stamp = self._rear_box_scan()
        cached = getattr(self, '_rear_box_detection_cache', None)
        if (cached is not None and cached[0] == stamp
                and cached[1] == (id(geometry), extent, half_width)):
            return cached[2]
        try:
            observation = estimate_rear_box_face(
                self._rear_scan_values(scan), geometry=geometry,
                rear_extent_m=extent, rear_half_width_m=half_width)
        except ValueError as error:
            self.emit('rear_box_face_unavailable', reason=str(error))
            raise RuntimeError('rear box face unavailable') from error
        try:
            transform = self.parking_tf.lookup_transform(
                'odom', self.parking_contract['robot_base_frame'],
                rclpy.time.Time(seconds=stamp)).transform
        except TransformException as error:
            raise RuntimeError('rear box stamped odom TF unavailable') from error
        q = transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        cosine, sine = math.cos(yaw), math.sin(yaw)
        x, y = observation['face_center_base_xy_m']
        nx, ny = observation['normal_base_xy']
        result = {**observation, 'stamp_s': stamp,
                  'face_odom': (transform.translation.x + cosine * x - sine * y,
                                transform.translation.y + sine * x + cosine * y),
                  'normal_odom': (cosine * nx - sine * ny, sine * nx + cosine * ny)}
        self._rear_box_detection_cache = (stamp, (id(geometry), extent, half_width), result)
        return result

    def _rear_corridor_observation(self, geometry, extent, half_width):
        scan, stamp = self._rear_box_scan()
        try:
            observed = rear_corridor_clearance(
                self._rear_scan_values(scan), geometry=geometry,
                rear_extent_m=extent, rear_half_width_m=half_width)
        except ValueError as error:
            raise RuntimeError('rear box corridor unavailable') from error
        return {**observed, 'stamp_s': stamp}

    def _rear_coarse_step(self, geometry, extent, half_width, distance_m,
                          command_watch=None):
        start = self._odom_pose()
        target = (start[0] - distance_m * math.cos(start[2]),
                  start[1] - distance_m * math.sin(start[2]), start[2])
        anchor = self._frozen_in_odom(RosPath())[1](0., 0., 0.)
        cosine, sine = math.cos(anchor[2]), math.sin(anchor[2])
        check = CachedStaticCorridorValidator(
            Path(self.registry['map']['yaml_path']),
            Path(self.registry['keepout']['yaml_path']), self._footprint_outline())

        def validate(actual):
            if math.dist(actual[:2], target[:2]) <= 0.05:
                poses = [actual, target]
            else:
                poses = reverse_waypoints(actual, target, max_distance_m=LIVE_APPROACH_MAX_M)
            mapped = []
            for x, y, yaw in poses:
                dx, dy = x - anchor[0], y - anchor[1]
                mapped.append((cosine * dx + sine * dy,
                               -sine * dx + cosine * dy, yaw - anchor[2]))
            if not check(mapped):
                raise RuntimeError('rear box coarse static corridor blocked')

        validate(start)
        validated_pose = [start]

        def watch():
            try:
                observed = self._rear_corridor_observation(geometry, extent, half_width)
                if observed['gap_m'] <= REAR_COARSE_HANDOFF_M:
                    return 'rear box coarse handoff ready'
                actual = self._odom_pose()
                if (math.dist(actual[:2], validated_pose[0][:2]) >= 0.025
                        or abs(_angle(actual[2] - validated_pose[0][2])) >= math.radians(.3)):
                    validate(actual)
                    validated_pose[0] = actual
            except (RuntimeError, ValueError, TransformException) as error:
                return str(error)
            return None

        kwargs = {} if command_watch is None else {'command_watch': command_watch}
        travelled = self._drive_zero_turn(
            -distance_m, watch=watch, hold_heading=True, **kwargs)
        failure = getattr(self, '_last_direct_motion_failure', None)
        if failure == 'rear box coarse handoff ready':
            return True
        if travelled is None or failure or self.stop_requested:
            self.emit('rear_box_coarse_paused', reason=failure or 'coarse motion incomplete')
            return False
        return travelled < 0.0

    def _approach_rear_corridor(self, geometry, extent, half_width, command_watch=None):
        """Range while reversing on the known dock axis; latch a face only nearby."""
        start = self._odom_pose()
        outline = self._footprint_outline()
        corridor_half_width = max(half_width, *(abs(point[1]) for point in outline))
        while not self.stop_requested and time.monotonic() < self.run_deadline_s:
            try:
                observed = self._rear_corridor_observation(
                    geometry, extent, corridor_half_width)
            except RuntimeError as error:
                if not self._rear_wait_for_input(str(error), command_watch):
                    return False
                continue
            if observed['gap_m'] <= REAR_COARSE_HANDOFF_M:
                self.emit('rear_box_fine_handoff', gap_m=observed['gap_m'])
                return True
            travelled = math.dist(start[:2], self._odom_pose()[:2])
            budget = MAX_REVERSE_DISTANCE_M - travelled
            if budget <= 0.0:
                raise RuntimeError('rear box coarse approach exceeds bounded travel')
            distance = min(REAR_COARSE_SEGMENT_M, budget,
                           observed['gap_m'] - REAR_COARSE_HANDOFF_M
                           + STRAIGHT_ESCAPE_TOLERANCE_M)
            self.emit('rear_box_coarse_approach_started', gap_m=observed['gap_m'],
                      distance_m=distance, face_latched=False)
            if not self._rear_coarse_step(
                    geometry, extent, corridor_half_width, distance, command_watch):
                reason = getattr(self, '_last_direct_motion_failure', None)
                if not reason or not self._rear_wait_for_input(reason, command_watch):
                    return False
        return False

    def _rear_wait_for_input(self, reason, command_watch=None):
        recoverable = {
            'rear box scan unavailable', 'rear box scan stale or future-dated',
            'rear box corridor unavailable', 'rear docking swept footprint occupied',
        }
        if reason not in recoverable and not self._input_gap_recoverable(reason):
            return False
        self.emit('rear_box_waiting_for_input', reason=reason, phase='coarse')
        expected = Twist()
        expected.linear.x = -0.05
        while not self.stop_requested and time.monotonic() < self.run_deadline_s:
            spin_node(self, timeout_sec=0.05)
            if not self._operator_link_ready():
                return False
            if self._guard_failure(False):
                continue
            try:
                self._rear_box_scan()
            except RuntimeError:
                continue
            if command_watch is not None and command_watch(expected):
                continue
            self.emit('rear_box_input_recovered', phase='coarse', target_preserved=True)
            return True
        return False

    def _observe_rear_box(self, geometry, extent, half_width):
        tracker = RearBoxFaceTracker(required_frames=8)
        observations = []
        last_stamp = None
        failure = 'rear box unstable'
        deadline = min(self.run_deadline_s, time.monotonic() + 5.0)
        while not self.stop_requested and time.monotonic() < deadline:
            spin_node(self, timeout_sec=0.02)
            if not self._operator_link_ready():
                raise RuntimeError('operator_link_lost')
            motion = getattr(self, 'latest_motion', None)
            if (motion is None or not 0.0 <= time.monotonic() - motion[0] <= 0.5
                    or abs(motion[1]) >= 0.01 or abs(motion[2]) >= 0.02):
                tracker.update(None)
                observations.clear()
                failure = 'rear box waiting for stopped odometry'
                continue
            try:
                observation = self._rear_box_observation(geometry, extent, half_width)
                if observation['stamp_s'] == last_stamp:
                    continue
                last_stamp = observation['stamp_s']
                observations.append(observation)
                observations = observations[-8:]
                if tracker.update(observation):
                    return _stationary_face(observations, extent, half_width)
                failure = 'rear box unstable'
                if tracker.frame_count == 1:
                    observations = observations[-1:]
            except (ValueError, RuntimeError) as error:
                tracker.update(None)
                observations.clear()
                failure = str(error)
        raise RuntimeError(f'rear box observation unavailable: {failure}')

    def _observe_rear_box_recovering(self, geometry, extent, half_width):
        """Keep a temporary perception gap inside the current return task."""
        while not self.stop_requested and time.monotonic() < self.run_deadline_s:
            if not self._operator_link_ready():
                raise RuntimeError('operator_link_lost')
            try:
                return self._observe_rear_box(geometry, extent, half_width)
            except RuntimeError as error:
                if not str(error).startswith('rear box observation unavailable:'):
                    raise
                self.emit('rear_box_waiting_for_observation', reason=str(error),
                          remaining_s=max(0., self.run_deadline_s - time.monotonic()))
        raise RuntimeError('operator_stop' if self.stop_requested
                           else 'rear box observation deadline exceeded')

    def _rear_box_update(self, initial, geometry, extent, half_width, target_gap,
                         reference=None):
        state = dict(initial)
        reference = initial if reference is None else reference
        reference = {'face_odom': tuple(reference['face_odom']),
                     'normal_odom': tuple(reference['normal_odom'])}
        start = self._odom_pose()
        validated = {'stamp': None, 'pose': None, 'heading': None}
        anchor = self._frozen_in_odom(RosPath())[1](0., 0., 0.)
        cosine, sine = math.cos(anchor[2]), math.sin(anchor[2])
        check = CachedStaticCorridorValidator(
            Path(self.registry['map']['yaml_path']),
            Path(self.registry['keepout']['yaml_path']), self._footprint_outline())

        def to_map(pose):
            dx, dy = pose[0] - anchor[0], pose[1] - anchor[1]
            return cosine * dx + sine * dy, -sine * dx + cosine * dy, pose[2] - anchor[2]

        def validate_control(actual, heading):
            if (validated['stamp'] == state['stamp_s'] and validated['pose'] is not None
                    and math.dist(actual[:2], validated['pose'][:2]) < 0.025
                    and abs(_angle(heading - validated['heading'])) < math.radians(0.3)):
                return
            preview = face_approach_waypoints(
                (*actual[:2], actual[2] + math.pi), state['face_odom'],
                state['normal_odom'], extent, half_width, target_gap,
                command_heading_rad=heading + math.pi)
            if len(preview) > 1 and not check([
                    to_map((x, y, yaw - math.pi)) for x, y, yaw in preview]):
                raise RuntimeError('rear box approach static corridor blocked')
            validated.update(stamp=state['stamp_s'], pose=actual, heading=heading)

        def update(actual):
            observation = self._rear_box_observation(geometry, extent, half_width)
            if observation['stamp_s'] < state['stamp_s']:
                raise RuntimeError('rear box scan clock regressed')
            if observation['stamp_s'] != state['stamp_s']:
                _same_target(reference, observation)
                old_heading = math.atan2(state['normal_odom'][1], state['normal_odom'][0])
                raw_heading = math.atan2(observation['normal_odom'][1],
                                         observation['normal_odom'][0])
                elapsed = observation['stamp_s'] - state['stamp_s']
                heading = filtered_face_heading(
                    old_heading, raw_heading, elapsed)
                old_offset = sum(state['normal_odom'][i] * state['face_odom'][i]
                                 for i in (0, 1))
                raw_offset = sum(observation['normal_odom'][i] * observation['face_odom'][i]
                                 for i in (0, 1))
                alpha = 1.0 - math.exp(-elapsed / FACE_HEADING_FILTER_S)
                offset = old_offset + alpha * (raw_offset - old_offset)
                state.update(observation)
                state['normal_odom'] = math.cos(heading), math.sin(heading)
                state['face_odom'] = tuple(offset*n for n in state['normal_odom'])
                self.emit('rear_box_face_updated', observation_stamp_s=state['stamp_s'],
                          gap_m=observation['gap_m'],
                          angle_rad=observation['normal_yaw_error_rad'])
            normal = state['normal_odom']
            heading = math.atan2(normal[1], normal[0])
            error = _angle(heading - actual[2])
            gap = (sum((actual[i] - state['face_odom'][i]) * normal[i] for i in (0, 1))
                   - extent * math.cos(error) - half_width * abs(math.sin(error)))
            if abs(error) > math.radians(30.0):
                raise RuntimeError('rear box heading outside approach range')
            remaining = (gap - target_gap) / math.cos(error)
            if math.dist(start[:2], actual[:2]) + max(0., remaining) > 0.6:
                raise RuntimeError('rear box approach exceeds bounded travel')
            return {'remaining_m': remaining, 'heading_rad': heading,
                    'validate_control': validate_control}

        return update

    def park_rear_box(self, geometry, route_path, *, execute, timeout_s,
                      target_gap_m=None):
        """Align to a live rear box and stop at the requested one-centimetre gap."""
        for key in ('frame_length', 'front_to_wheel_axis', 'frame_width'):
            item = geometry[key]
            if (item['unit'] != 'm' or isinstance(item['value'], bool)
                    or not isinstance(item['value'], (int, float))
                    or not math.isfinite(item['value']) or item['value'] <= 0):
                raise ValueError('invalid rear box chassis geometry')
        extent = geometry['frame_length']['value'] - geometry['front_to_wheel_axis']['value']
        half_width = geometry['frame_width']['value'] / 2.0
        if not all(math.isfinite(v) and v > 0 for v in (extent, half_width)):
            raise ValueError('invalid rear box chassis geometry')
        target_gap = self._final_gap_m() if target_gap_m is None else target_gap_m
        if (isinstance(target_gap, bool)
                or not isinstance(target_gap, (int, float))
                or not math.isfinite(target_gap)
                or not MINIMUM_FRONT_GAP_M <= target_gap <= MAXIMUM_FRONT_GAP_M):
            raise ValueError('target_front_gap_m must be in [0.01, 0.10] m')
        self.run_deadline_s = time.monotonic() + timeout_s
        self.table_id, self.pose_id = 'home_dock', 'rear_box'
        # A staging confirmation from the preceding home leg must not be reused as
        # evidence for this independent live-LiDAR terminal confirmation.
        self.confirmation = None
        self.verify_live_maps()
        if not self.wait_until_ready(timeout=10.0):
            return False
        if not execute:
            self.capture_stationary_pose()
            initial = self._observe_rear_box(geometry, extent, half_width)
            self.emit('rear_box_observed', observation=initial, target_gap_m=target_gap)
            return True
        if not self._reverse_smoother_ready() or not self._parking_parameters_ready(reverse=True):
            return False
        self._confirm_collision_monitor_on()
        with rear_collision_profile(self) as command_watch:
            if not self._approach_rear_corridor(
                    geometry, extent, half_width, command_watch):
                return False
            initial = self._observe_rear_box_recovering(geometry, extent, half_width)
            self.emit('rear_box_observed', observation=initial, target_gap_m=target_gap)
            return self._park_rear_face(
                initial, geometry, route_path, extent, half_width, target_gap, command_watch)

    def _park_rear_face(self, initial, geometry, route_path, extent, half_width,
                        target_gap, command_watch):
        """Keep the fine target fixed after the moving range-only handoff."""

        def make_straight_clearance(observation):
            # At contact range a turn sweeps a rear corner toward the box. Reuse
            # the validated straight dock departure before correcting heading.
            if not (observation['gap_m'] < 0.05 and abs(
                    observation['normal_yaw_error_rad']) > math.radians(0.3)):
                return observation
            self.config = load_route(route_path)
            self.waypoints = self.config['waypoints']
            self._dock_departure = None
            if not self._depart_dock_straight(0.2):
                return None
            reobserved = self._observe_rear_box_recovering(geometry, extent, half_width)
            _same_target(initial, reobserved)
            return reobserved

        def approach(observation):
            update = self._rear_box_update(
                observation, geometry, extent, half_width, target_gap,
                reference=initial)
            remaining = update(self._odom_pose())['remaining_m']
            if remaining <= 0.0:
                return True
            kwargs = {} if command_watch is None else {'command_watch': command_watch}
            return self._drive_zero_turn(
                -remaining, precision=True, hold_heading=True,
                target_update=update, approach_time_s=FACE_APPROACH_TIME_S, **kwargs) is not None

        approach_start = make_straight_clearance(initial)
        if approach_start is None or not approach(approach_start):
            return False

        final = self._observe_rear_box_recovering(geometry, extent, half_width)
        _same_target(initial, final)

        def confirmation(observation):
            error = observation['normal_yaw_error_rad']
            gap = observation['gap_m']
            far_gap = gap + 2.0 * half_width * abs(math.sin(error))
            confirmed = (target_gap - 0.003 <= gap <= target_gap + 0.003
                         and target_gap - 0.003 <= far_gap <= target_gap + 0.003)
            return confirmed, gap, far_gap, error

        confirmed, gap, far_gap, error = confirmation(final)
        if (not confirmed and not getattr(self, 'stop_requested', False)
                and self._operator_link_ready()):
            self.emit('rear_box_terminal_retry_started', nearest_gap_m=gap,
                      farthest_gap_m=far_gap, angle_rad=error,
                      target_gap_m=target_gap, target_preserved=True)
            retry_start = make_straight_clearance(final)
            if retry_start is not None and approach(retry_start):
                retried = self._observe_rear_box_recovering(geometry, extent, half_width)
                _same_target(initial, retried)
                final = retried
                confirmed, gap, far_gap, error = confirmation(final)
        self.confirmation = {
            'confirmed': confirmed,
            'reason': ('confirmed' if confirmed
                       else 'rear_box_gap_out_of_tolerance'),
            'reference': 'live_rear_lidar_face',
            'nearest_gap_m': gap,
            'farthest_gap_m': far_gap,
            'internal_estimate_only': True,
            'external_accuracy_verified': False,
        }
        self.emit('rear_box_parking_finished', confirmed=confirmed,
                  nearest_gap_m=gap, farthest_gap_m=far_gap, angle_rad=error,
                  target_gap_m=target_gap, basis='live_rear_lidar_face',
                  internal_estimate_only=True, external_accuracy_verified=False)
        return confirmed
