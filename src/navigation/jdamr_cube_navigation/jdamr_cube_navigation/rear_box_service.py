"""Rear LiDAR face feedback on the existing guarded precision motion path."""

import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist
from jdamr_cube_navigation.box_lidar_witness import (
    MAX_CANDIDATE_PLANE_DISTANCE_M, MAXIMUM_NORMAL_YAW_DIFFERENCE_RAD,
    MAXIMUM_RESIDUAL_RMS_M,
)
from jdamr_cube_navigation.corridor_route import (
    load_route, spin_node, TRANSIT_PLAN_END_TOLERANCE_M,
)
from jdamr_cube_navigation.parking import (
    MAXIMUM_FRONT_GAP_M, MINIMUM_FRONT_GAP_M,
)
from jdamr_cube_navigation.rear_box_docking import (
    derive_robot_pose_from_dock_reference, estimate_rear_box_face,
    MINIMUM_SUPPORT, MINIMUM_TANGENT_SPREAD_M, rear_corridor_clearance, RearBoxFaceTracker,
)
from jdamr_cube_navigation.rear_collision_profile import rear_collision_profile
from jdamr_cube_navigation.restaurant_service import (
    LIVE_APPROACH_MAX_M, STRAIGHT_ESCAPE_TOLERANCE_M,
)
from jdamr_cube_navigation.reverse_parking import (
    CachedStaticCorridorValidator, MAX_REVERSE_DISTANCE_M, reverse_waypoints,
)
from jdamr_cube_navigation.surface_approach import (
    FACE_APPROACH_TIME_S, FACE_HEADING_FILTER_S, filtered_face_heading,
)
from nav_msgs.msg import Path as RosPath
import rclpy
from tf2_ros import TransformException


REAR_COARSE_HANDOFF_M = 0.5
REAR_CENTER_HANDOFF_M = 1.0
REAR_CENTER_MIN_GAP_M = 0.9
REAR_COARSE_SEGMENT_M = 0.5
REAR_REOBSERVE_SEGMENT_M = 0.2
REAR_FACE_REOBSERVE_GAP_M = 0.2
# Observation only: the swept footprint and gap calculation retain the chassis width.
REAR_FACE_OBSERVATION_HALF_WIDTH_M = 1.2
REAR_FACE_OBSERVATION_WIDTHS_M = (0.6, 0.9, REAR_FACE_OBSERVATION_HALF_WIDTH_M)


def _angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _observe_rear_face_roi(scan, geometry, extent, half_width, expected_span_m=None):
    """Prefer the nearest useful view before including more side furniture."""
    candidate, failure = None, None
    for width_m in REAR_FACE_OBSERVATION_WIDTHS_M:
        try:
            observed = estimate_rear_box_face(
                scan, geometry=geometry, rear_extent_m=extent,
                rear_half_width_m=half_width, observation_half_width_m=width_m)
        except ValueError as error:
            failure = error
            continue
        if candidate is None:
            candidate = observed
        if observed['observation_roi_clipped']:
            continue
        if (expected_span_m is not None
                and abs(observed['span_m'] - expected_span_m) > max(.08, .25 * expected_span_m)):
            continue
        return observed
    if candidate is not None and expected_span_m is None:
        return candidate
    if candidate is not None:
        raise ValueError('rear dock reference face width or visibility changed')
    raise failure or ValueError('rear face unavailable in observation ROI')


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


def _set_collision_target(command_watch, observation, target_gap_m,
                          current_gap_m, source_point_filter=None):
    setter = getattr(command_watch, 'set_target', None)
    if callable(setter):
        setter(observation, target_gap_m, current_gap_m, source_point_filter)


def _clear_collision_target(command_watch):
    clearer = getattr(command_watch, 'clear_target', None)
    if callable(clearer):
        clearer()


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
    for midpoint in ('face_midpoint_base_xy_m', 'midpoint_odom'):
        if all(midpoint in item for item in observations):
            result[midpoint] = tuple(sum(item[midpoint][i] for item in observations)
                                     / len(observations) for i in (0, 1))
    result['observation_roi_clipped'] = any(
        item.get('observation_roi_clipped', False) for item in observations)
    return result


class RearBoxParkingMixin:
    """Observe a rear box without starting the depth camera or a transit task."""

    def _depart_dock_straight(self, distance_m):
        # Only the route's initial dock exit teaches a landmark. A terminal
        # clearance/retry must never replace the outbound reference.
        outbound = (getattr(self, 'home_rear_geometry', None) is not None
                    and self.config.get('dock_departure_distance_m') == distance_m
                    and self.registry.get('home', {}).get('rear_parking_reference') == 'lidar_box')
        if outbound and getattr(self, '_dock_departure', None) is None:
            self._rear_dock_reference = None
        teach = outbound and getattr(self, '_rear_dock_reference', None) is None
        before_exit = None
        if teach:
            geometry = self.home_rear_geometry
            extent = geometry['frame_length']['value'] - geometry['front_to_wheel_axis']['value']
            half_width = geometry['frame_width']['value'] / 2.
            try:
                before_exit = self._rear_box_observation(geometry, extent, half_width)
            except (ValueError, RuntimeError):
                pass
        if not super()._depart_dock_straight(distance_m):
            return False
        if teach:
            geometry = self.home_rear_geometry
            extent = geometry['frame_length']['value'] - geometry['front_to_wheel_axis']['value']
            half_width = geometry['frame_width']['value'] / 2.
            try:
                # The parent already confirmed stopped odometry at the exit.
                # Use its latest fresh stamped scan without another dwell.
                phase = 'after_straight_departure'
                fallback_reason = None
                try:
                    observation = self._rear_box_observation(geometry, extent, half_width)
                    if observation['observation_roi_clipped']:
                        fallback_reason = 'rear dock face is clipped by observation ROI'
                    elif (observation['support_count'] < 2 * MINIMUM_SUPPORT
                          or observation['span_m'] < 2 * MINIMUM_TANGENT_SPREAD_M):
                        fallback_reason = 'rear dock reference support is too small'
                except (ValueError, RuntimeError) as error:
                    if before_exit is None:
                        raise
                    fallback_reason = str(error)
                if fallback_reason is not None and before_exit is not None:
                    observation, phase = before_exit, 'before_straight_departure'
                if observation['observation_roi_clipped']:
                    raise RuntimeError('rear dock face is clipped by observation ROI')
                if (observation['support_count'] < 2 * MINIMUM_SUPPORT
                        or observation['span_m'] < 2 * MINIMUM_TANGENT_SPREAD_M):
                    raise RuntimeError('rear dock reference support is too small')
                departure = self._dock_departure
                end_map, end_odom = departure['end'], self._odom_pose()
                scan_odom = observation['base_pose_odom']
                turn = end_map[2] - end_odom[2]
                cosine, sine = math.cos(turn), math.sin(turn)
                dx, dy = scan_odom[0] - end_odom[0], scan_odom[1] - end_odom[1]
                # The scan can predate the stopped exit pose. Apply its stamped
                # odom displacement before associating the face with map.
                pose = {'x_m': end_map[0] + cosine * dx - sine * dy,
                        'y_m': end_map[1] + sine * dx + cosine * dy,
                        'yaw_rad': _angle(scan_odom[2] + turn)}
                self._rear_dock_reference = {
                    'observation': observation, 'pose_map': pose,
                    'start_pose_map': list(departure['start']),
                    'exit_pose_map': list(departure['end']),
                    'observation_phase': phase,
                }
                self.emit('rear_dock_reference_recorded', reference=self._rear_dock_reference,
                          source='same_run_straight_departure',
                          after_exit_failure=fallback_reason, internal_estimate_only=True)
            except (ValueError, RuntimeError) as error:
                self.emit('rear_dock_reference_unavailable', reason=str(error))
        return True

    def _rear_reference_stage(self, stage):
        reference = getattr(self, '_rear_dock_reference', None)
        if reference is None:
            return stage
        pose, observation = reference['pose_map'], reference['observation']
        angle = pose['yaw_rad'] + observation['normal_yaw_error_rad']
        start, exit_pose = reference['start_pose_map'], reference['exit_pose_map']
        distance = sum((exit_pose[i] - start[i]) * direction
                       for i, direction in enumerate((math.cos(angle), math.sin(angle))))
        if distance <= 0.0:
            raise RuntimeError('rear dock reference has no forward departure')
        # Preserve the outbound origin's tangent position; use the observed
        # box normal for the reverse axis instead of a separate map heading.
        return {**stage, 'x': start[0] + distance * math.cos(angle),
                'y': start[1] + distance * math.sin(angle), 'yaw': _angle(angle)}

    def _rear_reference_pose(self, observed, *, actual_pose_map=None):
        reference = self._rear_dock_reference
        taught = reference['observation']
        if observed.get('observation_roi_clipped', True):
            raise RuntimeError('rear dock reference face is clipped')
        if abs(observed['span_m'] - taught['span_m']) > max(.08, .25 * taught['span_m']):
            raise RuntimeError('rear dock reference face width changed')
        measured = derive_robot_pose_from_dock_reference(taught, observed, reference['pose_map'])
        if actual_pose_map is not None:
            inferred = tuple(measured[k] for k in ('x_m', 'y_m', 'yaw_rad'))
            if (math.dist(actual_pose_map[:2], inferred[:2]) > REAR_FACE_OBSERVATION_HALF_WIDTH_M
                    or abs(_angle(actual_pose_map[2] - inferred[2])) > math.radians(30.)):
                raise RuntimeError('rear dock landmark disagrees with return region')
        return measured

    def _align_rear_reference(self, stage, pose, *, observed=None, return_to_stage=True):
        """Correct observed dock drift, then center the existing reverse entry."""
        if getattr(self, '_rear_dock_reference', None) is None:
            return True
        reason = ('operator_stop' if self.stop_requested else
                  'operator_link_lost' if not self._operator_link_ready() else
                  self._guard_failure(False))
        if reason:
            raise RuntimeError(reason)
        geometry = self.home_rear_geometry
        extent = geometry['frame_length']['value'] - geometry['front_to_wheel_axis']['value']
        half_width = geometry['frame_width']['value'] / 2.
        if observed is None:
            observed = self._observe_rear_box(geometry, extent, half_width)
        actual, _ = self.capture_stationary_pose(intermediate=True)
        measured = self._rear_reference_pose(observed, actual_pose_map=actual)
        inferred = tuple(measured[k] for k in ('x_m', 'y_m', 'yaw_rad'))
        shift = math.dist(actual[:2], inferred[:2])
        turn = abs(_angle(actual[2] - inferred[2]))
        # This seed comes from a measured landmark, not the registered dock pose.
        if shift > .02 or turn > math.radians(.5):
            from jdamr_cube_navigation.activate_navigation import initialize_localization

            initialize_localization(self, {
                'frame_id': self.registry['frame_id'], **measured,
                'covariance_x_m2': .0025, 'covariance_y_m2': .0025,
                'covariance_yaw_rad2': math.radians(3.) ** 2,
            })
            localized, _ = self.capture_stationary_pose(intermediate=True)
            if (math.dist(localized[:2], inferred[:2]) > .05
                    or abs(_angle(localized[2] - inferred[2])) > math.radians(2.)):
                raise RuntimeError('rear dock landmark localization not retained')
            self.emit('rear_dock_landmark_localized', prior_pose_map=list(actual),
                      observed_pose_map=list(inferred), translation_correction_m=shift,
                      yaw_correction_rad=turn, source='same_run_rear_face_midpoint_and_normal')
            actual = localized
        if return_to_stage and (math.dist(actual[:2], (stage['x'], stage['y'])) > .05
                or abs(_angle(actual[2] - stage['yaw'])) > math.radians(3.)):
            stage_pose = {**pose, 'id': stage['id'], 'x_m': stage['x'], 'y_m': stage['y'],
                          'yaw_rad': stage['yaw'], 'parking_direction': 'forward'}
            planned = self._plan_with_input_recovery(
                stage_pose, single=True, end_tolerance_m=TRANSIT_PLAN_END_TOLERANCE_M)
            if not planned['ok']:
                return False
            actual = self._reach_staging(stage)
            if actual is None:
                return False
            observed = self._observe_rear_box(geometry, extent, half_width)
            self._rear_reference_pose(observed, actual_pose_map=actual)
        return True

    def _center_rear_box(self, observed, extent, *, after_clearance=False):
        """Use one guarded reverse curve while there is room to remove lateral error."""
        self._rear_center_moved = False
        if observed.get('observation_roi_clipped', True):
            raise RuntimeError('rear box midpoint is clipped')
        odom = self._odom_pose()
        normal = observed['normal_odom']
        midpoint = observed['midpoint_odom']
        tangent = -normal[1], normal[0]
        lateral = sum((odom[i] - midpoint[i]) * tangent[i] for i in (0, 1))
        self.emit('rear_dock_center_observed', lateral_error_m=lateral,
                  angle_rad=observed['normal_yaw_error_rad'], basis='live_rear_face_midpoint')
        if (not after_clearance and abs(lateral) <= .02
                and observed['gap_m'] <= REAR_COARSE_HANDOFF_M):
            return True
        target_odom = (
            *(midpoint[i] + (extent + REAR_FACE_REOBSERVE_GAP_M) * normal[i]
              for i in (0, 1)), math.atan2(normal[1], normal[0]))
        _path, to_odom = self._frozen_in_odom(RosPath())
        anchor = to_odom(0., 0., 0.)
        cosine, sine = math.cos(anchor[2]), math.sin(anchor[2])

        def to_map(pose):
            dx, dy = pose[0] - anchor[0], pose[1] - anchor[1]
            return (cosine * dx + sine * dy, -sine * dx + cosine * dy,
                    _angle(pose[2] - anchor[2]))

        actual, target = to_map(odom), to_map(target_odom)
        path = self._make_reverse_path(actual, target, curve=True)
        points = [(item.pose.position.x, item.pose.position.y,
                   2. * math.atan2(item.pose.orientation.z, item.pose.orientation.w))
                  for item in path.poses]
        check = CachedStaticCorridorValidator(
            Path(self.registry['map']['yaml_path']),
            Path(self.registry['keepout']['yaml_path']), self._footprint_outline())
        if not check(points):
            raise RuntimeError('rear dock centering static corridor blocked')
        frozen = self._odom_path([to_odom(*point) for point in points])
        self.waypoints = [{'id': 'rear_box_center', 'x': target[0], 'y': target[1],
                           'yaw': target[2]}]
        self.emit('rear_dock_centering_started', target_odom=list(target_odom),
                  lateral_error_m=lateral, target_gap_m=REAR_FACE_REOBSERVE_GAP_M)
        completed = self._execute_reverse_once(
            frozen, goal_checker_id='parking_goal_checker', final=False,
            motion='rear_dock_centering')
        self._rear_center_moved = bool(completed)
        return completed

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
        reference = getattr(self, '_rear_dock_reference', None)
        expected_span = (None if reference is None or getattr(self, '_rear_fine_face', None)
                         is not None else reference['observation']['span_m'])
        key = id(geometry), extent, half_width, expected_span
        failed = getattr(self, '_rear_box_detection_failure', None)
        if failed is not None and failed[:2] == (stamp, key):
            raise RuntimeError('rear box face unavailable')
        cached = getattr(self, '_rear_box_detection_cache', None)
        if (cached is not None and cached[0] == stamp
                and cached[1] == key):
            return cached[2]
        try:
            observation = _observe_rear_face_roi(
                self._rear_scan_values(scan), geometry, extent, half_width, expected_span)
        except ValueError as error:
            self._rear_box_detection_failure = stamp, key
            self.emit('rear_box_face_unavailable', reason=str(error))
            raise RuntimeError('rear box face unavailable') from error
        self._rear_box_detection_failure = None
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
                  'base_pose_odom': (transform.translation.x, transform.translation.y, yaw),
                  'face_odom': (transform.translation.x + cosine * x - sine * y,
                                transform.translation.y + sine * x + cosine * y),
                  'normal_odom': (cosine * nx - sine * ny, sine * nx + cosine * ny)}
        mx, my = observation['face_midpoint_base_xy_m']
        result['midpoint_odom'] = (
            transform.translation.x + cosine * mx - sine * my,
            transform.translation.y + sine * mx + cosine * my)
        self._rear_box_detection_cache = stamp, key, result
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
                          command_watch=None, *, handoff_m=REAR_COARSE_HANDOFF_M):
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
                if observed['gap_m'] <= handoff_m:
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

    def _approach_rear_corridor(self, geometry, extent, half_width, command_watch=None,
                               *, handoff_m=REAR_COARSE_HANDOFF_M):
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
            if observed['gap_m'] <= handoff_m:
                self.emit('rear_box_fine_handoff', gap_m=observed['gap_m'])
                return True
            travelled = math.dist(start[:2], self._odom_pose()[:2])
            budget = MAX_REVERSE_DISTANCE_M - travelled
            if budget <= 0.0:
                raise RuntimeError('rear box coarse approach exceeds bounded travel')
            distance = min(REAR_COARSE_SEGMENT_M, budget,
                           observed['gap_m'] - handoff_m
                           + STRAIGHT_ESCAPE_TOLERANCE_M)
            self.emit('rear_box_coarse_approach_started', gap_m=observed['gap_m'],
                      distance_m=distance, face_latched=False)
            if not self._rear_coarse_step(
                    geometry, extent, corridor_half_width, distance, command_watch,
                    **({'handoff_m': handoff_m} if handoff_m != REAR_COARSE_HANDOFF_M else {})):
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

    def _observe_rear_box(self, geometry, extent, half_width, *, timeout_s=5.0):
        tracker = RearBoxFaceTracker(required_frames=8)
        observations = []
        last_stamp = None
        failure = 'rear box unstable'
        deadline = min(self.run_deadline_s, time.monotonic() + timeout_s)
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

    def _observe_rear_box_recovering(self, geometry, extent, half_width, command_watch=None,
                                     *, allow_range_reapproach=False,
                                     range_handoff_m=REAR_FACE_REOBSERVE_GAP_M):
        """Keep a temporary perception gap inside the current return task."""
        range_budget_m = MAX_REVERSE_DISTANCE_M
        while not self.stop_requested and time.monotonic() < self.run_deadline_s:
            if not self._operator_link_ready():
                raise RuntimeError('operator_link_lost')
            try:
                return self._observe_rear_box(
                    geometry, extent, half_width,
                    **({'timeout_s': 1.0} if allow_range_reapproach else {}))
            except RuntimeError as error:
                if not str(error).startswith('rear box observation unavailable:'):
                    raise
                if self.stop_requested or time.monotonic() >= self.run_deadline_s:
                    break
                if (allow_range_reapproach and range_budget_m > 0.0
                        and str(error).endswith('rear box face unavailable')):
                    # The raw range handoff does not prove a transverse box face.
                    # Reobserve closer on the same axis, bounded by every live return.
                    outline = self._footprint_outline()
                    corridor_width = max(half_width, *(abs(p[1]) for p in outline))
                    observed = self._rear_corridor_observation(geometry, extent, corridor_width)
                    distance_m = min(REAR_REOBSERVE_SEGMENT_M, range_budget_m,
                                     observed['gap_m'] - range_handoff_m)
                    if distance_m > 0:
                        self.emit('rear_box_range_reobserve', distance_m=distance_m,
                                  gap_m=observed['gap_m'],
                                  handoff_m=range_handoff_m)
                        if not self._rear_coarse_step(
                                geometry, extent, corridor_width, distance_m, command_watch,
                                handoff_m=range_handoff_m):
                            reason = ('operator_stop' if self.stop_requested else
                                      'operator_link_lost' if not self._operator_link_ready() else
                                      getattr(self, '_last_direct_motion_failure', None)
                                      or 'rear box range reobservation blocked')
                            raise RuntimeError(reason)
                        range_budget_m -= distance_m
                        continue
                self.emit('rear_box_waiting_for_observation', reason=str(error),
                          remaining_s=max(0., self.run_deadline_s - time.monotonic()))
        raise RuntimeError('operator_stop' if self.stop_requested
                           else 'rear box observation deadline exceeded')

    def _rear_box_update(self, initial, geometry, extent, half_width, target_gap,
                         reference=None, command_watch=None,
                         source_point_filter=None):
        """Track the dock box in odom; live scan guards cover the final approach."""
        state = dict(initial)
        reference = initial if reference is None else reference
        reference = {'face_odom': tuple(reference['face_odom']),
                     'normal_odom': tuple(reference['normal_odom'])}
        start = self._odom_pose()

        def update(actual):
            observation = self._rear_box_observation(geometry, extent, half_width)
            if observation['stamp_s'] < state['stamp_s']:
                raise RuntimeError('rear box scan clock regressed')
            if observation['stamp_s'] != state['stamp_s']:
                _same_target(reference, observation)
                _set_collision_target(
                    command_watch, observation, target_gap,
                    observation['gap_m'], source_point_filter)
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
                    'allow_near_alignment': (gap > 0. and abs(error) < math.atan2(
                        half_width, extent))}

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
            self.emit('failed', phase='rear_box_readiness',
                      reason=self.readiness_failure_reason())
            return False
        if not execute:
            self.capture_stationary_pose()
            initial = self._observe_rear_box(geometry, extent, half_width)
            self.emit('rear_box_observed', observation=initial, target_gap_m=target_gap)
            return True
        if not self._reverse_smoother_ready() or not self._parking_parameters_ready(reverse=True):
            return False
        self._confirm_collision_monitor_on()
        if not self._approach_rear_corridor(
                geometry, extent, half_width, handoff_m=REAR_CENTER_HANDOFF_M):
            return False
        initial = self._observe_rear_box_recovering(
            geometry, extent, half_width, allow_range_reapproach=True)
        if getattr(self, '_rear_dock_reference', None) is not None:
            with self._localization_bound(True):
                if not self._align_rear_reference(
                        self._rear_reference_stage({'id': 'home_dock_approach'}),
                        self.registry['home'], observed=initial, return_to_stage=False):
                    return False
        tangent = -initial['normal_odom'][1], initial['normal_odom'][0]
        lateral = sum((initial['base_pose_odom'][i] - initial['midpoint_odom'][i]) * tangent[i]
                      for i in (0, 1))
        after_clearance = False
        if abs(lateral) > .02 and initial['gap_m'] < REAR_CENTER_MIN_GAP_M:
            # A near-range retry needs room for a curve, rather than a sideways
            # target at the same range which a differential drive cannot reach.
            self.config = load_route(route_path)
            self.config.pop('dock_departure_distance_m', None)
            self.waypoints = self.config['waypoints']
            self._dock_departure = None
            clearance = min(REAR_CENTER_HANDOFF_M, REAR_CENTER_HANDOFF_M - initial['gap_m'])
            self.emit('rear_dock_center_clearance', distance_m=clearance,
                      lateral_error_m=lateral)
            if not self._depart_dock_straight(clearance):
                return False
            after_clearance = True
            # The accepted plane is fixed in odom. The guarded straight exit
            # provides curve room; it need not rediscover the face from farther
            # away. Reconfirm it with fresh scans after the centering curve.
            self.emit('rear_dock_center_reference_carried',
                      observation_stamp_s=initial['stamp_s'],
                      basis='accepted_face_fixed_in_odom_during_guarded_clearance')
        if not self._center_rear_box(
                initial, extent, **({'after_clearance': True} if after_clearance else {})):
            return False
        if getattr(self, '_rear_center_moved', False):
            centered = self._observe_rear_box_recovering(geometry, extent, half_width)
            _same_target(initial, centered)
            tangent = -centered['normal_odom'][1], centered['normal_odom'][0]
            actual = self._odom_pose()
            lateral = sum((actual[i] - centered['midpoint_odom'][i]) * tangent[i]
                          for i in (0, 1))
            self.emit('rear_dock_centering_finished', lateral_error_m=lateral,
                      observation=centered)
            if abs(lateral) > .02:
                self.emit('failed', phase='rear_box_centering',
                          reason='rear_box_center_not_reached', lateral_error_m=lateral)
                return False
            initial = centered
        with rear_collision_profile(self) as command_watch:
            self.emit('rear_box_observed', observation=initial, target_gap_m=target_gap)
            # Landmark localization needs visible endpoints. After latching the
            # plane, near-range clipping must not replace plane feedback with
            # a requirement to see the complete box width again.
            self._rear_fine_face = initial
            try:
                return self._park_rear_face(
                    initial, geometry, route_path, extent, half_width, target_gap, command_watch)
            finally:
                _clear_collision_target(command_watch)
                self._rear_fine_face = None

    def _park_rear_face(self, initial, geometry, route_path, extent, half_width,
                        target_gap, command_watch):
        """Keep the fine target fixed after the moving range-only handoff."""

        source_cache = {}

        def source_goal_face_point(pose, scan, point):
            """Classify memory in its acquisition frame, before odom drift."""
            key = id(scan)
            acquisition_pose = tuple(pose)
            cached = source_cache.get(key)
            if (cached is None or cached[0] is not scan
                    or cached[1] != acquisition_pose):
                try:
                    observed = _observe_rear_face_roi(
                        self._rear_scan_values(scan), geometry, extent,
                        half_width, expected_span_m=initial['span_m'])
                    cosine, sine = math.cos(pose[2]), math.sin(pose[2])
                    x_m, y_m = observed['face_center_base_xy_m']
                    nx, ny = observed['normal_base_xy']
                    source = {
                        **observed,
                        'face_odom': (
                            pose[0] + cosine * x_m - sine * y_m,
                            pose[1] + sine * x_m + cosine * y_m),
                        'normal_odom': (
                            cosine * nx - sine * ny,
                            sine * nx + cosine * ny),
                    }
                    _same_target(initial, source)
                    source_cache[key] = (scan, acquisition_pose, observed)
                except (KeyError, RuntimeError, TypeError, ValueError):
                    source_cache[key] = (scan, acquisition_pose, None)
            observed = source_cache[key][2]
            if observed is None:
                return False
            normal = observed['normal_base_xy']
            tangent = observed['tangent_base_xy']
            residual_m = (point[0] * normal[0] + point[1] * normal[1]
                          - observed['line_offset_m'])
            tangent_m = point[0] * tangent[0] + point[1] * tangent[1]
            lower_m, upper_m = observed['tangent_bounds_m']
            residual_limit_m = min(
                4.0 * observed['residual_rms_m'], MAXIMUM_RESIDUAL_RMS_M)
            near_limit_m = min(residual_limit_m, 0.003)
            return (-residual_limit_m <= residual_m <= near_limit_m
                    and lower_m <= tangent_m <= upper_m)

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
            reobserved = self._observe_rear_box_recovering(
                geometry, extent, half_width, command_watch)
            _same_target(initial, reobserved)
            return reobserved

        def approach(observation):
            _set_collision_target(
                command_watch, observation, target_gap, observation['gap_m'],
                source_goal_face_point)
            update = self._rear_box_update(
                observation, geometry, extent, half_width, target_gap,
                reference=initial, command_watch=command_watch,
                source_point_filter=source_goal_face_point)
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

        final = self._observe_rear_box_recovering(geometry, extent, half_width, command_watch)
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
            retry_completed = retry_start is not None and approach(retry_start)
            if retry_start is not None:
                try:
                    retried = self._observe_rear_box_recovering(
                        geometry, extent, half_width, command_watch)
                    _same_target(initial, retried)
                    final = retried
                    confirmed, gap, far_gap, error = confirmation(final)
                except RuntimeError:
                    pass
            if not retry_completed:
                confirmed = False
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
