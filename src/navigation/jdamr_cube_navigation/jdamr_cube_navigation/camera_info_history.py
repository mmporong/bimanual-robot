"""Bounded timestamp association for depth CameraInfo messages."""

from collections import deque


_NANOSECONDS_PER_SECOND = 1_000_000_000


def stamp_nanoseconds(stamp):
    """Convert one ROS-style time value to validated integer nanoseconds."""
    try:
        seconds = stamp.sec
        nanoseconds = stamp.nanosec
    except AttributeError as error:
        raise ValueError('stamp fields are missing') from error
    if (isinstance(seconds, bool) or not isinstance(seconds, int)
            or isinstance(nanoseconds, bool)
            or not isinstance(nanoseconds, int)):
        raise ValueError('camera info stamp fields must be integers')
    if seconds < 0 or not 0 <= nanoseconds < _NANOSECONDS_PER_SECOND:
        raise ValueError('camera info stamp is invalid')
    return seconds * _NANOSECONDS_PER_SECOND + nanoseconds


def _camera_info_stamp_ns(message):
    try:
        stamp = message.header.stamp
    except AttributeError as error:
        raise ValueError('camera info header stamp is missing') from error
    return stamp_nanoseconds(stamp)


class CameraInfoHistory:
    """Associate depth frames with current-epoch camera calibration."""

    def __init__(self, capacity=8):
        if (isinstance(capacity, bool) or not isinstance(capacity, int)
                or capacity <= 0):
            raise ValueError('camera info history capacity must be positive')
        self._capacity = capacity
        self._stamped = deque()
        self._static = None
        self._saw_stamped = False
        self._last_received_stamped = None
        self._last_selected_image_stamp_ns = None
        self._arrival_serial = 0
        self._last_select_arrival_serial = 0
        self.rewound = False

    def add(self, info):
        """Add a CameraInfo message, replacing an equal stamped arrival."""
        try:
            stamp_ns = _camera_info_stamp_ns(info)
        except ValueError:
            self._stamped.clear()
            self._static = None
            self._last_received_stamped = None
            raise
        if stamp_ns == 0:
            self._static = info
            return

        self._saw_stamped = True
        self._arrival_serial += 1
        self._last_received_stamped = (
            stamp_ns, info, self._arrival_serial)
        self._stamped = deque(
            (existing_stamp, existing_info)
            for existing_stamp, existing_info in self._stamped
            if existing_stamp != stamp_ns)
        self._stamped.append((stamp_ns, info))
        while len(self._stamped) > self._capacity:
            self._stamped.popleft()

    def select(self, image_stamp_ns, *, maximum_lag_ns):
        """Select the latest non-future CameraInfo for one depth frame."""
        if (isinstance(image_stamp_ns, bool)
                or not isinstance(image_stamp_ns, int)
                or image_stamp_ns < 0):
            raise ValueError('depth image stamp must be a nonnegative integer')
        if (isinstance(maximum_lag_ns, bool)
                or not isinstance(maximum_lag_ns, int)
                or maximum_lag_ns < 0):
            raise ValueError(
                'maximum camera info lag must be a nonnegative integer')

        previous_stamp_ns = self._last_selected_image_stamp_ns
        self.rewound = (
            previous_stamp_ns is not None
            and image_stamp_ns < previous_stamp_ns)
        if self.rewound:
            exact = self._last_received_stamped
            self._stamped.clear()
            if (exact is not None
                    and exact[0] == image_stamp_ns
                    and exact[2] > self._last_select_arrival_serial):
                self._stamped.append(exact[:2])
        self._last_selected_image_stamp_ns = image_stamp_ns
        self._last_select_arrival_serial = self._arrival_serial

        eligible = [
            (stamp_ns, info) for stamp_ns, info in self._stamped
            if (stamp_ns <= image_stamp_ns
                and image_stamp_ns - stamp_ns <= maximum_lag_ns)
        ]
        if eligible:
            return max(eligible, key=lambda item: item[0])[1]
        if not self._saw_stamped:
            return self._static
        return None
