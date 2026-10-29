"""AST1 Windows camera receiver; publishes sensors only, never motion.

Wire order: AST1 + u32be JSON length + JSON + rgb PNG + depth PNG16.
PONG records (4 + 8 + 8 bytes) may occur only between complete AST1 records.
"""

import argparse
from collections import deque
import json
import math
import select
import socket
import struct
import threading
import time

import cv2
import numpy as np


FRAME = 'camera_color_optical_frame'
MAX_AGE_NS = 500_000_000
MAX_RTT_NS = 100_000_000


def exact(connection, size):
    chunks = bytearray()
    deadline = time.monotonic() + 3.0
    while len(chunks) < size:
        if time.monotonic() >= deadline:
            raise TimeoutError('incomplete AST1 record')
        block = connection.recv(min(size - len(chunks), 65536))
        if not block:
            raise EOFError('sender disconnected')
        chunks.extend(block)
    return bytes(chunks)


def integer(value, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError('invalid integer field')
    return value


def metadata(value, depth=False):
    if not isinstance(value, dict):
        raise ValueError('image metadata must be an object')
    width, height = (320, 240) if depth else (640, 480)
    if (value.get('width'), value.get('height')) != (width, height):
        raise ValueError('unexpected image resolution')
    if value.get('frame_id') != FRAME:
        raise ValueError('unexpected optical frame')
    if value.get('encoding') != ('16UC1' if depth else 'bgr8'):
        raise ValueError('unexpected image encoding')
    if value.get('compression') != ('png16' if depth else 'png'):
        raise ValueError('unexpected compression')
    if depth and value.get('unit') != 'mm':
        raise ValueError('depth must be millimetres')
    integer(value.get('stamp_ns'), 1, 2**63 - 1)
    length = integer(value.get('len'), 1, 2 * 1024 * 1024)
    info = value['camera_info']
    if not isinstance(info, dict):
        raise ValueError('CameraInfo must be an object')
    if (info.get('width'), info.get('height')) != (width, height):
        raise ValueError('CameraInfo resolution mismatch')
    if info.get('distortion_model') != 'plumb_bob':
        raise ValueError('unexpected distortion model')
    for key, size in (('D', 5), ('K', 9), ('R', 9), ('P', 12)):
        values = info.get(key)
        if (not isinstance(values, list) or len(values) != size
                or not all(type(v) in (int, float) and math.isfinite(v) for v in values)):
            raise ValueError('invalid CameraInfo ' + key)
    if min(info['K'][0], info['K'][4], info['P'][0], info['P'][5]) <= 0:
        raise ValueError('invalid focal length')
    return length


def decode(payload, meta, depth=False):
    # Check dimensions before allowing the PNG decoder to allocate memory.
    if (len(payload) < 33 or payload[:8] != b'\x89PNG\r\n\x1a\n'
            or payload[12:16] != b'IHDR'
            or struct.unpack('!II', payload[16:24]) != (meta['width'], meta['height'])
            or payload[24] != (16 if depth else 8)
            or payload[25] != (0 if depth else 2)):
        raise ValueError('PNG header does not match metadata')
    image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_UNCHANGED)
    expected = (240, 320) if depth else (480, 640, 3)
    if image is None or image.shape != expected or image.dtype != (np.uint16 if depth else np.uint8):
        raise ValueError('decoded image does not match metadata')
    return image


class Receiver:
    def __init__(self, host, port, peer):
        self.host, self.port, self.peer = host, port, peer
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.status = {'state': 'listening'}
        self.last_received = 0.0
        self.listener = socket.socket()
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind((host, port))
        self.listener.listen(1)
        self.listener.settimeout(.5)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def take(self):
        with self.lock:
            value, self.latest = self.latest, None
            return value

    def current_status(self):
        status = dict(self.status)
        if status.get('state') == 'receiving':
            silence = time.monotonic() - self.last_received
            status['silence_s'] = silence
            if silence > MAX_AGE_NS / 1e9:
                status['state'] = 'stale'
        return status

    def run(self):
        while not self.stop.is_set():
            try:
                connection, address = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with connection:
                if self.peer and address[0] != self.peer:
                    continue
                connection.settimeout(.5)
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                try:
                    self.session(connection, address[0])
                except (OSError, EOFError, ValueError, KeyError, TypeError, struct.error, cv2.error) as error:
                    self.status = {'state': 'disconnected', 'reason': str(error)}
                finally:
                    with self.lock:
                        self.latest = None

    def session(self, connection, peer):
        self.status = {'state': 'connected', 'peer': peer}
        clock = deque(maxlen=8)
        pending = None
        last_ping = 0.0
        last_seq = -1
        while not self.stop.is_set():
            now = time.monotonic()
            if now - last_ping >= .5:
                # One outstanding ping; late replies cannot update the clock.
                if pending is None or now - pending[1] > 2.0:
                    pi_ns = time.time_ns()
                    pending = (pi_ns, now)
                    connection.sendall(b'PING' + struct.pack('!Q', pi_ns))
                    last_ping = now
            if not select.select([connection], [], [], .1)[0]:
                continue
            magic = exact(connection, 4)
            if magic == b'PONG':
                echo, win_ns = struct.unpack('!QQ', exact(connection, 16))
                received_ns, received_mono = time.time_ns(), time.monotonic()
                if pending is not None and echo == pending[0]:
                    rtt = received_ns - echo
                    elapsed = int((received_mono - pending[1]) * 1e9)
                    pending = None
                    if abs(rtt - elapsed) > 20_000_000:
                        clock.clear()
                    elif 0 <= rtt <= MAX_RTT_NS:
                        offset = (echo + received_ns) // 2 - win_ns
                        if clock and abs(offset - min(clock, key=lambda x: x[0])[1]) > MAX_RTT_NS:
                            clock.clear()
                        clock.append((rtt, offset, received_mono))
                continue
            if magic != b'AST1':
                raise ValueError('unknown message magic or interleaved frame/PONG')
            size = integer(struct.unpack('!I', exact(connection, 4))[0], 1, 65536)
            header = json.loads(exact(connection, size).decode('utf-8'))
            if not isinstance(header, dict):
                raise ValueError('header must be an object')
            seq = integer(header['seq'], 0, 2**63 - 1)
            integer(header['send_ns'], 1, 2**63 - 1)
            rgb, depth = header['rgb'], header['depth']
            rgb_size, depth_size = metadata(rgb), metadata(depth, True)
            rgb_data, depth_data = exact(connection, rgb_size), exact(connection, depth_size)
            if seq <= last_seq:
                raise ValueError('frame sequence did not increase')
            last_seq = seq
            fresh = [sample for sample in clock if time.monotonic() - sample[2] < 10.0]
            if len(fresh) < 3:
                self.status = {'state': 'clock_sync', 'samples': len(fresh)}
                continue
            rtt, offset, _ = min(fresh, key=lambda x: x[0])
            stamps = [rgb['stamp_ns'] + offset, depth['stamp_ns'] + offset]
            age = [time.time_ns() - stamp for stamp in stamps]
            if not all(0 <= a <= MAX_AGE_NS for a in age):
                self.status = {'state': 'dropped', 'reason': 'stale_or_future_frame', 'age_ns': age}
                continue
            images = decode(rgb_data, rgb), decode(depth_data, depth, True)
            with self.lock:
                self.latest = (header, images, stamps)
            self.last_received = time.monotonic()
            self.status = {'state': 'receiving', 'peer': peer, 'seq': seq,
                           'clock_offset_ns': offset, 'clock_rtt_ns': rtt,
                           'clock_uncertainty_ns': rtt // 2, 'age_ns': age,
                           'rgb_depth_delta_ns': abs(stamps[0] - stamps[1])}

    def close(self):
        self.stop.set()
        self.listener.close()
        self.thread.join(timeout=4.0)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=5600)
    parser.add_argument('--peer', default='192.168.0.175')
    args = parser.parse_args()
    rclpy.init()
    node = Node('astra_tcp_receiver')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    pubs = [(node.create_publisher(CameraInfo, f'/camera/{kind}/camera_info', qos),
             node.create_publisher(Image, f'/camera/{kind}/image_raw', qos))
            for kind in ('color', 'depth')]
    status_pub = node.create_publisher(String, '/camera/tcp_status', 1)
    receiver = Receiver(args.host, args.port, args.peer)

    def publish():
        value = receiver.take()
        if value is None:
            return
        header, images, stamps = value
        if not all(0 <= time.time_ns() - stamp <= MAX_AGE_NS for stamp in stamps):
            return
        for index, kind in enumerate(('rgb', 'depth')):
            meta, image, stamp = header[kind], images[index], stamps[index]
            info = CameraInfo()
            info.header.frame_id = FRAME
            info.header.stamp.sec, info.header.stamp.nanosec = divmod(stamp, 10**9)
            info.width, info.height = meta['width'], meta['height']
            ci = meta['camera_info']
            info.distortion_model = ci['distortion_model']
            for key in ('D', 'K', 'R', 'P'):
                setattr(info, key.lower(), [float(v) for v in ci[key]])
            message = Image()
            message.header = info.header
            message.width, message.height = info.width, info.height
            message.encoding = meta['encoding']
            message.is_bigendian = 0
            message.step = info.width * (3 if index == 0 else 2)
            message.data = image.astype('<u2' if index else 'u1', copy=False).tobytes()
            pubs[index][0].publish(info)
            pubs[index][1].publish(message)

    node.create_timer(.01, publish)
    node.create_timer(1.0, lambda: status_pub.publish(String(data=json.dumps(receiver.current_status()))))
    node.get_logger().info(f'AST1 listening on {args.host}:{args.port}; peer={args.peer}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
