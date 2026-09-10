#!/usr/bin/env python3
"""Feetech STS/SMS 버스 최소 헬퍼.

이 파일은 통신만 한다. 무엇을 쓸지 판단하는 것은 각 도구 스크립트의 몫이다.
프로토콜은 반이중 TTL, 기본 1 Mbps 다. 명령은 PING·READ·WRITE 세 개만 쓴다.

레지스터 주소는 STS3215 기준이다. 다른 계열을 붙이려면 이 표부터 확인할 것.
"""
from __future__ import annotations

import glob
import time

BAUD = 1_000_000
RESOLUTION = 4096              # 한 바퀴 카운트

# EEPROM — 쓰려면 A_LOCK 을 0 으로 풀어야 한다
A_MIN_ANGLE, A_MAX_ANGLE = 9, 11
A_MAX_TEMP = 13
A_P, A_D, A_I = 21, 22, 23
A_OFFSET = 31                  # 위치 보정. 12bit 부호크기, bit11 이 음수 표시

# SRAM
A_TORQUE, A_ACCEL, A_GOAL, A_SPEED = 40, 41, 42, 46
A_LOCK = 55                    # 0 = EEPROM 쓰기 허용, 1 = 잠금
A_POS, A_PRESENT_SPEED, A_LOAD = 56, 58, 60
A_VOLT, A_TEMP = 62, 63


def find_port(explicit=None):
    """--port 를 안 주면 by-id 에서 하나를 고른다. ttyACM 번호는 순서가 바뀌므로 쓰지 않는다."""
    if explicit:
        return explicit
    found = sorted(glob.glob("/dev/serial/by-id/*"))
    if len(found) == 1:
        return found[0]
    if not found:
        raise SystemExit("시리얼 포트가 없습니다. 보드 USB 와 12V 를 확인하세요.")
    raise SystemExit("포트가 여러 개입니다. --port 로 지정하세요:\n  " + "\n  ".join(found))


def encode_offset(value):
    value = max(-2047, min(2047, int(value)))
    return (0x800 | -value) if value < 0 else value


def decode_offset(raw):
    return -(raw & 0x7FF) if raw & 0x800 else (raw & 0x7FF)


def signed(value):
    """속도·부하처럼 bit15 를 부호로 쓰는 값."""
    if value is None:
        return None
    return -(value & 0x7FFF) if value & 0x8000 else value


def _frame(sid, instr, params=b""):
    body = bytes([sid, len(params) + 2, instr]) + params
    return b"\xff\xff" + body + bytes([(~sum(body)) & 0xFF])


class Bus:
    """실물 버스."""

    def __init__(self, port, baud=BAUD, timeout=0.05):
        import serial
        self.ser = serial.Serial(port, baud, timeout=timeout)

    def _txn(self, sid, instr, params=b""):
        try:
            self.ser.reset_input_buffer()
            self.ser.write(_frame(sid, instr, params))
            self.ser.flush()
            head = self.ser.read(4)
        except Exception as exc:                       # USB 가 빠지거나 다른 프로그램이 포트를 잡은 경우
            raise SystemExit(f"포트 통신 실패: {exc}\n보드 USB·전원, 그리고 텔레옵 등 다른 프로그램이 같은 포트를 열고 있지 않은지 확인하세요.")
        if len(head) < 4 or head[:2] != b"\xff\xff":
            return None
        rest = self.ser.read(head[3])
        if len(rest) < head[3]:
            return None
        # 체크섬 검증 — 깨진 패킷이 그럴듯한 값으로 들어오는 것을 막는다 (2026-09-09 범위 기록 오염)
        if ((~(head[2] + head[3] + sum(rest[:-1]))) & 0xFF) != rest[-1]:
            return None
        return rest[1:-1]

    def ping(self, sid):
        return self._txn(sid, 0x01) is not None

    def read(self, sid, addr, n=1):
        data = self._txn(sid, 0x02, bytes([addr, n]))
        if data is None or len(data) < n:
            return None
        return data[0] if n == 1 else data[0] | (data[1] << 8)

    def write(self, sid, addr, value, n=1):
        if n == 1:
            params = bytes([addr, value & 0xFF])
        else:
            params = bytes([addr, value & 0xFF, (value >> 8) & 0xFF])
        return self._txn(sid, 0x03, params) is not None

    def write_eeprom(self, sid, addr, value, n=2):
        """EEPROM 은 잠금을 풀고 쓴 뒤 다시 잠근다. 중간 대기는 실물에서 필요했다."""
        self.write(sid, A_LOCK, 0)
        time.sleep(0.05)
        ok = self.write(sid, addr, value, n)
        time.sleep(0.08)
        self.write(sid, A_LOCK, 1)
        time.sleep(0.05)
        return ok

    def scan(self, lo=1, hi=20):
        return [sid for sid in range(lo, hi + 1) if self.ping(sid)]

    def close(self):
        self.ser.close()


class FakeBus:
    """리허설용 모의 버스.

    실물에 붙이기 전에 main() 을 끝까지 돌려 본다. py_compile 은 리허설이 아니다.
    `wall` 은 원시 엔코더 기준으로 더 이상 못 가는 하드 스톱 위치다.
    """

    def __init__(self, sid=6, pos=3600, lo=760, hi=3600, offset=0, sign=1, wall=None):
        self.sid = sid
        self.sign = sign
        self.wall = wall
        self.raw = pos - sign * offset
        self.reg = {
            A_MIN_ANGLE: lo, A_MAX_ANGLE: hi, A_MAX_TEMP: 65,
            A_P: 16, A_D: 32, A_I: 0, A_OFFSET: encode_offset(offset),
            A_TORQUE: 0, A_ACCEL: 0, A_GOAL: pos, A_SPEED: 0, A_LOCK: 1,
            A_PRESENT_SPEED: 0, A_LOAD: 20, A_VOLT: 123, A_TEMP: 34,
        }

    def _reported(self):
        return (self.raw + self.sign * decode_offset(self.reg[A_OFFSET])) % RESOLUTION

    def ping(self, sid):
        return sid == self.sid

    def read(self, sid, addr, n=1):
        if sid != self.sid:
            return None
        if addr == A_POS:
            if self.reg[A_TORQUE]:
                target = self.reg[A_GOAL] - self.sign * decode_offset(self.reg[A_OFFSET])
                if self.wall is not None:
                    target = max(target, self.wall)
                step = 40
                if abs(target - self.raw) <= step:
                    self.raw = target
                else:
                    self.raw += step if target > self.raw else -step
                blocked = self.wall is not None and self.raw == self.wall
                self.reg[A_LOAD] = 700 if blocked else 20
            return self._reported()
        return self.reg.get(addr, 0)

    def write(self, sid, addr, value, n=1):
        if sid != self.sid:
            return False
        if addr in (A_MIN_ANGLE, A_MAX_ANGLE, A_OFFSET) and self.reg[A_LOCK]:
            return True                     # 잠긴 채 쓰면 무시되는 실물 동작을 흉내낸다
        self.reg[addr] = value
        return True

    def write_eeprom(self, sid, addr, value, n=2):
        self.write(sid, A_LOCK, 0)
        ok = self.write(sid, addr, value, n)
        self.write(sid, A_LOCK, 1)
        return ok

    def scan(self, lo=1, hi=20):
        return [self.sid] if lo <= self.sid <= hi else []

    def close(self):
        pass
