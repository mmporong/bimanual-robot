"""Bounded AST1 reads preserve fragmented payloads and record boundaries."""

import socket

import pytest

from jdamr_cube_navigation.astra_tcp_receiver import exact


class Fragments:
    def __init__(self, payload, *, timeout_once=False):
        self.payload = payload
        self.offset = 0
        self.timeout_once = timeout_once

    def recv_into(self, view):
        if self.timeout_once:
            self.timeout_once = False
            raise socket.timeout()
        count = min(len(view), 17, len(self.payload) - self.offset)
        view[:count] = self.payload[self.offset:self.offset + count]
        self.offset += count
        return count


def test_exact_read_handles_fragments_timeout_and_next_record():
    payload = bytes(range(256)) * 300
    stream = Fragments(payload + b'PONG', timeout_once=True)
    assert exact(stream, len(payload)) == payload
    assert exact(stream, 4) == b'PONG'


def test_exact_read_rejects_mid_record_disconnect():
    with pytest.raises(EOFError, match='disconnected'):
        exact(Fragments(b'AST'), 4)


def test_exact_read_keeps_total_record_deadline(monkeypatch):
    from jdamr_cube_navigation import astra_tcp_receiver
    times = iter((0., 0., 3.1))
    monkeypatch.setattr(astra_tcp_receiver.time, 'monotonic', lambda: next(times))
    with pytest.raises(TimeoutError, match='incomplete AST1 record'):
        exact(Fragments(b'AST1'), 8)
