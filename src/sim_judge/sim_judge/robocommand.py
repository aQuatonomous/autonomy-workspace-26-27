"""Stand-in for the RoboCommand TCP server (RoboBoat 2026 communication protocol).

Frames are b'$R' + 1-byte payload length + serialized robocommand.roboboat.v1.Report + b'!!'.
Teams may send several frames per connection or reconnect for every frame (message_node does
the latter); both are accepted.
"""

import socket
import threading
import time

from google.protobuf import json_format

from . import report_compat as pb

HEADER = b'$R'
FOOTER = b'!!'


class FrameParser:
    """Incremental $R...!! frame parser. feed() returns decoded (payload bytes) frames."""

    def __init__(self):
        self.buf = b''
        self.errors = []

    def feed(self, data):
        self.buf += data
        frames = []
        while True:
            start = self.buf.find(HEADER)
            if start < 0:
                keep = 1 if self.buf.endswith(HEADER[:1]) else 0
                if len(self.buf) > keep:
                    self.errors.append(f'discarded {len(self.buf) - keep} bytes outside a $R frame')
                self.buf = self.buf[len(self.buf) - keep:]
                break
            if start > 0:
                self.errors.append(f'discarded {start} bytes before $R header')
                self.buf = self.buf[start:]
            if len(self.buf) < 3:
                break
            length = self.buf[2]
            end = 3 + length + len(FOOTER)
            if len(self.buf) < end:
                break
            payload, footer = self.buf[3:3 + length], self.buf[3 + length:end]
            if footer != FOOTER:
                self.errors.append(f'bad footer {footer!r} after {length}-byte payload; resyncing')
                self.buf = self.buf[1:]
                continue
            frames.append(payload)
            self.buf = self.buf[end:]
        return frames


def decode(payload):
    report = pb.Report()
    report.ParseFromString(payload)
    return report


def to_dict(report):
    return json_format.MessageToDict(report, preserving_proto_field_name=True)


class RoboCommandServer:
    """Threaded TCP server; calls on_report(wall_time, report, peer) for every valid frame and
    on_error(wall_time, message, peer) for framing/decoding problems."""

    def __init__(self, host, port, on_report, on_error):
        self.host, self.port = host, port
        self.on_report, self.on_error = on_report, on_error
        self._sock = None
        self._stop = threading.Event()

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.listen(16)
        self._sock.settimeout(0.5)
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def stop(self):
        self._stop.set()
        if self._sock:
            self._sock.close()

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                conn, addr = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._serve, args=(conn, f'{addr[0]}:{addr[1]}'), daemon=True).start()

    def _serve(self, conn, peer):
        parser = FrameParser()
        conn.settimeout(1.0)
        with conn:
            while not self._stop.is_set():
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not data:
                    break
                now = time.time()
                for payload in parser.feed(data):
                    try:
                        report = decode(payload)
                    except Exception as exc:  # noqa: BLE001  (any protobuf decode failure)
                        self.on_error(now, f'undecodable {len(payload)}-byte payload: {exc}', peer)
                        continue
                    self.on_report(now, report, peer)
                for err in parser.errors:
                    self.on_error(now, err, peer)
                parser.errors.clear()
        if parser.buf:
            self.on_error(time.time(), f'connection closed with {len(parser.buf)} bytes of incomplete frame', peer)


def encode_frame(report):
    payload = report.SerializeToString()
    if len(payload) > 255:
        raise ValueError(f'payload is {len(payload)} bytes; the 1-byte length field allows 255')
    return HEADER + bytes([len(payload)]) + payload + FOOTER
