"""
Head control over the same gRPC channel the phone app uses (no Booster SDK / DDS needed).

From head_side_to_side.pcapng: /booster.proto.rpc.RobotGrpc/RobotRequest, command id 1005,
body = optional field 1 and field 2, each an int64 in {-1, 0, 1}; an empty body ends the move.

Drop-in for robot.Head: same set_dir(x, y) (d-pad hat, right/up positive), start(), shutdown().

Which body field is yaw, and the sign of each, is NOT provable from the capture. Tune with env vars
(no code edits) after running k1_head.py probe:
    NUMOTE_HEAD_YAW_FIELD    1 or 2   (default 1; the other field is pitch)
    NUMOTE_HEAD_YAW_SIGN     1 or -1  (default -1)
    NUMOTE_HEAD_PITCH_SIGN   1 or -1  (default -1)
    NUMOTE_HEAD_REPEAT_S     resend interval while a direction is held (default 0.2)
"""
import os
import threading
import time
import uuid

METHOD = "/booster.proto.rpc.RobotGrpc/RobotRequest"
API_HEAD = 1005


def _varint(n):
    n &= (1 << 64) - 1  # two's complement: -1 is a 10-byte varint (robot.enc_varint would loop forever)
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        if n:
            out.append(b | 0x80)
        else:
            out.append(b)
            return bytes(out)


def _f_varint(num, v):
    return _varint(num << 3) + _varint(v)


def _f_bytes(num, b):
    return _varint(num << 3 | 2) + _varint(len(b)) + b


def head_message(yaw, pitch, yaw_field=2):
    """yaw/pitch each -1/0/1. Zero fields are omitted; all-zero means no body at all (stop)."""
    pitch_field = 3 - yaw_field
    body = b""
    for field, val in sorted(((yaw_field, yaw), (pitch_field, pitch))):
        if val:
            body += _f_varint(field, val)
    msg = _f_varint(1, API_HEAD) + _f_bytes(2, str(uuid.uuid4()).encode())
    if body:
        msg += _f_bytes(3, body)
    return msg + _f_varint(5, 1)


class GrpcHead(threading.Thread):
    def __init__(self, target, timeout=1.0):
        super().__init__(daemon=True)
        import grpc
        self.timeout = timeout
        self.repeat = float(os.environ.get("NUMOTE_HEAD_REPEAT_S", "0.2"))
        self.yaw_field = int(os.environ.get("NUMOTE_HEAD_YAW_FIELD", "1"))
        self.yaw_sign = int(os.environ.get("NUMOTE_HEAD_YAW_SIGN", "-1"))
        self.pitch_sign = int(os.environ.get("NUMOTE_HEAD_PITCH_SIGN", "-1"))
        self.channel = grpc.insecure_channel(target)
        self.call = self.channel.unary_unary(
            METHOD, request_serializer=lambda b: b, response_deserializer=lambda b: b)
        self.lock = threading.Lock()
        self.dir = (0, 0)
        self.running = True

    def set_dir(self, x, y):
        """d-pad direction, each -1/0/1; right and up are positive."""
        with self.lock:
            self.dir = (x, y)

    def _send(self, d):
        x, y = d
        msg = head_message(x * self.yaw_sign, y * self.pitch_sign, self.yaw_field)
        try:
            self.call(msg, timeout=self.timeout)
        except Exception as e:
            print(f"head error: {e}")
            time.sleep(1)

    def run(self):
        sent = (0, 0)
        next_send = 0.0
        while self.running:
            with self.lock:
                d = self.dir
            now = time.monotonic()
            if d != sent:
                self._send(d)  # includes (0, 0) = stop
                sent, next_send = d, time.monotonic() + self.repeat
            elif d != (0, 0) and now >= next_send:
                self._send(d)  # keep-alive while the d-pad is held
                next_send = time.monotonic() + self.repeat
            time.sleep(0.02)

    def shutdown(self):
        self.running = False
        try:
            self.call(head_message(0, 0, self.yaw_field), timeout=self.timeout)
        except Exception:
            pass