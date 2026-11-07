"""
Body (joystick) and head control over the gRPC channel the phone app uses (no Booster SDK / DDS needed).

Both go to /booster.proto.rpc.RobotGrpc/RobotRequest as hand-encoded protobuf messages:
    joystick, command id 1003: Sender; body = floats x, y (left stick) and z (right stick yaw).
    head, command id 1005: GrpcHead; from head_side_to_side.pcapng, body = optional field 1 and field 2,
    each an int64 in {-1, 0, 1}; an empty body ends the move.

Which head body field is yaw, and the sign of each, is NOT provable from the capture. Tune with env vars
(no code edits):
    NUMOTE_HEAD_YAW_FIELD    1 or 2   (default 1; the other field is pitch)
    NUMOTE_HEAD_YAW_SIGN     1 or -1  (default -1)
    NUMOTE_HEAD_PITCH_SIGN   1 or -1  (default -1)
    NUMOTE_HEAD_REPEAT_S     resend interval while a direction is held (default 0.2)

SAFETY - this can move a physical robot. Stop messages are sent on release and shutdown,
but the emergency stop is the real safety.
"""
import os
import struct
import threading
import time
import uuid

METHOD = "/booster.proto.rpc.RobotGrpc/RobotRequest"
CMD_ID = 1003
API_HEAD = 1005


# ---------- protobuf encoding ----------
def enc_varint(n):
    n &= (1 << 64) - 1  # two's complement: -1 is a 10-byte varint, not an endless loop
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        if n:
            out.append(b | 0x80)
        else:
            out.append(b)
            return bytes(out)


def f_varint(num, v):
    return enc_varint(num << 3) + enc_varint(v)


def f_bytes(num, b):
    return enc_varint(num << 3 | 2) + enc_varint(len(b)) + b


def f_float(num, f):
    return enc_varint(num << 3 | 5) + struct.pack("<f", f)


def joystick_message(x, y, z=-0.0):
    inner = f_float(1, x) + f_float(2, y) + f_float(3, z)
    uid = str(uuid.uuid4()).encode()
    return f_varint(1, CMD_ID) + f_bytes(2, uid) + f_bytes(3, inner) + f_varint(5, 1)


class Sender(threading.Thread):
    """Sends the current stick value at a fixed rate while active; sends stop when asked."""

    def __init__(self, target, rate, timeout):
        super().__init__(daemon=True)
        import grpc
        self.dt = 1.0 / rate
        self.timeout = timeout
        self.lock = threading.Lock()
        self.x = self.y = self.z = 0.0
        self.move_active = self.yaw_active = False
        self.stop_pending = False
        self.max_in_flight = 3
        self.in_flight = 0
        self.skipped_updates = 0
        self.completed_calls = 0
        self.failed_calls = 0
        self.last_rpc_ms = 0.0
        self.max_rpc_ms = 0.0
        self.last_rpc_error = ""
        self.next_diagnostic_at = time.monotonic() + 1.0
        self.grpc = grpc
        self.target = target
        self.consecutive_transport_failures = 0
        self.reconnect_requested = False
        self.reconnect_forced = False
        self.next_reconnect_at = 0.0
        self.rpc_generation = 0
        self.oldest_in_flight_at = None
        self.rpc_stall_timeout = max(3.0, timeout * 3)
        self.running = True
        self.channel, self.call = self._create_channel()

    def _create_channel(self):
        channel = self.grpc.insecure_channel(self.target)
        call = channel.unary_unary(
            METHOD, request_serializer=lambda b: b, response_deserializer=lambda b: b)
        return channel, call

    @staticmethod
    def _is_transient_rpc_error(error):
        code = getattr(error, "code", None)
        if not callable(code):
            return False
        try:
            code = code()
        except Exception:
            return False
        code_name = getattr(code, "name", str(code).rsplit(".", 1)[-1])
        return code_name in {"UNAVAILABLE", "DEADLINE_EXCEEDED"}

    def _reconnect_channel(self):
        new_channel, new_call = self._create_channel()
        with self.lock:
            old_channel = self.channel
            self.channel, self.call = new_channel, new_call
            self.rpc_generation += 1
            self.in_flight = 0
            self.oldest_in_flight_at = None
            self.consecutive_transport_failures = 0
            self.reconnect_requested = False
            self.reconnect_forced = False
            self.next_reconnect_at = 0.0
        old_channel.close()
        print("joystick rpc: recreated channel after repeated transport failures")

    def set(self, x, y):
        with self.lock:
            self.x, self.y, self.move_active = x, y, True

    def set_yaw(self, z):
        with self.lock:
            self.z, self.yaw_active = z, True

    def release_move(self):
        """Left stick released: zero x,y; stop entirely if yaw is idle too."""
        with self.lock:
            self.x = self.y = 0.0
            self.move_active = False
            if not self.yaw_active:
                self.stop_pending = True

    def release_yaw(self):
        """Right stick released: zero z; stop entirely if movement is idle too."""
        with self.lock:
            self.z = 0.0
            self.yaw_active = False
            if not self.move_active:
                self.stop_pending = True

    def release(self):
        """Emergency / disconnect: stop sending and send zeros."""
        with self.lock:
            self.x = self.y = self.z = 0.0
            self.move_active = self.yaw_active = False
            self.stop_pending = True

    def _send_stop(self):
        for _ in range(5):
            try:
                self.call(joystick_message(0.0, 0.0), timeout=self.timeout)
            except Exception:
                pass
            time.sleep(0.02)

    def _active_call_done(self, future, started_at, generation):
        error = None
        try:
            future.result()
        except Exception as e:
            error = e
        elapsed_ms = (time.monotonic() - started_at) * 1000
        with self.lock:
            if generation != self.rpc_generation:
                return
            self.in_flight -= 1
            if self.in_flight == 0:
                self.oldest_in_flight_at = None
            self.completed_calls += 1
            self.last_rpc_ms = elapsed_ms
            self.max_rpc_ms = max(self.max_rpc_ms, elapsed_ms)
            if error:
                self.failed_calls += 1
                self.last_rpc_error = str(error)
                if self._is_transient_rpc_error(error):
                    self.consecutive_transport_failures += 1
                    if self.consecutive_transport_failures >= 3:
                        self.reconnect_requested = True
                        self.next_reconnect_at = time.monotonic() + 0.25
            else:
                self.consecutive_transport_failures = 0
                self.last_rpc_error = ""

    def _active_call_submit_failed(self, started_at, error, generation):
        elapsed_ms = (time.monotonic() - started_at) * 1000
        with self.lock:
            if generation != self.rpc_generation:
                return
            self.in_flight -= 1
            if self.in_flight == 0:
                self.oldest_in_flight_at = None
            self.failed_calls += 1
            self.last_rpc_ms = elapsed_ms
            self.max_rpc_ms = max(self.max_rpc_ms, elapsed_ms)
            self.last_rpc_error = str(error)
            if self._is_transient_rpc_error(error):
                self.consecutive_transport_failures += 1
                if self.consecutive_transport_failures >= 3:
                    self.reconnect_requested = True
                    self.next_reconnect_at = time.monotonic() + 0.25

    def run(self):
        nxt = time.monotonic()
        while self.running:
            now = time.monotonic()
            with self.lock:
                x, y, z, stop = self.x, self.y, self.z, self.stop_pending
                active = self.move_active or self.yaw_active
                if stop:
                    self.stop_pending = False
                if (
                    active
                    and self.in_flight
                    and self.oldest_in_flight_at is not None
                    and now - self.oldest_in_flight_at >= self.rpc_stall_timeout
                ):
                    self.reconnect_requested = True
                    self.reconnect_forced = True
                    self.next_reconnect_at = now
                    self.last_rpc_error = f"RPCs stalled for {now - self.oldest_in_flight_at:.1f} s"
                reconnect = (
                    self.reconnect_requested
                    and (self.in_flight == 0 or self.reconnect_forced)
                    and now >= self.next_reconnect_at
                )
                send_active = active and not self.reconnect_requested and self.in_flight < self.max_in_flight
                if active and not send_active:
                    self.skipped_updates += 1
                if send_active:
                    self.in_flight += 1
                    generation = self.rpc_generation
                    if self.oldest_in_flight_at is None:
                        self.oldest_in_flight_at = now
            if reconnect:
                try:
                    self._reconnect_channel()
                except Exception as e:
                    with self.lock:
                        self.next_reconnect_at = time.monotonic() + 1.0
                        self.last_rpc_error = f"channel reconnect failed: {e}"
                    print(f"joystick rpc: channel reconnect failed: {e}")
            if stop:
                self._send_stop()
            elif send_active:
                started_at = time.monotonic()
                try:
                    future = self.call.future(joystick_message(x, y, z), timeout=self.timeout)
                    future.add_done_callback(
                        lambda result, started=started_at, token=generation:
                            self._active_call_done(result, started, token)
                    )
                except Exception as e:
                    self._active_call_submit_failed(started_at, e, generation)

            now = time.monotonic()
            if now >= self.next_diagnostic_at:
                with self.lock:
                    in_flight = self.in_flight
                    skipped = self.skipped_updates
                    completed = self.completed_calls
                    failures = self.failed_calls
                    last_rpc_ms = self.last_rpc_ms
                    max_rpc_ms = self.max_rpc_ms
                    last_error = self.last_rpc_error
                    self.skipped_updates = 0
                    self.completed_calls = 0
                    self.failed_calls = 0
                    self.max_rpc_ms = 0.0
                    self.last_rpc_error = ""  # reported once; a still-failing call sets it again
                error = f", last_error={last_error}" if last_error else ""
                if failures or last_error:  # healthy periods stay quiet; the viewer prints a heartbeat
                    print(
                        f"joystick rpc: completed={completed}, failed={failures}, "
                        f"in_flight={in_flight}/{self.max_in_flight}, skipped={skipped}, "
                        f"last={last_rpc_ms:.1f} ms, max={max_rpc_ms:.1f} ms{error}"
                    )
                self.next_diagnostic_at = now + 1.0
            nxt += self.dt
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.monotonic()

    def shutdown(self):
        self.release()
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            with self.lock:
                if not self.stop_pending:
                    break
            time.sleep(0.02)
        time.sleep(0.15)
        self.running = False
        self.channel.close()
        self.join(timeout=1.0)


def head_message(yaw, pitch, yaw_field=2):
    """yaw/pitch each -1/0/1. Zero fields are omitted; all-zero means no body at all (stop)."""
    pitch_field = 3 - yaw_field
    body = b""
    for field, val in sorted(((yaw_field, yaw), (pitch_field, pitch))):
        if val:
            body += f_varint(field, val)
    msg = f_varint(1, API_HEAD) + f_bytes(2, str(uuid.uuid4()).encode())
    if body:
        msg += f_bytes(3, body)
    return msg + f_varint(5, 1)


class GrpcHead(threading.Thread):
    """Drop-in for robot.Head: set_dir(x, y) (d-pad hat, right/up positive), start(), shutdown()."""

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