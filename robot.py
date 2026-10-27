"""
Robot protocol helpers: protobuf encode/decode, the joystick command, and a background sender.

Joystick command (id 1003), sent to /booster.proto.rpc.RobotGrpc/RobotRequest:
    1: 1003            command id
    2: "<uuid>"        fresh per message
    3 { 1: x  2: y  3: z }   floats; x,y = left stick (translation), z = right stick (yaw)
    5: 1

Video frames arrive over websockets as:
    1: <int>            (10001)
    2: "<uuid>"
    3 { 1: <JPEG bytes> ... }

SAFETY - this can move a physical robot. Stop messages are sent on release and shutdown,
but the emergency stop is the real safety.
"""
import math
import struct
import threading
import time
import uuid

METHOD = "/booster.proto.rpc.RobotGrpc/RobotRequest"
CMD_ID = 1003


# ---------- protobuf encoding ----------
def enc_varint(n):
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


# ---------- minimal protobuf reader ----------
def varint(b, i):
    n = shift = 0
    while True:
        c = b[i]
        i += 1
        n |= (c & 0x7F) << shift
        if not c & 0x80:
            return n, i
        shift += 7


def parse(b):
    i, out = 0, []
    while i < len(b):
        key, i = varint(b, i)
        num, wt = key >> 3, key & 7
        if wt == 0:
            v, i = varint(b, i)
        elif wt == 1:
            v = b[i:i + 8]
            i += 8
        elif wt == 5:
            v = b[i:i + 4]
            i += 4
        elif wt == 2:
            n, i = varint(b, i)
            v = b[i:i + n]
            i += n
        else:
            raise ValueError(f"bad wire type {wt}")
        out.append((num, wt, v))
    return out


def extract_jpeg(msg: bytes):
    """Return JPEG bytes from a message, or None."""
    try:
        for num, wt, v in parse(msg):
            if num == 3 and wt == 2:
                for n2, wt2, v2 in parse(v):
                    if wt2 == 2 and v2[:2] == b"\xff\xd8":
                        return v2
    except Exception:
        pass
    # Fallback: scan for JPEG start/end markers
    s = msg.find(b"\xff\xd8\xff")
    e = msg.rfind(b"\xff\xd9")
    if s != -1 and e > s:
        return msg[s:e + 2]
    return None


# ---------- stick ----------
def clamp_disc(x, y, max_mag):
    m = math.hypot(x, y)
    if m > max_mag and m > 0:
        return x * max_mag / m, y * max_mag / m
    return x, y


def map_stick(sx, sy, max_mag):
    """Stick (each -1..1) -> (field 3.1, field 3.2), capped at max_mag."""
    return clamp_disc(sx * max_mag, sy * max_mag, max_mag)


def map_yaw(sx, max_yaw):
    """Stick axis (-1..1) -> field 3.3, capped at max_yaw."""
    return max(-max_yaw, min(max_yaw, sx * max_yaw))


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
                error = f", last_error={last_error}" if last_error else ""
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


class Head(threading.Thread):
    """Moves the head through the Booster SDK (DDS); holding the d-pad sweeps the target angle."""

    SPEED = 1.0  # rad/s
    YAW_LIMIT = 0.785  # rad; documented RotateHead range is +/-0.785
    PITCH_MIN, PITCH_MAX = -0.3, 1.0  # documented range

    def __init__(self, domain_id=0, interface=""):
        super().__init__(daemon=True)
        import booster_robotics_sdk_python as sdk
        sdk.ChannelFactory.Instance().Init(domain_id, interface)
        self.client = sdk.B1LocoClient()
        self.client.Init()
        self.lock = threading.Lock()
        self.dir = (0, 0)
        self.yaw = self.pitch = 0.0
        self.running = True

    def set_dir(self, x, y):
        """d-pad direction, each -1/0/1; right and up are positive."""
        with self.lock:
            self.dir = (x, y)

    def run(self):
        last = time.monotonic()
        while self.running:
            time.sleep(0.05)
            now = time.monotonic()
            dt, last = now - last, now
            with self.lock:
                dx, dy = self.dir
            if not (dx or dy):
                continue
            # SDK: positive yaw looks left, positive pitch looks down
            self.yaw = max(-self.YAW_LIMIT, min(self.YAW_LIMIT, self.yaw - dx * self.SPEED * dt))
            self.pitch = max(self.PITCH_MIN, min(self.PITCH_MAX, self.pitch - dy * self.SPEED * dt))
            try:
                self.client.RotateHead(self.pitch, self.yaw)
            except Exception as e:
                print(f"head error: {e}")
                time.sleep(2)  # the failed call already blocked for its timeout
                last = time.monotonic()

    def shutdown(self):
        self.running = False
