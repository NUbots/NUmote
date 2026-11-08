"""
Robot helpers: video protobuf decoding, stick mapping, and SDK head control.

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
import threading
import time

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
