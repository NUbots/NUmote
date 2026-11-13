import asyncio, base64, os, queue, shutil, struct, subprocess, threading, time
from datetime import datetime
from pathlib import Path

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")  # no SDL window, so no focus
import pygame
import pygame._sdl2.controller as sdl_controller
from nicegui import app, ui
from grpc_control import GrpcHead, Sender
from robot import extract_jpeg, map_stick, map_yaw, parse
import hashlib, hmac, html, json, posixpath, random, secrets, shlex, socket
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
ASSETS = ROOT / "assets"
SETTINGS_PATH = ROOT / ".numote_settings.json"

def load_settings():
    try:
        return json.loads(SETTINGS_PATH.read_text())
    except (OSError, ValueError):
        return {}

def save_settings(**changes):
    try:
        SETTINGS_PATH.write_text(json.dumps({**load_settings(), **changes}))
    except OSError as e:
        print(f"couldn't save settings: {e}")

settings = load_settings()
robot_ip = "" # empty for now
if not robot_ip:
    robot_ip = str(settings.get("ip", ""))  # reconnect to the last robot used
ip_list = [] # this is the list of discovered Robot IPs

video_stream_port = 51111 # this seems to be standard for the Boosters.
discovery_port = 9000 # found with WireShark
KICK_ACTION = os.environ.get("NUMOTE_KICK_ACTION", "kick")
DEFAULT_AGENT_ID = "com.boosterobotics.default"
SOCCER_AGENT_ID = "com.boosterobotics.soccer"
SOCCER_KICK_ACTION = os.environ.get("NUMOTE_SOCCER_KICK_ACTION_ID", "visual_kick_ball")
SOCCER_POWER_KICK_ACTION = os.environ.get("NUMOTE_SOCCER_POWER_KICK_ACTION_ID", "visual_kick_ball_power")
SOCCER_CELEBRATIONS = (  # component ids from the soccer agent's shortcut_list.json
    ("Waka Waka", "robocup_wakawaka"),
    ("Siuuu", "robocup_siuuu"),
    ("Ring Kiss", "robocup_ring_kiss"),
    ("Arms Crossed", "robocup_arms_crossed"),
    ("Point to Sky", "robocup_point_to_sky"),
    ("Cradle Dance", "robocup_cradle"),
)
KICK_LATCH_SECONDS = 8  # the soccer kick stays latched on until it's switched off, so it's released after this long
CRITICAL_BATTERY = 15  # nularm
STRONG_KICK_PORT = int(os.environ.get("NUMOTE_STRONG_KICK_PORT", "51121"))
# shared with the bridge each time this app starts it, so the strong kick and dance work without any setup;
# set NUMOTE_STRONG_KICK_TOKEN to pin one instead
STRONG_KICK_TOKEN = os.environ.get("NUMOTE_STRONG_KICK_TOKEN") or secrets.token_hex(32)

CAMERAS = ("left", "right")  # index matches the camera id sent by NUmote_bridge.py
STREAM_DEPTH = len(CAMERAS)  # stream ids after the cameras, matching NUmote_bridge.py
STREAM_DETECTIONS = STREAM_DEPTH + 1
VIEWS = ("RAW", "DEPTH", "DETECTIONS")
DETECTION_BGR = (0, 174, 255)  # #FFAE00, the colour of the on-page boxes
CAMERA_RELAY_PORT = int(os.environ.get("NUMOTE_CAMERA_RELAY_PORT", "51120"))
VISION_INTERFACE_SETUP = os.environ.get(
    "NUMOTE_VISION_INTERFACE_SETUP",
    "/opt/booster/booster_agent_data/data/agents/extract/com.boosterobotics.soccer/agent/vision_interface/share/vision_interface/local_setup.bash",
)  # robot path; provides the message type the bridge needs for detections
relay_frame_at = 0.0  # monotonic time of the last relay frame
bridge_hashes = {"local": "", "remote": ""}  # short SHA-256 prefixes; remote is "", "missing" or "?" when unknown
bridge_status = ""  # progress of the SSH deploy and start, shown in the header

state = {
    "jpeg": {cam: None for cam in CAMERAS},
    "count": {cam: 0 for cam in CAMERAS},
    "depth": None, "depth_count": 0,  # raw 16-bit depth PNG
    "depth_view": None, "depth_view_count": -1,  # its colormapped JPEG, made on demand
    "detections": [], "detections_count": 0,  # [{label, confidence, box: [xmin, ymin, xmax, ymax]}]
    "battery": None, "waving": False, "chasing": False, "kicking": False, "walking": False,
}
recording_status = "Not recording"
recording_active = False
recording_started_at = 0.0  # monotonic time the current recording began
recording_stopping = False
recording_lock = threading.Lock()
video_recorders = {}  # "left" | "right" | "detections" -> VideoRecorder, "depth" -> DepthRecorder, "detections_log" -> DetectionLog

sender = None
discovery_sock = None
max_mag = max_yaw = min(max(float(settings.get("speed", 100)), 10), 100) / 100

pad = None
controller = None
controller_status = "No controller connected"
controller_layout = "none"  # "none" | "raw" | "sdl" | "switch"
pad_axes = (0.0, 0.0, 0.0, 0.0)  # lx, ly, rx, ry after dead zone
axes_armed = False  # False until a freshly connected pad has reported a neutral stick reading
deadzone = 0.15
last_axis_log_at = 0.0
# SDL joystick button index -> (name, action) on this pad
button_actions = {
    0: ("A", lambda: send_random_dance()),
    4: ("Y", lambda: send_action("get_up")),
    3: ("X", lambda: toggle_wave()),
    1: ("B", lambda: send_soccer_kick()),
    10: ("Back", lambda: send_robot_mode(1)),   # STAND
    11: ("Start", lambda: send_robot_mode(2)),  # WALK
}
controller_button_actions = {
    pygame.CONTROLLER_BUTTON_A: ("A", lambda: send_random_dance()),
    pygame.CONTROLLER_BUTTON_Y: ("Y", lambda: send_action("get_up")),
    pygame.CONTROLLER_BUTTON_X: ("X", lambda: toggle_wave()),
    pygame.CONTROLLER_BUTTON_B: ("B", lambda: send_soccer_kick()),
    pygame.CONTROLLER_BUTTON_RIGHTSHOULDER: ("RB", None),
    pygame.CONTROLLER_BUTTON_LEFTSTICK: ("L3", None),
    pygame.CONTROLLER_BUTTON_BACK: ("Back", lambda: send_robot_mode(1)),
    pygame.CONTROLLER_BUTTON_START: ("Start", lambda: send_robot_mode(2)),
}
switch_controller_button_actions = {
    **controller_button_actions,
    pygame.CONTROLLER_BUTTON_Y: ("X", lambda: toggle_wave()),
    pygame.CONTROLLER_BUTTON_X: ("Y", lambda: send_action("get_up")),
}
# keyed by printed button name; the Switch dict relabels so a name means the same action on every layout
action_descriptions = {
    "A": "Random dance", "B": "Kick", "X": "Toggle wave",
    "Y": "Get up", "RB": "Toggle chase", "Back": "Stand", "Start": "Walk",
}
# printed face button -> position on the pad; Switch swaps A/B and X/Y relative to Xbox
face_slots = {
    "sdl": {"Y": "north", "B": "east", "A": "south", "X": "west"},
    "switch": {"X": "north", "A": "east", "B": "south", "Y": "west"},
}
pad_labels = {
    "sdl": {"title": "Xbox / standard gamepad", "lt": "LT", "rt": "RT", "lb": "LB", "rb": "RB", "back": "Back", "start": "Start"},
    "switch": {"title": "Nintendo Switch controller", "lt": "ZL", "rt": "ZR", "lb": "L", "rb": "R", "back": "Minus", "start": "Plus"},
}
raw_mode_buttons = {10, 11}
controller_mode_buttons = {pygame.CONTROLLER_BUTTON_BACK, pygame.CONTROLLER_BUTTON_START}
buttons_down = set()
head_dir = (0, 0)  # d-pad hat (x, y), each -1/0/1
head = None
record_combo_stage = 0
left_trigger_was_pressed = False
right_trigger_was_pressed = False

SCP_LOCATION = "" # place we SCP the bridge to; empty = remote user's home directory
ROBOT_SSH_USER = os.environ.get("NUMOTE_ROBOT_SSH_USER", "booster")

class VideoRecorder:
    FPS = 30

    def __init__(self, output_dir, camera):
        ffmpeg = shutil.which(os.environ.get("NUMOTE_FFMPEG", "ffmpeg"))
        if not ffmpeg:
            raise RuntimeError("FFmpeg was not found; install it or set NUMOTE_FFMPEG")

        output_dir.mkdir(parents=True, exist_ok=True)
        self.path = output_dir / f"{camera}.mp4"
        self.frames = queue.Queue(maxsize=120)
        self.frame_interval = 1.0 / self.FPS
        self.schedule_lock = threading.Lock()
        self.last_frame = None
        self.next_frame_at = None
        self.stopping = threading.Event()
        self.dropped_frames = 0
        self.error = None
        self.process = subprocess.Popen(
            [
                ffmpeg, "-y", "-hide_banner", "-loglevel", "error",
                "-f", "image2pipe", "-framerate", str(self.FPS), "-vcodec", "mjpeg", "-i", "pipe:0",
                "-an", "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
                "-c:v", "libx264", "-preset", "ultrafast", "-crf", "12",
                "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(self.path),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        self.worker = threading.Thread(target=self._write_frames, daemon=True)
        self.worker.start()

    def _enqueue_frame(self, frame):
        try:
            self.frames.put_nowait(frame)
        except queue.Full:
            self.dropped_frames += 1

    def add_frame(self, frame, captured_at=None):
        if self.process.poll() is not None:
            return
        now = time.monotonic() if captured_at is None else captured_at
        with self.schedule_lock:
            if self.stopping.is_set():
                return
            if self.last_frame is None:
                self._enqueue_frame(frame)
                self.last_frame = frame
                self.next_frame_at = now + self.frame_interval
                return
            while now - self.next_frame_at > self.frame_interval * 1e-6:
                self._enqueue_frame(self.last_frame)
                self.next_frame_at += self.frame_interval
            self.last_frame = frame

    def _write_frames(self):
        try:
            while not self.stopping.is_set() or not self.frames.empty():
                try:
                    frame = self.frames.get(timeout=0.1)
                except queue.Empty:
                    continue
                self.process.stdin.write(frame)
        except Exception as e:
            self.error = str(e)
            self.stopping.set()
        finally:
            try:
                self.process.stdin.close()
            except Exception:
                pass
            if self.error and self.process.poll() is None:
                self.process.terminate()
            try:
                return_code = self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                return_code = self.process.wait()
                self.error = self.error or "FFmpeg did not finish in time"
            if return_code:
                reason = self.error or f"FFmpeg exited with code {return_code}"
                detail = self.process.stderr.read().decode(errors="replace").strip()
                self.error = f"{reason}: {detail}" if detail else reason
            self.process.stderr.close()

    def stop(self):
        with self.schedule_lock:
            self.stopping.set()
            if self.last_frame is not None:
                self._enqueue_frame(self.last_frame)
                self.last_frame = None
        self.worker.join(timeout=12)
        if self.worker.is_alive():
            self.process.terminate()
            self.worker.join(timeout=2)
        if self.worker.is_alive():
            self.process.kill()
            self.worker.join()
        if self.error:
            raise RuntimeError(self.error)
        return self.path

class DepthRecorder:
    """Saves each raw 16-bit depth PNG untouched as depth/<milliseconds since recording started>.png."""

    def __init__(self, output_dir):
        self.path = output_dir / "depth"
        self.path.mkdir(parents=True, exist_ok=True)
        self.frames = queue.Queue(maxsize=120)
        self.started = time.monotonic()
        self.dropped_frames = 0
        self.error = None
        self.worker = threading.Thread(target=self._write_frames, daemon=True)
        self.worker.start()

    def add_frame(self, png):
        name = f"{int((time.monotonic() - self.started) * 1000):09d}.png"
        try:
            self.frames.put_nowait((name, png))
        except queue.Full:
            self.dropped_frames += 1

    def _write_frames(self):
        while (item := self.frames.get()) is not None:
            if self.error:  # keep draining so stop() never blocks on a full queue
                continue
            try:
                (self.path / item[0]).write_bytes(item[1])
            except OSError as e:
                self.error = str(e)

    def stop(self):
        self.frames.put(None)
        self.worker.join()
        if self.error:
            raise RuntimeError(self.error)
        return self.path

class DetectionLog:
    """One JSON line per detections message: seconds since recording started, plus the objects."""
    dropped_frames = 0

    def __init__(self, output_dir):
        output_dir.mkdir(parents=True, exist_ok=True)
        self.path = output_dir / "detections.jsonl"
        self.file = open(self.path, "w")
        self.started = time.monotonic()
        self.lock = threading.Lock()

    def add(self, objects):
        record = {"t": round(time.monotonic() - self.started, 3), "objects": objects}
        with self.lock:
            if not self.file.closed:
                self.file.write(json.dumps(record, separators=(",", ":")) + "\n")

    def stop(self):
        with self.lock:
            self.file.close()
        return self.path

def stop_recorders(recorders):
    """Stop every recorder; returns (saved descriptions, error descriptions)."""
    saved, errors = [], []
    for recorder in recorders.values():
        try:
            path = recorder.stop()
            dropped = f" ({recorder.dropped_frames} frames dropped)" if recorder.dropped_frames else ""
            saved.append(f"{path.name}{dropped}")
        except Exception as e:
            errors.append(f"{recorder.path.name}: {e}")
    return saved, errors

def recordings_dir():
    return Path(os.environ.get("NUMOTE_RECORDING_DIR", ROOT / "recordings"))

def open_recordings_folder():
    """Opens the folder on the machine running this script."""
    path = recordings_dir()
    path.mkdir(parents=True, exist_ok=True)
    try:
        subprocess.Popen(["xdg-open", str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError as e:
        notify(f"Couldn't open {path}: {e}", type="negative")

def toggle_video_recording():
    global video_recorders, recording_status, recording_active, recording_started_at, recording_stopping
    with recording_lock:
        if recording_stopping:
            return
        if not video_recorders:
            cameras = [cam for cam in CAMERAS if state["jpeg"][cam]]
            if not cameras:
                recording_status = "No video stream to record"
                return
            recordings_root = recordings_dir()
            output_dir = recordings_root / f"numote_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
            recorders = {}
            try:
                for cam in cameras:
                    recorders[cam] = VideoRecorder(output_dir, cam)
                if state["depth"]:
                    recorders["depth"] = DepthRecorder(output_dir)
                if relay_active() and CAMERAS[0] in cameras:  # detections only come through the bridge
                    recorders["detections"] = VideoRecorder(output_dir, "detections")
                    recorders["detections_log"] = DetectionLog(output_dir)
            except Exception as e:
                stop_recorders(recorders)
                recording_status = f"Recording failed: {e}"
                print(recording_status)
                return
            video_recorders = recorders
            recording_started_at = time.monotonic()
            recording_active = True
            recording_status = f"Recording to {output_dir.name}/: " + ", ".join(r.path.name for r in recorders.values())
            print(recording_status)
            return
        recording_active = False
        recording_stopping = True
        current_recorders = video_recorders
        video_recorders = {}
        recording_status = "Finalizing recording..."

    try:
        output_dir = next(iter(current_recorders.values())).path.parent
        saved, errors = stop_recorders(current_recorders)
        recording_status = "; ".join(
            ([f"Saved to {output_dir}"] if saved else [])
            + (["Recording failed: " + ", ".join(errors)] if errors else [])
        )
        print(recording_status)
    finally:
        with recording_lock:
            recording_stopping = False
            recording_active = False

def add_recording_frame(name, frame):
    with recording_lock:
        recorder = video_recorders.get(name)
    if recorder:
        recorder.add_frame(frame)

def draw_detections(jpeg, detections):
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    for d in detections:
        x0, y0, x1, y1 = d["box"]
        cv2.rectangle(image, (x0, y0), (x1, y1), DETECTION_BGR, 2)
        cv2.putText(image, f'{d["label"]} {d["confidence"]:.0f}%', (x0, max(y0 - 4, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, DETECTION_BGR, 1, cv2.LINE_AA)
    return cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 98])[1].tobytes()

def add_detections_frame(jpeg):
    with recording_lock:
        recorder = video_recorders.get("detections")
    if recorder:  # drawn only while recording, since it costs a decode and re-encode per frame
        recorder.add_frame(draw_detections(jpeg, state["detections"]))

def add_detections_record(objects):
    with recording_lock:
        log = video_recorders.get("detections_log")
    if log:
        log.add(objects)

def on_frame(cam, jpeg):
    state["jpeg"][cam] = jpeg
    state["count"][cam] += 1
    add_recording_frame(cam, jpeg)
    if cam == CAMERAS[0]:
        add_detections_frame(jpeg)

def relay_active():
    return time.monotonic() - relay_frame_at < 2.0

def connection_summary():
    """(Quasar colour, text) for the header status line."""
    if not robot_ip:
        return "grey-6", "No robot selected"
    if relay_active():
        return "green-5", f"{robot_ip}: bridge running"
    status = bridge_status or "waiting"
    colour = "red-5" if status.startswith("Bridge failed") else "amber-5"
    stream = "video only" if state["jpeg"][CAMERAS[0]] else "no video yet"
    return colour, f"{robot_ip}: {stream} - {status}"

BUBBLES = {  # connection_summary colour -> (icon, style); the colours match the notification toasts
    "grey-6": ("videocam_off", "background: #323232; color: #fff"),
    "amber-5": ("hourglass_top", "background: #f2c037; color: #000"),
    "red-5": ("error", "background: #c10015; color: #fff"),
}

def robot_alive():
    """True while the robot's connection is delivering data."""
    return bool(robot_ip) and (relay_active() or time.monotonic() - state.get("seen", 0.0) < 6.0)

def stream_row(name, seen_key):
    """A relay stream is live while messages keep arriving; an empty detections list still counts."""
    if time.monotonic() - state.get(seen_key, 0.0) < 5.0:
        return (name, "green-5", "Receiving")
    return (name, "amber-5", "Waiting for data" if relay_active() else "Needs the bridge")

def connection_rows():
    """(label, Quasar colour, text) for each link, shown in the robot menu."""
    if not robot_ip:
        return [(name, "grey-6", "-") for name in ("Robot", "Bridge", "Bridge file", "Video", "Depth", "Detections")]
    has_video = any(state["jpeg"].values())
    robot_up = robot_alive()
    status = bridge_status or "Waiting"
    if relay_active():
        bridge = ("green-5", "Running")
    elif status.startswith("Bridge failed"):
        bridge = ("red-5", status)
    else:
        bridge = ("amber-5", status)
    local, remote = bridge_hashes["local"], bridge_hashes["remote"]
    if not remote:
        file_row = ("amber-5", f"Not checked ({local})" if local else "Not checked")
    elif remote == local:
        file_row = ("green-5", f"Up to date ({local})")
    elif remote == "missing":
        file_row = ("red-5", "Missing on robot")
    elif remote == "?":
        file_row = ("amber-5", "Couldn't check")
    else:
        file_row = ("amber-5", f"Out of date ({remote} vs {local})")
    return [
        ("Robot", "green-5" if robot_up else "amber-5", "Connected" if robot_up else "Connecting..."),
        ("Bridge", *bridge),
        ("Bridge file", *file_row),
        ("Video", "green-5" if has_video else "amber-5", "Streaming" if has_video else "No video yet"),
        stream_row("Depth", "depth_seen"),
        stream_row("Detections", "detections_seen"),
    ]

def robot_icon_props():
    colour = connection_summary()[0]
    return f"icon=smart_toy color={'grey-7' if colour == 'grey-6' else colour}"

def controller_icon_props():
    connected = controller_layout != "none"
    return f"icon={'sports_esports' if connected else 'videogame_asset_off'} color={'green-5' if connected else 'grey-7'}"

def set_speed_limit(percent):
    global max_mag, max_yaw
    max_mag = max_yaw = percent / 100
    save_settings(speed=percent)

def notify(message, **kwargs):
    ui.notify(message, position="top", **kwargs)

def clear_frames(cameras):
    for cam in cameras:
        if state["jpeg"][cam]:
            state["jpeg"][cam] = None
            state["count"][cam] += 1

def clear_relay_streams():
    if state["depth"]:
        state["depth"] = None
        state["depth_count"] += 1
    if state["detections"]:
        state["detections"] = []
        state["detections_count"] += 1

def colormap_depth(png):
    depth = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED)
    valid = depth > 0  # 0 means no measurement
    colored = np.zeros((*depth.shape, 3), np.uint8)
    if valid.any():
        # scaled per frame because the depth units are unconfirmed
        near, far = np.percentile(depth[valid][::16], (1, 99))
        scaled = np.clip((depth.astype(np.float32) - near) / max(far - near, 1) * 255, 0, 255).astype(np.uint8)
        colored = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
        colored[~valid] = 0
    return cv2.imencode(".jpg", colored, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()

def depth_view_jpeg():
    if state["depth_view_count"] != state["depth_count"]:
        state["depth_view"] = colormap_depth(state["depth"]) if state["depth"] else None
        state["depth_view_count"] = state["depth_count"]
    return state["depth_view"]

def detections_svg(detections):
    """Boxes and labels in image pixels; the overlay's viewport has the image's dimensions."""
    return "".join(
        f'<rect x="{x0}" y="{y0}" width="{x1 - x0}" height="{y1 - y0}" fill="none" stroke="#FFAE00" stroke-width="2"/>'
        f'<text x="{x0}" y="{max(y0 - 3, 10)}" fill="#FFAE00" font-size="12">{html.escape(d["label"])} {d["confidence"]:.0f}%</text>'
        for d in detections for x0, y0, x1, y1 in [d["box"]]
    )

async def relay_loop(ip):
    """Both head cameras from k1_camera_relay.py running on the robot."""
    global relay_frame_at
    last_error = None
    while True:
        writer = None
        try:
            reader, writer = await asyncio.wait_for(asyncio.open_connection(ip, CAMERA_RELAY_PORT), timeout=5)
            print(f"camera relay connected ({ip}:{CAMERA_RELAY_PORT})")
            last_error = None
            while True:
                stream, size = struct.unpack("<BI", await asyncio.wait_for(reader.readexactly(5), timeout=5))
                payload = await reader.readexactly(size)
                if stream < len(CAMERAS):
                    relay_frame_at = time.monotonic()
                    on_frame(CAMERAS[stream], payload)
                elif stream == STREAM_DEPTH:
                    state["depth"] = payload
                    state["depth_seen"] = time.monotonic()
                    state["depth_count"] += 1
                    add_recording_frame("depth", payload)
                elif stream == STREAM_DETECTIONS:
                    state["detections"] = json.loads(payload)
                    state["detections_seen"] = time.monotonic()
                    state["detections_count"] += 1
                    add_detections_record(state["detections"])
        except asyncio.CancelledError:
            raise
        except Exception as e:
            error = f"{type(e).__name__}: {e}"
            if error != last_error:  # the relay may simply not be running; don't spam
                print(f"camera relay unavailable ({error}); showing the app stream, retrying every 2s")
                last_error = error
            clear_frames(CAMERAS[1:])
            clear_relay_streams()
        finally:
            if writer:
                writer.close()
        await asyncio.sleep(2)

def trigger_pressed(value):
    return value > (0.5 if abs(value) <= 1 else 16384)

def mapping_html():
    """Controller mapping for the connected layout; built only from the constants above."""
    if controller_layout == "none":
        return "<p>No controller connected</p>"
    if controller_layout == "raw":
        rows = "".join(
            f"<tr><td>button {i}</td><td>{name}</td><td>{action_descriptions[name]}</td></tr>"
            for i, (name, _) in button_actions.items()
        )
        return f"<table class='map'>{rows}</table>"
    labels = pad_labels[controller_layout]
    slots = face_slots[controller_layout]
    scale, pad_cx, pad_cy = 1.3, 380, 208  # the pad is drawn around (380, 208); scaled up about that point
    pt = lambda x, y: (round(450 + scale * (x - pad_cx), 1), round(pad_cy + scale * (y - pad_cy), 1))
    desc = action_descriptions

    def desc_lines(text, x, y, anchor):
        # "|" splits a description into lines
        return "".join(
            f"<text class='desc' x='{x}' y='{y + 18 * i}' text-anchor='{anchor}'>{line}</text>"
            for i, line in enumerate(text.split("|"))
        )

    def callout(cx, cy, side, ly, title, text):
        # title is the printed button name; ly is where the label sits, cx/cy the control in pad coordinates
        w = max(30, 7.4 * len(title) + 18)
        left_side = side == "left"
        edge = 180 if left_side else 720
        px = edge - w if left_side else edge
        end, bend = (edge + 4, edge + 34) if left_side else (edge - 4, edge - 34)
        ax, ay = pt(cx, cy)
        return (
            f"<polyline class='leader' points='{ax},{ay} {bend},{ly} {end},{ly}'/>"
            f"<circle class='dot' cx='{ax}' cy='{ay}' r='3'/>"
            f"<rect class='pill' x='{px}' y='{ly - 11}' width='{w}' height='22' rx='11'/>"
            f"<text class='cap' x='{px + w / 2}' y='{ly}' dy='.35em'>{title}</text>"
            + desc_lines(text, edge, ly + 28, "end" if left_side else "start")
        )

    def below(cx, cy, mx, title, text):
        w = max(30, 7.4 * len(title) + 18)
        ax, ay = pt(cx, cy)
        return (
            f"<polyline class='leader' points='{ax},{ay + 10} {ax},272 {mx},294'/>"
            f"<circle class='dot' cx='{ax}' cy='{ay}' r='3'/>"
            f"<rect class='pill' x='{mx - w / 2}' y='294' width='{w}' height='22' rx='11'/>"
            f"<text class='cap' x='{mx}' y='305' dy='.35em'>{title}</text>"
            + desc_lines(text, mx, 333, "middle")
        )

    face_offsets = {"north": (0, -24), "east": (24, 0), "south": (0, 24), "west": (-24, 0)}
    face_buttons = "".join(
        f"<circle class='btn' cx='{490 + face_offsets[slot][0]}' cy='{175 + face_offsets[slot][1]}' r='12'/>"
        f"<text class='btn-t' x='{490 + face_offsets[slot][0]}' y='{175 + face_offsets[slot][1]}' dy='.35em'>{name}</text>"
        for name, slot in slots.items()
    )
    face_legend = "".join(
        f"<circle class='btn' cx='732' cy='{204 + 24 * i}' r='9'/>"
        f"<text class='btn-t' x='732' y='{204 + 24 * i}' dy='.35em'>{name}</text>"
        f"<text class='desc' x='750' y='{204 + 24 * i}' dy='.35em'>{desc[name]}</text>"
        for i, name in enumerate(sorted(slots))
    )
    return f"""
    <div class='mapping-title'>{labels['title']}</div>
    <svg class='mapping' viewBox='47 25 820 353' xmlns='http://www.w3.org/2000/svg'>
      <defs>
        <linearGradient id='padbody' x1='0' y1='0' x2='0' y2='1'>
          <stop offset='0' stop-color='#323232'/><stop offset='1' stop-color='#1f1f1f'/>
        </linearGradient>
      </defs>
      <g transform='translate(450,{pad_cy}) scale({scale}) translate({-pad_cx},{-pad_cy})'>
        <rect class='ctl on' x='245' y='82' width='80' height='20' rx='8'/><text class='lbl' x='285' y='92' dy='.35em'>{labels['lt']}</text>
        <rect class='ctl on' x='435' y='82' width='80' height='20' rx='8'/><text class='lbl' x='475' y='92' dy='.35em'>{labels['rt']}</text>
        <rect class='ctl' x='245' y='106' width='80' height='12' rx='6'/><text class='lbl' x='285' y='112' dy='.35em'>{labels['lb']}</text>
        <rect class='ctl on' x='435' y='106' width='80' height='12' rx='6'/><text class='lbl' x='475' y='112' dy='.35em'>{labels['rb']}</text>
        <path class='body' d='M 270 120 H 490 C 520 120 540 140 548 180 L 575 290 C 582 322 548 334 528 312 L 470 250 H 290 L 232 312 C 212 334 178 322 185 290 L 212 180 C 220 140 240 120 270 120 Z'/>
        <circle class='ctl' cx='380' cy='150' r='8'/>
        <circle class='ctl on' cx='270' cy='175' r='24'/><circle class='stick' cx='270' cy='175' r='14'/>
        <path class='ctl on' d='M 309 207 h12 v9.5 h9.5 v12 h-9.5 v9.5 h-12 v-9.5 h-9.5 v-12 h9.5 z'/>
        <circle class='ctl on' cx='445' cy='222' r='22'/><circle class='stick' cx='445' cy='222' r='13'/>
        <circle class='ctl on' cx='355' cy='170' r='6'/><circle class='ctl on' cx='405' cy='170' r='6'/>
        {face_buttons}
      </g>
      {callout(245, 92, "left", 57, labels['lt'], f"Celebration (+A)|Recording combo|({labels['lt']} + {labels['rt']} + L3)")}
      {callout(248, 175, "left", 165, 'Left stick', 'Walk (press: L3)')}
      {callout(300, 222, "left", 226, 'D-pad', 'Head yaw / pitch')}
      {callout(515, 92, "right", 57, labels['rt'], 'Power kick (+B)|Danger dance (+A)')}
      {callout(515, 112, "right", 130, labels['rb'], desc['RB'])}
      <polyline class='leader' points='640,165 678,240 712,240'/><circle class='dot' cx='640' cy='165' r='3'/>
      <polyline class='leader' points='716,194 712,194 712,286 716,286'/>
      <text class='head' x='720' y='184'>FACE BUTTONS</text>
      {face_legend}
      {callout(465, 222, "right", 325, 'Right stick', 'Turn')}
      {below(355, 170, 405, labels['back'], desc['Back'])}
      {below(405, 170, 495, labels['start'], desc['Start'])}
    </svg>"""

def shutdown_video_recording():
    global video_recorders, recording_active, recording_stopping
    with recording_lock:
        if not video_recorders or recording_stopping:
            return
        recording_active = False
        recording_stopping = True
        current_recorders = video_recorders
        video_recorders = {}
    try:
        _, errors = stop_recorders(current_recorders)
        if errors:
            print(f"recording shutdown failed: {', '.join(errors)}")
    finally:
        with recording_lock:
            recording_stopping = False

def start_head(ip):
    global head
    if head:
        head.shutdown()
    head = GrpcHead(f"{ip}:50051")
    head.start()

def start_pad():
    pygame.display.init()
    pygame.joystick.init()
    sdl_controller.init()

def poll_pad():
    """Read the controller through SDL and drive the robot."""
    global pad, controller, controller_status, controller_layout, pad_axes, buttons_down, last_axis_log_at
    global record_combo_stage, left_trigger_was_pressed, right_trigger_was_pressed, axes_armed
    pygame.event.pump()
    if any(event.type in (pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED) for event in pygame.event.get()):
        pad = None
        controller = None
        axes_armed = False
        buttons_down.clear()

    if pygame.joystick.get_count() == 0:
        pad = None
        controller = None
        axes_armed = False
        controller_status = "No controller connected"
        controller_layout = "none"
    elif pad is None:
        pad = pygame.joystick.Joystick(0)
        controller = sdl_controller.Controller(0) if sdl_controller.is_controller(0) else None
        axes_armed = False
        is_switch_layout = "nintendo switch" in pad.get_name().lower()
        controller_layout = "raw" if controller is None else "switch" if is_switch_layout else "sdl"
        mapping_status = (
            "SDL Switch face labels" if controller and is_switch_layout else
            "SDL mapping" if controller else
            "raw button mapping"
        )
        controller_status = f"{pad.get_name()} ({mapping_status})"
        print(f"controller connected: {controller_status}")

    if controller and "nintendo switch" in pad.get_name().lower():
        actions = switch_controller_button_actions
    else:
        actions = controller_button_actions if controller else button_actions
    if controller:
        now_down = {button for button in actions if controller.get_button(button)}
    elif pad:
        now_down = {i for i in range(pad.get_numbuttons()) if pad.get_button(i)}
    else:
        now_down = set()
    newly_pressed = now_down - buttons_down
    recording_combo = False
    if controller:
        left_trigger = trigger_pressed(controller.get_axis(pygame.CONTROLLER_AXIS_TRIGGERLEFT))
        right_trigger = trigger_pressed(controller.get_axis(pygame.CONTROLLER_AXIS_TRIGGERRIGHT))
        left_trigger_rising = left_trigger and not left_trigger_was_pressed
        right_trigger_rising = right_trigger and not right_trigger_was_pressed
        if not left_trigger:
            record_combo_stage = 0
        elif left_trigger_rising:
            record_combo_stage = 1
        if record_combo_stage == 1 and left_trigger and right_trigger_rising:
            record_combo_stage = 2
        if (
            record_combo_stage == 2
            and left_trigger
            and right_trigger
            and pygame.CONTROLLER_BUTTON_LEFTSTICK in newly_pressed
        ):
            recording_combo = True
            record_combo_stage = 0
        left_trigger_was_pressed = left_trigger
        right_trigger_was_pressed = right_trigger
    else:
        record_combo_stage = 0
        left_trigger_was_pressed = False
        right_trigger_was_pressed = False

    for i in newly_pressed:
        name, action = actions.get(i, (None, None))
        if controller and i == pygame.CONTROLLER_BUTTON_B and right_trigger:
            name, action = "B+RT", send_soccer_power_kick
        elif controller and i == pygame.CONTROLLER_BUTTON_A and left_trigger and right_trigger:
            name, action = "A+LT+RT", None  # ambiguous, so neither dance fires
        elif controller and i == pygame.CONTROLLER_BUTTON_A and right_trigger:
            name, action = "A+RT", send_dangerous_dance
        elif controller and i == pygame.CONTROLLER_BUTTON_A and left_trigger:
            name, action = "A+LT", send_soccer_celebration
        print(f"button {i} pressed" + (f" ({name})" if name else ""))
        if recording_combo and i == pygame.CONTROLLER_BUTTON_LEFTSTICK:
            asyncio.create_task(asyncio.to_thread(toggle_video_recording))
            continue
        if controller and i == pygame.CONTROLLER_BUTTON_RIGHTSHOULDER:
            asyncio.create_task(toggle_chase(notify_user=False))
            continue
        if not action or not robot_ip:
            continue
        mode_buttons = controller_mode_buttons if controller else raw_mode_buttons
        if i in mode_buttons and any(pad_axes):
            print("ignored: sticks are deflected")  # don't change mode mid-walk
            continue
        asyncio.create_task(asyncio.to_thread(run_button_action, action, name, time.perf_counter()))
    buttons_down = now_down

    # d-pad is the head only; it must never touch the walking sticks below
    global head_dir
    if controller:
        hat = (
            int(controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_RIGHT))
            - int(controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_LEFT)),
            int(controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_UP))
            - int(controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_DOWN)),
        )
        if hat == (0, 0) and pad.get_numhats() > 0:
            hat = pad.get_hat(0)
    else:
        hat = pad.get_hat(0) if pad and pad.get_numhats() > 0 else (0, 0)
    if hat != head_dir:
        head_dir = hat
        print(f"d-pad (head): yaw={hat[0]} pitch={hat[1]}")
        if head:
            head.set_dir(*hat)

    if pad and pad.get_numaxes() >= 4:
        raw_axes = tuple(pad.get_axis(i) for i in range(4))
        if not axes_armed:
            # SDL reports unread axes as -1 until the first axis event; wait for a neutral reading
            axes_armed = all(abs(v) < deadzone for v in raw_axes)
            raw_axes = (0.0, 0.0, 0.0, 0.0)
        axes = tuple(v if abs(v) >= deadzone else 0.0 for v in raw_axes)
    else:
        axes = (0.0, 0.0, 0.0, 0.0)

    prev, pad_axes = pad_axes, axes
    if not any(axes) and not any(prev):
        return

    lx, ly, rx, ry = axes
    now = time.monotonic()
    if any(axes) and now - last_axis_log_at >= 0.5:  # a held stick prints a couple of times a second at most
        print(f"sticks: left=({lx:.2f}, {ly:.2f}) right=({rx:.2f}, {ry:.2f})")
        last_axis_log_at = now
    if not sender:
        return
    if lx or ly:
        sender.set(*map_stick(-ly, -lx, max_mag))
    else:
        sender.release_move()
    if rx:
        sender.set_yaw(map_yaw(-rx, max_yaw))
    else:
        sender.release_yaw()

def start_sender(ip):
    global sender
    if sender:
        sender.shutdown()

    sender = Sender(f"{ip}:50051", 50, 1.0)
    sender.start()
    start_head(ip)

def extract_battery(msg: bytes):
    """Return battery value from a status (id 10000) message, or None."""
    try:
        top = parse(msg)
        if (1, 0, 10000) not in top:
            return None
        for num, wt, v in top:
            if num == 3 and wt == 2:
                for n2, wt2, v2 in parse(v):
                    if n2 == 6 and wt2 == 2:
                        f = {n: val for n, _, val in parse(v2)}
                        if f.get(1) == b"battery" and 3 in f:
                            return struct.unpack("<f", f[3])[0]
    except Exception:
        pass
    return None

def run_button_action(action, name, detected_at):

    action()

def _recv_exact(sock, size):
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("robot closed the connection before sending the full response")
        data.extend(chunk)
    return bytes(data)

def _send_robot_request(msg, label):
    if not robot_ip:
        print("No robot selected")  # also runs on worker threads, where ui.notify has no client
        return False

    started_at = time.perf_counter()
    with socket.create_connection((robot_ip, 6868), timeout=5) as s:
        connected_at = time.perf_counter()
        body = json.dumps(msg, separators=(",", ":")).encode()
        s.sendall(struct.pack("<I", len(body)) + body)
        sent_at = time.perf_counter()
        size = struct.unpack("<I", _recv_exact(s, 4))[0]
        response_text = _recv_exact(s, size).decode()
        finished_at = time.perf_counter()

    try:
        response = json.loads(response_text)
        succeeded = isinstance(response, dict) and response.get("success", True) is not False and not response.get("error")
    except json.JSONDecodeError:
        succeeded = False
    print(
        f"{label}: connect={(connected_at - started_at) * 1000:.1f} ms, "
        f"response={(finished_at - sent_at) * 1000:.1f} ms, "
        f"total={(finished_at - started_at) * 1000:.1f} ms, "
        f"success={succeeded}"
    )
    print(response_text)
    return succeeded

def send_action(action: str, value: int = 0, agent: str = "com.boosterobotics.default"):
    """Send an agent button press. Returns True if the robot answered."""
    msg = {
        "request": "send_to_agent",
        "params": {"agent_id": agent},
        "agent_req": {"event": "on_component_click", "component_id": action, "state": value},
        "language": "en",
        "app_api_level": 10800,
        "app_platform": "Android",
    }
    return _send_robot_request(msg, f"action {action} agent={agent}")

def send_relay_command(command):
    label = {"shoot": "Strong Kick", "dance": "Random Dance", "dangerous_dance": "Dangerous Dance"}[command]
    if not robot_ip:
        print(f"{label} failed: no robot selected")
        return None
    if not STRONG_KICK_TOKEN:
        print(f"{label} disabled: set NUMOTE_STRONG_KICK_TOKEN on the app and robot")
        return None

    timestamp = int(time.time())
    nonce = secrets.token_hex(16)
    signature = hmac.new(
        STRONG_KICK_TOKEN.encode(),
        f"{command}\n{timestamp}\n{nonce}".encode(),
        hashlib.sha256,
    ).hexdigest()
    request = {
        "command": command,
        "timestamp": timestamp,
        "nonce": nonce,
        "signature": signature,
    }
    try:
        with socket.create_connection((robot_ip, STRONG_KICK_PORT), timeout=5) as conn:
            conn.sendall(json.dumps(request, separators=(",", ":")).encode() + b"\n")
            with conn.makefile("rb") as response_stream:
                response_line = response_stream.readline(4097)
        if not response_line:
            raise ConnectionError("robot-action relay closed without a response")
        response = json.loads(response_line)
        succeeded = response.get("success") is True
        if not succeeded:
            print(f"{label} rejected: {response.get('error', 'relay request failed')}")
    except Exception as e:
        print(f"{label} request failed: {e}")
        succeeded = False

    if not succeeded:
        return None
    return response


def send_strong_kick():
    if not send_relay_command("shoot"):
        return False
    print("Strong Kick accepted")
    return True

def confirm_bridge_with_robot(robot_ip):
    """Compare the local bridge's checksum with the robot's copy.

    True: copies match. False: robot's copy differs or is missing, so it needs a transfer.
    None: couldn't tell (no robot selected, SSH failed, or no sha256sum on the robot).
    """
    if not robot_ip:
        print("Bridge check skipped: no robot selected")
        return None

    # compute the checksum of the bridge we have on file (presumed to be latest)
    bridge_path = Path(__file__).with_name("NUmote_bridge.py")
    with open(bridge_path, "rb") as f:
        local_checksum = hashlib.file_digest(f, "sha256").hexdigest() # SHA256 seems alright
    bridge_hashes["local"] = local_checksum[:8]

    remote_path = posixpath.join(SCP_LOCATION, bridge_path.name)
    command = [
        "ssh",
        "-o", "BatchMode=yes",  # no terminal behind the UI, so fail instead of waiting on a password prompt
        "-o", "ConnectTimeout=5",
        f"{ROBOT_SSH_USER}@{robot_ip}",
        f"sha256sum -- {shlex.quote(remote_path)}",
    ]
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.TimeoutExpired) as e:
        print(f"Bridge check failed: {e}")
        bridge_hashes["remote"] = "?"
        return None

    # sha256sum exits 1 when it can't read the file; ssh itself exits 255 and a missing command exits 127
    if result.returncode == 1:
        print(f"Bridge missing or unreadable on {robot_ip}: {result.stderr.strip()}")
        bridge_hashes["remote"] = "missing"
        return False
    if result.returncode != 0 or not result.stdout.strip():
        print(f"Bridge check failed: {result.stderr.strip()}")
        bridge_hashes["remote"] = "?"
        return None

    remote_checksum = result.stdout.split()[0]
    bridge_hashes["remote"] = remote_checksum[:8]
    if remote_checksum != local_checksum:
        print(f"Bridge out of date on {robot_ip}: local {local_checksum}, robot {remote_checksum}")
        return False

    print(f"Bridge up to date on {robot_ip}")
    return True

def transfer_bridge_to_robot(robot_ip):
    # assuming the robot needs a new copy of the bridge script, SCP it to them
    if not robot_ip:
        print("Bridge transfer skipped: no robot selected")
        return False

    bridge_path = Path(__file__).with_name("NUmote_bridge.py")
    command = [
        "scp",
        "-o", "BatchMode=yes",  # no terminal behind the UI, so fail instead of waiting on a password prompt
        "-o", "ConnectTimeout=5",
        str(bridge_path),
        f"{ROBOT_SSH_USER}@{robot_ip}:{SCP_LOCATION}",
    ]
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired) as e:
        print(f"Bridge transfer failed: {e}")
        return False

    if result.returncode != 0:
        print(f"Bridge transfer failed: {result.stderr.strip()}")
        return False

    print(f"Bridge transferred to {robot_ip}:{SCP_LOCATION}")
    return True

def wait_for_bridge_report(listener, nonce, ssh_process, timeout):
    """Wait for the bridge to connect back with this launch's nonce. Gives up early if the launch script fails."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            conn, _ = listener.accept()
        except TimeoutError:
            if ssh_process.poll() not in (None, 0):
                return False
            continue
        with conn:
            conn.settimeout(2)
            try:
                report = conn.recv(256)
            except OSError:
                continue
        if report.strip() == nonce.encode():
            return True
    return False

def start_bridge_on_robot(robot_ip):
    """(Re)start the bridge on the robot. Returns True once the bridge reports back that its ports are open."""
    if not robot_ip:
        print("Bridge start skipped: no robot selected")
        return False

    if not STRONG_KICK_TOKEN:
        print("Bridge starting without NUMOTE_STRONG_KICK_TOKEN: its robot-action relay will be disabled")

    remote_path = shlex.quote(posixpath.join(SCP_LOCATION, "NUmote_bridge.py"))
    log_path = shlex.quote(posixpath.join(SCP_LOCATION, "NUmote_bridge.log"))
    report_nonce = secrets.token_hex(16)  # so only this launch's bridge can confirm the start
    command = [
        "ssh",
        "-o", "BatchMode=yes",  # no terminal behind the UI, so fail instead of waiting on a password prompt
        "-o", "ConnectTimeout=5",
        f"{ROBOT_SSH_USER}@{robot_ip}",
        "bash -s",
    ]
    try:
        listener = socket.create_server(("", 0))  # the bridge connects back here once it's up
    except OSError as e:
        print(f"Bridge start failed: {e}")
        return False

    with listener:
        listener.settimeout(0.5)
        # The env vars match the robot's own camera process, otherwise no frames arrive.
        # Sent over stdin so the token never shows up in a process list, and so pkill can't match this shell.
        # SSH_CONNECTION's first field is this host's address as the robot sees it.
        script = f"""
source /opt/ros/humble/setup.bash || exit 1
source {shlex.quote(VISION_INTERFACE_SETUP)} || exit 1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp RMW_FASTRTPS_USE_QOS_FROM_XML=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/config/dds_configs/single-board_configs/fastdds_profile.xml
export NUMOTE_STRONG_KICK_TOKEN={shlex.quote(STRONG_KICK_TOKEN)}
export NUMOTE_STRONG_KICK_PORT={STRONG_KICK_PORT}
export NUMOTE_BRIDGE_REPORT_ADDR="${{SSH_CONNECTION%% *}}:{listener.getsockname()[1]}"
export NUMOTE_BRIDGE_REPORT_NONCE={report_nonce}

pkill -f 'NUmote_bridge\\.py' && sleep 1  # a running copy would hold the port and may be out of date
nohup python3 {remote_path} {CAMERA_RELAY_PORT} > {log_path} 2>&1 < /dev/null &
bridge_pid=$!

# only watches for a crash, so the host doesn't sit out its whole timeout; the bridge's report is what confirms the start
for _ in $(seq 1 25); do
    if ! kill -0 "$bridge_pid" 2>/dev/null; then
        echo "bridge exited during startup:" >&2
        tail -n 20 {log_path} >&2
        exit 1
    fi
    sleep 1
done
"""
        try:
            ssh_process = subprocess.Popen(
                command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True,
            )
        except OSError as e:
            print(f"Bridge start failed: {e}")
            return False

        try:
            ssh_process.stdin.write(script)
            ssh_process.stdin.close()
        except OSError:
            pass  # ssh already exited; its exit code and stderr are reported below

        reported = wait_for_bridge_report(listener, report_nonce, ssh_process, timeout=30)

    if reported:
        ssh_process.kill()  # the bridge is detached, so this only ends the crash watch
        ssh_process.wait()
        ssh_process.stderr.close()
        print(f"Bridge reported running on {robot_ip}:{CAMERA_RELAY_PORT}")
        return True

    launch_failed = ssh_process.poll() not in (None, 0)
    if not launch_failed:
        ssh_process.kill()
    ssh_process.wait()
    errors = ssh_process.stderr.read().strip()
    ssh_process.stderr.close()
    if launch_failed:
        print(f"Bridge start failed: {errors}")
    else:
        print(
            f"Bridge start unconfirmed: no report from {robot_ip} within 30s. It may still be running. "
            "Check that the robot's copy is current, that this machine accepts inbound connections from it, "
            f"and its log at ~/{posixpath.join(SCP_LOCATION, 'NUmote_bridge.log')}"
        )
    return False

def set_bridge_status(ip, text):
    global bridge_status
    if ip == robot_ip:  # a slower attempt for a previous robot must not overwrite the current status
        bridge_status = text

def start_bridged_mode(robot_ip):
    # regular mode will work on the usual robot <-> app comms
    # bridged mode gives you more features, but relies on a bridge script running on board the robot.

    # returns True once the bridge is running; if not, regular mode is all that's available.

    set_bridge_status(robot_ip, "Checking bridge...")
    bridge_current = confirm_bridge_with_robot(robot_ip)
    if bridge_current is None:  # couldn't reach the robot or read its checksum, so don't touch its copy
        print("Bridged mode unavailable: could not check the robot's bridge")
        set_bridge_status(robot_ip, "Bridge failed: couldn't check the robot's copy")
        return False

    if not bridge_current:
        set_bridge_status(robot_ip, "Copying bridge...")
        if not transfer_bridge_to_robot(robot_ip):
            print("Bridged mode unavailable: could not update the robot's bridge")
            set_bridge_status(robot_ip, "Bridge failed: couldn't copy it to the robot")
            return False

    set_bridge_status(robot_ip, "Starting bridge...")
    started = start_bridge_on_robot(robot_ip)
    set_bridge_status(robot_ip, "Bridge running" if started else "Bridge failed: start not confirmed (see console)")
    return started


bridge_start_lock = threading.Lock()

def refresh_bridge_file(ip):
    """Re-check the robot's bridge against the local copy and SCP it over if they differ. Returns a (message, type) toast."""
    if not bridge_start_lock.acquire(blocking=False):  # a start in progress is already doing this
        return "The bridge is already being checked or started", "warning"
    try:
        current = confirm_bridge_with_robot(ip)
        if current is None:
            return "Couldn't check the robot's bridge", "negative"
        if current:
            return "Bridge is up to date", "positive"
        if not transfer_bridge_to_robot(ip):
            return "Couldn't copy the bridge to the robot", "negative"
        if confirm_bridge_with_robot(ip) is not True:
            return "Copied the bridge but it still doesn't match", "negative"
        set_bridge_status(ip, "Starting bridge...")
        started = start_bridge_on_robot(ip)
        set_bridge_status(ip, "Bridge running" if started else "Bridge failed: start not confirmed (see console)")
        return ("Bridge updated and restarted", "positive") if started else ("Bridge updated but it didn't start (see console)", "negative")
    finally:
        bridge_start_lock.release()

def start_bridged_mode_for_connection(ip):
    # runs on its own thread so connect() doesn't wait on SSH; relay_loop picks the bridge up once it's running.
    with bridge_start_lock:  # a second start would kill the bridge the first is still waiting on
        if ip == robot_ip:  # otherwise another robot was picked while this one waited its turn
            start_bridged_mode(ip)


def send_random_dance(command="dance"):
    response = send_relay_command(command)
    if not response:
        return None
    dance_name = response.get("dance", "random dance")
    print(f"{command} accepted: {dance_name}")
    return dance_name

def send_dangerous_dance():
    return send_random_dance("dangerous_dance")


async def random_dance_from_ui():
    dance_name = await asyncio.to_thread(send_random_dance)
    if dance_name:
        notify(f"Playing {dance_name}")
    else:
        notify("Dance request failed", type="negative")

async def run_robot_action(label, action, *args):
    """Run a blocking robot request off the event loop and report the outcome."""
    try:
        succeeded = await asyncio.to_thread(action, *args)
    except Exception as e:
        print(f"{label} failed: {e}")
        notify(f"{label} failed: {e}", type="negative")
        return
    if succeeded:
        notify(f"{label} sent", type="positive", timeout=1500)
    else:
        notify(f"{label} was rejected", type="negative")

def ensure_agent(agent):
    """Make sure the robot is running this agent, switching to it if needed. Returns True once it is."""
    reply = start_agent(agent)  # replies just_started false when it was already running, so this is always safe
    if not reply.get("success"):
        return False
    if reply.get("result", {}).get("just_started"):
        # switching agents resets the other agent's toggles
        if agent == DEFAULT_AGENT_ID:
            state.update(chasing=False, kicking=False)
        else:
            state.update(waving=False)
        time.sleep(4)  # its buttons show up ~2.7 s after start_agent
    return True

kick_lock = threading.RLock()
kick_timer = None
kick_latched = None  # (label, component id) of the kick currently latched on

def stop_soccer_kick():
    """Release the kick latch; if the robot won't take that, leave the soccer agent instead."""
    global kick_timer, kick_latched
    with kick_lock:
        if kick_timer:
            kick_timer.cancel()
            kick_timer = None
        if not state["kicking"]:
            return True
        state["kicking"] = False
        label, action_id = kick_latched
        kick_latched = None
        try:
            released = send_action(action_id, 1, SOCCER_AGENT_ID)
        except Exception as e:
            print(f"{label} release failed: {e}")
            released = False
        if released:
            print(f"{label} released")
            return True
        print(f"{label} release rejected; leaving the soccer agent")
        try:
            start_agent(DEFAULT_AGENT_ID)
            state.update(chasing=False)
        except Exception as e:
            print(f"Couldn't leave the soccer agent: {e}")
        return False

def latch_soccer_kick(label, action_id):
    """Latch a soccer kick on (starting the soccer agent if needed; chasing is left alone). Pressing any kick again releases it."""
    global kick_timer, kick_latched
    with kick_lock:
        if state["kicking"]:
            return stop_soccer_kick()
        try:
            succeeded = ensure_agent(SOCCER_AGENT_ID) and send_action(action_id, 0, SOCCER_AGENT_ID)
        except Exception as e:
            print(f"{label} request failed: {e}")
            succeeded = False

        if not succeeded:
            return False

        state["kicking"] = True
        kick_latched = (label, action_id)
        kick_timer = threading.Timer(KICK_LATCH_SECONDS, stop_soccer_kick)
        kick_timer.daemon = True
        kick_timer.start()
        print(f"{label} accepted; releasing in {KICK_LATCH_SECONDS}s")
        return True

def send_soccer_kick():
    return latch_soccer_kick("Soccer Kick Ball", SOCCER_KICK_ACTION)

def send_soccer_power_kick():
    return latch_soccer_kick("Soccer Power Kick", SOCCER_POWER_KICK_ACTION)

def send_soccer_celebration():
    """A random soccer-agent celebration (a single click, starting the soccer agent if needed)."""
    label, action_id = random.choice(SOCCER_CELEBRATIONS)
    print(f"soccer celebration selected: {label} ({action_id})")
    try:
        succeeded = ensure_agent(SOCCER_AGENT_ID) and send_action(action_id, 0, SOCCER_AGENT_ID)
    except Exception as e:
        print(f"{label} request failed: {e}")
        return False
    if succeeded:
        print(f"{label} accepted")
    return succeeded

def send_robot_mode(mode: int):
    """Switch between STAND and WALK through the robot's app control endpoint."""
    msg = {
        "request": "set_robot_mode",
        "params": {"mode_value": mode},
        "language": "en",
        "app_api_level": 10800,
        "app_platform": "Android",
    }
    succeeded = _send_robot_request(msg, f"robot mode {mode}")
    if succeeded:
        state["walking"] = mode == 2  # only flip once the robot has the command
    return succeeded

def toggle_robot_mode():
    return send_robot_mode(1 if state["walking"] else 2)

def toggle_wave():
    # "state" is the toggle's current position, like the official app's switch:
    # 0 (currently off) starts the wave, 1 (currently on) stops it
    # wave lives on the default agent, which isn't running after the soccer agent (chase / kick) was started
    succeeded = ensure_agent(DEFAULT_AGENT_ID) and send_action("wave", 1 if state["waving"] else 0, DEFAULT_AGENT_ID)
    if succeeded:
        state["waving"] = not state["waving"]  # only flip once the robot has the command
    return succeeded

def start_agent(agent: str):
    """Switch the robot to another agent. Returns the robot's reply, e.g. {"result":{"just_started":true},"success":true}."""
    msg = {"request": "start_agent", "params": {"id": agent},
       "language": "en", "app_api_level": 10800, "app_platform": "Android"}
    body = json.dumps(msg, separators=(",", ":")).encode()

    with socket.create_connection((robot_ip, 6868), timeout=5) as s:
        s.sendall(struct.pack("<I", len(body)) + body)
        n = struct.unpack("<I", s.recv(4))[0]
        reply = s.recv(n).decode()
        print(reply)
    return json.loads(reply)

async def toggle_chase(notify_user: bool = True):
    def report(message, notification_type="info"):
        print(f"chase: {message}")
        if notify_user:
            notify(message, type=notification_type)

    # same toggle as wave, but chase_ball lives on the soccer agent
    if not robot_ip:
        report("No robot selected", "warning")
        return
    if not state["chasing"]:
        # starting an already running agent replies just_started false, so this is safe to send every time
        try:
            reply = await asyncio.to_thread(start_agent, SOCCER_AGENT_ID)
        except Exception as e:
            report(f"Couldn't start the Soccer agent: {e}", "negative")
            return
        if not reply.get("success"):
            report("Couldn't start the Soccer agent", "negative")
            return
        if reply.get("result", {}).get("just_started"):
            state["waving"] = False  # the default agent's toggles reset when it stops
            report("Switching to Soccer agent...")
            await asyncio.sleep(4)  # its buttons show up ~2.7 s after start_agent
    try:
        succeeded = await asyncio.to_thread(
            send_action,
            "chase_ball",
            1 if state["chasing"] else 0,
            SOCCER_AGENT_ID,
        )
    except Exception as e:
        report(f"Chase Ball request failed: {e}", "negative")
        return
    if succeeded:
        state["chasing"] = not state["chasing"]
        report("Chase Ball started" if state["chasing"] else "Chase Ball stopped")
    else:
        report("Chase Ball request was rejected", "negative")

def print_heartbeat():
    link = "up" if robot_alive() else "down"
    bridge = "up" if relay_active() else "down"
    pad_name = controller_status if controller_layout != "none" else "none"
    print(f"heartbeat: robot={robot_ip or 'none'} link={link} bridge={bridge} controller={pad_name}")

def start_discovery():
    global discovery_sock
    discovery_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    discovery_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    discovery_sock.bind(("", discovery_port))  # all interfaces, so broadcasts arrive
    discovery_sock.setblocking(False)  # polled from the event loop, must never block

def find_booster_ip(sock: socket.socket):
    # performs a sort of crude discovery by filtering some data out of the UDP traffic
    global ip_list
    while True:  # drain everything that arrived since the last poll
        try:
            data, addr = sock.recvfrom(1024)

            if "Booster" in str(data) and addr[0] not in ip_list:
                ip_list.append(addr[0])
        except BlockingIOError:
            return

async def ws_loop(url):
    import websockets
    while True:
        try:
            async with websockets.connect(url, max_size=None) as ws:
                async for msg in ws:
                    state["seen"] = time.monotonic()
                    if isinstance(msg, bytes):
                        if (b := extract_battery(msg)) is not None:
                            state["battery"] = b
                        elif not relay_active() and (j := extract_jpeg(msg)):
                            on_frame(CAMERAS[0], j)
        except asyncio.CancelledError:
            raise
        except Exception as e:
            print(f"ws error ({url}): {e}; retrying in 2s")
            if not relay_active():  # show the no-video frame while disconnected
                clear_frames(CAMERAS[:1])
            await asyncio.sleep(2)

ws_task = None
relay_task = None

def connect(ip):
    global ws_task, relay_task
    for task in (ws_task, relay_task):
        if task:
            task.cancel()
    state.update(battery=None, waving=False, chasing=False, kicking=False, walking=False, seen=0.0, depth_seen=0.0, detections_seen=0.0)
    bridge_hashes["remote"] = ""
    clear_frames(CAMERAS)
    clear_relay_streams()
    ws_task = asyncio.create_task(ws_loop(f"ws://{ip}:{video_stream_port}/"))
    relay_task = asyncio.create_task(relay_loop(ip))
    set_bridge_status(ip, "Checking bridge...")
    threading.Thread(target=start_bridged_mode_for_connection, args=(ip,), daemon=True).start()

# only auto-connect if an IP is hardcoded above; otherwise wait for one to be picked from the menu
app.on_startup(lambda: robot_ip and connect(robot_ip))
app.on_startup(lambda: robot_ip and start_sender(robot_ip))
app.on_startup(start_discovery)
app.on_startup(start_pad)
app.on_shutdown(lambda: head and head.shutdown())
app.on_shutdown(shutdown_video_recording)
app.timer(1 / 30, poll_pad)
app.timer(30.0, print_heartbeat)
app.on_shutdown(lambda: discovery_sock and discovery_sock.close())
app.timer(1.0, lambda: discovery_sock and find_booster_ip(discovery_sock))
app.on_disconnect(lambda: sender and sender.release())
app.on_shutdown(lambda: sender and sender.shutdown())

@ui.page("/")
def index():
    menu_state = {"open": False}  # the robot list isn't rebuilt while it's open, which would close it

    @ui.refreshable
    def ip_menu():
        # rebuilt by refresh() whenever discovery finds a new robot
        with ui.button(icon="lan").props("flat round dense color=grey-4").classes("icon-btn"):
            ui.badge(str(len(ip_list))).props("floating rounded color=amber-8 text-color=black").classes("count-badge")
            ui.tooltip(f"Select robot IP ({robot_ip})" if robot_ip else "Select robot IP")
            with ui.menu().on("show", lambda: menu_state.update(open=True)).on("hide", lambda: menu_state.update(open=False)):
                if not ip_list:
                    ui.item("Searching for robots...").props("disable")
                for address in ip_list:
                    with ui.item(on_click=lambda a=address: reconnect(a)):  # a=address binds this loop's value
                        with ui.item_section().props("avatar").classes("min-w-0"):
                            ui.icon("check" if address == robot_ip else "smart_toy", size="xs", color="amber-5" if address == robot_ip else "grey-6")
                        ui.item_section(address)

    @ui.refreshable
    def mapping_view():
        ui.html(mapping_html(), sanitize=False).classes("w-full")

    def open_mapping():
        mapping_view.refresh()  # rebuilt per open, so it always matches the connected pad
        mapping_dialog.open()

    def refresh():
        if controller_status != last.get("controller"):
            last["controller"] = controller_status
            controller_tooltip.text = f"{controller_status} (click for mapping)"
            controller_button.props(controller_icon_props())
            controller_button.classes(add="icon-on") if controller_layout != "none" else controller_button.classes(remove="icon-on")

        if recording_status != last.get("recording_status"):
            last["recording_status"] = recording_status
            last["recording_status_at"] = time.monotonic()
            recording_display.text = recording_status
        # only the post-recording result (e.g. the save directory) is shown, and only for 10 s
        show_status = (
            recording_status != "Not recording"
            and not recording_active
            and (recording_stopping or time.monotonic() - last.get("recording_status_at", 0) < 10)
        )
        if show_status != last.get("show_recording_status"):
            last["show_recording_status"] = show_status
            recording_display.set_visibility(show_status)
        if recording_active != last.get("recording_active"):
            last["recording_active"] = recording_active
            if recording_active:
                rec_light.classes(add="rec-on")
            else:
                rec_light.classes(remove="rec-on")
        if recording_active:
            elapsed = int(time.monotonic() - recording_started_at)
            record_text = f"STOP {elapsed // 60:02d}:{elapsed % 60:02d}"
        else:
            record_text = "RECORD"
        if record_text != last.get("record_text"):
            last["record_text"] = record_text
            recording_button.text = record_text
        recording_button.classes(add="btn-on" if recording_active else "", remove="" if recording_active else "btn-on")

        if (len(ip_list), robot_ip) != last.get("ip_built") and not menu_state["open"]:  # ip_list only ever grows
            last["ip_built"] = (len(ip_list), robot_ip)
            ip_menu.refresh()

        if robot_ip != last["ip"]:
            last["ip"] = robot_ip
            ip_label.text = robot_ip or "NO ROBOT"
            menu_title.text = robot_ip or "NO ROBOT SELECTED"
            robot_badge.set_visibility(bool(robot_ip))
            robot_button.classes(add="icon-on") if robot_ip else robot_button.classes(remove="icon-on")

        if state["waving"] != last.get("waving"):  # global, so every open page shows the same
            last["waving"] = state["waving"]
            wave_label.text = "STOP WAVE" if state["waving"] else "WAVE"

        dance_ready = relay_active() and bool(STRONG_KICK_TOKEN)  # dance goes through the bridge's authenticated relay
        if dance_ready != last.get("dance"):
            last["dance"] = dance_ready
            dance_item.set_visibility(dance_ready)

        if state["chasing"] != last.get("chasing"):
            last["chasing"] = state["chasing"]
            chase_button.text = "STOP CHASE" if state["chasing"] else "CHASE"
            chase_button.classes(add="btn-on" if state["chasing"] else "", remove="" if state["chasing"] else "btn-on")

        if state["walking"] != last.get("walking"):
            last["walking"] = state["walking"]
            mode_button.text = "STAND" if state["walking"] else "WALK"
            mode_button.classes(add="btn-on" if state["walking"] else "", remove="" if state["walking"] else "btn-on")

        for cam, img in imgs.items():
            if view["mode"] == "DEPTH" and cam == CAMERAS[0]:
                jpeg, count = depth_view_jpeg(), state["depth_count"]
            else:
                jpeg, count = state["jpeg"][cam], state["count"][cam]
            key = (view["mode"] == "DEPTH", count)  # the flag forces a redraw when the mode swaps the source
            if key == last["n"].get(cam):
                continue
            last["n"][cam] = key
            if jpeg:
                img.set_source("data:image/jpeg;base64," + base64.b64encode(jpeg).decode())
            else:
                img.set_source(ASSETS / "no_video.jpg")

        det_key = (view["mode"] == "DETECTIONS", state["detections_count"])
        if det_key != last.get("det"):
            last["det"] = det_key
            imgs[CAMERAS[0]].content = detections_svg(state["detections"]) if det_key[0] else ""
        apply_video_layout()

        conn = connection_summary()
        if conn != last.get("conn"):
            last["conn"] = conn
            robot_button.props(robot_icon_props())
            robot_badge.props(f"color={conn[0]}")
            robot_tooltip.text = conn[1]

        alive = robot_alive()
        if robot_ip != last.get("alive_ip"):  # a different robot starts a fresh history
            last.update(alive_ip=robot_ip, alive=False, had_link=False)
        if alive != last["alive"]:
            last["alive"] = alive
            if alive:
                if last["had_link"]:
                    notify(f"Reconnected to {robot_ip}", type="positive")
                last["had_link"] = True
            elif last["had_link"]:
                notify(f"Lost connection to {robot_ip}", type="negative")

        rows = connection_rows()
        if rows != last.get("rows"):
            last["rows"] = rows
            for name, colour, text in rows:
                dot, status_text = status_rows[name]
                dot.props(f"color={colour}")
                status_text.text = text

        ready = bool(robot_ip)
        if ready != last.get("ready"):
            last["ready"] = ready
            for control in robot_controls:
                control.set_enabled(ready)
        can_record = recording_active or any(state["jpeg"].values())
        if can_record != last.get("can_record"):
            last["can_record"] = can_record
            recording_button.set_enabled(can_record)

        left_shown = depth_view_jpeg() if view["mode"] == "DEPTH" else state["jpeg"][CAMERAS[0]]
        if left_shown:
            bubble = ("", "")
        elif view["mode"] == "DEPTH" and state["jpeg"][CAMERAS[0]]:
            bubble = ("No depth data - the bridge must be running", "amber-5")
        else:
            bubble = (conn[1], conn[0])
        if bubble != last.get("bubble"):
            last["bubble"] = bubble
            no_video_bubble.set_visibility(bool(bubble[0]))
            if bubble[0]:
                icon, style = BUBBLES.get(bubble[1], BUBBLES["grey-6"])
                no_video_label.text = bubble[0]
                reason_icon.props(f"name={icon}")
                reason_bubble.style(replace=style)

        # decode some battery stuff and update the battery label
        b = state["battery"]
        if b != last.get("b", "unset"):
            last["b"] = b
            battery_display.value = 0 if b is None else b / 100
            battery_label.text = "--" if b is None else f"{b:.0f}%"
            battery_tooltip.text = "No battery data" if b is None else f"Battery {b:.0f}% ({'good' if b > 80 else 'ok' if b >= 30 else 'low'})"

            if b is None:  # no robot or no status message yet
                battery_display.style('color: #6b6b6b')
            elif b > 80: # your battery is fine :)
                battery_display.style('color: #38c25d')
            elif b >= 30: # a lil drained
                battery_display.style('color: #cfa923')
            else: # low battery
                battery_display.style('color: #c91c1c')
            battery_display.classes(add="battery-low") if b is not None and b < 30 else battery_display.classes(remove="battery-low")

        # beep in the browser while the battery is critically low
        critical = b is not None and b < CRITICAL_BATTERY
        if critical != last.get("critical"):
            last["critical"] = critical
            ui.run_javascript(f"setLowBatteryAlarm({'true' if critical else 'false'})")

    async def refresh_bridge():
        if not robot_ip:
            notify("No robot selected", type="warning")
            return
        notify("Checking bridge...")
        message, kind = await asyncio.to_thread(refresh_bridge_file, robot_ip)
        notify(message, type=kind)

    async def reconnect(new_ip):
        # change the IP it wants to connect to (video + joystick)
        global robot_ip
        new_ip = new_ip.strip()
        if new_ip:
            robot_ip = new_ip
            save_settings(ip=new_ip)
            connect(new_ip)
            notify(f"Connecting to {new_ip}")
            await asyncio.to_thread(start_sender, new_ip)  # shutdown of old sender blocks briefly

    async def toggle_recording_from_ui():
        await asyncio.to_thread(toggle_video_recording)
        failed = "failed" in recording_status or recording_status.startswith("No video")
        notify(recording_status, type="negative" if failed else "positive")

    def apply_video_layout():
        # the second view stays hidden until it has frames; depth and detections are left camera only
        live = {cam: cam == CAMERAS[0] or (view["mode"] == "RAW" and state["jpeg"][cam] is not None) for cam in CAMERAS}
        if focus["cam"] and not live[focus["cam"]]:
            focus["cam"] = None
        visible = tuple(live[cam] and focus["cam"] in (None, cam) for cam in CAMERAS)
        if visible != last.get("visible"):
            last["visible"] = visible
            for img, shown in zip(imgs.values(), visible):
                img.set_visibility(shown)

    def toggle_focus(cam):
        """Tap a feed to fill the video area with it; tap again to go back to side by side."""
        focus["cam"] = None if focus["cam"] == cam else cam
        apply_video_layout()

    focus = {"cam": None}  # per page, so each viewer picks its own layout
    view = {"mode": VIEWS[0]}  # per page, one of VIEWS

    ui.add_css("""
        .nicegui-content { padding: 0.5rem; }
        @media (min-width: 640px) { .nicegui-content { padding: 1rem; } }

        .amber-btn { color: #FFAE00 !important; }
        .amber-btn.btn-on { background: #FFAE00 !important; color: #1a1a1a !important; }
        .amber-toggle { border: 1px solid #FFAE00; }
        .amber-toggle .q-btn { border-radius: 0; }

        .rec-light { display: inline-block; width: 0.8em; height: 0.8em; margin-left: 0.4em; border: 2px solid currentColor; border-radius: 50%; }
        .rec-light.rec-on { background: #ff2d2d; border-color: #ff2d2d; box-shadow: 0 0 8px #ff2d2d; animation: rec-blink 1s steps(1) infinite; }
        @keyframes rec-blink { 50% { background: transparent; box-shadow: none; } }
        .rec-btn { font-variant-numeric: tabular-nums; }
        .rec-btn .q-btn__content { flex-wrap: nowrap; white-space: nowrap; }
        .rec-btn .rec-light { flex: 0 0 auto; margin-left: 0.3em; }
        .rec-btn.btn-on { background: #2a1111 !important; color: #FFAE00 !important; opacity: 1 !important; border-color: #ff2d2d; }
        .rec-btn.btn-on .q-btn__content { color: #FFAE00 !important; }

        .icon-btn { background: #242424 !important; border: 1px solid #333; transition: background 0.15s, border-color 0.15s, box-shadow 0.15s; }
        .icon-btn:hover { background: #2e2e2e !important; border-color: #FFAE00; }
        .icon-btn.icon-on { border-color: #FFAE0080; box-shadow: 0 0 8px #FFAE0040; }
        .count-badge { top: -4px; right: -4px; font-size: 10px; min-width: 14px; padding: 1px 4px; }
        .speed-pill { display: flex; align-items: center; gap: 0.5rem; height: 2.25rem; padding: 0 0.75rem; background: #242424; border: 1px solid #333; border-radius: 999px; }
        .status-badge { top: -2px; right: -2px; min-width: 10px; min-height: 10px; padding: 0; border: 2px solid #1a1a1a; }
        .battery-low { animation: battery-pulse 1.2s ease-in-out infinite; }
        @keyframes battery-pulse { 50% { opacity: 0.45; } }

        .panel { background: #1a1a1a; border: 1px solid #2e2e2e; border-radius: 14px; }
        .header { padding: 0.5rem 0.75rem; display: flex; flex-direction: column; gap: 0.5rem; }
        .video.panel { padding: 0.5rem; }
        .reason-bubble { max-width: 90%; }
        .page { max-width: 2200px; }

        .mapping-title { font-size: 0.8rem; opacity: 0.7; }
        .mapping { width: 100%; min-width: 600px; height: auto; display: block; font-family: inherit; }
        .mapping .body { fill: url(#padbody); stroke: #3f3f3f; stroke-width: 2; }
        .mapping .ctl { fill: #262626; stroke: #4a4a4a; stroke-width: 1.5; }
        .mapping .ctl.on { fill: #2b2417; stroke: #FFAE00; }
        .mapping .stick { fill: #343434; stroke: #5a5a5a; stroke-width: 1.5; }
        .mapping .btn { fill: #FFAE00; }
        .mapping .btn-t { fill: #161616; font-size: 14px; font-weight: 700; text-anchor: middle; }
        .mapping .lbl { fill: #9a9a9a; font-size: 11px; text-anchor: middle; }
        .mapping .leader { fill: none; stroke: #FFAE00; stroke-opacity: 0.55; stroke-width: 1.2; }
        .mapping .dot { fill: #FFAE00; }
        .mapping .pill { fill: rgba(255, 174, 0, 0.14); stroke: #FFAE00; stroke-width: 1.2; }
        .mapping .cap { fill: #FFAE00; font-size: 13px; font-weight: 600; text-anchor: middle; }
        .mapping .desc { fill: #dcdcdc; font-size: 14px; }
        .mapping .head { fill: #9a9a9a; font-size: 12px; letter-spacing: 0.08em; }
        .map td { padding: 0.15rem 0.75rem; }

        /* phone portrait + desktop: video on top, sticks in the corners below it */
        .stage {
            display: grid; width: 100%; gap: 0.5rem; align-items: center;
            grid-template-areas: "video video" "left right";
            grid-template-columns: 1fr 1fr;
        }
        .stage .video { grid-area: video; }
        .stage .video .frames, .stage .video :is(img, svg) { max-height: 65dvh; object-fit: contain; }
        .stage .video .frames { overflow: hidden; }
        .stage .stick-left { grid-area: left; justify-self: start; padding-left: 40%; }
        .stage .stick-right { grid-area: right; justify-self: end; padding-right: 40%; }
        .stage .stick-left, .stage .stick-right { background-color: transparent; }

        /* phone landscape: gamepad layout, sticks either side of the video */
        @media (orientation: landscape) and (max-height: 500px) {
            .stage { grid-template-areas: "left video right"; grid-template-columns: auto 1fr auto; }
            .stage .video .frames, .stage .video :is(img, svg) { max-height: calc(100dvh - 11rem); }
        }

        /* desktop: video fills the window width, sticks overlaid on its bottom corners */
        @media (min-width: 1024px) and (min-height: 501px) {
            .stage { grid-template-areas: "video"; grid-template-columns: 1fr; }
            .stage .video .frames, .stage .video :is(img, svg) { max-height: calc(100dvh - 13.5rem); }
            .stage .stick-left, .stage .stick-right {
                grid-area: video; align-self: end; z-index: 1;
                width: 200px; height: 200px; padding: 0; margin: 1.5rem;
            }
        }

        /* wide desktop: the header collapses to one row to leave the picture more height */
        @media (min-width: 1280px) {
            .header { flex-direction: row; align-items: center; gap: 1rem; }
        }
        @media (min-width: 1280px) and (min-height: 501px) {
            .stage .video .frames, .stage .video :is(img, svg) { max-height: calc(100dvh - 11.5rem); }
        }
    """)

    with ui.column().classes("page w-full items-stretch mx-auto gap-2 sm:gap-3"):
        with ui.element("div").classes("panel header"):
            # phone: the status chip drops to its own line. desktop: one row, chip fills the middle.
            with ui.row().classes("xl:contents w-full items-center gap-x-2 gap-y-1 sm:gap-x-4"):
                ui.image(source=ASSETS / "numote.png").props("fit=contain").classes("shrink-0 w-12 h-7 sm:w-20 sm:h-12")  # 517x302 logo

                ip_menu()
                robot_button = ui.button().props(f"flat round dense {robot_icon_props()}").classes("icon-btn" + (" icon-on" if robot_ip else ""))
                with robot_button:
                    robot_badge = ui.badge().props("floating rounded color=" + connection_summary()[0]).classes("status-badge")
                    robot_badge.set_visibility(bool(robot_ip))
                    robot_tooltip = ui.tooltip(connection_summary()[1])
                    with ui.menu():
                        with ui.column().classes("gap-1 p-3").style("min-width: 260px"):
                            menu_title = ui.label(robot_ip or "NO ROBOT SELECTED").classes("text-base font-bold tracking-wide")
                            status_rows = {}
                            for name, colour, text in connection_rows():
                                with ui.row().classes("items-center no-wrap gap-2 w-full"):
                                    dot = ui.icon("circle", size="xs", color=colour)
                                    ui.label(name).classes("text-sm opacity-80")
                                    status_text = ui.label(text).classes("text-sm ml-auto text-right")
                                    if name == "Bridge file":
                                        ui.button(icon="refresh", on_click=refresh_bridge).props("flat round dense size=sm color=amber-8").tooltip("Re-check the bridge; copy it and restart if it differs")
                                status_rows[name] = (dot, status_text)
                ip_label = ui.label(robot_ip or "NO ROBOT").classes("max-sm:hidden text-xs font-medium opacity-80 whitespace-nowrap")

                with ui.dialog() as mapping_dialog, ui.card().classes("gap-1").style("max-width: 95vw !important; width: 1000px; overflow-x: auto"):
                    with ui.row().classes("w-full items-center justify-between no-wrap"):
                        ui.label("Controller mapping").classes("text-lg font-medium")
                        ui.button(icon="close", on_click=lambda: mapping_dialog.close()).props("flat round dense")
                    mapping_view()
                controller_button = ui.button(on_click=open_mapping).props(f"flat round dense {controller_icon_props()}").classes("xl:order-2 icon-btn" + (" icon-on" if controller_layout != "none" else ""))
                with controller_button:
                    controller_tooltip = ui.tooltip(f"{controller_status} (click for mapping)")

                with ui.row().classes("items-center no-wrap gap-2 w-28 sm:w-56 xl:order-3"):
                    ui.label("Battery").classes("max-sm:hidden whitespace-nowrap")
                    battery_display = ui.linear_progress(show_value=False, size="20px", color=None).classes("grow rounded")
                    with battery_display:
                        battery_label = ui.label().classes("absolute-center text-xs sm:text-sm text-white")
                    battery_tooltip = ui.tooltip("No battery data")

            with ui.row().classes("xl:contents w-full items-center gap-2"):
                # phone: 2 columns, WALK + speed share the first row. desktop: one inline row.
                with ui.element("div").classes("grid grid-cols-2 gap-2 w-full sm:flex sm:flex-1 sm:w-auto sm:items-stretch"):
                    action = dict(color="#FFAE00")
                    action_classes = "min-w-0 h-9 px-1 text-xs sm:flex-1 sm:min-w-24 sm:px-3 sm:text-sm"
                    mode_button = ui.button(text="WALK", on_click=lambda: run_robot_action("Stand" if state["walking"] else "Walk", toggle_robot_mode), **action).props("dense outline").classes(action_classes + " amber-btn")
                    with ui.element("div").classes("speed-pill min-w-0 sm:w-56 sm:shrink-0"):
                        ui.icon("speed", size="xs", color="amber-8")
                        speed_slider = ui.slider(min=10, max=100, step=5, value=round(max_mag * 100),
                                                on_change=lambda e: set_speed_limit(e.value)).props('color=amber-8 track-size=4px thumb-size=16px thumb-path="M7,3 h6 v14 h-6 z"').classes("grow")
                        ui.label().classes("w-9 text-right text-xs opacity-80").bind_text_from(speed_slider, "value", lambda v: f"{v:.0f}%")
                    chase_button = ui.button(text="CHASE", on_click=toggle_chase, **action).props("dense outline").classes(action_classes + " amber-btn")
                    with ui.dropdown_button("ACTIONS", auto_close=True, **action).props("dense outline").classes(action_classes + " amber-btn") as actions_button:
                        with ui.item(on_click=lambda: run_robot_action("Get up", send_action, "get_up")):
                            ui.item_section("GET UP")
                        with ui.item(on_click=lambda: run_robot_action("Wave", toggle_wave)):
                            wave_label = ui.item_section("WAVE")
                        with ui.item(on_click=lambda: run_robot_action("Kick", send_soccer_kick)):
                            ui.item_section("KICK")
                        with ui.item(on_click=random_dance_from_ui) as dance_item:
                            ui.item_section("DANCE")
                        dance_item.set_visibility(False)  # refresh() shows it once the bridge is up
                    with ui.element("div").classes("col-span-2 flex items-center gap-2 sm:col-span-1 sm:flex-1"):
                        recording_button = ui.button(text="RECORD", on_click=toggle_recording_from_ui, **action).props("dense outline").classes(action_classes + " amber-btn rec-btn grow")
                        with recording_button:
                            rec_light = ui.element("span").classes("rec-light" + (" rec-on" if recording_active else ""))
                        ui.button(icon="folder_open", on_click=open_recordings_folder).props("flat round dense color=white").classes("icon-btn").tooltip("Open recordings folder")

        with ui.element("div").classes("stage"):
            with ui.element("div").classes("panel video w-full flex flex-col gap-2") as video:
                with ui.row().classes("w-full items-center justify-between gap-2"):
                    ui.toggle(list(VIEWS), value=view["mode"], on_change=lambda e: view.update(mode=e.value)).props("dense unelevated color=transparent text-color=amber-8 toggle-color=amber-8 toggle-text-color=black").classes("amber-toggle")
                    recording_display = ui.label(recording_status).classes("order-last w-full sm:order-none sm:w-auto sm:flex-1 truncate text-xs opacity-80 text-center")
                    recording_display.set_visibility(False)  # refresh() shows it for 10 s after a recording ends
                    ui.button(icon="fullscreen", on_click=lambda: ui.run_javascript(f"getHtmlElement({video.id}).requestFullscreen()")).props("flat round dense color=white").classes("icon-btn")
                with ui.element("div").classes("frames relative w-full flex gap-1"):
                    imgs = {cam: ui.interactive_image().classes("min-w-0 flex-1 cursor-pointer") for cam in CAMERAS}
                    for cam, img in imgs.items():
                        img.on("click", lambda _, cam=cam: toggle_focus(cam))
                    with ui.element("div").classes("absolute inset-x-0 top-6 flex justify-center pointer-events-none") as no_video_bubble:
                        with ui.row().classes("reason-bubble items-center no-wrap gap-2 px-4 py-2 rounded-lg shadow-lg") as reason_bubble:
                            reason_icon = ui.icon("videocam_off", size="sm")
                            no_video_label = ui.label().classes("text-sm")
                    no_video_bubble.set_visibility(False)
            left_stick = ui.joystick(
                color='#FFAE00',
                mode='static',
                size=200,
            ).classes("stick-left")
            right_stick = ui.joystick(
                color='#FFAE00',
                mode='static',
                size=200,
            ).classes("stick-right")

    robot_controls = (actions_button, chase_button, mode_button)
    last = {
        "n": {},
        "ips": len(ip_list),
        "ip": robot_ip,
        "controller": controller_status,
        "recording_status": recording_status,
        "recording_status_at": time.monotonic(),
        "recording_active": recording_active,
    }

    # sender is None until a robot is picked
    left_stick.on_move(lambda e: sender and sender.set(*map_stick(e.y, -e.x, max_mag)))
    left_stick.on_end(lambda e: sender and sender.release_move())
    right_stick.on_move(lambda e: sender and sender.set_yaw(map_yaw(-e.x, max_yaw)))
    right_stick.on_end(lambda e: sender and sender.release_yaw())

    ui.run_javascript("""
        // nipplejs knob; max travel is half the stick size (200 / 2)
        window.moveKnob = (i, x, y) => {
            const knob = document.querySelectorAll(".nipple .front")[i];
            if (!knob) return;
            const m = Math.hypot(x, y);
            const k = m > 1 ? 1 / m : 1;
            knob.style.transform = `translate(${x * k * 100}px, ${y * k * 100}px)`;
        };

        // low battery alarm: three short beeps every 10 s while enabled.
        // browsers keep audio muted until the user interacts, so unlock it on the first click / key press
        window.audioCtx = window.audioCtx || new (window.AudioContext || window.webkitAudioContext)();
        const unlockAudio = () => window.audioCtx.state === "suspended" && window.audioCtx.resume();
        ["pointerdown", "keydown", "touchstart"].forEach(e => document.addEventListener(e, unlockAudio));
        const beep = (start) => {
            const ctx = window.audioCtx, osc = ctx.createOscillator(), gain = ctx.createGain();
            osc.type = "square";
            osc.frequency.value = 1760;
            gain.gain.setValueAtTime(0.0001, start);
            gain.gain.exponentialRampToValueAtTime(0.2, start + 0.01);
            gain.gain.exponentialRampToValueAtTime(0.0001, start + 0.15);
            osc.connect(gain).connect(ctx.destination);
            osc.start(start);
            osc.stop(start + 0.16);
        };
        const beepBurst = () => {
            const t = window.audioCtx.currentTime;
            [0, 0.25, 0.5].forEach(d => beep(t + d));
        };
        window.setLowBatteryAlarm = (on) => {
            clearInterval(window.lowBatteryTimer);
            window.lowBatteryTimer = null;
            if (!on) return;
            beepBurst();
            window.lowBatteryTimer = setInterval(beepBurst, 10000);
        };
    """)

    def show_pad():
        # mirror the physical controller onto the on-screen knobs
        if pad_axes != last.get("pad"):
            last["pad"] = pad_axes
            lx, ly, rx, ry = pad_axes
            ui.run_javascript(f"moveKnob(0,{lx},{ly});moveKnob(1,{rx},{ry})")

    ui.timer(1 / 30, show_pad)

    ui.timer(1 / 30, refresh)

ui.run(port=int(os.environ.get("NUMOTE_WEB_PORT", "8080")), dark=True, title="NUmote", favicon=ASSETS / "fav.png")