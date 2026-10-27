import asyncio, base64, os, queue, shutil, struct, subprocess, threading, time
from datetime import datetime
from pathlib import Path

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")  # no SDL window, so no focus
import pygame
import pygame._sdl2.controller as sdl_controller
from nicegui import app, ui
from gprc_head import GrpcHead
from robot import Sender, extract_jpeg, map_stick, map_yaw, parse
import json, socket

robot_ip = "" # empty for now
ip_list = [] # this is the list of discovered Robot IPs

video_stream_port = 51111 # this seems to be standard for the Boosters.
discovery_port = 9000 # found with WireShark
KICK_ACTION = os.environ.get("NUMOTE_KICK_ACTION", "kick")

state = {"jpeg": None, "battery": 0, "count": 0, "waving": False}
recording_status = "Not recording"
recording_active = False
recording_stopping = False
recording_lock = threading.Lock()
video_recorder = None

sender = None
discovery_sock = None
max_mag = 1
max_yaw = 1

pad = None
controller = None
controller_status = "No controller connected"
pad_axes = (0.0, 0.0, 0.0, 0.0)  # lx, ly, rx, ry after dead zone
deadzone = 0.15
# SDL joystick button index -> (name, action) on this pad
button_actions = {
    4: ("Y", lambda: send_action("get_up")),
    3: ("X", lambda: toggle_wave()),
    1: ("B", lambda: send_action(KICK_ACTION)),
    10: ("Back", lambda: send_robot_mode(1)),   # STAND
    11: ("Start", lambda: send_robot_mode(2)),  # WALK
}
controller_button_actions = {
    pygame.CONTROLLER_BUTTON_Y: ("Y", lambda: send_action("get_up")),
    pygame.CONTROLLER_BUTTON_X: ("X", lambda: toggle_wave()),
    pygame.CONTROLLER_BUTTON_B: ("B", lambda: send_action(KICK_ACTION)),
    pygame.CONTROLLER_BUTTON_LEFTSTICK: ("L3", None),
    pygame.CONTROLLER_BUTTON_BACK: ("Back", lambda: send_robot_mode(1)),
    pygame.CONTROLLER_BUTTON_START: ("Start", lambda: send_robot_mode(2)),
}
raw_mode_buttons = {10, 11}
controller_mode_buttons = {pygame.CONTROLLER_BUTTON_BACK, pygame.CONTROLLER_BUTTON_START}
buttons_down = set()
head_dir = (0, 0)  # d-pad hat (x, y), each -1/0/1
head = None
record_combo_stage = 0
left_trigger_was_pressed = False
right_trigger_was_pressed = False

class VideoRecorder:
    FPS = 30

    def __init__(self):
        ffmpeg = shutil.which(os.environ.get("NUMOTE_FFMPEG", "ffmpeg"))
        if not ffmpeg:
            raise RuntimeError("FFmpeg was not found; install it or set NUMOTE_FFMPEG")

        output_dir = Path(os.environ.get("NUMOTE_RECORDING_DIR", Path(__file__).resolve().parent / "recordings"))
        output_dir.mkdir(parents=True, exist_ok=True)
        self.path = output_dir / f"numote_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.mp4"
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
                "-c:v", "libx264", "-preset", "ultrafast", "-crf", "23",
                "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(self.path),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
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
            if return_code and not self.error:
                self.error = f"FFmpeg exited with code {return_code}"

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

def toggle_video_recording():
    global video_recorder, recording_status, recording_active, recording_stopping
    with recording_lock:
        if recording_stopping:
            return
        if video_recorder is None:
            if not state["jpeg"]:
                recording_status = "No video stream to record"
                return
            try:
                video_recorder = VideoRecorder()
            except Exception as e:
                recording_status = f"Recording failed: {e}"
                print(recording_status)
                return
            recording_active = True
            recording_status = f"Recording: {video_recorder.path.name}"
            print(recording_status)
            return
        recording_active = False
        recording_stopping = True
        current_recorder = video_recorder
        video_recorder = None
        recording_status = "Finalizing recording..."

    try:
        path = current_recorder.stop()
        recording_status = f"Saved: {path}"
        if current_recorder.dropped_frames:
            recording_status += f" ({current_recorder.dropped_frames} frames dropped)"
        print(recording_status)
    except Exception as e:
        recording_status = f"Recording failed: {e}"
        print(recording_status)
    finally:
        with recording_lock:
            recording_stopping = False
            recording_active = False

def add_recording_frame(frame):
    with recording_lock:
        current_recorder = video_recorder
    if current_recorder:
        current_recorder.add_frame(frame)

def trigger_pressed(value):
    return value > (0.5 if abs(value) <= 1 else 16384)

def shutdown_video_recording():
    global video_recorder, recording_active, recording_stopping
    with recording_lock:
        if video_recorder is None or recording_stopping:
            return
        recording_active = False
        recording_stopping = True
        current_recorder = video_recorder
        video_recorder = None
    try:
        current_recorder.stop()
    except Exception as e:
        print(f"recording shutdown failed: {e}")
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
    global pad, controller, controller_status, pad_axes, buttons_down
    global record_combo_stage, left_trigger_was_pressed, right_trigger_was_pressed
    pygame.event.pump()
    if any(event.type in (pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED) for event in pygame.event.get()):
        pad = None
        controller = None
        buttons_down.clear()

    if pygame.joystick.get_count() == 0:
        pad = None
        controller = None
        controller_status = "No controller connected"
    elif pad is None:
        pad = pygame.joystick.Joystick(0)
        controller = sdl_controller.Controller(0) if sdl_controller.is_controller(0) else None
        mapping_status = "SDL mapping" if controller else "raw button mapping"
        controller_status = f"{pad.get_name()} ({mapping_status})"
        print(f"controller connected: {controller_status}")

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
        print(f"button {i} pressed" + (f" ({name})" if name else ""))
        if recording_combo and i == pygame.CONTROLLER_BUTTON_LEFTSTICK:
            asyncio.create_task(asyncio.to_thread(toggle_video_recording))
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
        axes = tuple(v if abs(v) >= deadzone else 0.0 for v in (pad.get_axis(i) for i in range(4)))
    else:
        axes = (0.0, 0.0, 0.0, 0.0)

    prev, pad_axes = pad_axes, axes
    if not any(axes) and not any(prev):
        return

    lx, ly, rx, ry = axes
    print(f"right=({rx:.2f}, {ry:.2f}) left=({lx:.2f}, {ly:.2f})")
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
    worker_started = time.perf_counter()
    print(f"button {name}: worker queue={(worker_started - detected_at) * 1000:.1f} ms")
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
        ui.notify("No robot selected")
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

def send_action(action: str, value: int = 0):
    """Send an agent button press and return the robot's reported success."""
    msg = {
        "request": "send_to_agent",
        "params": {"agent_id": "com.boosterobotics.default"},
        "agent_req": {"event": "on_component_click", "component_id": action, "state": value},
        "language": "en",
        "app_api_level": 10800,
        "app_platform": "Android",
    }
    return _send_robot_request(msg, f"action {action}")

def send_robot_mode(mode: int):
    """Switch between STAND and WALK through the robot's app control endpoint."""
    msg = {
        "request": "set_robot_mode",
        "params": {"mode_value": mode},
        "language": "en",
        "app_api_level": 10800,
        "app_platform": "Android",
    }
    return _send_robot_request(msg, f"robot mode {mode}")

def toggle_wave():
    # "state" is the toggle's current position, like the official app's switch:
    # 0 (currently off) starts the wave, 1 (currently on) stops it
    if send_action("wave", 1 if state["waving"] else 0):
        state["waving"] = not state["waving"]  # only flip once the robot has the command

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
                    if isinstance(msg, bytes):
                        if (b := extract_battery(msg)) is not None:
                            state["battery"] = b
                        elif (j := extract_jpeg(msg)):
                            state["jpeg"] = j
                            state["count"] += 1
                            add_recording_frame(j)
        except asyncio.CancelledError:
            raise
        except Exception as e:
            print(f"ws error ({url}): {e}; retrying in 2s")
            if state["jpeg"]:  # show the no-video frame while disconnected
                state["jpeg"] = None
                state["count"] += 1
            await asyncio.sleep(2)

ws_task = None

def connect(ip):
    global ws_task
    if ws_task:
        ws_task.cancel()
    state.update(jpeg=None, battery=0, count=0, waving=False)
    ws_task = asyncio.create_task(ws_loop(f"ws://{ip}:{video_stream_port}/"))

# only auto-connect if an IP is hardcoded above; otherwise wait for one to be picked from the menu
app.on_startup(lambda: robot_ip and connect(robot_ip))
app.on_startup(lambda: robot_ip and start_sender(robot_ip))
app.on_startup(start_discovery)
app.on_startup(start_pad)
app.on_shutdown(lambda: head and head.shutdown())
app.on_shutdown(shutdown_video_recording)
app.timer(1 / 30, poll_pad)
app.on_shutdown(lambda: discovery_sock and discovery_sock.close())
app.timer(1.0, lambda: discovery_sock and find_booster_ip(discovery_sock))
app.on_disconnect(lambda: sender and sender.release())
app.on_shutdown(lambda: sender and sender.shutdown())

@ui.page("/")
def index():
    @ui.refreshable
    def ip_menu():
        # rebuilt by refresh() whenever discovery finds a new robot
        with ui.dropdown_button('IP Address: ', color="#FFAE00").props("dense").classes("px-2 text-xs sm:px-3 sm:text-sm"):
            for address in ip_list:
                ui.item(address, on_click=lambda a=address: reconnect(a))  # a=address binds this loop's value

    def refresh():
        if controller_status != last.get("controller"):
            last["controller"] = controller_status
            controller_display.text = f"Controller: {controller_status}"

        if recording_status != last.get("recording_status"):
            last["recording_status"] = recording_status
            recording_display.text = recording_status
        if recording_active != last.get("recording_active"):
            last["recording_active"] = recording_active
            recording_button.text = "STOP REC" if recording_active else "RECORD"

        if len(ip_list) != last["ips"]:  # ip_list only ever grows
            last["ips"] = len(ip_list)
            ip_menu.refresh()

        if state["waving"] != last.get("waving"):  # global, so every open page shows the same
            last["waving"] = state["waving"]
            wave_button.text = "STOP WAVE" if state["waving"] else "WAVE"

        if state["count"] != last["n"]:
            last["n"] = state["count"]
            if state["jpeg"]:
                img.set_source("data:image/jpeg;base64," + base64.b64encode(state["jpeg"]).decode())
            else:
                img.set_source("no_video.jpg")

        # decode some battery stuff and update the battery label
        b = state["battery"]
        if b != last.get("b"):
            last["b"] = b
            battery_display.value = b / 100
            battery_label.text = f"{b:.0f}%"

            if b > 80: # your battery is fine :)
                battery_display.style('color: #38c25d')
            elif b >= 30: # a lil drained
                battery_display.style('color: #cfa923')
            else: # low battery
                battery_display.style('color: #c91c1c')

    async def reconnect(new_ip):
        # change the IP it wants to connect to (video + joystick)
        global robot_ip
        new_ip = new_ip.strip()
        if new_ip:
            robot_ip = new_ip
            connect(new_ip)
            ui.notify(f"Connecting to {new_ip}")
            await asyncio.to_thread(start_sender, new_ip)  # shutdown of old sender blocks briefly

    async def toggle_recording_from_ui():
        await asyncio.to_thread(toggle_video_recording)

    ui.add_css("""
        .nicegui-content { padding: 0.5rem; }
        @media (min-width: 640px) { .nicegui-content { padding: 1rem; } }

        /* phone portrait + desktop: video on top, sticks in the corners below it */
        .stage {
            display: grid; width: 100%; gap: 0.5rem; align-items: center;
            grid-template-areas: "video video" "left right";
            grid-template-columns: 1fr 1fr;
        }
        .stage .video { grid-area: video; }
        .stage .video, .stage .video :is(img, svg) { max-height: 65dvh; object-fit: contain; }
        .stage .stick-left { grid-area: left; justify-self: start; padding-left: 40%; }
        .stage .stick-right { grid-area: right; justify-self: end; padding-right: 40%; }
        .stage .stick-left, .stage .stick-right { background-color: transparent; }

        /* phone landscape: gamepad layout, sticks either side of the video */
        @media (orientation: landscape) and (max-height: 500px) {
            .stage { grid-template-areas: "left video right"; grid-template-columns: auto 1fr auto; }
            .stage .video, .stage .video :is(img, svg) { max-height: calc(100dvh - 7rem); }
        }

        /* desktop: video at 80% of the window, sticks overlaid on its bottom corners */
        @media (min-width: 1024px) and (min-height: 501px) {
            .stage { width: 80vw; grid-template-areas: "video"; grid-template-columns: 1fr; }
            .stage .video, .stage .video :is(img, svg) { max-height: calc(100dvh - 7rem); }
            .stage .stick-left, .stage .stick-right {
                grid-area: video; align-self: end; z-index: 1;
                width: 200px; height: 200px; padding: 0; margin: 1.5rem;
            }
            .header { width: 80vw; }
        }
    """)

    with ui.column().classes("w-full max-w-5xl lg:max-w-none items-center mx-auto gap-2 sm:gap-4"):
        # phone: compact logo + IP + battery on one line, action buttons underneath. desktop: all on one line.
        with ui.row().classes("header w-full items-center gap-x-2 gap-y-1 sm:gap-x-4"):
            ui.image(source="numote.png").props("fit=contain").classes("shrink-0 w-12 h-7 sm:w-24 sm:h-14")  # 517x302 logo

            ip_menu()
            
            # phone: small fixed-width bar. desktop: stretches to fill the line.
            with ui.row().classes("w-20 items-center no-wrap gap-2 sm:flex-1 sm:w-auto sm:min-w-24"):
                ui.label("Battery").classes("max-sm:hidden whitespace-nowrap")
                battery_display = ui.linear_progress(show_value=False, size="20px", color=None).classes("grow rounded")
                with battery_display:
                    battery_label = ui.label().classes("absolute-center text-xs sm:text-sm text-white")

            # phone: full-width 4-column row under the header. desktop: inline at the end.
            with ui.element("div").classes("grid grid-cols-6 gap-2 w-full sm:flex sm:w-auto"):
                action = dict(color="#FFAE00")
                action_classes = "min-w-0 px-1 text-xs sm:px-3 sm:text-sm"
                get_up_button = ui.button(text="GET UP", on_click=lambda: send_action("get_up"), **action).props("dense").classes(action_classes)
                wave_button = ui.button(text="WAVE", on_click=toggle_wave, **action).props("dense").classes(action_classes)
                kick_button = ui.button(text="KICK", on_click=lambda: send_action(KICK_ACTION), **action).props("dense").classes(action_classes)
                stand_button = ui.button(text="STAND", on_click=lambda: send_robot_mode(1), **action).props("dense").classes(action_classes)
                walk_button = ui.button(text="WALK", on_click=lambda: send_robot_mode(2), **action).props("dense").classes(action_classes)
                recording_button = ui.button(text="RECORD", on_click=toggle_recording_from_ui, **action).props("dense").classes(action_classes)

            controller_display = ui.label(f"Controller: {controller_status}").classes("w-full truncate text-center text-xs opacity-70")
            recording_display = ui.label(recording_status).classes("w-full truncate text-center text-xs opacity-70")

        with ui.element("div").classes("stage"):
            img = ui.interactive_image().classes("video w-full")
            with img:
                ui.button(icon="fullscreen", on_click=lambda: ui.run_javascript(f"getHtmlElement({img.id}).requestFullscreen()")).props("flat round dense color=white").classes("absolute top-2 right-2")
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

    last = {
        "n": -1,
        "ips": len(ip_list),
        "controller": controller_status,
        "recording_status": recording_status,
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
    """)

    def show_pad():
        # mirror the physical controller onto the on-screen knobs
        if pad_axes != last.get("pad"):
            last["pad"] = pad_axes
            lx, ly, rx, ry = pad_axes
            ui.run_javascript(f"moveKnob(0,{lx},{ly});moveKnob(1,{rx},{ry})")

    ui.timer(1 / 30, show_pad)

    ui.timer(1 / 30, refresh)

ui.run(port=int(os.environ.get("NUMOTE_WEB_PORT", "8080")), dark=True, title="NUmote", favicon="fav.png")