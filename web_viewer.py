import asyncio, base64, struct
from nicegui import app, ui
from robot import Sender, extract_jpeg, map_stick, map_yaw, parse
import json, socket

robot_ip = "" # empty for now
ip_list = [] # this is the list of discovered Robot IPs

video_stream_port = 51111 # this seems to be standard for the Boosters.
discovery_port = 9000 # found with WireShark

state = {"jpeg": None, "battery": 0, "count": 0, "waving": False, "chasing": False}

sender = None
discovery_sock = None
max_mag = 1
max_yaw = 1

def start_sender(ip):
    global sender
    if sender:
        sender.shutdown()

    sender = Sender(f"{ip}:50051", 50, 1.0)
    sender.start()

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
    body = json.dumps(msg, separators=(",", ":")).encode()

    if not robot_ip:
        ui.notify("No robot selected")
        return False

    with socket.create_connection((robot_ip, 6868), timeout=5) as s:
        s.sendall(struct.pack("<I", len(body)) + body)
        n = struct.unpack("<I", s.recv(4))[0]
        print(s.recv(n).decode())
    return True

def send_robot_mode(mode: int):
    """Send an agent button press. Returns True if the robot answered."""
    msg = {"request": "set_robot_mode", "params": {"mode_value": mode},
       "language": "en", "app_api_level": 10800, "app_platform": "Android"}
    body = json.dumps(msg, separators=(",", ":")).encode()

    if not robot_ip:
        ui.notify("No robot selected")
        return False

    with socket.create_connection((robot_ip, 6868), timeout=5) as s:
        s.sendall(struct.pack("<I", len(body)) + body)
        n = struct.unpack("<I", s.recv(4))[0]
        print(s.recv(n).decode())
    return True

def toggle_wave():
    # "state" is the toggle's current position, like the official app's switch:
    # 0 (currently off) starts the wave, 1 (currently on) stops it
    if send_action("wave", 1 if state["waving"] else 0):
        state["waving"] = not state["waving"]  # only flip once the robot has the command

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

async def toggle_chase():
    # same toggle as wave, but chase_ball lives on the soccer agent
    if not robot_ip:
        ui.notify("No robot selected")
        return
    if not state["chasing"]:
        # starting an already running agent replies just_started false, so this is safe to send every time
        reply = await asyncio.to_thread(start_agent, "com.boosterobotics.soccer")
        if not reply.get("success"):
            ui.notify("Couldn't start the soccer agent")
            return
        if reply.get("result", {}).get("just_started"):
            ui.notify("Switching to soccer agent...")
            await asyncio.sleep(4)  # its buttons show up ~2.7 s after start_agent
    if send_action("chase_ball", 1 if state["chasing"] else 0, "com.boosterobotics.soccer"):
        state["chasing"] = not state["chasing"]

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
    state.update(jpeg=None, battery=0, count=0, waving=False, chasing=False)
    ws_task = asyncio.create_task(ws_loop(f"ws://{ip}:{video_stream_port}/"))

# only auto-connect if an IP is hardcoded above; otherwise wait for one to be picked from the menu
app.on_startup(lambda: robot_ip and connect(robot_ip))
app.on_startup(lambda: robot_ip and start_sender(robot_ip))
app.on_startup(start_discovery)
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
        if len(ip_list) != last["ips"]:  # ip_list only ever grows
            last["ips"] = len(ip_list)
            ip_menu.refresh()

        if state["waving"] != last.get("waving"):  # global, so every open page shows the same
            last["waving"] = state["waving"]
            wave_button.text = "STOP WAVE" if state["waving"] else "WAVE"

        if state["chasing"] != last.get("chasing"):
            last["chasing"] = state["chasing"]
            chase_button.text = "STOP CHASE" if state["chasing"] else "CHASE"

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

            # phone: full-width 5-column row under the header. desktop: inline at the end.
            with ui.element("div").classes("grid grid-cols-5 gap-2 w-full sm:flex sm:w-auto"):
                action = dict(color="#FFAE00")
                action_classes = "min-w-0 px-1 text-xs sm:px-3 sm:text-sm"
                get_up_button = ui.button(text="GET UP", on_click=lambda: send_action("get_up"), **action).props("dense").classes(action_classes)
                wave_button = ui.button(text="WAVE", on_click=toggle_wave, **action).props("dense").classes(action_classes)
                chase_button = ui.button(text="CHASE", on_click=toggle_chase, **action).props("dense").classes(action_classes)
                stand_button = ui.button(text="STAND", on_click=lambda: send_robot_mode(1), **action).props("dense").classes(action_classes)
                walk_button = ui.button(text="WALK", on_click=lambda: send_robot_mode(2), **action).props("dense").classes(action_classes)

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

    last = {"n": -1, "ips": len(ip_list)}

    # sender is None until a robot is picked
    left_stick.on_move(lambda e: sender and sender.set(*map_stick(e.y, -e.x, max_mag)))
    left_stick.on_end(lambda e: sender and sender.release_move())
    right_stick.on_move(lambda e: sender and sender.set_yaw(map_yaw(-e.x, max_yaw)))
    right_stick.on_end(lambda e: sender and sender.release_yaw())

    ui.timer(1 / 30, refresh)

ui.run(port=8080, dark=True, title="NUmote", favicon="fav.png")