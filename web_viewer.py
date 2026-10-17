import asyncio, base64, struct
from nicegui import app, ui
from robot import Sender, extract_jpeg, map_stick, parse
import json, socket

ip_address = "10.1.168.33" # Gauss, on the lab network
port = 51111 # this seems to be standard for the Boosters.

state = {"jpeg": None, "battery": 0, "count": 0}

sender = None
max_mag = 1

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

def send_get_up():
    msg = {
        "request": "send_to_agent",
        "params": {"agent_id": "com.boosterobotics.default"},
        "agent_req": {"event": "on_component_click", "component_id": "get_up", "state": 0},
        "language": "en",
        "app_api_level": 10800,
        "app_platform": "Android",
    }
    body = json.dumps(msg, separators=(",", ":")).encode()

    with socket.create_connection((ip_address, 6868), timeout=5) as s:
        s.sendall(struct.pack("<I", len(body)) + body)
        n = struct.unpack("<I", s.recv(4))[0]
        print(s.recv(n).decode())
        

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
    state.update(jpeg=None, battery=0, count=0)
    ws_task = asyncio.create_task(ws_loop(f"ws://{ip}:{port}/"))

app.on_startup(lambda: connect(ip_address))
app.on_startup(lambda: start_sender(ip_address))
app.on_disconnect(lambda: sender and sender.release())
app.on_shutdown(lambda: sender and sender.shutdown())

@ui.page("/")
def index():
    def refresh():
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

    async def reconnect():
        # change the IP it wants to connect to (video + joystick)
        new_ip = ip_input.value.strip()
        if new_ip:
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
        .stage .stick-left { grid-area: left; justify-self: start; }
        .stage .stick-right { grid-area: right; justify-self: end; }

        /* phone landscape: gamepad layout, sticks either side of the video */
        @media (orientation: landscape) and (max-height: 500px) {
            .stage { grid-template-areas: "left video right"; grid-template-columns: auto 1fr auto; }
            .stage .video, .stage .video :is(img, svg) { max-height: calc(100dvh - 7rem); }
        }
    """)

    with ui.column().classes("w-full max-w-5xl mx-auto gap-2 sm:gap-4"):
        # phone: logo + battery on one line, IP underneath. desktop: all on one line.
        with ui.row().classes("w-full items-center gap-x-4 gap-y-1"):
            ui.image(source="numote.png").props("fit=contain").classes("shrink-0 w-16 h-10 sm:w-24 sm:h-14")  # 517x302 logo
            ip_input = ui.input(label="Robot IP", value=ip_address).props("dense") \
                .classes("order-last w-full sm:order-none sm:w-48").on("keydown.enter", reconnect)
            with ui.row().classes("flex-1 min-w-0 items-center no-wrap gap-2"):
                ui.label("Battery").classes("max-sm:hidden whitespace-nowrap")
                battery_display = ui.linear_progress(show_value=False, size="20px", color=None).classes("grow rounded")
                with battery_display:
                    battery_label = ui.label().classes("absolute-center text-sm text-white")

        get_up_button = ui.button(text="GET UP", on_click=send_get_up, color="#FFAE00").style("width: 25%")

        with ui.element("div").classes("stage"):
            img = ui.interactive_image().classes("video w-full")
            left_stick = ui.joystick(
                color='#FFAE00',
                mode='static',
                size=200,
            ).classes("stick-left").style('background-color: transparent; padding-left: 40%')
            right_stick = ui.joystick(
                color='#FFAE00',
                mode='static',
                size=200,
            ).classes("stick-right").style('background-color: transparent; padding-right: 40%')

    last = {"n": -1}

    left_stick.on_move(lambda e: sender.set(*map_stick(e.y, -e.x, max_mag)))
    left_stick.on_end(lambda e: sender.release())

    ui.timer(1 / 30, refresh)

ui.run(port=8080, dark=True, title="NUmote", favicon="fav.png")