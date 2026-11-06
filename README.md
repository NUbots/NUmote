# NUmote

A browser-based remote for Booster K1 robots. It shows the robot's cameras (plus depth and detections when the bridge is running), drives and steers it from on-screen sticks or a gamepad, triggers actions such as wave, chase, kick and dance, and records video.

## Requirements

- Python 3.11 or newer
- `ssh` and `scp` on the laptop, with key-based access to the robot (`booster@<robot ip>` by default; override the user with `NUMOTE_ROBOT_SSH_USER`)
- `ffmpeg` on the laptop for recording (or set `NUMOTE_FFMPEG` to its path)

## How to install

`python3 -m pip install -r requirements.txt`

## How to run

`python3 web_viewer.py`, then open http://localhost:8080 (set `NUMOTE_WEB_PORT` to change the port).

Note: Ensure that you are on the epsilon-x wifi network otherwise you will not be able to see the available robots.

## Using the viewer

- **IP button (network icon):** lists the robots found on the network, with a count of how many. Pick one to connect. The last robot used is reconnected on the next start.
- **Robot button:** its badge shows the overall connection state. Open it for the status of the robot link, bridge, bridge file, video, depth and detections, and press refresh next to "Bridge file" to compare the robot's copy of `NUmote_bridge.py` with the local one. If they differ, it is copied over and the bridge is restarted.
- **Controller button:** opens the controller mapping.
- **Header buttons:** WALK/STAND, a speed limit slider (remembered between runs), CHASE, ACTIONS (get up, wave, kick, dance) and RECORD. The folder button next to RECORD opens the recordings folder.
- **Recording:** while recording, the button shows the elapsed time and a blinking light. When it stops, the save location is shown for 10 seconds.
- A toast appears if the robot stops sending data, and again when it comes back.

## Controller layout

![Controller layout](controller_layout.png)

A and Y are as shown on an Xbox controller; on a Nintendo Switch controller the X and Y labels are swapped to match its face buttons.

## Controller kicks

Press B for the soccer kick. It starts the soccer agent if needed, works while chasing, and releases itself after 5 seconds (press B again to release it sooner). Hold RT and press B for the SDK powerful shot.

The powerful shot runs through `NUmote_bridge.py` on the robot, where the Booster SDK can reach its local DDS service. `web_viewer.py` generates a token and starts the bridge with it over SSH, so no setup is needed; set `NUMOTE_STRONG_KICK_TOKEN` to use a fixed value instead. The token is never sent to the robot by the relay protocol itself, only used to sign requests. The relay uses DDS domain 0 and interface `lo` by default; set `NUMOTE_BOOSTER_SDK_DOMAIN_ID` or `NUMOTE_BOOSTER_SDK_NETWORK_INTERFACE` on the robot if your setup differs. This K1 currently rejects `kShoot` with status 501, so RT+B requires Booster to enable that motion in its firmware/configuration.

## Random dance

Press DANCE (or A on the controller) to randomly choose a built-in K1 whole-body routine: Arabic, Michael 1/2/3, Shan He Gu Ren, Gai Ge Chun Feng, or Michael 1+2. Kick-style routines and T2-only dances are excluded. The dance request uses the same authenticated robot relay and token as the strong-kick command.
