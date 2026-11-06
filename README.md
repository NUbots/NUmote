# NUmote

A browser remote for Booster K1 robots: live cameras, driving from on-screen sticks or a gamepad, robot actions and video recording.

![NUmote viewer](viewer_screenshot.png)

## Requirements

- Python 3.11 or newer
- `ssh`, `scp` and `ffmpeg`, with key-based SSH access to the robot
- The epsilon-x wifi network, so robots can be discovered

## Install

`python3 -m pip install -r requirements.txt`

## Run

`python3 web_viewer.py`, then open http://localhost:8080

Set `NUMOTE_WEB_PORT` to change the port.

## Using the viewer

- **Network icon:** pick a robot.
- **Robot icon:** connection status, and a refresh button to update the bridge on the robot.
- **Controller icon:** shows the controller mapping.
- **Header:** WALK, speed slider, CHASE, ACTIONS and RECORD.

## Controller layout

![Controller layout](controller_layout.png)

## Kick

- **B:** soccer kick. It releases itself after 5 seconds, or press B again.
- **RT + B:** powerful shot. The robot currently rejects it until Booster enables it.
