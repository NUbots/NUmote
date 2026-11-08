# NUmote

A browser remote for Booster K1 robots: live cameras, driving from on-screen sticks or a gamepad, robot actions and video recording.

![NUmote viewer](docs/viewer_screenshot.png)

## Requirements

- Python 3.11 or newer
- `ssh`, `scp` and `ffmpeg`, with key-based SSH access to the robot
- The epsilon-x wifi network, so robots can be discovered

## Install

`scripts/numote.sh` creates `.venv` and installs `requirements.txt` on first run, and again when they change.

## Run

`scripts/numote.sh` (picks a free port if 8080 is taken), then open http://localhost:8080

Set `NUMOTE_WEB_PORT` to change the port. 

Note: use `lsof -i :<NUMOTE_WEB_PORT>` to return the PIDs running on that port then use `kill -9 <PID>` to terminate zombie processes.

( RECOMMENDED ) `scripts/numote.sh --install-launcher` adds an applications menu entry.

<p align="center">
  <img src="docs/launcher.png" alt="NUmote launcher" width="600">
</p>




## Using the viewer

- **Network icon:** pick a robot.
- **Robot icon:** connection status, and a refresh button to update the bridge on the robot.
- **Controller icon:** shows the controller mapping.
- **Header:** WALK, speed slider, CHASE, ACTIONS and RECORD.

## Controller layout

![Controller layout](docs/controller_layout.png)

<div align="center">

|  | Input | Action |
| :--- | :---: | :--- |
| **Movement** | <kbd>Left stick</kbd> | Walk |
|  | <kbd>Right stick</kbd> | Turn |
|  | <kbd>D-pad</kbd> | Head yaw / pitch |
| **Actions** | <kbd>A</kbd> | Random dance |
|  | <kbd>B</kbd> | Kick |
|  | <kbd>X</kbd> | Toggle wave |
|  | <kbd>Y</kbd> | Get up |
| **Modes** | <kbd>RB</kbd> | Toggle chase |
|  | <kbd>Back</kbd> | Stand |
|  | <kbd>Start</kbd> | Walk |
| **Combos** | <kbd>LT</kbd> + <kbd>A</kbd> | Random soccer celebration |
|  | <kbd>RT</kbd> + <kbd>A</kbd> | Danger dance |
|  | <kbd>RT</kbd> + <kbd>B</kbd> | Power kick |
|  | <kbd>LT</kbd>, <kbd>RT</kbd>, then <kbd>L3</kbd> | Start / stop recording |

</div>

Nintendo Switch controllers have the same actions: ZL/ZR/L/R are LT/RT/LB/RB, Minus/Plus are Back/Start, and face buttons follow the printed label, so A/B and X/Y sit in swapped positions.
