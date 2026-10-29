# NUmote

## How to install

`python3 -m pip install -r requirements.txt`

## How to run

`python3 web_viewer.py`

Note: Ensure that you are on the epsilon-x wifi network otherwise you will not be able to see the available robots.

## Controller kicks

Press B for the normal soccer kick. Hold RT and press B for the SDK powerful shot. Release B and press it again to kick again; RB is not needed to re-arm.

The powerful shot runs through `k1_camera_relay.py` on the robot, where the Booster SDK can reach its local DDS service. Start that relay on the robot with the same `NUMOTE_STRONG_KICK_TOKEN` value you set before starting `web_viewer.py` on the laptop. The token is not sent over the network. The relay uses DDS domain 0 and interface `lo` by default; set `NUMOTE_BOOSTER_SDK_DOMAIN_ID` or `NUMOTE_BOOSTER_SDK_NETWORK_INTERFACE` on the robot if your setup differs. This K1 currently rejects `kShoot` with status 501, so RT+B requires Booster to enable that motion in its firmware/configuration.

## Random dance

Press DANCE (or A on the controller) to randomly choose a built-in K1 whole-body routine: Arabic, Michael 1/2/3, Shan He Gu Ren, Gai Ge Chun Feng, or Michael 1+2. Kick-style routines and T2-only dances are excluded. The dance request uses the same authenticated robot relay and token as the strong-kick command.
