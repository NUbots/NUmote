#!/usr/bin/env bash
# Starts the NUmote web viewer, moving to the next port if the first is taken.
#   ./numote.sh                    run (starts at $NUMOTE_WEB_PORT, default 8080)
#   ./numote.sh --install-launcher add a NUmote entry with the icon to the applications menu
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
START_PORT="${NUMOTE_WEB_PORT:-8080}"
MAX_TRIES=100

APPS="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
DESKTOP="$APPS/numote.desktop"

launcher_entry() {
    cat <<EOF
[Desktop Entry]
Type=Application
Name=NUmote
Comment=Booster K1 remote
Exec="$DIR/numote.sh"
Icon=$DIR/numote_icon.svg
Terminal=true
Categories=Utility;
EOF
}

if [[ "${1:-}" == "--install-launcher" ]]; then
    mkdir -p "$APPS"
    launcher_entry > "$DESKTOP"
    chmod +x "$DESKTOP"
    echo "Installed $DESKTOP"
    exit 0
fi

# Keeps an installed launcher on the current icon and path after a git pull.
if [[ -f "$DESKTOP" ]] && ! launcher_entry | cmp -s - "$DESKTOP"; then
    launcher_entry > "$DESKTOP" && echo "Updated launcher $DESKTOP"
fi

if [[ -x "$DIR/.venv/bin/python" ]]; then
    PYTHON="$DIR/.venv/bin/python"
else
    PYTHON="python3"
fi

port_free() {
    ! (exec 3<>"/dev/tcp/127.0.0.1/$1") 2>/dev/null
}

port="$START_PORT"
for ((i = 0; i < MAX_TRIES; i++, port++)); do
    port_free "$port" && break
done
if ((i == MAX_TRIES)); then
    echo "No free port found between $START_PORT and $((START_PORT + MAX_TRIES - 1))" >&2
    exit 1
fi

if [[ "$port" != "$START_PORT" ]]; then
    echo "Port $START_PORT is in use, using $port instead"
fi
echo "NUmote: http://localhost:$port"

cd "$DIR"
NUMOTE_WEB_PORT="$port" exec "$PYTHON" web_viewer.py
