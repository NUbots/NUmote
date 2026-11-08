#!/usr/bin/env bash
# Starts the NUmote web viewer, moving to the next port if the first is taken.
#   ./scripts/numote.sh                    run (starts at $NUMOTE_WEB_PORT, default 8080)
#   ./scripts/numote.sh --install-launcher add a NUmote entry with the icon to the applications menu
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
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
Exec="$DIR/scripts/numote.sh"
Icon=$DIR/assets/numote_icon.svg
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

# .venv is rebuilt when python3 changes version (its packages are per-version) and reinstalled when requirements.txt changes.
VENV="$DIR/.venv"
STAMP="$VENV/.numote_stamp"
py_version="$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
req_hash="$(sha256sum "$DIR/requirements.txt" | cut -d' ' -f1)"
if [[ "$(sed -n 1p "$STAMP" 2>/dev/null)" != "$py_version" ]]; then
    echo "Creating virtual environment (Python $py_version)"
    rm -rf "$VENV"
    python3 -m venv "$VENV"
fi
if [[ "$(sed -n 2p "$STAMP" 2>/dev/null)" != "$req_hash" ]]; then
    echo "Installing requirements"
    "$VENV/bin/python" -m pip install -r "$DIR/requirements.txt"
    printf '%s\n%s\n' "$py_version" "$req_hash" > "$STAMP"
fi
# shellcheck disable=SC1091
source "$VENV/bin/activate"

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
NUMOTE_WEB_PORT="$port" exec python numote/web_viewer.py
