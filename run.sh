#!/usr/bin/env bash
# Start the passing machine on the Pi (headless; dashboard on :8080).
#
#   ./run.sh                  dry run: nothing moves
#   ./run.sh --live           REALLY moves the arm (asks for confirmation)
#   ./run.sh --port 9000      any other passer.app option is passed through
#
# Config: configs/rpi.yaml, or set PASSER_CONFIG=path/to/other.yaml
set -euo pipefail
cd "$(dirname "$0")"

PY=.venv/bin/python
CONFIG=${PASSER_CONFIG:-configs/rpi.yaml}
[ -x "$PY" ] || { echo "No .venv found -- see the README's Raspberry Pi setup." >&2; exit 1; }
[ -f "$CONFIG" ] || { echo "Config not found: $CONFIG" >&2; exit 1; }

port=8080 live=0 prev=""
for a in "$@"; do
    [ "$a" = "--live" ] && live=1
    [ "$prev" = "--port" ] && port=$a
    case $a in --port=*) port=${a#--port=} ;; esac
    prev=$a
done

if [ "$live" = 1 ]; then
    echo "LIVE MODE: the launcher arm WILL move and throw."
    read -r -p "Is the area in front of the launcher clear? Type 'yes' to continue: " ok
    [ "$ok" = "yes" ] || { echo "Aborted."; exit 1; }
fi

ip=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "Dashboard: http://${ip:-localhost}:${port}/   (Ctrl+C to stop)"
exec "$PY" -m passer.app --config "$CONFIG" --headless "$@"
