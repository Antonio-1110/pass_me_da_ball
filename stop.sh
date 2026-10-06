#!/usr/bin/env bash
# Stop the passing machine started with ./run.sh.
#
# Sends Ctrl+C (SIGINT) first so the app shuts down cleanly (camera, dashboard,
# turret), then SIGTERM, and SIGKILL only as a last resort.
#
#   ./stop.sh
set -uo pipefail

pids=$(pgrep -u "$(id -u)" -f -- "-m passer\.app") || { echo "Not running."; exit 0; }

wait_gone() {   # wait up to $1 seconds for every pid to exit
    for _ in $(seq $(( $1 * 10 ))); do
        kill -0 $pids 2>/dev/null || return 0
        sleep 0.1
    done
    return 1
}

echo "Stopping passer (pid $(echo $pids | tr ' ' ','))..."
kill -INT $pids 2>/dev/null
wait_gone 8 && { echo "Stopped."; exit 0; }

echo "Still running, sending SIGTERM..."
kill -TERM $pids 2>/dev/null
wait_gone 3 && { echo "Stopped."; exit 0; }

echo "Forcing it (SIGKILL)..."
kill -KILL $pids 2>/dev/null
wait_gone 2 && { echo "Killed."; exit 0; }
echo "Could not stop pid(s): $pids" >&2
exit 1
