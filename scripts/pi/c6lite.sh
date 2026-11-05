#!/usr/bin/env bash
# Start the C6-lite loop on the Pi in one tmux session, so a dropped SSH connection does not stop a flight.
#
#   bash scripts/pi/c6lite.sh                       gateway + panel simulator, logs in data/c4_pi
#   bash scripts/pi/c6lite.sh data/c4_pi2           same, other log folder
#   bash scripts/pi/c6lite.sh data/c4_real 192.168.1.20   gateway + the real Arduino panel at that address
#
# Windows: gateway, panel (simulator only), drone. The drone window has the fly command typed in;
# check the drone is on its take-off spot, then press Enter. After landing stop the panel window first,
# then the gateway (Ctrl+C in each), so the panel watchdog does not log a fault.
# Reattach after a dropped connection with:  tmux attach -t c6lite
set -euo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
VENV="${AGRI_VENV:-$HOME/agri_venv}"
OUT="${1:-data/c4_pi}"
PANEL_HOST="${2:-}"
PY="$VENV/bin/python"
SESSION=c6lite

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "tmux session $SESSION is already running: tmux attach -t $SESSION"
    exit 1
fi

cd "$REPO"
mkdir -p "$OUT"
hold='; echo "Stopped. Press Enter to close this window."; read -r _'
if [ -n "$PANEL_HOST" ]; then
    tmux new-session -d -s "$SESSION" -n gateway \
        "$PY -m agri_loop.link.udp_nodes --role gateway --out '$OUT' --host-panel '$PANEL_HOST'$hold"
else
    tmux new-session -d -s "$SESSION" -n gateway "$PY -m agri_loop.link.udp_nodes --role gateway --out '$OUT'$hold"
    tmux new-window -t "$SESSION" -n panel "$PY -m agri_loop.link.udp_nodes --role panel --out '$OUT'$hold"
fi
tmux new-window -t "$SESSION" -n drone
tmux send-keys -t "$SESSION:drone" \
    "$PY -m agri_loop.drone.c1_run --fly --passes 4 --length 1.5 --fps-min 5 --beacon 127.0.0.1 --label c6lite_pi"
tmux attach -t "$SESSION"
