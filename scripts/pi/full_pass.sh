#!/usr/bin/env bash
# Run a whole autonomous test (take-off, passes over the plant, panel tilts out of the way, land, report)
# inside one tmux session, so a dropped SSH connection does not stop a flight.
#
#   bash scripts/pi/full_pass.sh --points data/measure/<run>/points.json --arduino 192.168.1.19
#   bash scripts/pi/full_pass.sh --points data/measure/<run>/points.json --sim --fake-arduino --yes   (dry run)
#
# All arguments go to agri_loop.drone.full_pass. It prints the plan and waits for Enter in the tmux window.
# Ctrl+C there lands the drone. Reattach after a dropped connection with:  tmux attach -t fullpass
# AGRI_ARDUINO_IP, AGRI_AIDECK_IP, AGRI_AIDECK_PROTO and ROS_DOMAIN_ID are passed into the session if set.
# Started from inside tmux ($TMUX set), the script runs directly in the current window instead.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
VENV="${AGRI_VENV:-$HOME/agri_venv}"
PY="$VENV/bin/python"
SESSION=fullpass

cd "$REPO"

if [ -n "${TMUX:-}" ]; then
    exec "$PY" -m agri_loop.drone.full_pass "$@"
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "tmux session $SESSION is already running: tmux attach -t $SESSION"
    exit 1
fi

envargs=()
for v in AGRI_ARDUINO_IP AGRI_AIDECK_IP AGRI_AIDECK_PROTO ROS_DOMAIN_ID; do
    if [ -n "${!v:-}" ]; then envargs+=(-e "$v=${!v}"); fi
done
# The session command is one bash -c with every value passed as a positional argument, so no quoting is needed:
# $1 = repo, $2 = python, the rest = the full_pass arguments.
inner='cd "$1" && py="$2" && shift 2 && "$py" -m agri_loop.drone.full_pass "$@"; echo "Finished. Press Enter to close this window."; read -r _'
tmux new-session -d -s "$SESSION" -n fullpass ${envargs[@]+"${envargs[@]}"} \
    bash -c "$inner" fullpass-inner "$REPO" "$PY" "$@"
tmux attach -t "$SESSION"
