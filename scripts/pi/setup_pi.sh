#!/usr/bin/env bash
# One-time setup of the Raspberry Pi 4 base station (Ubuntu Server 24.04).
# Run from anywhere inside the repository:  bash scripts/pi/setup_pi.sh
# It asks for the sudo password for the system packages and the udev rule.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
VENV="${AGRI_VENV:-$HOME/agri_venv}"
URI="${AGRI_CF_URI:-radio://0/100/2M/E7E7E7E715}"
PROTO="${AGRI_AIDECK_PROTO:-nanocockpit}"

echo "== System packages"
sudo apt-get update
# avahi and libnss-mdns let the Pi resolve the NanoCockpit deck's name, aideck.local
sudo apt-get install -y python3-venv python3-dev git tmux libusb-1.0-0 avahi-daemon avahi-utils libnss-mdns

echo "== Crazyradio and Crazyflie USB access"
sudo install -m 644 "$REPO/scripts/pi/99-bitcraze.rules" /etc/udev/rules.d/99-bitcraze.rules
sudo groupadd -f plugdev
sudo usermod -aG plugdev "$USER"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "== Python environment in $VENV"
python3 -m venv "$VENV"
"$VENV/bin/python" -m pip install --upgrade pip
"$VENV/bin/python" -m pip install -r "$REPO/requirements-pi.txt"
"$VENV/bin/python" -m pip install -e "$REPO"

echo "== Shell settings in ~/.bashrc"
grep -q "AGRI_CF_URI" "$HOME/.bashrc" || echo "export AGRI_CF_URI=\"$URI\"" >> "$HOME/.bashrc"
grep -q "AGRI_AIDECK_PROTO" "$HOME/.bashrc" || echo "export AGRI_AIDECK_PROTO=\"$PROTO\"" >> "$HOME/.bashrc"
grep -q "$VENV/bin/activate" "$HOME/.bashrc" || echo "source \"$VENV/bin/activate\"" >> "$HOME/.bashrc"

echo "== Tests"
(cd "$REPO" && "$VENV/bin/python" -m pytest -q)

echo
echo "Done. Log out and back in once (the plugdev group only applies to new logins),"
echo "then plug in the Crazyradio and run:  python -m agri_loop.drone.c1_preflight"
echo "Camera check (drone on, deck on the router Wi-Fi):  avahi-resolve -n aideck.local"
