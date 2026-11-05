# Raspberry Pi 4 as the base station

The Pi runs everything in the loop: it flies the Crazyflie over the Crazyradio, records the AI deck camera, runs the gateway and talks to the Arduino panel. A laptop or phone is only used to log in over SSH.

```
Crazyflie --Crazyradio (USB)----> Pi: drone code (cflib, passes, safety)
          --AI deck Wi-Fi (wlan0)-> Pi: camera recorder
                                    Pi: gateway --UDP (eth0, router)--> Arduino panel
Operator --SSH (eth0, router)-----> Pi
```

Why this layout:
- One clock for everything: drone log, camera frames, beacons and gateway events all carry the Pi's time, so latencies and frame positions line up without clock sync.
- The Pi has one Wi-Fi radio, and the camera needs it on the AI deck's own access point. The router (Arduino, SSH) therefore comes in over the Ethernet cable.

## What you need

- Raspberry Pi 4 with Ubuntu Server 24.04 (already set up by Anton), on the group router by Ethernet cable.
- The Crazyradio 2.0 in a Pi USB port.
- The Crazyflie E7E7E7E715 with the Lighthouse deck and the AI deck (Wi-Fi streamer in access point mode, "WiFi streaming example").
- A machine to log in from, on the router's network.

## Step 0: code on GitHub

- The Pi installs from the GitHub repository, so the branch with the current drone code (`drone/position-control`) has to be pushed first.
- On the Pi:

```
git clone https://github.com/Ananth-02/deis-agrivoltaics-group3.git
cd deis-agrivoltaics-group3
git switch drone/position-control
```

## Step 1: install

```
bash scripts/pi/setup_pi.sh
```

It installs `python3-venv`, `tmux` and `libusb`, adds the udev rule for the Crazyradio (`scripts/pi/99-bitcraze.rules`), adds you to the `plugdev` group, creates the Python environment `~/agri_venv` from `requirements-pi.txt`, sets `AGRI_CF_URI` in `~/.bashrc`, and runs the tests.

- Log out and back in once afterwards (the group change only applies to new logins).
- Check: `groups` lists `plugdev`, and `python -c "import cflib, cv2"` gives no error.

## Step 2: radio test, motors off

Drone on the floor, battery in, Crazyradio plugged in:

```
python -m agri_loop.drone.c1_preflight
```

- Check: it reads the battery voltage, lists the Lighthouse and AI decks, and reports Lighthouse positioning with 2 or more base stations.
- If it cannot find or open the radio: replug it, check `lsusb` shows `1915:7777`, and check you logged in again after step 1.

## Step 3: network for the camera

Do this with the Ethernet cable plugged in and working, so you keep SSH access.

1. Find the Pi's Ethernet address on the router (`ip -4 addr show eth0`) and reserve it in the router's DHCP settings. Do the same for the Arduino.
2. Back up the current network file and install the new one:

```
sudo cp /etc/netplan/50-cloud-init.yaml ~/50-cloud-init.yaml.bak
sudo cp scripts/pi/netplan-agri.yaml /etc/netplan/50-cloud-init.yaml
sudo chmod 600 /etc/netplan/50-cloud-init.yaml
sudo netplan try
```

- `netplan try` rolls back by itself after 120 s unless you press Enter, so a mistake cannot lock you out.
- Use the file name that is actually in `/etc/netplan/`. If there are several files, any other `wlan0` entry must go, or the Pi may join the old Wi-Fi instead of the AI deck.
- If the AI deck network has a password, add it as `"WiFi streaming example": {password: "..."}`.
3. Power-cycle the drone, wait for the AI deck network, then check:

```
ip -4 addr show wlan0      # an address 192.168.4.x
ip route                   # default route via eth0, not wlan0
python -m agri_loop.drone.c1_preflight --stream 10
```

- Check: the stream test reports 5 fps or more (14 fps on 20 Oct).

No Ethernet port on the router: use a USB Wi-Fi adapter for the router (`wlan1`) and keep `wlan0` for the AI deck, with the same no-default-route rule.

## Step 4: first flight from the Pi

Always fly inside `tmux`. If the SSH connection drops outside `tmux`, the flight program is killed, the setpoints stop and the Crazyflie falls after its timeout. Inside `tmux` the run carries on and lands normally.

```
tmux new -s fly
python -m agri_loop.drone.c1_run --fly --passes 1 --length 1.0 --fps-min 5 --label pi_hover
```

- Same rules as on the laptop: battery 4.1 V or more at rest, drone on the green patch facing +x, base stations in clear view, power-cycle the drone before camera runs.
- Ctrl+C lands. Detach with Ctrl+B then D; reattach with `tmux attach -t fly`.
- Results land in `data/c1/<date_time>_pi_hover` as before.

## Step 5: C6-lite on the Pi

```
bash scripts/pi/c6lite.sh data/c4_pi
```

- Opens one `tmux` session with three windows: gateway, panel simulator, drone. The drone window has the fly command typed in (4 passes of 1.5 m); press Enter when the drone is ready.
- Switch windows with Ctrl+B then a window number (0 gateway, 1 panel, 2 drone).
- After landing, stop the panel window first, then the gateway (Ctrl+C in each).
- Report:

```
python -m agri_loop.link.c4_report data/c4_pi/gateway.jsonl data/c4_pi/panel.jsonl data/c1/<run>/drone.jsonl
```

## Step 6: real Arduino panel

When the Arduino sketch is ready (listens on UDP 5002, sends PanelState to the Pi's Ethernet address on UDP 5001):

```
bash scripts/pi/c6lite.sh data/c4_real <Arduino address>
```

The panel simulator window is left out (windows are then 0 gateway, 1 drone); the gateway sends to the Arduino instead.

## Later: ROS2

The steps above use the plain UDP tools, which already run on the Pi. The ROS2 version (FR-3) wraps the same pieces as nodes: drone node publishing the position, camera recorder, gateway node (written, untested) and a panel bridge to the Arduino. One decision is open for then: cflib needs numpy 2.2, while ROS2 Jazzy's own Python packages are built against numpy 1.26, so check that `rclpy` works with the newer numpy before the drone node and ROS2 share one Python environment.
