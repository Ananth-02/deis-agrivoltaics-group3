# Raspberry Pi 4 as the base station

The Pi runs everything in the loop: it flies the Crazyflie over the Crazyradio, records the AI deck camera, runs the gateway and talks to the Arduino panel. A laptop or phone is only used to log in over SSH.

```
Crazyflie --Crazyradio (USB)-----------> Pi: drone code (cflib, passes, safety)
AI deck   --router Wi-Fi (TCP 5000)----> Pi: camera recorder
                                         Pi: gateway --UDP (router Wi-Fi)--> Arduino panel
Operator  --SSH (router Wi-Fi)---------> Pi
```

Why this layout:
- One clock for everything: drone log, camera frames, beacons and gateway events all carry the Pi's time, so latencies and frame positions line up without clock sync.
- The AI deck joins the group router as a normal Wi-Fi client, like the Pi and the Arduino. The Pi's single Wi-Fi radio then reaches all of them, and no second network or route rules are needed.

## What you need

- Raspberry Pi 4 with Ubuntu Server 24.04 (already set up by Anton), on the group router's Wi-Fi.
- The Crazyradio 2.0 in a Pi USB port.
- The Crazyflie E7E7E7E715 with the Lighthouse deck and the AI deck running the NanoCockpit streamer (ESP32 built with the router SSID and password in menuconfig, flashed over JTAG). It names itself `aideck.local`.
- A machine to log in from, on the router's network.
- Internet for step 1 only: the router has none, so plug in a phone by USB tethering while installing.

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

It installs `python3-venv`, `tmux`, `libusb` and the mDNS tools (`avahi`, `libnss-mdns`), adds the udev rule for the Crazyradio (`scripts/pi/99-bitcraze.rules`), adds you to the `plugdev` group, creates the Python environment `~/agri_venv` from `requirements-pi.txt`, sets `AGRI_CF_URI` and `AGRI_AIDECK_PROTO=nanocockpit` in `~/.bashrc`, and runs the tests.

- Log out and back in once afterwards (the group change only applies to new logins).
- Check: `groups` lists `plugdev`, and `python -c "import cflib, cv2"` gives no error.

## Step 2: radio test, motors off

Drone on the floor, battery in, Crazyradio plugged in:

```
python -m agri_loop.drone.c1_preflight
```

- Check: it reads the battery voltage, lists the Lighthouse and AI decks, and reports Lighthouse positioning with 2 or more base stations.
- If it cannot find or open the radio: replug it, check `lsusb` shows `1915:7777`, and check you logged in again after step 1.

## Step 3: camera on the router Wi-Fi

Power-cycle the drone and give the AI deck about 10 s to join the router. Then:

```
avahi-resolve -n aideck.local
python -m agri_loop.drone.c1_preflight --stream 10 --no-radio
```

- Check: the first line prints the deck's address, and the stream test reports 5 fps or more with 160 by 160 frames.
- The recorder reads `AGRI_AIDECK_PROTO` (set to `nanocockpit` by step 1) and connects to `aideck.local`. Every command below picks these up, so no `--aideck-ip` is needed.
- If the name does not resolve, set the address by hand for this login: `export AGRI_AIDECK_IP=<address>`. Without access to the router page the address can change after a router restart, so check it again with `avahi-resolve` each lab session.
- The old Bitcraze streamer still works with `--aideck-proto bitcraze` (or `AGRI_AIDECK_PROTO=bitcraze`).

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

When the Arduino sketch is ready (listens on UDP 5002, sends PanelState to the Pi's Wi-Fi address on UDP 5001):

```
bash scripts/pi/c6lite.sh data/c4_real <Arduino address>
```

The panel simulator window is left out (windows are then 0 gateway, 1 drone); the gateway sends to the Arduino instead.

## Later: ROS2

The steps above use the plain UDP tools, which already run on the Pi. The ROS2 version (FR-3) wraps the same pieces as nodes: drone node publishing the position, camera recorder, gateway node (written, untested) and a panel bridge to the Arduino. One decision is open for then: cflib needs numpy 2.2, while ROS2 Jazzy's own Python packages are built against numpy 1.26, so check that `rclpy` works with the newer numpy before the drone node and ROS2 share one Python environment.
