# Drone integration with the ROS2 mission coordinator

Plan for joining our Crazyflie code (this repository, branch `drone/position-control`) with Anton's ROS2 nodes on the Pi (`~/ros2_ws/src/group3_nodes`). Written 15 Nov 2026 from Anton's hand-off. Target: full automated mission demonstrated and measured by **Fri 27 Nov 2026** (soft deadline), leaving the days before TG3 (Wed 2 Dec) as buffer.

## 1. Starting point

What already works on real hardware:

| Piece | Owner | Status |
|---|---|---|
| `arduino_bridge` + `mission_coordinator` (IDLE, WAIT_CLEAR, CLEAR) with the real panel | Anton | Tested 11 Nov: request to confirmed in 1.50-1.80 s |
| Crazyflie flight from the Pi over the Crazyradio (`vehicle.py`, `mission.py`) | Ananth | First Pi flight 9 Nov: position control on Lighthouse, 2.9 cm sideways error |
| Lighthouse positioning, origin at the green patch, x along the 2.8 m side | Ananth | Working since 20 Oct (Flow deck is faulty and not used) |
| AI deck stream to the Pi (NanoCockpit, 160x160 at 30 fps), `FrameRecorder` | Ananth, Mai-Lill | Recorded 180 frames in flight on 9 Nov |
| Pre-flight checks (battery, decks, base stations, stream) | Ananth | In use |
| Safety checks in flight (link, battery, tilt, supervisor crash, geofence) | Ananth | In use, run at 20 Hz |

This answers three of Anton's open questions already:
- Positioning: Lighthouse (2 base stations), absolute frame, no drift.
- URI: `radio://0/100/2M/E7E7E7E715`, Crazyradio 2.0 on the Pi.
- Flight stack: cflib 0.1.34 in `~/agri_venv`, already flying. Crazyswarm2 is not installed.

## 2. Key decisions

1. **No Crazyswarm2.** Our cflib code already flies on the Pi with tested safety checks. Crazyswarm2 would mean a long C++ build on the Pi 4, a new config format, and (for its cflib backend) cflib inside the ROS2 Python, which clashes on numpy (cflib needs numpy 2.x, Jazzy's packages are built for 1.26). It adds risk without adding anything the mission needs.
2. **cflib never runs inside a ROS2 node.** A small process `cf_server` runs in `~/agri_venv` and owns the radio. A ROS2 node `drone_bridge` (system Python, standard library only) translates between ROS topics and plain-text UDP on localhost. This is the same split Anton already uses for the panel: `drone_bridge` is to `cf_server` what `arduino_bridge` is to the Arduino.
3. **The coordinator stays the only node that decides**, and stays hardware-agnostic. It sends goals on a topic; it never imports cflib or knows the URI. This answers the "who owns go_to" question in favour of a separate bridge.
4. **Two layers of safety.** `cf_server` keeps everything about the drone itself (link, battery, tilt, arena fence, what to do if commands stop). The coordinator keeps everything about the mission (panel confirmation, where the drone is allowed to be). Neither trusts the other blindly.
5. **Anton's tested panel path is the panel path.** Our binary C-ITS layer (`link/gateway.py`, `arduino/panel_node.ino`, ports 5001/5002) is not flashed or used. It is parked as material for Part 2.1 (V2X), where it can later run alongside, fed from `/group3/cf/pose`.

## 3. Architecture

```
                        /group3/mission (plant | panel | abort | land | retry | done)
operator (ros2 topic pub) -----------------------------> mission_coordinator   (decides, 10 Hz tick)
                                                          |     ^       |     ^
                                     /group3/panel/cmd    |     |       |     |  /group3/cf/pose  (PoseStamped, 20 Hz)
                                                          v     |       |     |  /group3/cf/state (String)
                                                 arduino_bridge |       v     |
                                     /group3/panel/state -------+   drone_bridge   (translates, ROS2 Python)
                                                 | UDP 5005                | UDP 127.0.0.1:5010 / 5011
                                                 v                         v
                                          Arduino + servo           cf_server      (agri_venv, cflib, drone safety)
                                                                           | Crazyradio
                                                                           v
                                                                     Crazyflie (Lighthouse)

  /group3/capture (start,n | stop,n): coordinator -> image_saver <- camera_bridge <- AI deck (Wi-Fi)
```

All processes run on the Pi, so `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and `ROS_DOMAIN_ID=3` keep other groups out, and every log carries the Pi's clock.

### New ROS2 topics

| Topic | Type | Publisher -> subscriber | Content |
|---|---|---|---|
| `/group3/cf/pose` | `geometry_msgs/PoseStamped` | drone_bridge -> coordinator, image_saver | Lighthouse frame (`frame_id: "world"`), yaw as a quaternion, 20 Hz |
| `/group3/cf/state` | `std_msgs/String` | drone_bridge -> coordinator | `STATE,vbat`, e.g. `FLYING,3.95` |
| `/group3/cf/cmd` | `std_msgs/String` | coordinator -> drone_bridge | `Takeoff,0.8`, `Goto,1.40,-0.10,0.80`, `Land` |
| `/group3/capture` | `std_msgs/String` | coordinator -> image_saver | `start,3` and `stop,3` (mission number) |

### drone_bridge <-> cf_server protocol (plain text over UDP on localhost)

Same rules as the panel protocol, so the same reasoning applies.

| Direction | Message | Meaning |
|---|---|---|
| bridge -> server (5010) | `Status` | Change nothing, just report. Default after a bridge restart, so a restart never moves the drone. |
| bridge -> server | `Takeoff,z` | Arm, climb to z above the current spot. Ignored if already flying. |
| bridge -> server | `Goto,x,y,z` | Fly to this point at limited speed (0.25 m/s) and hold there. Same target resent = no change. |
| bridge -> server | `Land` | Descend where it is, stop motors, disarm. |
| server -> bridge (5011) | `STATE,x,y,z,yaw,vbat` | e.g. `FLYING,1.40,0.60,0.80,2.1,3.95`. States: `LANDED`, `TAKING_OFF`, `FLYING`, `LANDING`, `NO_CMD`, `FAULT`. |

- The bridge resends the latest command 5 times per second (like `arduino_bridge`). The server sends `STATE` at 20 Hz, but **only while the radio log is fresh** (newest sample < 0.3 s old). A stale drone therefore goes silent instead of reporting an old position, so the coordinator's 1 s staleness rule works end to end.
- If no command arrives for 1 s while airborne, `cf_server` holds its current position (`NO_CMD`); after 5 s it lands. Never wait forever.
- No motor-cut command over the network. Ctrl+C in the `cf_server` window lands; a second Ctrl+C cuts the motors. That window is the human kill switch.

### cf_server (new, about 150 lines, reuses tested code)

- `vehicle.CrazyflieVehicle` as is (or `sim.SimVehicle` with `--sim`, so the whole chain runs without hardware).
- `c1_common.preflight` before arming; refuse take-off below 4.0 V resting (the 9 Nov flight started at 3.86 V).
- Safety checks from `Mission._check` (link, battery, tilt, supervisor), with the geofence changed from the pass line to the arena box (2.8 x 2.0 m minus 0.2 m margin, z below 1.2 m).
- `Goto` slides a position target towards the goal at limited speed and streams `send_position_setpoint` at 20 Hz. This is the same position control that gave 1.9-2.9 cm sideways error in the pass tests. Low-level setpoints also mean that if the Pi or the server dies, the firmware's setpoint watchdog stops the drone instead of letting it hover on its own until the battery runs out.
- Writes `flight_log.csv`, `events.jsonl` and `console.log` to a run folder, as `c1_run` does.
- Optional `--record`: run `FrameRecorder` for the whole flight. This is the fallback if `camera_bridge` is not ready (CAPTURE windows are then cut out afterwards from the coordinator's log by timestamp, same clock).

### drone_bridge (new, about 60 lines)

A copy of `arduino_bridge` with three changes: UDP to `127.0.0.1:5010`, parse `STATE,...` into a `PoseStamped` and a `String`, and default command `Status`. It decides nothing.

## 4. Coordinator extension

The agreed state machine, with one-time actions on entering a state and `tick` only checking conditions. Two small additions: TAKEOFF and LAND, since the drone now takes off and lands by itself.

| State | On entry | Leaves when (checked every tick) |
|---|---|---|
| IDLE | (drone landed) | `plant` or `panel` request -> TAKEOFF |
| TAKEOFF | `Takeoff,Z` | state `FLYING` and height within 0.15 m -> APPROACHING; 10 s -> ABORT |
| APPROACHING | `Goto STAGING` | near(STAGING) for 0.5 s -> WAIT_CLEAR; 15 s -> ABORT |
| WAIT_CLEAR | panel command (this is REQUEST_TILT) | panel confirmed -> FLY_OVER; 5 s -> ABORT |
| FLY_OVER | `Goto OVER_PLANT` | panel lost -> ABORT; near(OVER_PLANT) for 0.5 s -> CAPTURE; 10 s -> ABORT |
| CAPTURE | `start,n` on `/group3/capture` | panel lost -> ABORT; 3 s -> send `stop,n`, RETURN |
| RETURN | `Goto STAGING` | near(STAGING) -> RESTORE_PANEL; 10 s -> ABORT |
| RESTORE_PANEL | panel `Done` | panel reports `SUN_CATCH,45` -> LAND; 5 s -> warn and LAND anyway (drone is already clear) |
| LAND | `Goto HOME`, then `Land` once near HOME | state `LANDED` -> IDLE |
| ABORT | `Goto STAGING`, hover | human: `land` -> LAND, `retry` -> WAIT_CLEAR |

Rules that apply in every state:
- Anton's three rules stay: only the panel's own confirmation counts, messages older than 1 s don't count, never wait forever.
- Pose older than 1 s while airborne -> ABORT. (`cf_server` handles the radio side on its own.)
- `cf/state` showing `LANDED` or `FAULT` during a flying state -> error, IDLE (the drone has already landed itself).
- `abort` from any flying state -> ABORT.
- **The panel is only told `Done` when the drone is landed, or its pose is fresh and outside the panel keep-out box.** One helper, `drone_clear_of_panel()`, guards every `Done`, so this cannot be broken by a new state later.
- With parameter `use_drone:=false` the coordinator behaves exactly as today (IDLE, WAIT_CLEAR, CLEAR), so Anton's panel-only tests keep working.

Positions live as constants at the top of the file, in the Lighthouse frame, measured in the room (section 6). If the take-off pad is placed directly under STAGING, TAKEOFF ends at STAGING, APPROACHING becomes a no-op and LAND is a straight descent.

Later optimisation (not in the first version): send the panel command on entering APPROACHING instead of at staging. The panel then moves while the drone flies, which hides most of the 1.5-1.8 s, and safety is unchanged because FLY_OVER still needs the confirmation.

## 5. Steps

Each step is tested on its own before the next starts. Steps 1 and 3 are mine; step 2 and 5 touch Anton's coordinator, so we do them together.

| # | Step | Test that must pass |
|---|---|---|
| 0 | Agree this design with Anton; measure positions and the panel keep-out box (section 6) | Numbers written into the coordinator constants |
| 1 | `cf_server` (pose only, no commands) + `drone_bridge`. Sim first, then the real drone on the floor, carried by hand | `ros2 topic hz /group3/cf/pose` shows 20 Hz (NFR-6 needs 5); pose follows the drone by hand; switching the drone off makes the topic go silent within 0.3 s |
| 2 | Coordinator subscribes to pose and state, logs them, staleness check. No flying | Killing `cf_server` gives a stale-pose log within 1 s |
| 3 | `cf_server` commands, driven by hand with `ros2 topic pub` (no coordinator): take off, go to STAGING, go to OVER_PLANT with the panel set vertical by hand, back, land | Flies the points within 0.15 m; stopping `drone_bridge` gives `NO_CMD` hold, then landing after 5 s; Ctrl+C lands |
| 4 | One automated move (Anton's step 3): after CLEAR, go over the plant; on `done`, back to staging, then panel `Done`, land | 5 runs without intervention, net up, hand on Ctrl+C |
| 5 | Full state machine from section 4 | Fault tests below |
| 6 | Measured runs for TG3: 10 missions with `ros2 bag record` of all `/group3` topics | Report NFR-1, NFR-3, NFR-6 from the bag (section 7) |
| 7 | `camera_bridge` + `image_saver` (Mai-Lill's step 4) hooked to `/group3/capture` and `/group3/cf/pose` | Each mission folder holds tagged frames with the pose at capture time |

Fault tests for step 5 (each must end with the drone at staging or landed, panel never restored above the drone):
- Arduino powered off during FLY_OVER -> ABORT within about 1 s.
- Panel never confirms -> ABORT 5 s after WAIT_CLEAR.
- `drone_bridge` killed in CAPTURE -> `cf_server` holds, then lands.
- Coordinator killed mid-mission -> `drone_bridge` keeps resending the last goal, drone holds; Ctrl+C in `cf_server` lands.
- Battery below 3.3 V for 1 s -> `cf_server` lands, coordinator sees `LANDING` and goes to IDLE with an error.

Timing against the 27 Nov soft deadline:

| Dates | Steps |
|---|---|
| Mon 16 - Wed 18 Nov | 0 (agree, measure), 1 on the Pi, 2 |
| Thu 19 - Fri 20 Nov | 3 (hand-driven flights) |
| Mon 23 - Tue 24 Nov | 4 and 5 (automated mission, fault tests) |
| Wed 25 - Fri 27 Nov | 6 (10 measured missions) |
| Mon 30 Nov - Wed 2 Dec | buffer, TG3 material |

Step 7 runs in parallel; `cf_server --record` (added in step 3) covers the photos if it slips. If a step slips by more than a day, cut the later optimisation and the full fault list first, never the safety rules.

### Step 1 status and how to run it on the Pi

Written and tested on the simulator (15 Nov): `agri_loop/drone/cf_server.py` and `ros2_ws/src/group3_nodes/group3_nodes/drone_bridge.py`, with tests in `tests/test_cf_server.py` (cf_server alone, and simulator -> cf_server -> drone_bridge -> PoseStamped end to end with stand-ins for ROS2). In this step `cf_server` reports `LANDED` and refuses every flying command; the motors cannot start.

1. Pull the branch into the repository clone on the Pi (`git pull` on `drone/position-control`).
2. Copy the node into Anton's package (tell Anton, it is his workspace), then add `"drone_bridge = group3_nodes.drone_bridge:main",` to `console_scripts` in his `setup.py`:

```
cp ros2_ws/src/group3_nodes/group3_nodes/drone_bridge.py ~/ros2_ws/src/group3_nodes/group3_nodes/
deactivate; cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

3. Three `tmux` windows:

```
# window 0, in ~/agri_venv (simulator first, then without --sim for the real drone)
python -m agri_loop.drone.cf_server --sim
# window 1, system Python
ros2 run group3_nodes drone_bridge
# window 2, system Python
ros2 topic hz /group3/cf/pose
ros2 topic echo /group3/cf/state
```

Pass: `ros2 topic hz` shows about 20 Hz; with the real drone the pose follows it when carried by hand; switching the drone off stops the topic within 0.3 s, and cf_server prints growing `stale_ticks`. Sending `ros2 topic pub --once /group3/cf/cmd std_msgs/String "data: Goto,1.0,0.0,0.8"` makes cf_server print that it refuses the command.

## 6. To measure in the room (before step 3)

All in the Lighthouse frame. Use `python -m agri_loop.drone.measure_points --arduino <Arduino IP>` (in `~/agri_venv`, with cf_server, arduino_bridge and mission_coordinator stopped): it lists 15 points in walking order, records each one by itself once the drone has been still for 2.5 s at a new place (the drone's LEDs flash), moves the panel for the panel points, and writes `points.csv` and `summary.md` (fence, heights, HOME, STAGING, OVER_PLANT, keep-out box, warnings) to `data/measure/<time>/`. `--start <point>` redoes a part.

- HOME: the take-off pad.
- STAGING: beside the plant, outside the panel's swing in every position (handoff example: 0.6 m to the side, 0.8 m up).
- OVER_PLANT: above the pot centre.
- Panel keep-out box: footprint of the panel swing plus 0.15 m, and the height of the panel's top edge when vertical (`Plant_inspection`). OVER_PLANT must be at least 0.3 m above that edge, or the drone hits the panel it asked to move.
- Camera footprint at the chosen height (160x160 frames) to check the whole pot is in view.

Panel facts read from Anton's `panel.ino` (16 Nov): states `SUN_CATCH` (servo 45, after `Done` and after every Arduino reset), `PLANT_INSPECTION` (0), `PANEL_INSPECTION` (90); the reply angle is the servo angle the sketch has commanded, moved 1 deg per loop with a 15 ms delay.

## 7. Requirements and how each is shown

| Requirement | Where it is checked | Measured from |
|---|---|---|
| NFR-3 clearance < 2 s | coordinator: WAIT_CLEAR entry to confirmed | coordinator log / bag, 10 missions. 1.50-1.80 s today; Anton's 2 deg per step change should give about 1 s margin |
| NFR-6 telemetry >= 5 Hz | `/group3/cf/pose` rate and gaps | `ros2 topic hz`, bag timestamps (expect 20 Hz) |
| NFR-1 delivery >= 95% | panel commands sent vs replies; cf commands sent vs `STATE` replies | bridge counters or bag |
| NFR-4 FSM design | coordinator (mission) and cf_server (drone) state machines | state table above, transition log |
| NFR-2 positioning +-4 deg | panel angle report vs target | Arduino replies (unchanged) |

## 8. Risks

- **AI deck stream drops near motor start** (errno 128 seen on 20 Oct). `FrameRecorder` reconnects; start the stream before take-off and check frames arrive during CAPTURE in step 4.
- **Shared Wi-Fi** carries the panel and camera. The radio link and Lighthouse do not use it, so a Wi-Fi drop cannot crash the drone, but it can delay panel confirmation (ABORT then handles it).
- **One radio user.** `cfclient` cannot run while `cf_server` holds the Crazyradio.
- **SSH drops.** Run every node in `tmux`, as for the C1 flights.
- **Arduino reset during a mission.** After any reset (for example a brownout when the servo draws current) `panel.ino` starts in `SUN_CATCH` and writes servo 45 straight away, at full servo speed, before Wi-Fi is back. The coordinator only notices once replies resume (ABORT). This is why OVER_PLANT must sit above the panel's highest point in every position (the vertical top edge plus 0.3 m): then a panel moving on its own spoils the photo but cannot reach the drone. Check that the servo supply cannot pull down the Arduino's supply.
- **Arduino Wi-Fi drop.** `panel.ino` connects only in `setup()`, so after a Wi-Fi drop it stays silent until reset. Safe (the coordinator times out and aborts) but it stops a test session; a reconnect check in `loop()` would fix it.

### Suggestions for Anton's nodes (small, optional)

- `arduino_bridge` sends and then reads in the same 5 Hz tick, so each Arduino reply waits about 0.2 s in the socket before it is published. A separate 50 Hz receive timer (as in `drone_bridge`) makes every panel state 0.2 s fresher and takes about 0.2 s off the NFR-3 latency.
- The measured 1.50-1.80 s is more than the 0.7 s the sketch comment expects for 45 deg. Likely causes: `udp.parsePacket()` on the UNO R4 goes through the ESP32 module and adds time to every loop, plus up to 0.1 s coordinator tick, 0.2 s command resend wait and 0.2 s reply wait. Moving 2 deg per loop and the receive timer together should bring it to about 1 s.
- `panel.ino` holds the Wi-Fi password, so it must not be committed to the public repository. Use an `arduino_secrets.h` that git ignores, as the repository already does for `panel_node.ino`.

## 9. Questions for Anton

1. Is the bridge split (`drone_bridge` + `cf_server`) fine with you, rather than the coordinator calling go_to itself?
2. Can `group3_nodes` move into this GitHub repository (`ros2_ws/src/group3_nodes`) so we both work on one copy? Our old untested `agri_ros` and `agri_msgs` packages would be removed.
3. Are the topic names and `std_msgs/String` text messages for `/group3/cf/state` and `/group3/cf/cmd` fine, to match your panel style?
4. Do you want to write the coordinator extension, or should I draft it and you review?
