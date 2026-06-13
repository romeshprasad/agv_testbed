# AGV Testbed — Setup and Handoff Guide

This system runs a fleet of Arduino Alvik AGVs on an 8×8 grid using ROS 2 and micro-ROS over WiFi. The PC (or VM) runs the dispatch node; each Alvik runs a micro-ROS sketch that receives token sequences and executes them blindly.

---

## Repository layout

```
AGV_MULTI_WS_DISPATCH/
    AGV_MULTI_WS_DISPATCH.ino   ← Original full sketch (PD + yaw-blend line follower)
Line_Follow_Simple.ino          ← Current sketch flashed to every Alvik (see below)
ORACLE_VM/
    dispatch_node.py            ← ROS 2 node: mission commands → token sequences → Alviks
    route_planner.py            ← Routing engine (no ROS dependency, pure Python)
    agv_robots.yaml             ← Robot names and MAC addresses
    workstations.json           ← Workstation positions on the 8×8 grid
    INSTRUCTIONS.txt            ← Quick-reference launch cheat sheet
```

### `Line_Follow_Simple.ino` vs `AGV_MULTI_WS_DISPATCH.ino`

`Line_Follow_Simple.ino` is the version currently flashed to the Alviks. It keeps the
same ROS/token/state-machine/color-detection scaffolding as `AGV_MULTI_WS_DISPATCH.ino`,
but:

- **Line following** is reduced to the plain proportional centroid controller from the
  stock Arduino `Line_follower` example (`error = centroid(L,C,R)`, `control = error * KP`,
  no D term, no yaw blend, no intersection blind window).
- **Marker clearance is distance-based, not time-based** (see next section) — this is the
  one behavioral change from the original dispatch sketch's marker-ignore logic.
- Turn sequence (`turnGenericState`) is otherwise unchanged from `AGV_MULTI_WS_DISPATCH.ino`.

The instructions below (flashing, network setup, MAC IDs, launch order) apply the same way
to `Line_Follow_Simple.ino` — just open/flash that file instead of the one in
`AGV_MULTI_WS_DISPATCH/`.

---

## Prerequisites

### On the PC / VM running the dispatch node

| Requirement | Version tested |
|---|---|
| Ubuntu 24.04 (or WSL2 on Windows) | 24.04 LTS |
| ROS 2 Jazzy | jazzy |
| Python 3.10+ | 3.10 |
| Python packages | `pyyaml` (`pip install pyyaml`) |
| micro-ROS agent | see below |

Install micro-ROS agent (first time only):

```bash
sudo snap install micro-ros-agent
# OR build from source:
# https://micro.ros.arduino.io/
```

### On each Alvik

- Arduino IDE 2.x
- Arduino Alvik library (install via Arduino Library Manager → search "Arduino Alvik")
- micro_ros_arduino library — download the `.zip` for ESP32 from:
  https://github.com/micro-ROS/micro_ros_arduino/releases
  Install via Arduino IDE → Sketch → Include Library → Add .ZIP Library

---

## Network setup

All Alviks and the dispatch PC must be on the **same WiFi network**.

The sketch hard-codes the network credentials. Open `AGV_MULTI_WS_DISPATCH.ino` and change these three lines near the top:

```cpp
char WIFI_SSID[]     = "AGV_SWARM";      // ← your network SSID
char WIFI_PASSWORD[] = "ISECap123";      // ← your network password
char AGENT_IP[]      = "192.168.1.141";  // ← IP of the PC running the micro-ROS agent
const uint32_t AGENT_PORT = 8888;        // leave as 8888 unless you changed it
```

To find the PC's IP on Linux/WSL:

```bash
ip addr show | grep "inet " | grep -v 127
```

---

## Alvik MAC addresses and IDs

Each Alvik is identified by its WiFi MAC address. The mapping is in two places and must match:

**`agv_robots.yaml`** — controls which ROS topics the dispatch node creates:

```yaml
agvs:
  agv_1:
    name: "Alvik1"
    mac_address: "3C:84:27:C2:87:50"
  agv_2:
    name: "Alvik2"
    mac_address: "3C:84:27:C3:E7:DC"
```

**`AGV_MULTI_WS_DISPATCH.ino`** — `getAlvikID()` function (around line 338):

```cpp
int getAlvikID() {
  String mac = WiFi.macAddress();
  mac.toUpperCase();
  if (mac == "3C:84:27:C2:87:50") return 1;   // Alvik1
  if (mac == "3C:84:27:C3:E7:DC") return 2;   // Alvik2
  if (mac == "74:4D:BD:A2:1B:70") return 3;   // Alvik3
  if (mac == "48:CA:43:2E:1D:CC") return 4;   // Alvik4
  return 1;                                    // fallback
}
```

To add a new Alvik:
1. Flash the sketch to it, connect it to WiFi, open Serial Monitor — it will print its MAC address.
2. Add the MAC → ID mapping to `getAlvikID()`.
3. Add the corresponding entry to `agv_robots.yaml` with the matching name `AlvikN`.

---

## Launch order

### 1. Start the micro-ROS agent on the PC

```bash
micro-ros-agent udp4 --port 8888
```

Leave this running in its own terminal. Every Alvik connects to this agent over UDP.

### 2. Flash the Alviks

Open `AGV_MULTI_WS_DISPATCH/AGV_MULTI_WS_DISPATCH.ino` in Arduino IDE.

- Select board: **Arduino Nano ESP32** (or the ESP32 variant your Alvik uses)
- Select the correct COM port
- Flash each Alvik (no changes needed per-unit — the MAC lookup handles identity automatically)

Place each Alvik on its blue start sticker facing **north** (toward the grid). The LED will blink red/off while searching for the blue sticker, then go solid blue once confirmed.

### 3. Source ROS 2 and start the dispatch node

```bash
source /opt/ros/jazzy/setup.bash
cd ~/VRP/ORACLE_VM          # or wherever you cloned the repo
python dispatch_node.py
```

You should see:

```
[INFO] AGV DISPATCH NODE starting
[INFO] Loaded 28 workstations
[INFO] AGVs: ['agv_1', 'agv_2', 'agv_3', 'agv_4']
[INFO] AGV DISPATCH NODE ready
```

### 4. Send a mission

From a second terminal (with ROS 2 sourced):

```bash
# Grid traversal test — serpentine 8×8, counts all 64 red stickers
ros2 topic pub --once /agv_dispatch/command std_msgs/msg/String \
  "{data: 'GRID_TEST agv_1'}"

# Specific workstation route
ros2 topic pub --once /agv_dispatch/command std_msgs/msg/String \
  "{data: 'DISPATCH {\"routes\":[{\"agv\":\"agv_1\",\"workstations\":[\"WS01\",\"WS04\",\"WS07\"]}]}'}"

# Random 3-stop route
ros2 topic pub --once /agv_dispatch/command std_msgs/msg/String \
  "{data: 'RANDOM {\"agv\":\"agv_1\",\"n_stops\":3,\"seed\":42}'}"

# Emergency stop all
ros2 topic pub --once /agv_dispatch/command std_msgs/msg/String \
  "{data: 'STOP_ALL'}"
```

---

## Monitor telemetry

```bash
# Global dispatch status (1 Hz JSON summary of all AGVs)
ros2 topic echo /agv_dispatch/status

# Single Alvik raw telemetry (200 ms — includes yaw, line sensors, color, red_count)
ros2 topic echo /Alvik1_status
ros2 topic echo /Alvik1_color
```

The color topic JSON format:

```json
{"color":"RED","h":0.0,"s":1.000,"v":0.060,"L":784,"C":314,"R":598,
 "red_count":21,"yaw":-287.5,"tgt":102.8,"bot":9.8}
```

- `yaw` — actual IMU heading (degrees, raw -180..+180 from IMU)
- `tgt` — `leg_target_yaw`, the heading reference for the current leg
- `bot` — bottom ToF distance in cm (6–7 cm = over sticker, 9–11 cm = over black tape)
- `red_count` — cumulative red stickers detected this run

---

## Workstation layout (`workstations.json`)

Workstations are defined as pairs of adjacent grid nodes. The grid is 8×8, numbered row-major from the bottom-left:

```
Row 8:  57 58 59 60 61 62 63 64
Row 7:  49 50 51 52 53 54 55 56
...
Row 1:   1  2  3  4  5  6  7  8
```

Depot is node 1 (bottom-left). The AGV starts facing north on the blue sticker at node 1.

To add or change workstations, edit `workstations.json`. Each entry needs:

```json
{"id": "WS01", "between_nodes": [10, 11]}
```

`between_nodes` is the pair of grid nodes the workstation spur sits between.

---

## Marker clearance (position-based, in `Line_Follow_Simple.ino`)

After the robot stops on/near a colored marker (red intersection sticker, yellow
workstation sticker, or after completing a turn), color detection must be briefly
suppressed — otherwise the robot immediately "re-detects" the marker it's still
sitting on top of.

`AGV_MULTI_WS_DISPATCH.ino` did this with **fixed-time ignore windows**
(`MARKER_IGNORE_AFTER_RED_MS`, `EXIT_WORKSTATION_IGNORE_MS`, `YELLOW_ARM_AFTER_RED_MS`,
`MARKER_IGNORE_AFTER_TURN_MS` — 600–1000 ms each). This caused intermittent missed
red/yellow detections: the workstation legs (red→yellow, yellow→yellow, yellow→red,
all ~4.5–5 in) are roughly **half** the length of a normal grid edge (red→red, ~10 in).
A 1000 ms ignore window that's a safe fraction of a 10 in leg can cover most of a 4.5 in
leg, leaving little time to detect the next marker before the robot is told to brake.

`Line_Follow_Simple.ino` replaces all four time-based windows with a **single
distance-based clearance** using odometry (`alvik.get_pose()`):

| Constant | Value | Purpose |
|---|---|---|
| `MARKER_CLEAR_DISTANCE_CM` | 3.8 (~1.5 in) | Distance the robot must travel from the last marker event before color detection resumes |

When a marker event occurs (red detected, `YENTRY`/`EXIT`/`CLEAR` token issued, turn
finished, or a new `run` script starts), `armMarkerClear()` records the current
`(x, y)` pose. `targetColorDetectedStable()` ignores all color readings until the
robot has moved `MARKER_CLEAR_DISTANCE_CM` from that point — regardless of how long
that takes. Because the clearance distance is sized to the marker sticker (0.8 in
diameter) rather than the leg length, it leaves several inches of margin even on the
shortest 4.5 in workstation legs.

The five old `*_IGNORE_*_MS` / `*_AFTER_*_MS` constants and the
`marker_ignore_until_ms` / `pending_marker_ignore_ms` variables are left commented out
in `Line_Follow_Simple.ino` for reference.

**If the robot re-triggers on the marker it just left**: increase
`MARKER_CLEAR_DISTANCE_CM`.

**If the robot still misses a marker on a short leg**: decrease
`MARKER_CLEAR_DISTANCE_CM`, or check `RED_STABLE_SAMPLES`/`YELLOW_STABLE_SAMPLES` and
loop rate (`LOOP_DELAY_MS`) — a faster loop gives more chances to sample the marker
during its (short) dwell time under the sensor.

---

## Tuning constants (in the Arduino sketch)

All tuning is at the top of `AGV_MULTI_WS_DISPATCH.ino`. The values that are currently working best:

| Constant | Value | Purpose |
|---|---|---|
| `BASE_SPEED` | 40.0 | Forward drive speed (RPM) |
| `KP` | 120.0 | Line-following proportional gain |
| `KD` | 8.0 | Line-following derivative gain |
| `MAX_CORRECTION` | 30.0 | Max RPM differential from line PD |
| `DRIVE_TRIM` | 0.0 | Constant RPM offset to cancel motor asymmetry (negative = slow right wheel) |
| `KP_YAW_BLEND` | 1.0 | Heading correction strength on clean tape |
| `KP_YAW_CROSS` | 2.0 | Heading hold strength during sticker blind window |
| `MARKER_BLIND_MS` | 350 | Duration (ms) to suppress line sensors over a sticker |
| `YAW_TOLERANCE` | 3.0 | Degrees within which a turn is considered settled |

**If the robot drifts right** (yaw goes increasingly negative on a straight north leg): make `DRIVE_TRIM` more negative (try -2.0, -3.0, -4.0).

**If the robot oscillates** side-to-side on the tape: reduce `KP` or `KP_YAW_BLEND`.

**If the robot loses the tape after a turn**: `KP_YAW_CROSS` is too low, or the turn is not settling — check that `YAW_TOLERANCE` is reachable (3° is usually fine).

---

## Debugging a route without a robot

```bash
cd ~/VRP/ORACLE_VM
python route_planner.py route WS01 WS04 WS07 --format explain
python route_planner.py random 4 --seed 42 --format json
python route_planner.py route WS02 WS06 --format tokens
```

---

## Common problems

**Alvik LED stays blinking red/off after placing on blue sticker**
→ The color sensor is not seeing the blue sticker. Adjust position — the sensor needs to be roughly centered on the sticker. The robot must stay still for 1.5 seconds for confirmation.

**dispatch_node prints "AGV considered offline"**
→ The Alvik is not publishing status. Check: WiFi connected? micro-ROS agent running? Correct `AGENT_IP` in sketch?

**`loadScript` returns false / robot stays IDLE after `run` command**
→ The token string contains an unknown token. Check the command for typos. Valid tokens: `RED YENTRY YWORK DOCK DWELL EXIT BLUE R L YAW0 CLEAR`.

**Robot spins endlessly during a turn**
→ The target yaw is numerically degenerate (e.g. computed from an un-normalized `leg_target_yaw` that wraps past ±180°). Use `GRID_TEST` to confirm turns work before running full routes.

**`red_count` stops incrementing mid-run**
→ Robot has drifted off the tape. Check `DRIVE_TRIM` — set it so yaw stays flat with `KP_YAW_BLEND=0` on a straight run, then re-enable blend.
