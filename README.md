# AGV Testbed — Setup and Handoff Guide

This system runs a fleet of Arduino Alvik AGVs on an 8×8 grid using ROS 2 and micro-ROS over WiFi.

**Current architecture (`juan_code.ino` + `juan_supervisor.py`)**: the Alvik is a
**thin executor** — it knows only one atomic motion command at a time
(`FORWARD_UNTIL_RED`, `RIGHT_UNTIL_COLOR`, `ROTATE_180`, etc.), executes it, and reports
back over ROS when done. All routing intelligence — the full route, what command comes
next, workstation visit sequences — lives on the PC side, in `juan_supervisor.py`. The
robot has no notion of the grid, the mission, or what "next" means; it just does what
it's told and reports the outcome. The PC is the brain, the robot is the hands.

This replaced an earlier design (`Line_Follow_Simple.ino` / `AGV_MULTI_WS_DISPATCH.ino`,
documented further below for reference) where the robot received a full token script up
front and executed the entire route autonomously, including its own turn sequencing and
heading bookkeeping. That approach worked but accumulated a lot of robot-side state
(leg target yaw, multi-phase turn sequences, per-transition timing windows) that was
fragile to tune. The step-by-step supervisor model avoids most of that — each command is
self-contained, so there's no cross-leg drift or timing-window overlap to manage.

---

## Repository layout

```
juan_code/
    juan_code.ino               ← Current sketch flashed to every Alvik
ORACLE_VM/
    juan_supervisor.py          ← Current PC-side brain: sends one command at a time,
                                   waits for the robot to report done, sends the next
    full_route_juan.txt         ← Example full-route command file for juan_supervisor.py
    dispatch_node.py            ← Older ROS 2 node: mission commands → token sequences → Alviks
    route_planner.py            ← Routing engine (no ROS dependency, pure Python)
    agv_robots.yaml             ← Robot names and MAC addresses
    workstations.json           ← Workstation positions on the 8×8 grid
    INSTRUCTIONS.txt            ← Quick-reference launch cheat sheet
AGV_MULTI_WS_DISPATCH/
    AGV_MULTI_WS_DISPATCH.ino   ← Older full sketch (PD + yaw-blend line follower, full
                                   token-script execution on-robot)
Line_Follow_Simple.ino          ← Older sketch: same scaffolding as
                                   AGV_MULTI_WS_DISPATCH.ino but with the simplified
                                   centroid line follower and distance-based marker
                                   clearance
```

---

## `juan_code.ino` + `juan_supervisor.py` — current system

### Command protocol

`juan_code.ino` subscribes to `/agv/cmd` and publishes `/agv/status`. It accepts exactly
one command at a time:

| Command | Behavior |
|---|---|
| `FORWARD_UNTIL_RED` | Line-follow forward until a stable red marker, then advance `ADVANCE_AFTER_DETECT_MS` more and brake |
| `FORWARD_UNTIL_YELLOW` | Same, for yellow |
| `FORWARD_UNTIL_BLUE` | Same, for blue |
| `FORWARD_UNTIL_COLOR` | Same, but accepts any of red/yellow/blue (reports which one) |
| `BACKWARD_UNTIL_COLOR` / `_YELLOW` / `_BLUE` | Same as above, but reverses instead of line-following forward |
| `RIGHT_UNTIL_COLOR` | Rotate in place ~83° right (`RIGHT_TURN_DEG`) |
| `LEFT_UNTIL_COLOR` | Rotate in place ~83° left (`LEFT_TURN_DEG`) |
| `ROTATE_180` | Rotate in place 180° (used to turn around inside a workstation bay) |
| `STOP` | Brake immediately, return to IDLE |
| `RESET_POSE` | Brake, zero odometry (`alvik.reset_pose`) |
| `GET_STATUS` | Publish current state + last detected color/HSV |

While executing, the robot publishes `BUSY <command>`. On completion it publishes
`DETECTED <COLOR> H=.. S=.. V=..`, `TURN COMPLETE`, `STOPPED`, or `POSE_RESET`, followed
by `IDLE`. `juan_supervisor.py` waits for one of `IDLE | STOPPED | POSE_RESET | ERROR`
before sending the next command — this is the synchronization point that makes the
"thin executor" model work.

### Running a route

```bash
cd ~/VRP/ORACLE_VM   # or wherever you cloned the repo

# Run a route from a file (one command per line, '#' comments allowed)
python3 juan_supervisor.py --file full_route_juan.txt

# Run a route_planner-style token sequence (R/L/RED/YELLOW/BLUE only)
python3 juan_supervisor.py --tokens R,RED,RED,L,RED

# Run a few commands directly
python3 juan_supervisor.py FORWARD_UNTIL_RED RIGHT_UNTIL_COLOR FORWARD_UNTIL_RED
```

`full_route_juan.txt` is an example of a full 12-workstation tour ending back at the
depot (`FORWARD_UNTIL_BLUE`). Each workstation visit follows the same shape:

```
FORWARD_UNTIL_YELLOW   # reach entry marker
<L|R>_UNTIL_COLOR      # turn into the bay
FORWARD_UNTIL_YELLOW   # reach the dropoff sticker
ROTATE_180             # face back out
FORWARD_UNTIL_YELLOW   # reach yellow again
<L|R>_UNTIL_COLOR      # turn back onto the main loop
```

### Tuning constants (`juan_code.ino`)

| Constant | Value | Purpose |
|---|---|---|
| `BASE_SPEED` | 50.0 | Forward line-follow speed (RPM) |
| `BACKWARD_SPEED` | 20.0 | Reverse speed (RPM) |
| `KP` | 25.0 | Line-following proportional gain |
| `MAX_CORRECTION` | 20.0 | Max RPM differential from line P-control |
| `STICKER_CROSS_SPEED` | 60.0 | Speed while line sensors briefly lose tape (crossing a sticker) |
| `RIGHT_TURN_DEG` / `LEFT_TURN_DEG` | -83° / +83° | Relative in-place turn angle (slightly under 90° — tuned to compensate observed overshoot) |
| `YAW_TOLERANCE` | 2.0° | Turn settle tolerance |
| `TURN_MIN_SPEED` / `TURN_MAX_SPEED` | 18 / 20 | Turn rotation speed range (RPM) |
| `MARKER_STABLE_SAMPLES` (unused alias) / `RED_STABLE_SAMPLES`, `YELLOW_STABLE_SAMPLES`, `BLUE_STABLE_SAMPLES` | 4, 6, 5 | Consecutive samples required before a color is "confirmed" |
| `CMD_MARKER_IGNORE_MS` | 700 | After receiving a command, ignore color detection for this long — armed fresh per command, so it can't overlap into the next leg |
| `ADVANCE_AFTER_DETECT_MS` | 200 | After a marker is confirmed, keep driving this long before braking (centers the robot over/past the marker) |
| `LOST_LINE_FAILSAFE_MS` | 1500 | If no tape detected for this long while line-following, enter `STATE_ERROR` |

**If a turn over/undershoots 90°**: adjust `RIGHT_TURN_DEG`/`LEFT_TURN_DEG` (currently
83°, i.e. 7° under, tuned empirically).

**If the robot stops short of / past a marker**: adjust `ADVANCE_AFTER_DETECT_MS`.

**If the robot misses a marker or false-triggers on the one it just left**: adjust
`CMD_MARKER_IGNORE_MS` and/or the `*_STABLE_SAMPLES` constants. Because the ignore
window is armed per-command (not per fixed schedule), it doesn't compete with leg
length the way the old `Line_Follow_Simple.ino` time-based windows did.

### Launch order (`juan_code.ino` system)

1. **Start the micro-ROS agent on the PC**

   ```bash
   micro-ros-agent udp4 --port 8888
   ```

2. **Flash `juan_code/juan_code.ino`** to the Alvik via Arduino IDE.
   - Check `WIFI_SSID`, `WIFI_PASSWORD`, `AGENT_IP`, `AGENT_PORT` near the top match your
     network and the PC running the micro-ROS agent (see Network setup below).
   - Place the robot on the grid, facing the direction your route file assumes as
     "forward."

3. **Source ROS 2 and run the supervisor**

   ```bash
   source /opt/ros/jazzy/setup.bash
   cd ~/VRP/ORACLE_VM
   python3 juan_supervisor.py --file full_route_juan.txt
   ```

   The supervisor logs each command sent and each status received
   (`-> FORWARD_UNTIL_RED`, `status: BUSY FORWARD_UNTIL_RED`, ... `status: IDLE`), and
   stops with an error if the robot doesn't report done within 30s of a command.

---

## Older architecture (`Line_Follow_Simple.ino` / `AGV_MULTI_WS_DISPATCH.ino`)

The sections below describe the earlier full-autonomy design, kept for reference. The
robot received a complete token script (`run RED,L,YENTRY,...`) and executed the whole
route on its own, including turn sequencing and heading tracking.

### `Line_Follow_Simple.ino` vs `AGV_MULTI_WS_DISPATCH.ino`

`Line_Follow_Simple.ino` keeps the same ROS/token/state-machine/color-detection
scaffolding as `AGV_MULTI_WS_DISPATCH.ino`, but:

- **Line following** is reduced to the plain proportional centroid controller from the
  stock Arduino `Line_follower` example (`error = centroid(L,C,R)`, `control = error * KP`,
  no D term, no yaw blend, no intersection blind window).
- **Marker clearance is distance-based, not time-based** (see further below) — this is
  the one behavioral change from the original dispatch sketch's marker-ignore logic.
- Turn sequence (`turnGenericState`) is otherwise unchanged from `AGV_MULTI_WS_DISPATCH.ino`.

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

The sketch hard-codes the network credentials near the top. For `juan_code.ino`:

```cpp
char WIFI_SSID[]     = "AGV_SWARM";      // ← your network SSID
char WIFI_PASSWORD[] = "ISECap123";      // ← your network password
char AGENT_IP[]      = "192.168.1.143";  // ← IP of the PC running the micro-ROS agent
const uint32_t AGENT_PORT = 8888;        // leave as 8888 unless you changed it
```

(`AGV_MULTI_WS_DISPATCH.ino` / `Line_Follow_Simple.ino` have the same four lines, with
`AGENT_IP = "192.168.1.141"` — update whichever sketch you're flashing to match the PC
actually running the micro-ROS agent.)

To find the PC's IP on Linux/WSL:

```bash
ip addr show | grep "inet " | grep -v 127
```

`juan_code.ino` is single-robot: it uses a fixed node name (`agv_alvik_node`) and fixed
topics (`/agv/cmd`, `/agv/status`), so there's no MAC-based identity lookup or
`agv_robots.yaml` entry needed for it. The MAC address / multi-robot ID mapping below
applies only to the older `AGV_MULTI_WS_DISPATCH.ino` / `dispatch_node.py` system.

---

## Alvik MAC addresses and IDs (older multi-robot dispatch system)

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

## Launch order (older multi-robot dispatch system)

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

### `juan_code.ino` / `juan_supervisor.py`

**Supervisor times out waiting for a command ("Timed out waiting for robot to finish")**
→ Robot didn't publish a done marker (`IDLE`/`STOPPED`/`POSE_RESET`/`ERROR`) within 30s.
Check: robot on WiFi? micro-ROS agent running? Is the robot stuck in `STATE_ERROR`
(line lost — see `LOST_LINE_FAILSAFE_MS`)?

**Robot brakes on `STATE_ERROR` mid-route**
→ Line sensors saw no tape for `LOST_LINE_FAILSAFE_MS` (1500ms). Robot drifted off the
tape — check `KP`/`MAX_CORRECTION`/`BASE_SPEED`, or the grid tape itself near that
intersection.

**Turn overshoots/undershoots, robot ends up off the next line**
→ Adjust `RIGHT_TURN_DEG` / `LEFT_TURN_DEG` (currently ±83°).

**Robot stops too early/late on a marker**
→ Adjust `ADVANCE_AFTER_DETECT_MS`.

### Older multi-robot dispatch system

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
