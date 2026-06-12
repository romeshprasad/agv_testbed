# AGV Testbed

An 8×8 grid warehouse testbed demonstrating autonomous guided vehicle (AGV) navigation using an Arduino Alvik robot. The grid simulates a factory floor where AGVs transport goods between workstations — similar to how Amazon warehouse robots operate.

---

## Physical Setup

### Grid Layout

- **8×8 grid** of cells, each cell represents one workstation bay
- **64 nodes** (intersections) marked with **red dot stickers**, numbered row-major from the bottom-left
  - Row 1: nodes 1–8 (bottom), Row 8: nodes 57–64 (top)
- **Edges** between nodes marked with **black tape**
- **Depot** marked with a **blue sticker** at the bottom-left of the grid (node 1 area)

### Workstation Entry Points

Each cell has an imaginary workstation inside it. To enter:

- A **yellow sticker** is placed at the top-center of the cell's top edge (the "entry marker")
- A short **black tape spur** runs perpendicular from that yellow sticker down to the center of the cell
- A second **yellow sticker** at the center marks the **drop-off / pick-up point**

The AGV enters via the yellow entry marker, follows the spur south to the drop-off yellow, reverses in to dock, waits, then exits the same way back to the grid.

**The grid edges and nodes are transit-only — workstation entry always goes through the yellow spur.**

### Workstation Map (12 workstations)

| ID | Name | Entry Node | Col | Row |
|----|------|-----------|-----|-----|
| 1  | WS01 | 9         | 1   | 2   |
| 2  | WS02 | 13        | 5   | 2   |
| 3  | WS03 | 18        | 2   | 3   |
| 4  | WS04 | 19        | 3   | 3   |
| 5  | WS05 | 23        | 7   | 3   |
| 6  | WS06 | 27        | 3   | 4   |
| 7  | WS07 | 33        | 1   | 5   |
| 8  | WS08 | 35        | 3   | 5   |
| 9  | WS09 | 38        | 6   | 5   |
| 10 | WS10 | 42        | 2   | 6   |
| 11 | WS11 | 43        | 3   | 6   |
| 12 | WS12 | 55        | 7   | 8   |

---

## Robot

**Hardware:** Arduino Alvik

**Sensors used:**
- 3-channel line sensor (left / center / right) — follows black tape
- Color sensor (HSV + RGB) — detects red nodes, yellow entry markers, blue depot
- IMU (yaw) — heading hold during turns, sticker crossings, and reverse docking

**Actuators:** Two wheel motors (RPM control), RGB LEDs

---

## Architecture

Two firmware versions exist side by side:

| File | Description |
|------|-------------|
| `agv_coded.ino` | **Original** — fully self-contained, hardcoded mission sequence and map |
| `agv_level2.ino` | **Level 2** — micro-ROS command executor, no map knowledge on the robot |

### Level 2 Architecture

```
┌─────────────────────────────┐        serial / micro-ROS        ┌─────────────────────────────┐
│        ROS2 Host (PC)       │ ────────────────────────────────> │      Arduino Alvik           │
│                             │   /agv/commands                   │                             │
│  mission_node.py            │   UInt8MultiArray                 │  agv_level2.ino             │
│  ─ holds workstation map    │                                   │  ─ command queue executor   │
│  ─ plans paths              │ <──────────────────────────────── │  ─ line following           │
│  ─ dispatches commands      │   /agv/status                     │  ─ color detection          │
│  ─ tracks AGV state         │   String                          │  ─ turn control             │
│                             │                                   │  ─ workstation docking      │
└─────────────────────────────┘                                   └─────────────────────────────┘
```

**The robot knows how to move — ROS knows where to go.**

Adding new workstations or changing mission sequences requires no Arduino reflash. Only the ROS-side map is updated.

### Command Protocol

Commands are sent as a flat byte array: `[opcode, param, opcode, param, ...]`

| Opcode | Name                 | Param                              |
|--------|----------------------|------------------------------------|
| 1      | `CMD_MOVE_NORTH`     | Number of red nodes to count       |
| 2      | `CMD_MOVE_SOUTH`     | Number of red nodes to count       |
| 3      | `CMD_MOVE_EAST`      | Number of red nodes to count       |
| 4      | `CMD_MOVE_WEST`      | Number of red nodes to count       |
| 5      | `CMD_ENTER_WORKSTATION` | 0 (unused)                      |
| 6      | `CMD_RETURN_TO_DEPOT`   | Red nodes to travel west to col 1 |

### Status Feedback

The robot publishes to `/agv/status`:

| Value        | Meaning                                              |
|--------------|------------------------------------------------------|
| `IDLE`       | Powered on, waiting for a mission                    |
| `EXECUTING`  | Command queue in progress                            |
| `DONE`       | All commands completed, AGV at depot                 |
| `ERROR`      | Emergency stop triggered, lost line, or bad command  |
| `BUSY`       | New mission received while already executing         |

### LED Status Indicators

| Color  | Meaning                        |
|--------|--------------------------------|
| Blue   | Idle, waiting for mission      |
| Yellow | Turning                        |
| Green  | Mission complete               |
| Red    | Emergency stop / error         |
| Blink Yellow | Docked at workstation    |

---

## Software Setup

### Arduino (agv_level2.ino)

**Dependencies:**
- [Arduino Alvik library](https://github.com/arduino-libraries/Arduino_Alvik)
- [micro-ROS Arduino library](https://github.com/micro-ROS/micro_ros_arduino)

**Flash:**
1. Open `agv_level2.ino` in Arduino IDE
2. Select the correct board (Arduino Alvik)
3. Upload

### ROS2 Node (ros_mission_node)

**Dependencies:** ROS2 (tested on Humble/Iron), Python 3

**Build and install:**
```bash
cd ~/ros2_ws/src
ln -s /home/labfab/agv_testbed/ros_mission_node .
cd ~/ros2_ws
colcon build --packages-select ros_mission_node
source install/setup.bash
```

**Start the micro-ROS agent** (connects to the Arduino over serial):
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

---

## Running a Mission

**Dispatch via command line parameter:**
```bash
ros2 run ros_mission_node mission_node --ros-args -p mission:="7,8,9,10,11,12"
```

**Monitor status:**
```bash
ros2 topic echo /agv/status
```

**Send a raw command manually** (e.g. MOVE_NORTH 2 reds):
```bash
ros2 topic pub /agv/commands std_msgs/msg/UInt8MultiArray \
  "data: [1, 2]" --once
```

**Send a full sequence manually** (MOVE_NORTH 4 → ENTER_WORKSTATION → RETURN_TO_DEPOT 1 red west):
```bash
ros2 topic pub /agv/commands std_msgs/msg/UInt8MultiArray \
  "data: [1, 4, 5, 0, 6, 1]" --once
```

---

## Extending the Map

To add workstations WS13–WS15, edit only `mission_node.py`:

```python
WORKSTATIONS = {
    ...
    13: WorkstationInfo(13, col=4, row=7, name='WS13'),
    14: WorkstationInfo(14, col=6, row=7, name='WS14'),
    15: WorkstationInfo(15, col=2, row=8, name='WS15'),
}
```

No Arduino reflash required.

---

## Verification Steps

1. Flash `agv_level2.ino` → start micro-ROS agent → `ros2 topic echo /agv/status` should show `IDLE`
2. Publish `data: []` → robot responds `DONE` immediately (empty mission)
3. Publish `data: [1, 2]` → robot drives north, counts 2 red nodes, stops, responds `DONE`
4. Position robot at WS07 entry → publish `data: [5, 0]` → full dock/undock cycle
5. Run `dispatch_mission([7])` from the ROS node → robot completes WS07 and returns to depot
6. Run `dispatch_mission([7,8,9,10,11,12])` → compare path to `agv_coded.ino` reference run
