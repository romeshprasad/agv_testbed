"""
AGV Level 2 Mission Dispatch Node

Holds the warehouse map, plans paths as primitive commands, and dispatches
them to the AGV over micro-ROS via /agv/commands (std_msgs/UInt8MultiArray).
Receives execution status from /agv/status (std_msgs/String).

Usage examples
--------------
# Dispatch a mission to workstations 7, 8, 9, 10, 11, 12 then return to depot:
ros2 run ros_mission_node mission_node --ros-args -p mission:="7,8,9,10,11,12"

# Single workstation:
ros2 run ros_mission_node mission_node --ros-args -p mission:="7"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String
from collections import namedtuple

# ── Command opcodes (must match agv_level2.ino) ──────────────────────────────
CMD_MOVE_NORTH        = 1
CMD_MOVE_SOUTH        = 2
CMD_MOVE_EAST         = 3
CMD_MOVE_WEST         = 4
CMD_ENTER_WORKSTATION = 5
CMD_RETURN_TO_DEPOT   = 6

# ── Workstation map ───────────────────────────────────────────────────────────
# Derived from agv_coded.ino WORKSTATIONS[] table.
# entry_col = (left_node - 1) % 8 + 1
# entry_row = (left_node - 1) // 8 + 1
WorkstationInfo = namedtuple('WorkstationInfo', ['id', 'col', 'row', 'name'])

WORKSTATIONS = {
     1: WorkstationInfo( 1, col=1, row=2, name='WS01'),  # left_node=9
     2: WorkstationInfo( 2, col=5, row=2, name='WS02'),  # left_node=13
     3: WorkstationInfo( 3, col=2, row=3, name='WS03'),  # left_node=18
     4: WorkstationInfo( 4, col=3, row=3, name='WS04'),  # left_node=19
     5: WorkstationInfo( 5, col=7, row=3, name='WS05'),  # left_node=23
     6: WorkstationInfo( 6, col=3, row=4, name='WS06'),  # left_node=27
     7: WorkstationInfo( 7, col=1, row=5, name='WS07'),  # left_node=33
     8: WorkstationInfo( 8, col=3, row=5, name='WS08'),  # left_node=35
     9: WorkstationInfo( 9, col=6, row=5, name='WS09'),  # left_node=38
    10: WorkstationInfo(10, col=2, row=6, name='WS10'),  # left_node=42
    11: WorkstationInfo(11, col=3, row=6, name='WS11'),  # left_node=43
    12: WorkstationInfo(12, col=7, row=8, name='WS12'),  # left_node=55
}

GridPos = namedtuple('GridPos', ['col', 'row'])
DEPOT_POS = GridPos(col=1, row=1)


class MissionNode(Node):

    def __init__(self):
        super().__init__('agv_mission_node')

        self.cmd_pub = self.create_publisher(UInt8MultiArray, '/agv/commands', 10)
        self.status_sub = self.create_subscription(
            String, '/agv/status', self._status_callback, 10)

        self.declare_parameter('mission', '')

        self._current_pos = DEPOT_POS
        self._agv_status  = 'UNKNOWN'
        self._mission_dispatched = False

        # Dispatch mission from parameter once the node is spinning.
        self.create_timer(1.0, self._startup_dispatch)

    # ── Status feedback ───────────────────────────────────────────────────────

    def _status_callback(self, msg: String):
        self._agv_status = msg.data
        self.get_logger().info(f'AGV status: {msg.data}')

        if msg.data == 'DONE':
            self.get_logger().info('Mission complete. AGV returned to depot.')
            self._current_pos = DEPOT_POS
        elif msg.data == 'ERROR':
            self.get_logger().error('AGV reported an error — mission aborted.')
        elif msg.data == 'BUSY':
            self.get_logger().warn('AGV is busy — mission dispatch was ignored.')

    # ── Startup timer: dispatch mission from ROS parameter ───────────────────

    def _startup_dispatch(self):
        if self._mission_dispatched:
            return

        mission_param = self.get_parameter('mission').get_parameter_value().string_value
        if not mission_param:
            self.get_logger().info('No mission parameter set. Waiting for dispatch_mission() call.')
            return

        try:
            ws_ids = [int(x.strip()) for x in mission_param.split(',') if x.strip()]
        except ValueError:
            self.get_logger().error(f'Invalid mission parameter: "{mission_param}"')
            return

        self._mission_dispatched = True
        self.dispatch_mission(ws_ids)

    # ── Public API ────────────────────────────────────────────────────────────

    def dispatch_mission(self, workstation_ids: list):
        """
        Plan and publish a full mission: visit each workstation in order,
        then return to depot.

        Args:
            workstation_ids: ordered list of workstation IDs (e.g. [7, 8, 9])
        """
        unknown = [w for w in workstation_ids if w not in WORKSTATIONS]
        if unknown:
            self.get_logger().error(f'Unknown workstation IDs: {unknown}')
            return

        all_commands = []
        pos = self._current_pos

        for ws_id in workstation_ids:
            cmds, pos = self._plan_path(pos, ws_id)
            all_commands.extend(cmds)

        depot_cmds = self._plan_return_to_depot(pos)
        all_commands.extend(depot_cmds)

        self._publish_commands(all_commands)

        ws_names = [WORKSTATIONS[w].name for w in workstation_ids]
        self.get_logger().info(
            f'Mission dispatched: {ws_names} → depot '
            f'({len(all_commands)} commands, '
            f'{len(all_commands) * 2} bytes)'
        )

    # ── Path planning ─────────────────────────────────────────────────────────

    def _plan_path(self, from_pos: GridPos, to_ws_id: int):
        """
        Compute the MOVE + ENTER_WORKSTATION commands to reach a workstation.

        Returns:
            (commands, arrival_pos) where commands is a list of (opcode, param)
            tuples and arrival_pos is the grid position after the workstation entry.
        """
        ws = WORKSTATIONS[to_ws_id]
        commands = []

        row_delta = ws.row - from_pos.row
        if row_delta > 0:
            commands.append((CMD_MOVE_NORTH, row_delta))
        elif row_delta < 0:
            commands.append((CMD_MOVE_SOUTH, abs(row_delta)))

        col_delta = ws.col - from_pos.col
        if col_delta > 0:
            commands.append((CMD_MOVE_EAST, col_delta))
        elif col_delta < 0:
            commands.append((CMD_MOVE_WEST, abs(col_delta)))

        commands.append((CMD_ENTER_WORKSTATION, 0))

        arrival_pos = GridPos(col=ws.col, row=ws.row)
        return commands, arrival_pos

    def _plan_return_to_depot(self, from_pos: GridPos):
        """
        Compute the RETURN_TO_DEPOT command to go from from_pos back to depot.
        param = number of red nodes to travel west to reach column 1.
        """
        west_reds = from_pos.col
        return [(CMD_RETURN_TO_DEPOT, west_reds)]

    # ── Transport ─────────────────────────────────────────────────────────────

    def _publish_commands(self, commands: list):
        msg = UInt8MultiArray()
        msg.data = []
        for opcode, param in commands:
            msg.data.append(int(opcode))
            msg.data.append(int(param))
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
