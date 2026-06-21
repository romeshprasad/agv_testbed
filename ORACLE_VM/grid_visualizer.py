#!/usr/bin/env python3
"""
grid_visualizer.py — "Etch-a-sketch" trace of all 4 AGVs on the 8x8 grid.

Subscribes to <ROBOT_NAME>_status for each configured robot, replays that
robot's route file in lock-step (see grid_tracker.py) to maintain its
(row, col, heading), and animates the result with matplotlib: each AGV is a
colored marker that leaves a trail as it moves.

Works both:
  - LIVE, while juan_supervisor.py instances are driving the real robots, and
  - IN REPLAY, while `ros2 bag play` republishes a recorded bag's *_status
    topics (this node only subscribes — it doesn't care whether the
    publisher is a real robot or a bag).

Usage:
    source /opt/ros/jazzy/setup.bash
    cd ~/agv_testbed_1/agv_testbed/ORACLE_VM
    python3 grid_visualizer.py \\
        --robot Alvik1:test_1agv_alvik1.txt \\
        --robot Alvik2:test_1agv_alvik2.txt \\
        --robot Alvik3:test_1agv_alvik3.txt \\
        --robot Alvik4:test_1agv_alvik4.txt
"""

from __future__ import annotations

import argparse
import threading

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from grid_tracker import AgvTrack, GRID_SIZE, load_route

COLORS = ["tab:blue", "tab:orange", "tab:green", "tab:red",
          "tab:purple", "tab:brown", "tab:pink", "tab:gray"]


class GridVisualizerNode(Node):
    def __init__(self, robots: dict[str, str]):
        super().__init__("grid_visualizer")

        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self._lock = threading.Lock()
        self.tracks: dict[str, AgvTrack] = {}
        self._subs = []

        for robot_name, route_path in robots.items():
            route = load_route(route_path)
            self.tracks[robot_name] = AgvTrack(route=route)
            sub = self.create_subscription(
                String, f"{robot_name}_status",
                self._make_callback(robot_name), qos)
            self._subs.append(sub)
            self.get_logger().info(
                f"tracking {robot_name} ({len(route)} commands) "
                f"from {robot_name}_status")

    def _make_callback(self, robot_name: str):
        def _on_status(msg: String):
            text = msg.data.strip()
            with self._lock:
                track = self.tracks[robot_name]
                if track.on_status(text):
                    self.get_logger().info(
                        f"{robot_name}: -> ({track.row}, {track.col}) "
                        f"facing {track.heading}")
        return _on_status

    def snapshot(self):
        with self._lock:
            return {
                name: (track.row, track.col, track.heading, list(track.trail))
                for name, track in self.tracks.items()
            }


def run_visualizer(node: GridVisualizerNode):
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(0.5, GRID_SIZE + 0.5)
    ax.set_ylim(0.5, GRID_SIZE + 0.5)
    ax.set_xticks(range(1, GRID_SIZE + 1))
    ax.set_yticks(range(1, GRID_SIZE + 1))
    ax.set_xlabel("col")
    ax.set_ylabel("row")
    ax.set_title("AGV grid trace")
    ax.grid(True)
    ax.set_aspect("equal")

    trail_lines = {}
    markers = {}
    labels = {}

    for i, name in enumerate(node.tracks):
        color = COLORS[i % len(COLORS)]
        (line,) = ax.plot([], [], "-", color=color, alpha=0.5, linewidth=1.5)
        (marker,) = ax.plot([], [], "o", color=color, markersize=14)
        label = ax.text(0, 0, name, fontsize=8, ha="center", va="bottom",
                         color=color, fontweight="bold")
        trail_lines[name] = line
        markers[name] = marker
        labels[name] = label

    def update(_frame):
        snap = node.snapshot()
        artists = []
        for name, (row, col, heading, trail) in snap.items():
            xs = [c for _, c in trail]
            ys = [r for r, _ in trail]
            trail_lines[name].set_data(xs, ys)
            markers[name].set_data([col], [row])
            labels[name].set_position((col, row + 0.2))
            labels[name].set_text(f"{name} ({heading})")
            artists.extend([trail_lines[name], markers[name], labels[name]])
        return artists

    anim = FuncAnimation(fig, update, interval=300, blit=False)
    plt.tight_layout()
    plt.show()


def parse_robots(args_list: list[str]) -> dict[str, str]:
    robots = {}
    for item in args_list:
        name, _, route_path = item.partition(":")
        if not name or not route_path:
            raise ValueError(f"--robot expects NAME:ROUTE_FILE, got: {item!r}")
        robots[name] = route_path
    return robots


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot", action="append", default=[], required=True,
        metavar="NAME:ROUTE_FILE",
        help="Robot name and its route file, e.g. Alvik1:test_1agv_alvik1.txt "
             "(repeat for each AGV)")
    args = parser.parse_args()

    robots = parse_robots(args.robot)

    rclpy.init()
    node = GridVisualizerNode(robots)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        run_visualizer(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
