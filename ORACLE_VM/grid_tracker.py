#!/usr/bin/env python3
"""
grid_tracker.py — Dead-reckoning grid tracker for juan_code.ino AGVs.

juan_code.ino reports only line-follow *events*, not (x, y) pose. Each
command type has its own distinct completion event (see COMPLETION_PREFIX
below) — e.g. a FORWARD_UNTIL_RED finishes with "DETECTED RED ...", a
RIGHT_UNTIL_COLOR finishes with "TURN COMPLETE". This module replays a route
file in lock-step with those completion events from <ROBOT_NAME>_status to
maintain each AGV's (row, col, heading) on the shared 8x8 grid
(workstations.json convention: row 1 = bottom, col 1 = leftmost, node 1 =
depot).

Note: juan_code.ino also republishes a plain "IDLE" every ~1s while at rest
(a heartbeat, unrelated to command completion) — this tracker never matches
on bare "IDLE", so the heartbeat is naturally ignored.

Each route file's first 5 commands are the shared "merge onto the loop"
maneuver every AGV performs from its own entry spur:

    FORWARD_UNTIL_BLUE   -> reach blue depot marker (off-grid)
    RIGHT_UNTIL_COLOR    -> turn onto the loop
    FORWARD_UNTIL_RED    -> advance (off-grid)
    RIGHT_UNTIL_COLOR    -> turn onto the loop
    FORWARD_UNTIL_RED    -> arrive at node 1 (row=1, col=1), facing East

So every AGV is seeded at (row=1, col=1, heading=EAST) and tracking begins
at command index 5 (0-based) — the first 5 commands are consumed without
emitting grid moves.

Heading convention (matches route_planner.py / workstations.json):
    NORTH = +row, EAST = +col, SOUTH = -row, WEST = -col
    RIGHT_UNTIL_COLOR rotates -90 (clockwise: N->E->S->W->N)
    LEFT_UNTIL_COLOR  rotates +90 (counter-clockwise)
    ROTATE_180 reverses heading, position unchanged
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple

GRID_SIZE = 8

# Each command type's distinct completion-event prefix, as published on
# <ROBOT_NAME>_status by juan_code.ino. "ERROR" always ends the route early
# regardless of the command in flight.
COMPLETION_PREFIX = {
    "FORWARD_UNTIL_RED": "DETECTED",
    "FORWARD_UNTIL_YELLOW": "DETECTED",
    "FORWARD_UNTIL_BLUE": "DETECTED",
    "FORWARD_UNTIL_COLOR": "DETECTED",
    "BACKWARD_UNTIL_COLOR": "DETECTED",
    "BACKWARD_UNTIL_YELLOW": "DETECTED",
    "BACKWARD_UNTIL_BLUE": "DETECTED",
    "RIGHT_UNTIL_COLOR": "TURN COMPLETE",
    "LEFT_UNTIL_COLOR": "TURN COMPLETE",
    "ROTATE_180": "ROTATE_180 COMPLETE",
    "DWELL": "DWELL COMPLETE",
    "STOP": "STOPPED",
    "RESET_POSE": "POSE_RESET",
    "GET_STATUS": "IDLE",
}

HEADINGS = ["N", "E", "S", "W"]  # clockwise order
HEADING_DELTA = {
    "N": (1, 0),
    "E": (0, 1),
    "S": (-1, 0),
    "W": (0, -1),
}

# Commands that advance one grid cell in the current heading on completion.
FORWARD_CMDS = {
    "FORWARD_UNTIL_RED",
    "FORWARD_UNTIL_YELLOW",
    "FORWARD_UNTIL_BLUE",
    "FORWARD_UNTIL_COLOR",
}
BACKWARD_CMDS = {
    "BACKWARD_UNTIL_COLOR",
    "BACKWARD_UNTIL_YELLOW",
    "BACKWARD_UNTIL_BLUE",
}

# Number of leading commands consumed by the shared off-grid merge maneuver
# before the AGV is at (row=1, col=1) facing East.
MERGE_PREFIX_LEN = 5

START_ROW = 1
START_COL = 1
START_HEADING = "E"


def load_route(path: str | Path) -> List[str]:
    """Parse a route file: one command per line, blank lines and '#' comments skipped."""
    commands = []
    for line in Path(path).read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            commands.append(line)
    return commands


@dataclass
class AgvTrack:
    """Tracks one AGV's grid pose as its route commands complete."""

    route: List[str]
    row: int = START_ROW
    col: int = START_COL
    heading: str = START_HEADING
    cmd_index: int = 0  # index into route of the command currently in flight
    started: bool = False
    finished: bool = False
    trail: List[Tuple[int, int]] = field(default_factory=list)

    def __post_init__(self):
        self.trail.append((self.row, self.col))

    @property
    def current_command(self) -> str | None:
        if self.cmd_index >= len(self.route):
            return None
        return self.route[self.cmd_index]

    def on_status(self, status: str) -> bool:
        """Feed one status-line update. Returns True if the AGV's pose changed."""
        if self.finished:
            return False

        cmd = self.current_command
        if cmd is None:
            self.finished = True
            return False

        if status.startswith("ERROR"):
            self.finished = True
            return False

        if not status.startswith(COMPLETION_PREFIX[cmd]):
            return False

        changed = self._apply_command(cmd)
        self.cmd_index += 1
        self.started = True

        if self.cmd_index >= len(self.route):
            self.finished = True

        return changed

    def _apply_command(self, cmd: str) -> bool:
        # The first MERGE_PREFIX_LEN commands move the AGV from its own
        # entry spur onto the shared loop; final pose is fixed (node 1,
        # facing East) regardless of how it got there, so skip dead-reckoning
        # for them.
        if self.cmd_index < MERGE_PREFIX_LEN:
            return False

        if cmd in FORWARD_CMDS:
            dr, dc = HEADING_DELTA[self.heading]
            self._move(dr, dc)
            return True

        if cmd in BACKWARD_CMDS:
            dr, dc = HEADING_DELTA[self.heading]
            self._move(-dr, -dc)
            return True

        if cmd == "RIGHT_UNTIL_COLOR":
            idx = HEADINGS.index(self.heading)
            self.heading = HEADINGS[(idx + 1) % 4]
            return False

        if cmd == "LEFT_UNTIL_COLOR":
            idx = HEADINGS.index(self.heading)
            self.heading = HEADINGS[(idx - 1) % 4]
            return False

        if cmd == "ROTATE_180":
            idx = HEADINGS.index(self.heading)
            self.heading = HEADINGS[(idx + 2) % 4]
            return False

        # DWELL, STOP, RESET_POSE, GET_STATUS: no pose change.
        return False

    def _move(self, dr: int, dc: int) -> None:
        new_row = self.row + dr
        new_col = self.col + dc
        # Clamp to the 8x8 grid in case of an off-by-one in the route/status
        # stream; keeps the trace on-screen instead of erroring out.
        new_row = max(1, min(GRID_SIZE, new_row))
        new_col = max(1, min(GRID_SIZE, new_col))
        self.row, self.col = new_row, new_col
        self.trail.append((self.row, self.col))
