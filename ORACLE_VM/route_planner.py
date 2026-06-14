#!/usr/bin/env python3
"""
route_planner.py  —  Off-robot routing engine for AGV_MULTI_WS_DISPATCH

This module contains ALL grid / map intelligence.  The Arduino sketch is a
dumb token executor; every routing decision is made here and sent as a flat
token string.

Token vocabulary (matches the Arduino sketch)
---------------------------------------------
  RED     drive forward to the next red intersection marker
  CLEAR   creep forward off the current marker before the next RED
  R       turn 90° right (absolute yaw decreases by 90)
  L       turn 90° left  (absolute yaw increases by 90)
  YENTRY  drive slowly until the yellow entry sticker (side of the aisle)
  R_SPUR  right-turn version of the YENTRY→spur turn (same as R with short
          yellow-ignore window — emitted separately so callers can read the
          intent, collapsed to R in to_token_string)
  YWORK   drive slowly until the yellow workstation stop marker
  YAW0    rotate to absolute yaw 0 (face north) before reversing into dock
  DOCK    reverse slowly until the yellow dock marker
  DWELL   wait at the workstation (WORKSTATION_WAIT_MS on the robot)
  EXIT    drive forward until back at the yellow entry sticker
  BLUE    drive forward until the blue depot marker
  YAW0    (also used at the end of the return leg to face north at depot)

Grid conventions  (same as the Arduino sketch and workstations.json)
--------------------------------------------------------------------
  8×8 grid, 64 nodes numbered row-major from the BOTTOM-LEFT.
  Row 1 = nodes  1-8   (bottom row)
  Row 2 = nodes  9-16
  …
  Row 8 = nodes 57-64  (top row)

  Depot is at the bottom-left corner (node 1, column 1).
  The AGV starts facing NORTH (yaw = 0) on the blue sticker.

  Red intersection markers sit at every grid node.
  Yellow entry stickers sit between two adjacent nodes (workstation spurs).
"""

from __future__ import annotations

import json
import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple


# ---------------------------------------------------------------------------
# Grid helpers
# ---------------------------------------------------------------------------

GRID_COLS = 8


def node_row(node: int) -> int:
    """1-based row of a node (row 1 = bottom)."""
    return (node - 1) // GRID_COLS + 1


def node_col(node: int) -> int:
    """1-based column of a node (col 1 = leftmost)."""
    return (node - 1) % GRID_COLS + 1


# ---------------------------------------------------------------------------
# Workstation data class
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Workstation:
    id: str            # e.g. "WS01"
    left_node: int     # lower-numbered node of the pair
    right_node: int    # higher-numbered node of the pair

    @property
    def row(self) -> int:
        return node_row(self.left_node)

    @property
    def left_col(self) -> int:
        return node_col(self.left_node)

    @property
    def right_col(self) -> int:
        return node_col(self.right_node)

    # --- approach from depot (north-bound then east-bound) ------------------

    @property
    def northbound_red_count(self) -> int:
        """Red markers to count going north from depot before the east turn."""
        return self.row - 1

    @property
    def eastbound_red_count(self) -> int:
        """Red markers to count going east before the yellow entry (APPROACH_EAST)."""
        return self.left_col - 1

    @property
    def westbound_red_count(self) -> int:
        """Red markers to count going west from the entry segment back to depot col."""
        return self.left_col


# ---------------------------------------------------------------------------
# Load workstations from JSON
# ---------------------------------------------------------------------------

_DEFAULT_JSON = Path(__file__).parent / "workstations.json"


def load_workstations(path: Path = _DEFAULT_JSON) -> dict[str, Workstation]:
    data = json.loads(path.read_text(encoding="utf-8"))
    registry: dict[str, Workstation] = {}
    for row in data["workstations"]:
        a, b = int(row["between_nodes"][0]), int(row["between_nodes"][1])
        ws = Workstation(
            id=str(row["id"]),
            left_node=min(a, b),
            right_node=max(a, b),
        )
        registry[ws.id] = ws
    return registry


# ---------------------------------------------------------------------------
# Token sequence builders
# ---------------------------------------------------------------------------

def _red_tokens(count: int) -> List[str]:
    """Tokens to traverse `count` red intersection markers in one direction."""
    tokens: List[str] = []
    for i in range(count):
        if i > 0:
            tokens.append("CLEAR")
        tokens.append("RED")
    return tokens


def depot_to_workstation_tokens(ws: Workstation) -> List[str]:
    """
    Token sequence: blue depot → docked at workstation.

    The robot starts at the depot facing NORTH.
    Steps:
      1. Drive north, counting red markers until entry row.
      2. Turn east.
      3. Drive east, counting red markers until entry column.
      4. Drive slowly to yellow entry sticker.
      5. Turn south (right turn) into the workstation spur.
      6. Drive slowly to workstation yellow marker.
      7. Align yaw to north (YAW0) so we can reverse straight.
      8. Reverse to dock yellow marker.
    """
    tokens: List[str] = []

    north_count = ws.northbound_red_count
    east_count  = ws.eastbound_red_count

    if north_count > 0:
        tokens.extend(_red_tokens(north_count))

    tokens.append("R")          # face east

    if east_count > 0:
        tokens.extend(_red_tokens(east_count))

    tokens.append("YENTRY")     # creep east to yellow entry sticker
    tokens.append("R")          # face south into the spur
    tokens.append("YWORK")      # approach workstation yellow
    tokens.append("YAW0")       # align north before reversing
    tokens.append("DOCK")       # reverse to dock

    return tokens


def workstation_to_next_tokens(from_ws: Workstation, to_ws: Workstation) -> List[str]:
    """
    Token sequence: docked at from_ws → docked at to_ws.

    Heading turn table (CW = right, CCW = left):
      R from N → E    L from N → W
      R from E → S    L from E → N
      R from S → W    L from S → E
      R from W → N    L from W → S

    After EXIT the robot is back at the yellow entry sticker, on the main
    grid line, facing NORTH (away from the spur) — it must turn before it
    can drive along the row.

    Minimal-turn strategy (one turn per direction change, same as the
    working WS-to-WS and WS-to-depot patterns):
      1. EXIT, then turn once toward to_ws.left_col (R→E if east, L→W if west).
      2. Drive the column delta in reds.
      3. If the row also changes, turn once onto the column (toward to_ws.row),
         drive the row delta in reds, then turn once to face EAST for YENTRY.
      4. If the row doesn't change, no further turn is needed before YENTRY
         (already facing E from step 1) — or if delta_col == 0, turn directly
         from N to face the row-delta direction, then turn to E afterward.
    """
    tokens: List[str] = []

    tokens.append("EXIT")           # drive N, stop at yellow entry (heading = N)

    delta_col = to_ws.left_col - from_ws.left_col   # positive = go east
    delta_row = to_ws.row - from_ws.row             # positive = go north

    if delta_col == 0:
        # already in the right column — just turn onto it and go to to_ws.row
        if delta_row > 0:
            tokens.extend(_red_tokens(delta_row))   # heading stays N
            tokens.append("R")      # N→R→E
        elif delta_row < 0:
            tokens.append("R")      # N→R→E
            tokens.append("R")      # E→R→S
            tokens.extend(_red_tokens(-delta_row))
            tokens.append("L")      # S→L→E
        else:
            tokens.append("R")      # N→R→E
        # heading = E
    elif delta_col > 0:
        tokens.append("R")          # N→R→E
        tokens.extend(_red_tokens(delta_col))
        # heading = E
        if delta_row > 0:
            tokens.append("L")      # E→L→N
            tokens.extend(_red_tokens(delta_row))
            tokens.append("R")      # N→R→E
        elif delta_row < 0:
            tokens.append("R")      # E→R→S
            tokens.extend(_red_tokens(-delta_row))
            tokens.append("L")      # S→L→E
        # heading = E
    else:
        tokens.append("L")          # N→L→W
        tokens.extend(_red_tokens(-delta_col))
        # heading = W
        if delta_row > 0:
            tokens.append("R")      # W→R→N
            tokens.extend(_red_tokens(delta_row))
            tokens.append("R")      # N→R→E
        elif delta_row < 0:
            tokens.append("L")      # W→L→S
            tokens.extend(_red_tokens(-delta_row))
            tokens.append("L")      # S→L→E
        else:
            tokens.append("R")      # W→R→N
            tokens.append("R")      # N→R→E
        # heading = E

    # --- Dock at to_ws (heading = E, at to_ws.left_col, to_ws.row) ---
    tokens.append("YENTRY")         # creep E to yellow entry sticker
    tokens.append("R")              # E→R→S (into spur)
    tokens.append("YWORK")
    tokens.append("YAW0")           # align N before reversing
    tokens.append("DOCK")

    return tokens


def workstation_to_depot_tokens(ws: Workstation) -> List[str]:
    """
    Token sequence: docked at workstation → blue depot.

    After EXIT the robot faces NORTH at the yellow entry sticker.
    Turn west (N→L→W), count reds west to col 1, turn south (W→L→S),
    drive to BLUE, then YAW0 to face north ready for the next mission.
    """
    tokens: List[str] = []
    tokens.append("EXIT")           # heading = N
    tokens.append("L")              # N→L→W
    tokens.extend(_red_tokens(ws.westbound_red_count))
    tokens.append("L")              # W→L→S
    tokens.append("BLUE")
    tokens.append("YAW0")
    return tokens


def snake_sweep_tokens(registry: dict[str, "Workstation"]) -> List[str]:
    """
    Connectivity test: serpentine ("snake") sweep through rows 1..7,
    visiting every workstation whose entry row is on that row.

    Not a real planner — just proves out RED/CLEAR/turn/YENTRY/.../EXIT
    sequencing works end to end across the whole grid. The real
    route_planner / MAPF solver replaces this later.

    Robot starts at depot (row 1, col 1) facing NORTH.

    For each row 1..7:
      - turn onto the row (alternating East / West each row)
      - drive across the row, counting RED/CLEAR between columns
      - at each workstation whose entry row == this row, detour into
        the spur at its left_col: turn toward the spur (R if heading
        East, L if heading West), YENTRY,YWORK,YAW0,DOCK,DWELL,EXIT,
        then turn back the same direction to resume the row heading
      - turn to face the next row (North) and drive 1 RED to it

    After row 7, return to depot: turn to face the depot column,
    drive back, turn south, BLUE, YAW0.
    """
    # group workstations by entry row -> list of (left_col, ws_id)
    by_row: dict[int, list[tuple[int, str]]] = {}
    for ws_id, ws in registry.items():
        by_row.setdefault(ws.row, []).append((ws.left_col, ws_id))
    for row in by_row:
        by_row[row].sort()

    tokens: List[str] = []
    heading = "E"   # current travel heading along the row (E or W)
    cur_col = 1
    cur_row = 1

    for row in range(1, 8):
        if row == 1:
            # already at (row1, col1) facing N — turn onto the row
            tokens.append("R" if heading == "E" else "L")  # N -> E or W
        else:
            # turn to face North, drive 1 RED to the new row, turn onto it
            tokens.append("L" if heading == "E" else "R")  # heading -> N
            tokens.append("RED")
            cur_row = row
            tokens.append("R" if heading == "E" else "L")  # N -> heading

        stops = by_row.get(row, [])
        stops = stops if heading == "E" else list(reversed(stops))

        for left_col, ws_id in stops:
            # drive along the row to this workstation's column
            steps = abs(left_col - cur_col)
            if steps > 0:
                tokens.extend(_red_tokens(steps))
            cur_col = left_col

            # detour into the workstation spur and back
            turn = "R" if heading == "E" else "L"
            tokens.append(turn)          # heading -> S (into spur)
            tokens.append("YENTRY")
            tokens.append("YWORK")
            tokens.append("YAW0")
            tokens.append("DOCK")
            tokens.append("DWELL")
            tokens.append("EXIT")        # ends heading N
            tokens.append(turn)          # N -> resume row heading (E/W)

        # finish driving to the end of the row (col 8 if heading E, col 1 if W)
        target_col = 8 if heading == "E" else 1
        steps = abs(target_col - cur_col)
        if steps > 0:
            tokens.extend(_red_tokens(steps))
        cur_col = target_col

        heading = "W" if heading == "E" else "E"

    # --- return to depot (row 7, col cur_col, heading = current) -> (row1, col1) ---
    # turn to face South, drive back down to row 1, turn to face West (if needed), BLUE
    tokens.append("R" if heading == "E" else "L")   # heading -> S
    tokens.extend(_red_tokens(cur_row - 1))
    cur_row = 1
    # now at (row1, cur_col) heading S; turn to face West, drive to col1
    tokens.append("L")                              # S -> W
    tokens.extend(_red_tokens(cur_col - 1))
    tokens.append("L")                              # W -> S
    tokens.append("BLUE")
    tokens.append("YAW0")

    return tokens


# ---------------------------------------------------------------------------
# High-level route builders
# ---------------------------------------------------------------------------

def build_route_tokens(
    workstation_ids: List[str],
    registry: Optional[dict[str, Workstation]] = None,
    return_to_depot: bool = True,
) -> List[str]:
    """
    Build the complete flat token list for an ordered workstation route.

    Parameters
    ----------
    workstation_ids : e.g. ["WS01", "WS04", "WS07"]
    registry        : workstation registry (loaded from JSON if None)
    return_to_depot : whether to append return-to-depot tokens after last stop

    Returns
    -------
    Flat list of token strings ready to join with commas and send via ROS.
    """
    if registry is None:
        registry = load_workstations()

    if not workstation_ids:
        raise ValueError("workstation_ids must not be empty")

    tokens: List[str] = []

    # First stop: depot → workstation
    first_ws = registry[workstation_ids[0]]
    tokens.extend(depot_to_workstation_tokens(first_ws))
    tokens.append("DWELL")

    # Subsequent stops: workstation → workstation
    for prev_id, next_id in zip(workstation_ids, workstation_ids[1:]):
        prev_ws = registry[prev_id]
        next_ws = registry[next_id]
        tokens.extend(workstation_to_next_tokens(prev_ws, next_ws))
        tokens.append("DWELL")

    # Return leg
    if return_to_depot:
        last_ws = registry[workstation_ids[-1]]
        tokens.extend(workstation_to_depot_tokens(last_ws))

    return tokens


def to_token_string(tokens: List[str]) -> str:
    """Join tokens into the comma-separated string expected by the Arduino."""
    return ",".join(tokens)


def to_run_command(tokens: List[str]) -> str:
    """Format tokens as a 'run ...' ROS command string."""
    return "run " + to_token_string(tokens)


# Arduino's cmd_buf is 512 bytes; leave headroom for the "run "/"append "
# prefix and the trailing NUL the micro-ROS string transport adds.
_CMD_BUF_LIMIT = 512
_SAFE_CMD_LEN = 480


def to_chunked_commands(tokens: List[str], max_len: int = _SAFE_CMD_LEN) -> List[str]:
    """
    Split tokens into a sequence of ROS command strings that each fit within
    the Arduino's cmd_buf (512 bytes). The first chunk uses 'run', every
    subsequent chunk uses 'append' so the script keeps building on the robot
    without resetting token_index.
    """
    commands: List[str] = []
    chunk: List[str] = []
    for tok in tokens:
        prefix = "run " if not commands and not chunk else "append "
        candidate = chunk + [tok]
        if len(prefix + to_token_string(candidate)) > max_len and chunk:
            verb = "run" if not commands else "append"
            commands.append(f"{verb} " + to_token_string(chunk))
            chunk = [tok]
        else:
            chunk = candidate
    if chunk:
        verb = "run" if not commands else "append"
        commands.append(f"{verb} " + to_token_string(chunk))
    return commands


# ---------------------------------------------------------------------------
# Random route generator  (useful for testing and scheduling experiments)
# ---------------------------------------------------------------------------

def random_route(
    n_stops: int,
    registry: Optional[dict[str, Workstation]] = None,
    seed: Optional[int] = None,
    return_to_depot: bool = True,
) -> Tuple[List[str], List[str]]:
    """
    Generate a random ordered route visiting `n_stops` distinct workstations.

    Returns (workstation_id_list, token_list).
    """
    if registry is None:
        registry = load_workstations()

    rng = random.Random(seed)
    ws_ids = rng.sample(sorted(registry.keys()), k=min(n_stops, len(registry)))
    tokens = build_route_tokens(ws_ids, registry=registry, return_to_depot=return_to_depot)
    return ws_ids, tokens


# ---------------------------------------------------------------------------
# Route plan dataclass  (for scheduling integration)
# ---------------------------------------------------------------------------

@dataclass
class RoutePlan:
    """A resolved route for one AGV, ready to dispatch."""
    agv_id: str
    workstation_ids: List[str]
    tokens: List[str] = field(default_factory=list)
    return_to_depot: bool = True

    def build(self, registry: Optional[dict[str, Workstation]] = None) -> "RoutePlan":
        """Compute tokens from workstation_ids. Returns self for chaining."""
        self.tokens = build_route_tokens(
            self.workstation_ids,
            registry=registry,
            return_to_depot=self.return_to_depot,
        )
        return self

    @property
    def run_command(self) -> str:
        return to_run_command(self.tokens)

    def to_dict(self) -> dict:
        return {
            "agv_id": self.agv_id,
            "workstation_ids": self.workstation_ids,
            "token_count": len(self.tokens),
            "tokens": self.tokens,
            "run_command": self.run_command,
        }


# ---------------------------------------------------------------------------
# Token stream inspector  (debugging helper)
# ---------------------------------------------------------------------------

def explain_tokens(tokens: List[str]) -> str:
    """
    Return a human-readable summary of a token list, grouping consecutive
    movement tokens and annotating each DWELL.
    """
    lines: List[str] = []
    step = 0
    for i, tok in enumerate(tokens):
        lines.append(f"  {i:3d}  {tok}")
        if tok == "DWELL":
            step += 1
            lines[-1] += f"  ← stop #{step}"
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse
    import sys

    parser = argparse.ArgumentParser(
        description="Compute token routes for AGV_MULTI_WS_DISPATCH."
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_route = sub.add_parser("route", help="Build route for given workstation list.")
    p_route.add_argument("workstations", nargs="+",
                         help="Workstation IDs in visit order, e.g. WS01 WS04 WS07")
    p_route.add_argument("--agv", default="Alvik1", help="AGV name (for ROS topic prefix)")
    p_route.add_argument("--no-return", action="store_true",
                         help="Do not append return-to-depot tokens")
    p_route.add_argument("--format", choices=("tokens", "command", "explain", "json"),
                         default="command")
    p_route.add_argument("--ws-json", type=Path, default=_DEFAULT_JSON,
                         help="Path to workstations.json")

    p_sweep = sub.add_parser("sweep", help="Build the full-grid snake-sweep test route "
                             "(visits all workstations, connectivity test only).")
    p_sweep.add_argument("--agv", default="Alvik1")
    p_sweep.add_argument("--format", choices=("tokens", "command", "explain", "chunked"),
                          default="command")
    p_sweep.add_argument("--ws-json", type=Path, default=_DEFAULT_JSON)

    p_rand = sub.add_parser("random", help="Build a random route.")
    p_rand.add_argument("n", type=int, help="Number of workstations to visit")
    p_rand.add_argument("--seed", type=int, default=None)
    p_rand.add_argument("--agv", default="Alvik1")
    p_rand.add_argument("--no-return", action="store_true")
    p_rand.add_argument("--format", choices=("tokens", "command", "explain", "json"),
                        default="command")
    p_rand.add_argument("--ws-json", type=Path, default=_DEFAULT_JSON)

    args = parser.parse_args()
    registry = load_workstations(args.ws_json)

    if args.cmd == "route":
        ids   = [i.upper() if i.upper().startswith("WS") else f"WS{int(i):02d}"
                 for i in args.workstations]
        plan  = RoutePlan(agv_id=args.agv, workstation_ids=ids,
                          return_to_depot=not args.no_return).build(registry)
    elif args.cmd == "sweep":
        tokens = snake_sweep_tokens(registry)
        plan = RoutePlan(agv_id=args.agv, workstation_ids=sorted(registry.keys()))
        plan.tokens = tokens
    else:
        ids, tokens = random_route(args.n, registry=registry, seed=args.seed,
                                   return_to_depot=not args.no_return)
        plan = RoutePlan(agv_id=args.agv, workstation_ids=ids,
                         return_to_depot=not args.no_return)
        plan.tokens = tokens

    fmt = args.format
    if fmt == "tokens":
        print(to_token_string(plan.tokens))
    elif fmt == "command":
        print(plan.run_command)
    elif fmt == "explain":
        print(f"Route: {plan.workstation_ids}")
        print(f"Tokens ({len(plan.tokens)}):")
        print(explain_tokens(plan.tokens))
    elif fmt == "json":
        print(json.dumps(plan.to_dict(), indent=2))
    elif fmt == "chunked":
        topic = f"/{args.agv}_cmd"
        for cmd in to_chunked_commands(plan.tokens):
            print(
                f"ros2 topic pub --once {topic} std_msgs/msg/String "
                f"\"{{data: '{cmd}'}}\" --qos-reliability best_effort"
            )
