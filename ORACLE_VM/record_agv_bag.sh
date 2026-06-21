#!/usr/bin/env bash
# record_agv_bag.sh — Record cmd/status topics for all 4 Alviks to a rosbag.
#
# Usage:
#   source /opt/ros/jazzy/setup.bash
#   source ~/microros_ws/install/setup.bash
#   ./record_agv_bag.sh [output_bag_name]
#
# Stop with Ctrl-C. The bag is written to ./<output_bag_name>/ (default:
# agv_run_<timestamp>).

set -euo pipefail

ROBOTS=(Alvik1 Alvik2 Alvik3 Alvik4)
OUT="${1:-agv_run_$(date +%Y%m%d_%H%M%S)}"

TOPICS=()
for r in "${ROBOTS[@]}"; do
    TOPICS+=("/${r}_cmd" "/${r}_status")
done

echo "Recording topics: ${TOPICS[*]}"
echo "Output bag: ${OUT}"

ros2 bag record -o "${OUT}" "${TOPICS[@]}"
