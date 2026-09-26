#!/usr/bin/env bash
# Run each ROS node in order; stop immediately when a phase reports failure.
set -euo pipefail

mission_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
report_dir="${REBAR_MISSION_REPORT_DIR:-/tmp/rebar_tester_mission_reports}"
mkdir -p "$report_dir"

for phase in 01_plan 02_pick 03_navigate 04_insert 05_handoff 06_retreat; do
  echo "=== ${phase} ==="
  python3 "$mission_dir/${phase}.py" --output "$report_dir/${phase}.json" "$@"
done

echo "Mission complete. Reports: $report_dir"
