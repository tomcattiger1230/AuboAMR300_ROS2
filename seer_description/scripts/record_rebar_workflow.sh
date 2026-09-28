#!/usr/bin/env bash
# Run the eight verified stages against an already running, recording Isaac stack.
set -euo pipefail

VIDEO_DIR="${REBAR_VIDEO_DIR:?Set REBAR_VIDEO_DIR to the Isaac recording directory}"
mkdir -p "$VIDEO_DIR"
STATE_FILE="$VIDEO_DIR/workflow_state.json"
REPORT_DIR="$VIDEO_DIR/reports"
mkdir -p "$REPORT_DIR"
COMPACT=false
if [[ "${1:-}" == --compact ]]; then
  COMPACT=true
elif [[ $# -gt 0 ]]; then
  printf 'Usage: %s [--compact]\n' "$0" >&2
  exit 2
fi

run_stage() {
  local number="$1" label="$2" script="$3"
  shift 3
  printf '%s\n' "$number" > "$VIDEO_DIR/stage.txt"
  printf '%s\t%s\t%s\n' "$(date -u +%FT%TZ)" "$number" "$label" | tee -a "$VIDEO_DIR/stages.tsv"
  ros2 run seer_description "$script" "$@" --output "$REPORT_DIR/$number.json" \
    > "$REPORT_DIR/$number.log" 2>&1 || {
      tail -60 "$REPORT_DIR/$number.log" >&2
      exit 1
    }
  tail -3 "$REPORT_DIR/$number.log"
}

run_stage 01 '取筋并装车' test_rebar_grasp.py --onboard-slot 1
run_stage 02 '载筋导航' 02_navigate_standoff.py --onboard-slot 1
if [[ "$COMPACT" == true ]]; then
  run_stage 03 '车载取筋' pickup_onboard.py --onboard-slot 1 --at-standoff --compact-start --state-file "$STATE_FILE"
  compact_args=()
  if [[ -n "${REBAR_COMPACT_SPEED:-}" ]]; then
    compact_args+=(--speed-scale "$REBAR_COMPACT_SPEED")
  fi
  if [[ -n "${REBAR_COMPACT_SCHEDULE:-}" ]]; then
    compact_args+=(--schedule-file "$REBAR_COMPACT_SCHEDULE")
  fi
  run_stage 04 '同步竖直化与正面预定位' 04_compact_vertical.py --state-file "$STATE_FILE" "${compact_args[@]}"
  run_stage 05 '预定位状态确认' 04_preposition_standoff.py --state-file "$STATE_FILE"
else
  run_stage 03 '车载取筋' pickup_onboard.py --onboard-slot 1 --at-standoff --state-file "$STATE_FILE"
  run_stage 04 '竖直化钢筋' 03_verticalize_standoff.py --state-file "$STATE_FILE"
  run_stage 05 '测试台前预定位' 04_preposition_standoff.py --state-file "$STATE_FILE"
fi
run_stage 06 '竖直持筋接近' 05_drive_vertical_rebar.py --state-file "$STATE_FILE"
run_stage 07 '夹持线微调' 06_micro_insert.py --state-file "$STATE_FILE"
run_stage 08 '测试机接管' 05_handoff.py --state-file "$STATE_FILE"
printf '%s\n' complete > "$VIDEO_DIR/stage.txt"
printf 'All eight rebar workflow stages completed.\n'
