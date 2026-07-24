#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

WAREHOUSE_USD=""
for candidate in \
  "$SCRIPT_DIR/../urdf/warehouse_stick_demo.usda" \
  "$SCRIPT_DIR/../../share/seer_description/urdf/warehouse_stick_demo.usda"; do
  if [[ -f "$candidate" ]]; then
    WAREHOUSE_USD="$(cd -- "$(dirname -- "$candidate")" && pwd)/$(basename -- "$candidate")"
    break
  fi
done

if [[ -z "$WAREHOUSE_USD" ]]; then
  printf 'warehouse_stick_demo.usda was not found next to the installed package.\n' >&2
  exit 1
fi

exec "$SCRIPT_DIR/start_isaac_ros2_stack.sh" \
  --usd "$WAREHOUSE_USD" \
  --robot-prim "/World/seer_aubo_composite/base_footprint" \
  --robot-xacro "composite_robot_stick.urdf.xacro" \
  --moveit-package "seer_aubo_stick_moveit_config" \
  --action-name "/aubo_arm_controller/follow_joint_trajectory" \
  "$@"
