#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-/home/arnold/isaacsim}"
HEADLESS=true
START_RVIZ=true
USD_PATH=""
ROBOT_PRIM="/World/seer_aubo_composite/base_footprint"
ROBOT_XACRO="composite_robot.urdf.xacro"
MOVEIT_PACKAGE="seer_aubo_moveit_config"
ACTION_NAME="/aubo_arm_controller_wo_gripper/follow_joint_trajectory"
COMMAND_TOPIC="/isaac_joint_commands"
STATE_TOPIC="/joint_states"

usage() {
  printf '%s\n' \
    "Usage: $0 [options]" \
    "" \
    "Options:" \
    "  --gui                  Show the Isaac Sim window (default: headless)" \
    "  --no-rviz              Do not start RViz" \
    "  --usd PATH             Override seer_aubo.usd path" \
    "  --robot-prim PATH      Articulation root prim in the USD" \
    "  --robot-xacro FILE     Robot xacro from seer_description/urdf" \
    "  --moveit-package NAME  MoveIt configuration package" \
    "  --action-name NAME     FollowJointTrajectory action name" \
    "  --isaac-sim PATH       Isaac Sim directory (default: $ISAAC_SIM_PATH)" \
    "  --domain-id ID         Set ROS_DOMAIN_ID" \
    "  --help                 Show this help"
}

while (($#)); do
  case "$1" in
    --gui)
      HEADLESS=false
      ;;
    --no-rviz)
      START_RVIZ=false
      ;;
    --usd)
      USD_PATH="${2:?--usd requires a path}"
      shift
      ;;
    --robot-prim)
      ROBOT_PRIM="${2:?--robot-prim requires a path}"
      shift
      ;;
    --robot-xacro)
      ROBOT_XACRO="${2:?--robot-xacro requires a file name}"
      shift
      ;;
    --moveit-package)
      MOVEIT_PACKAGE="${2:?--moveit-package requires a package name}"
      shift
      ;;
    --action-name)
      ACTION_NAME="${2:?--action-name requires a name}"
      shift
      ;;
    --isaac-sim)
      ISAAC_SIM_PATH="${2:?--isaac-sim requires a path}"
      shift
      ;;
    --domain-id)
      export ROS_DOMAIN_ID="${2:?--domain-id requires an integer}"
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown option: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

ROS_DOMAIN_KEY="${ROS_DOMAIN_ID:-0}"
LOCK_FILE="/tmp/seer_isaac_ros2_domain_${ROS_DOMAIN_KEY}.lock"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  printf '%s\n' \
    "Another SEER Isaac ROS 2 stack is already running in ROS_DOMAIN_ID=${ROS_DOMAIN_KEY}." \
    "Stop it first, or start this stack with a different --domain-id." >&2
  exit 1
fi

if [[ ! -x "$ISAAC_SIM_PATH/python.sh" ]]; then
  printf 'Isaac Sim python.sh was not found under %s\n' "$ISAAC_SIM_PATH" >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash

WORKSPACE_ROOT=""
for candidate in \
  "$SCRIPT_DIR/../../../.." \
  "$SCRIPT_DIR/../../.."; do
  candidate="$(cd -- "$candidate" 2>/dev/null && pwd || true)"
  if [[ -n "$candidate" && -f "$candidate/install/setup.bash" ]]; then
    WORKSPACE_ROOT="$candidate"
    break
  fi
done

if [[ -z "$WORKSPACE_ROOT" ]]; then
  printf 'Could not locate the built ROS 2 workspace. Build it with colcon first.\n' >&2
  exit 1
fi

source "$WORKSPACE_ROOT/install/setup.bash"
set -u

if [[ -z "$USD_PATH" ]]; then
  PACKAGE_SHARE="$(ros2 pkg prefix --share seer_description)"
  USD_PATH="$PACKAGE_SHARE/urdf/seer_aubo.usd"
fi

if [[ ! -f "$USD_PATH" ]]; then
  printf 'USD file does not exist: %s\n' "$USD_PATH" >&2
  exit 1
fi

# Running two Kit/Isaac instances on this workstation has already caused a
# renderer crash. Make the operator close a manually opened GUI first.
if pgrep -u "$USER" -f '/isaacsim[^ ]*/kit/kit .*isaacsim' >/dev/null; then
  printf '%s\n' \
    'An Isaac Sim GUI process is already running.' \
    'Close it before starting this automated stack.' >&2
  exit 1
fi

RUNNER="$SCRIPT_DIR/isaac_ros2_runner.py"
if [[ ! -f "$RUNNER" ]]; then
  printf 'Isaac runner was not found: %s\n' "$RUNNER" >&2
  exit 1
fi

RUNNER_ARGS=(
  --usd "$USD_PATH"
  --robot-prim "$ROBOT_PRIM"
  --command-topic "$COMMAND_TOPIC"
  --state-topic "$STATE_TOPIC"
)
if [[ "$HEADLESS" == true ]]; then
  RUNNER_ARGS+=(--headless)
fi

ISAAC_PID=""
cleanup() {
  if [[ -n "$ISAAC_PID" ]]; then
    kill -TERM -- "-$ISAAC_PID" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$ISAAC_PID" 2>/dev/null || break
      sleep 0.5
    done
    if kill -0 "$ISAAC_PID" 2>/dev/null; then
      kill -KILL -- "-$ISAAC_PID" 2>/dev/null || true
    fi
    wait "$ISAAC_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

printf 'Starting Isaac Sim with %s\n' "$USD_PATH"
setsid "$ISAAC_SIM_PATH/python.sh" "$RUNNER" "${RUNNER_ARGS[@]}" &
ISAAC_PID=$!

printf 'Waiting for %s' "$STATE_TOPIC"
ready=false
for _ in $(seq 1 120); do
  if ! kill -0 "$ISAAC_PID" 2>/dev/null; then
    printf '\nIsaac Sim exited before the ROS 2 bridge became ready.\n' >&2
    wait "$ISAAC_PID" || true
    exit 1
  fi
  if ros2 topic list 2>/dev/null | grep -Fxq "$STATE_TOPIC"; then
    ready=true
    break
  fi
  printf '.'
  sleep 1
done
printf '\n'

if [[ "$ready" != true ]]; then
  printf 'Timed out waiting for %s\n' "$STATE_TOPIC" >&2
  exit 1
fi

ros2 launch seer_description bringup_isaac.launch.py \
  start_rviz:="$START_RVIZ" \
  command_topic:="$COMMAND_TOPIC" \
  robot_xacro:="$ROBOT_XACRO" \
  moveit_package:="$MOVEIT_PACKAGE" \
  action_name:="$ACTION_NAME"
