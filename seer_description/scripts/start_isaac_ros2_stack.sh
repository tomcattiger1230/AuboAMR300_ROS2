#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-$HOME/isaacsim}"
ROS_BRIDGE_MODE="auto"
BRIDGE_DISTRO="jazzy"
RENDERER=""
HEADLESS=true
START_RVIZ=true
USD_PATH=""
ROBOT_PRIM="/World/seer_aubo_composite/base_footprint"
ROBOT_XACRO="composite_robot.urdf.xacro"
MOVEIT_PACKAGE="seer_aubo_moveit_config"
ACTION_NAME="/aubo_arm_controller_wo_gripper/follow_joint_trajectory"
COMMAND_TOPIC="/isaac_joint_commands"
STATE_TOPIC="/joint_states"
CAMERA_PROFILE="gemini"
CAMERA_RESOLUTION="preview"

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
    "  --ros-bridge-mode MODE auto (Lyrical: internal), system, or internal" \
    "  --bridge-distro NAME   Bundled backend: jazzy (default) or humble" \
    "  --renderer NAME       RaytracedLighting or RealTimePathTracing" \
    "  --camera-profile NAME gemini or mv-ch100-60um" \
    "  --camera-resolution MODE preview or full" \
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
    --ros-bridge-mode)
      ROS_BRIDGE_MODE="${2:?--ros-bridge-mode requires a value}"
      shift
      ;;
    --bridge-distro)
      BRIDGE_DISTRO="${2:?--bridge-distro requires a value}"
      shift
      ;;
    --renderer)
      RENDERER="${2:?--renderer requires a value}"
      shift
      ;;
    --camera-profile)
      CAMERA_PROFILE="${2:?--camera-profile requires a value}"
      shift
      ;;
    --camera-resolution)
      CAMERA_RESOLUTION="${2:?--camera-resolution requires a value}"
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

case "$CAMERA_PROFILE" in gemini|mv-ch100-60um) ;; *) usage >&2; exit 2 ;; esac
case "$CAMERA_RESOLUTION" in preview|full) ;; *) usage >&2; exit 2 ;; esac

case "$ROS_BRIDGE_MODE" in auto|system|internal) ;; *) usage >&2; exit 2 ;; esac
case "$BRIDGE_DISTRO" in jazzy|humble) ;; *) usage >&2; exit 2 ;; esac
case "$RENDERER" in ""|RaytracedLighting|RealTimePathTracing) ;; *) usage >&2; exit 2 ;; esac

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
ROS_SETUP_FILE=""
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  ROS_SETUP_FILE="/opt/ros/${ROS_DISTRO}/setup.bash"
else
  for candidate in /opt/ros/*/setup.bash; do
    if [[ -f "$candidate" ]]; then
      ROS_SETUP_FILE="$candidate"
      break
    fi
  done
fi

if [[ -z "$ROS_SETUP_FILE" ]]; then
  printf 'No ROS 2 installation was found under /opt/ros.\n' >&2
  exit 1
fi

unset COLCON_CURRENT_PREFIX
source "$ROS_SETUP_FILE"

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

if [[ "$ROS_BRIDGE_MODE" == auto ]]; then
  if [[ "$ROS_DISTRO" == lyrical ]]; then
    ROS_BRIDGE_MODE=internal
  else
    ROS_BRIDGE_MODE=system
  fi
fi

ISAAC_ENV=()
if [[ "$ROS_BRIDGE_MODE" == internal ]]; then
  BRIDGE_LIB=""
  for extension in isaacsim.ros2.core isaacsim.ros2.bridge; do
    candidate="$ISAAC_SIM_PATH/exts/$extension/$BRIDGE_DISTRO/lib"
    if [[ -f "$candidate/librmw_implementation.so" ]]; then
      BRIDGE_LIB="$candidate"
      break
    fi
  done
  if [[ -z "$BRIDGE_LIB" ]]; then
    printf 'Bundled ROS backend %s was not found under %s\n' "$BRIDGE_DISTRO" "$ISAAC_SIM_PATH" >&2
    exit 1
  fi
  ISAAC_ENV=(env -u ROS_DISTRO -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH
    -u VIRTUAL_ENV -u PYTHONPATH -u PYTHONHOME
    "LD_LIBRARY_PATH=$BRIDGE_LIB" "RMW_IMPLEMENTATION=rmw_fastrtps_cpp")
  RENDERER="${RENDERER:-RaytracedLighting}"
else
  RENDERER="${RENDERER:-RealTimePathTracing}"
fi

for candidate in "$WORKSPACE_ROOT"/.venv-isaac/lib/python*/site-packages; do
  if [[ -d "$candidate" ]]; then
    export PYTHONPATH="$candidate:${PYTHONPATH:-}"
    break
  fi
done

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
if pgrep -u "$USER" -f '/isaacsim[^ ]*/kit/(kit .*isaacsim|python/bin/python3 .*\.py)' >/dev/null; then
  printf '%s\n' \
    'An Isaac Sim process is already running.' \
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
  --cmd-vel-topic /isaac_cmd_vel
  --renderer "$RENDERER"
  --camera-profile "$CAMERA_PROFILE"
  --camera-resolution "$CAMERA_RESOLUTION"
)
if [[ "$ROS_BRIDGE_MODE" == internal ]]; then
  RUNNER_ARGS+=(--internal-ros-distro "$BRIDGE_DISTRO")
fi
if [[ "$HEADLESS" == true ]]; then
  RUNNER_ARGS+=(--headless)
fi

ISAAC_PID=""
READY_DIR="$(mktemp -d /tmp/seer_isaac_ready.XXXXXX)"
RUNNER_ARGS+=(--ready-file "$READY_DIR/ready")
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
  rm -rf -- "$READY_DIR"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

printf 'Starting Isaac Sim with %s\n' "$USD_PATH"
printf 'Host ROS: %s; Isaac ROS mode: %s; renderer: %s\n' "$ROS_DISTRO" "$ROS_BRIDGE_MODE" "$RENDERER"
setsid "${ISAAC_ENV[@]}" "$ISAAC_SIM_PATH/python.sh" "$RUNNER" "${RUNNER_ARGS[@]}" &
ISAAC_PID=$!

printf 'Waiting for Isaac simulation startup'
ready=false
for _ in $(seq 1 120); do
  if ! kill -0 "$ISAAC_PID" 2>/dev/null; then
    printf '\nIsaac Sim exited before the ROS 2 bridge became ready.\n' >&2
    wait "$ISAAC_PID" || true
    exit 1
  fi
  if [[ -f "$READY_DIR/ready" ]]; then
    ready=true
    break
  fi
  printf '.'
  sleep 1
done
printf '\n'

if [[ "$ready" != true ]]; then
  printf 'Timed out waiting for Isaac simulation startup\n' >&2
  exit 1
fi

ros2 launch seer_description bringup_isaac.launch.py \
  start_rviz:="$START_RVIZ" \
  command_topic:="$COMMAND_TOPIC" \
  robot_xacro:="$ROBOT_XACRO" \
  moveit_package:="$MOVEIT_PACKAGE" \
  camera_profile:="$CAMERA_PROFILE" \
  action_name:="$ACTION_NAME"
