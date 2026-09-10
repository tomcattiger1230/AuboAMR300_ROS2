#!/bin/bash
set -e
SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
GUI_ENV="${AUBO_GUI_ENV:-$HOME/.venvs/aubo-ros-lyrical}"
if [[ ! -x "$GUI_ENV/bin/python" ]]; then
  echo "请先按 aubo_control_gui/README.md 安装 macOS ROS 环境：$GUI_ENV"
  exit 1
fi
export ROS_DISTRO=lyrical ROS_VERSION=2 ROS_PYTHON_VERSION=3
export PATH="$GUI_ENV/bin:$PATH"
export AMENT_PREFIX_PATH="$GUI_ENV${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
GUI_OVERLAY="${AUBO_GUI_OVERLAY:-$HOME/.local/share/aubo-gui-overlay}/install"
if [[ -f "$GUI_OVERLAY/share/moveit_msgs/local_setup.bash" ]]; then
  source "$GUI_OVERLAY/share/moveit_msgs/local_setup.bash"
else
  echo "请先执行 scripts/build_macos_moveit_msgs.sh 对齐远端 MoveIt 消息版本" >&2
  exit 1
fi
export DYLD_LIBRARY_PATH="$GUI_OVERLAY/lib:$GUI_ENV/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
exec "$GUI_ENV/bin/python" "$SCRIPT_DIR/start_macos_gui.py" "$@"
