#!/bin/bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
DEFAULT_GUI_ENV="$HOME/.venvs/aubo-ros-lyrical"
DEFAULT_GUI_OVERLAY="$HOME/.local/share/aubo-gui-overlay"
if [[ -x "$HOME/.venvs/aubo-unified-ros-lyrical/bin/python" ]]; then
  DEFAULT_GUI_ENV="$HOME/.venvs/aubo-unified-ros-lyrical"
  DEFAULT_GUI_OVERLAY="$HOME/.local/share/aubo-unified-gui-overlay"
fi
GUI_ENV="${AUBO_GUI_ENV:-$DEFAULT_GUI_ENV}"
if [[ ! -x "$GUI_ENV/bin/python" ]]; then
  echo "未找到 ROS GUI 环境：$GUI_ENV。离线预览可运行：.venv/bin/python scripts/start_unified_gui.py" >&2
  exit 1
fi
export ROS_DISTRO=lyrical ROS_VERSION=2 ROS_PYTHON_VERSION=3
export PATH="$GUI_ENV/bin:$PATH"
export AMENT_PREFIX_PATH="$GUI_ENV${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
GUI_OVERLAY="${AUBO_GUI_OVERLAY:-$DEFAULT_GUI_OVERLAY}/install"
set +u  # Generated ROS setup scripts allow unset tracing variables.
for GUI_PACKAGE in moveit_msgs aubo_bridge_msgs; do
  if [[ -f "$GUI_OVERLAY/share/$GUI_PACKAGE/local_setup.bash" ]]; then
    source "$GUI_OVERLAY/share/$GUI_PACKAGE/local_setup.bash"
  fi
done
set -u
export DYLD_LIBRARY_PATH="$GUI_OVERLAY/lib:$GUI_ENV/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
exec "$GUI_ENV/bin/python" "$SCRIPT_DIR/start_unified_gui.py" "$@"
