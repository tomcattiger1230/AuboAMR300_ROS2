#!/bin/bash
# Build the ROS interface package used by the real-arm GUI; no SDK/device access.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
GUI_ENV="${AUBO_GUI_ENV:-$HOME/.venvs/aubo-ros-lyrical}"
GUI_OVERLAY="${AUBO_GUI_OVERLAY:-$HOME/.local/share/aubo-gui-overlay}"
export PATH="$GUI_ENV/bin:$PATH"
export AMENT_PREFIX_PATH="$GUI_ENV"
export CMAKE_PREFIX_PATH="$GUI_ENV"
export ROS_DISTRO=lyrical ROS_VERSION=2 ROS_PYTHON_VERSION=3
cmake -S "$SCRIPT_DIR/../aubo_bridge_msgs" -B "$GUI_OVERLAY/build-aubo-bridge-msgs" -G Ninja \
  -DCMAKE_INSTALL_PREFIX="$GUI_OVERLAY/install" -DPython3_EXECUTABLE="$GUI_ENV/bin/python" \
  -DCMAKE_SHARED_LINKER_FLAGS="-L$GUI_ENV/lib" -DCMAKE_MODULE_LINKER_FLAGS="-L$GUI_ENV/lib" \
  -DCMAKE_INSTALL_RPATH="$GUI_OVERLAY/install/lib;$GUI_ENV/lib" \
  -DPYTHON_EXECUTABLE="$GUI_ENV/bin/python" -DBUILD_TESTING=OFF
cmake --build "$GUI_OVERLAY/build-aubo-bridge-msgs" --parallel 4
cmake --install "$GUI_OVERLAY/build-aubo-bridge-msgs"
