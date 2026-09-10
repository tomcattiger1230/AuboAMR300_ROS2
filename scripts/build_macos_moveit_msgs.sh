#!/bin/bash
# Build only the matching interface package; no local MoveIt server is needed.
set -euo pipefail
GUI_ENV="${AUBO_GUI_ENV:-$HOME/.venvs/aubo-ros-lyrical}"
OVERLAY="${AUBO_GUI_OVERLAY:-$HOME/.local/share/aubo-gui-overlay}"
export PATH="$GUI_ENV/bin:$PATH"
export AMENT_PREFIX_PATH="$GUI_ENV"
export CMAKE_PREFIX_PATH="$GUI_ENV"
export ROS_DISTRO=lyrical ROS_VERSION=2 ROS_PYTHON_VERSION=3
mkdir -p "$OVERLAY/src"
if [[ ! -d "$OVERLAY/src/moveit_msgs" ]]; then
  git clone --depth 1 --branch 2.7.2 https://github.com/moveit/moveit_msgs.git "$OVERLAY/src/moveit_msgs"
fi
if [[ "$(git -C "$OVERLAY/src/moveit_msgs" rev-parse HEAD)" != 15daf790c8008665f1e577755562f14b43bbd410 ]]; then
  echo "moveit_msgs source must match the tested 2.7.2 commit" >&2
  exit 1
fi
cmake -S "$OVERLAY/src/moveit_msgs" -B "$OVERLAY/build" -G Ninja \
  -DCMAKE_INSTALL_PREFIX="$OVERLAY/install" -DPython3_EXECUTABLE="$GUI_ENV/bin/python" \
  -DCMAKE_SHARED_LINKER_FLAGS="-L$GUI_ENV/lib" -DCMAKE_MODULE_LINKER_FLAGS="-L$GUI_ENV/lib" \
  -DCMAKE_INSTALL_RPATH="$OVERLAY/install/lib;$GUI_ENV/lib" \
  -DPYTHON_EXECUTABLE="$GUI_ENV/bin/python" -DBUILD_TESTING=OFF
cmake --build "$OVERLAY/build" --parallel 4
cmake --install "$OVERLAY/build"
