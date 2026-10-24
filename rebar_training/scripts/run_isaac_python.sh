#!/usr/bin/env bash
# Run the isolated Lab Python with the existing Isaac Sim binary's libraries.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export REBAR_REPO_ROOT="${REBAR_REPO_ROOT:-$(cd "$script_dir/../.." && pwd)}"
export ISAACLAB_ROOT="${ISAACLAB_ROOT:-$HOME/Develop/IsaacLab-rebar}"
export ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-$HOME/isaacsim}"
export PYTHONEXE="${REBAR_TRAINING_PYTHON:-$ISAACLAB_ROOT/.venv/bin/python}"

if [[ ! -x "$PYTHONEXE" || ! -f "$ISAAC_SIM_ROOT/python.sh" ]]; then
    echo "Set ISAACLAB_ROOT, ISAAC_SIM_ROOT and optionally REBAR_TRAINING_PYTHON to the prepared runtime." >&2
    exit 1
fi

cd "$REBAR_REPO_ROOT"
exec bash "$ISAAC_SIM_ROOT/python.sh" "$@"
