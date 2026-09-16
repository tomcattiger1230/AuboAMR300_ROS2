#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for candidate in "$SCRIPT_DIR/../urdf" "$SCRIPT_DIR/../../share/seer_description/urdf"; do
  if [[ -f "$candidate/warehouse_finger_rebar_loading_demo.usda" ]]; then
    LOADING_USD="$(cd "$candidate" && pwd)/warehouse_finger_rebar_loading_demo.usda"
    break
  fi
done
exec "$SCRIPT_DIR/start_warehouse_finger_rebar_mono_demo.sh" \
  --usd "${LOADING_USD:?Loading scene not found}" --rebar-loading "$@"
