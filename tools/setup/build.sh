#!/usr/bin/env bash
set -eo pipefail
SIM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../ros_ws" && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$SIM_ROOT"
if [[ -f "$SIM_ROOT/../datasets/reference/boat_export.json" ]]; then
  python3 "$SIM_ROOT/../tools/modeling/build_pbr_materials.py"
  python3 "$SIM_ROOT/../tools/modeling/prepare_hull.py"
fi
exec colcon build --base-paths src ../dependencies/src/vrx --parallel-workers 2 "$@"
