#!/usr/bin/env bash
set -eo pipefail
SIM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$SIM_ROOT/install/setup.bash" ]]; then
  echo "请先执行 ../tools/setup/build.sh" >&2
  exit 1
fi
source "$SIM_ROOT/install/setup.bash"
export ROS_LOG_DIR="$SIM_ROOT/log/runtime"
export GZ_PARTITION="${GZ_PARTITION:-usv_vrx_baseline}"
# model://package_name/... requires each isolated install share directory.
for pkg in wamv_description wamv_gazebo vrx_gazebo; do
  export GZ_SIM_RESOURCE_PATH="$SIM_ROOT/install/$pkg/share:${GZ_SIM_RESOURCE_PATH:-}"
done
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch vrx_gz competition.launch.py \
  world:="$SIM_ROOT/src/usv_simulation/worlds/usv_baseline_offline.sdf" \
  headless:=True extra_gz_args:=--headless-rendering "$@"
