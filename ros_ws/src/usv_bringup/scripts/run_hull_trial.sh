#!/usr/bin/env bash
set -eo pipefail
SIM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$SIM_ROOT/install/setup.bash"
export ROS_LOG_DIR="$SIM_ROOT/log/hull-runtime"
export GZ_IP="${GZ_IP:-127.0.0.1}"
export GZ_PARTITION="${GZ_PARTITION:-usv_hull_trial}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$SIM_ROOT/install/usv_physics/lib:$SIM_ROOT/install/usv_simulation/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch vrx_gz vrx_environment.launch.py \
  world:="$SIM_ROOT/src/usv_simulation/worlds/usv_hull_trial.sdf" \
  headless:=True extra_gz_args:=--headless-rendering bridge_competition_topics:=False "$@"
