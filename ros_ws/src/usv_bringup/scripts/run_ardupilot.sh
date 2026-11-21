#!/usr/bin/env bash
set -eo pipefail
SIM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$SIM_ROOT/install/setup.bash"
export GZ_IP="${GZ_IP:-127.0.0.1}"
export GZ_PARTITION="${GZ_PARTITION:-usv_hull_trial}"
exec python3 "$SIM_ROOT/src/usv_ardupilot/scripts/json_bridge.py" "$@"
