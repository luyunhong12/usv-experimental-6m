#!/usr/bin/env bash
set -eo pipefail
SIM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
scene_args=(--wave-gain 0.3)
launch_args=()
while (($#)); do
  case "$1" in
    --name|--second-name|--wave-gain|--profile|--task|--payload-kg|--fuel-l|--mass-case|--power-mode) scene_args+=("$1" "$2"); shift 2 ;;
    --payload-center) scene_args+=("$1" "$2" "$3" "$4"); shift 4 ;;
    *) launch_args+=("$1"); shift ;;
  esac
done
python3 "$SIM_ROOT/../tools/modeling/prepare_sydney.py" "${scene_args[@]}"
exec bash "$SIM_ROOT/src/usv_bringup/scripts/run_hull_trial.sh" \
  world:="$SIM_ROOT/src/usv_simulation/worlds/usv_sydney.sdf" headless:=False "${launch_args[@]}"
