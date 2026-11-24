#!/usr/bin/env bash
# Start the 15 km/h engineering simulation with correctly matched interfaces.
set -eo pipefail
SIM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
mapfile -t platform_cfg < <(python3 -c 'import json,sys; c=json.load(open(sys.argv[1])); print(c["profile"]); print(c["speed_feedforward_m_s"]); print(c["wave_gain_default"])' "$SIM_ROOT/src/usv_bringup/config/platform.json")
[[ ${#platform_cfg[@]} -eq 3 ]] || { echo 'Invalid platform configuration' >&2; exit 2; }
control=ros
if [[ "${1:-}" == --control ]]; then control="$2"; shift 2; fi
case "$control" in ros|ardupilot|ardupilot-external) ;; *) echo 'Use --control ros, ardupilot or ardupilot-external' >&2; exit 2;; esac
# The coupled launcher intentionally fixes boat/profile; custom multi-boat use the lower-level launchers.
for arg in "$@"; do
  case "$arg" in --name|--second-name|--profile|world:=*) echo "Use run_sydney.sh for custom boat/profile/world: $arg" >&2; exit 2;; esac
done
export GZ_PARTITION="${GZ_PARTITION:-usv_platform15}"
children=()
cleanup() {
  trap '' INT TERM
  for pid in "${children[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for attempt in {1..40}; do
    alive=0
    for pid in "${children[@]}"; do if kill -0 "$pid" 2>/dev/null; then alive=1; fi; done
    [[ "$alive" == 0 ]] && break
    sleep 0.2
  done
  for pid in "${children[@]}"; do if kill -0 "$pid" 2>/dev/null; then kill -KILL -- "-$pid" 2>/dev/null || true; fi; done
  for pid in "${children[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT TERM
setsid bash "$SIM_ROOT/src/usv_bringup/scripts/run_sydney.sh" --profile "${platform_cfg[0]}" --wave-gain "${platform_cfg[2]}" "$@" & children+=("$!")
if [[ "$control" == ros ]]; then
  setsid bash "$SIM_ROOT/src/usv_bringup/scripts/run_interfaces.sh" "speed_scale:=${platform_cfg[1]}" & children+=("$!")
else
  setsid bash "$SIM_ROOT/src/usv_bringup/scripts/run_interfaces.sh" control:=False & children+=("$!")
  setsid bash "$SIM_ROOT/src/usv_bringup/scripts/run_ardupilot.sh" --port 9102 & children+=("$!")
  if [[ "$control" == ardupilot ]]; then
    setsid python3 "$SIM_ROOT/src/usv_bringup/scripts/run_sitl.py" --instance 10 --profile platform15 & children+=("$!")
  fi
fi
if [[ "$control" == ros ]]; then
  source /opt/ros/jazzy/setup.bash
  source "$SIM_ROOT/install/setup.bash"
  setsid python3 "$SIM_ROOT/src/usv_bringup/scripts/wait_control_ready.py" --output "$SIM_ROOT/log/platform-ready-${GZ_PARTITION}.json" & readiness_pid=$!
  children+=("$readiness_pid")
  wait "$readiness_pid" || { echo 'Platform not ready; see readiness diagnostic JSON.' >&2; exit 1; }
  children=("${children[@]:0:2}")
  echo 'Boat ready: physics, energy, actuators, IMU and GPS are fresh.'
fi
# Any child failure tears down the stack rather than leaving a half-running vessel.
wait -n "${children[@]}"
