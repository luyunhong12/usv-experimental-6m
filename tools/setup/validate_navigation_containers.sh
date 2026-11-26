#!/usr/bin/env bash
# Independent instance 43; never connects to the desktop SITL/QGC instance 10.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
label="${1:-navigation_$(date -u +%Y%m%dT%H%M%SZ)}"
[[ "$label" =~ ^[A-Za-z][A-Za-z0-9_]*$ ]] || { echo 'Invalid label' >&2; exit 2; }
shift "$(( $# > 0 ? 1 : 0 ))"
# Only forward supported acceptance options; do not allow endpoint/output overrides.
remaining=("$@")
while (($#)); do
  case "$1" in
    --failsafe-only|--mirror-course|--current-navigation|--response-only) shift ;;
    --timeout|--current-east|--current-north|--wind-east|--wind-north|--wave-gain) [[ $# -ge 2 ]] || { echo 'Missing numeric argument' >&2; exit 2; }; shift 2 ;;
    *) echo "Unsupported option: $1" >&2; exit 2 ;;
  esac
done
set -- "${remaining[@]}"
out="$ROOT/artifacts/navigation/$label"
[[ ! -e "$out" ]] || { echo "Output already exists: $out" >&2; exit 2; }
mkdir -p "$out/sitl" "$out/inputs/config"
config_source="${USV_NAV_CONFIG_DIR:-$ROOT/ros_ws/src/usv_ardupilot/config}"
cp "$config_source/"*.parm "$out/inputs/config/"
cp "$ROOT/tests/integration/validate_navigation.py" "$out/inputs/"
cp "$ROOT/tools/lib/navigation_acceptance.py" "$out/inputs/"
cp "$0" "$out/inputs/runner.sh"
(cd "$out/inputs" && sha256sum config/*.parm *.py runner.sh) > "$out/input-sha256.txt"
sim="usv-navigation-sim"
sitl="usv-navigation-sitl"
# Fixed names prevent two tests sharing ports 6190/9432/5931.
for name in "$sim" "$sitl"; do
  if docker container inspect "$name" >/dev/null 2>&1; then
    echo "Existing test container: $name; inspect it before retrying" >&2
    exit 1
  fi
done
sim_created=false
sitl_created=false
cleanup() {
  if $sim_created; then docker stop -t 15 "$sim" >/dev/null 2>&1 || true; docker rm "$sim" >/dev/null 2>&1 || true; fi
  if $sitl_created; then docker logs "$sitl" > "$out/sitl-console.log" 2>&1 || true; docker stop -t 10 "$sitl" >/dev/null 2>&1 || true; docker rm "$sitl" >/dev/null 2>&1 || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
common=(--init --network host --user "$(id -u):$(id -g)" --cap-drop ALL --security-opt no-new-privileges)
sim_image="$(docker image inspect -f '{{.Id}}' "${USV_SIM_IMAGE:-usv-simulation:sitl-gui}")"
sitl_image="$(docker image inspect -f '{{.Id}}' "${USV_SITL_IMAGE:-usv-sitl:rover-4.7.3}")"
docker image inspect "$sim_image" "$sitl_image" > "$out/images.json"
docker create --name "$sitl" "${common[@]}" \
  -v "$out/inputs/config:/config:ro" -v "$out/sitl:/data" \
  "$sitl_image" --model JSON --sim-address 127.0.0.1 --sim-port-out 9432 \
  --instance 43 --home=-33.724223,150.679736,0,90 \
  --defaults /config/boat.parm,/config/platform15.parm,/config/navigation_low_speed.parm \
  --speedup 1 --rate 100 --wipe > "$out/sitl-id"
sitl_created=true
docker create --name "$sim" "${common[@]}" \
  -v "$ROOT/dependencies/vendor/python:/work/usv/dependencies/vendor/python:ro" \
  -v "$out/inputs/navigation_acceptance.py:/work/usv/tools/lib/navigation_acceptance.py:ro" \
  -e ROS_DOMAIN_ID=143 -e GZ_IP=127.0.0.1 \
  -v "$out:/work/usv/artifacts/navigation/$label" \
  -v "$out/inputs/validate_navigation.py:/work/usv/tests/integration/validate_navigation.py:ro" \
  -v "$out/inputs/config:/work/usv/ros_ws/src/usv_ardupilot/config:ro" \
  "$sim_image" bash -c \
  'source /opt/ros/jazzy/setup.bash; source /work/usv/ros_ws/install/setup.bash; exec python3 /work/usv/tests/integration/validate_navigation.py --external-sitl --isolated-water --no-rendering --label "$1" --output "/work/usv/artifacts/navigation/$1/run" "${@:2}"' \
  bash "$label" "$@" > "$out/sim-id"
sim_created=true
docker start "$sitl" >/dev/null
set +e
docker start -a "$sim" 2>&1 | tee "$out/console.log"
attach_code=${PIPESTATUS[0]}
set -e
code="$(docker inspect -f '{{.State.ExitCode}}' "$sim")"
if [[ "$code" == 0 && "$attach_code" != 0 ]]; then code="$attach_code"; fi
echo "Navigation evidence: $out (exit $code)"
exit "$code"
