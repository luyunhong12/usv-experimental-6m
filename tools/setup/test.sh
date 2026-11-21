#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
mkdir -p "$ROOT/artifacts/validation/unit-tests"
EVIDENCE_DIR="$(mktemp -d "$ROOT/artifacts/validation/unit-tests/$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")"
echo "测试证据：$EVIDENCE_DIR/tests.log"
{
source /opt/ros/jazzy/setup.bash
source "$ROOT/ros_ws/install/setup.bash"
cd "$ROOT/ros_ws"
colcon test --packages-select usv_simulation usv_energy --event-handlers console_direct+
colcon test-result --verbose
python3 -m unittest discover -s "$ROOT/tests/unit" -p 'test_*.py' -v
python3 -m unittest discover -s "$ROOT/design/azimuth_thruster/仿真接口" -p 'test_*.py' -v
} 2>&1 | tee "$EVIDENCE_DIR/tests.log"
