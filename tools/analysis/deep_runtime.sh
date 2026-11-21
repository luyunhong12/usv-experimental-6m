#!/usr/bin/env bash
# 原生物理响应与受影响推进工况；每次保存独立证据。
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/jazzy/setup.bash
# Supervisor 的自定义命令彼此独立；在本命令内确保构建成功，禁止使用旧插件。
"$ROOT/usv" build
source "$ROOT/ros_ws/install/setup.bash"
mkdir -p "$ROOT/artifacts/validation/deep-runtime"
EVIDENCE_DIR="$(mktemp -d "$ROOT/artifacts/validation/deep-runtime/$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")"
echo "运行时证据：$EVIDENCE_DIR"
for mode in force torque; do
  python3 "$ROOT/tests/integration/validate_inertia_backend.py" --mode "$mode" --output-dir "$EVIDENCE_DIR/inertia-$mode"
done
for scenario in forward reverse left right reversal; do
  python3 "$ROOT/tests/integration/validate_propulsion.py" --case "$scenario" --output-dir "$EVIDENCE_DIR/propulsion-$scenario"
done
