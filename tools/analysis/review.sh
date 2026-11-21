#!/usr/bin/env bash
# 手动审查入口；不会在 build/test/run 时自动触发。
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
SUPERVISOR_BIN="${CODE_SUPERVISOR_BIN:-$ROOT/../code-supervisor/.venv/bin/supervisor}"
if [[ ! -x "$SUPERVISOR_BIN" ]]; then
  echo "未找到 Code Supervisor：$SUPERVISOR_BIN" >&2
  echo "请设置 CODE_SUPERVISOR_BIN 为已安装的 supervisor 可执行文件路径。" >&2
  exit 2
fi
# 独立 CMake 检查也需要 Gazebo/ROS 的依赖搜索路径。
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
fi
exec "$SUPERVISOR_BIN" check "$ROOT" \
  --config "$ROOT/supervisor.toml" \
  --output "$ROOT/artifacts/validation/supervisor" "$@"
