#!/usr/bin/env bash
# 使用当前会话的X11凭据启动本机容器；失败返回非零，down保留实验数据。
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime="$ROOT/artifacts/container-runtime"
action="${1:-help}"
case "$action" in
  up)
    : "${DISPLAY:?A graphical desktop session is required}"
    command -v xauth >/dev/null
    [[ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]] || { echo "NVIDIA EGL vendor manifest is missing" >&2; exit 1; }
    mkdir -p "$runtime/sitl" "$runtime/roslogs" "$ROOT/artifacts/control-locks"
    chmod 700 "$runtime"
    cookie="$runtime/xauthority"
    touch "$cookie"
    chmod 600 "$cookie"
    records="$(xauth nlist "$DISPLAY")"
    [[ -n "$records" ]] || { echo 'No X11 authorization cookie found' >&2; exit 1; }
    printf '%s\n' "$records" | sed 's/^..../ffff/' | xauth -f "$cookie" nmerge -
    printf 'USV_UID=%s\nUSV_GID=%s\nDISPLAY=%s\nUSV_XAUTHORITY=%s\n' "$(id -u)" "$(id -g)" "$DISPLAY" "$cookie" > "$runtime/runtime.env"
    args=(up -d) ;;
  down) args=(down) ;;
  logs) args=(logs --tail 100) ;;
  status) args=(ps) ;;
  help|--help|-h) echo 'Usage: ./usv container {up|down|logs|status}'; exit 0 ;;
  *) echo "Unknown container action: $action" >&2; exit 2 ;;
esac
[[ -f "$runtime/runtime.env" ]] || { echo 'Run container up from the desktop session first' >&2; exit 1; }
exec docker compose --project-name usv-desktop --env-file "$runtime/runtime.env" -f "$ROOT/deploy/compose.yaml" "${args[@]}"
