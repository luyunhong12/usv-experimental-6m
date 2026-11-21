#!/usr/bin/env bash
set -eu
base="$(cd "$(dirname "$0")" && pwd)"
export BLENDER_MCP_DISABLE_TELEMETRY=1
export BLENDER_MCP_SAFE_MODE=1
exec "${BLENDER_BIN:-blender}" "$@" --python "$base/../../../dependencies/vendor/blender-mcp/addon.py" --python-expr 'import bpy; bpy.context.scene.blendermcp_port=19876; bpy.ops.blendermcp.start_server()'
