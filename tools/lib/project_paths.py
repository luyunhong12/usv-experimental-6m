"""Repository paths shared by development tools (no dependence on cwd)."""
from pathlib import Path
import json
PROJECT_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = PROJECT_ROOT / "ros_ws"
_TOOL_PATHS = json.loads((Path(__file__).parent / "tool_paths.json").read_text())
def tool_path(name):
    return PROJECT_ROOT / _TOOL_PATHS[name]
