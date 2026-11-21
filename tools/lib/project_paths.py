"""统一定位项目根、ROS 工作区与登记工具路径。

Repository paths shared by development tools (no dependence on cwd).
"""

from pathlib import Path
import json

PROJECT_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = PROJECT_ROOT / "ros_ws"
_TOOL_PATHS = json.loads((Path(__file__).parent / "tool_paths.json").read_text())


def tool_path(name):
    """从工具索引返回相对于项目根的绝对路径。

    Args:
        name: tool_paths.json 中登记的工具文件名。
    """
    return PROJECT_ROOT / _TOOL_PATHS[name]
