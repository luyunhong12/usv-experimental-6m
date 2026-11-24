"""将场景的递归 Fuel 引用解析为本地生成副本，保留第三方原文件。"""

from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET


def localize_fuel_asset(name, source_root, output_root, resolved=None):
    """复制模型并递归解析其模型、网格与纹理的 Fuel URL。

    Args:
        name: 官方模型目录名，仅允许字母、数字和下划线。
        source_root: 已下载的第三方模型根目录，不修改其中内容。
        output_root: 可写生成目录，缺失依赖会抛错，不自动联网。
        resolved: 当前递归链已建立的模型路径映射，避免循环包含。

    Returns:
        可离线解析的生成模型目录。

    Raises:
        ValueError: 模型名称或文件路径不安全，或URL格式不支持。
        FileNotFoundError: 模型或被引用文件缺失。
    """
    if not re.fullmatch(r"[A-Za-z0-9_]+", name):
        raise ValueError(f"Invalid Fuel model name: {name}")
    resolved = {} if resolved is None else resolved
    if name in resolved:
        return resolved[name]
    source = Path(source_root) / name
    if not (source / "model.sdf").is_file():
        raise FileNotFoundError(f"Missing local Fuel dependency: {source}")
    target = (Path(output_root) / name).resolve()
    shutil.copytree(source, target, dirs_exist_ok=True)
    resolved[name] = target
    tree = ET.parse(source / "model.sdf")
    for element in tree.iter():
        text = (element.text or "").strip()
        if not text.startswith("https://fuel.gazebosim.org/"):
            continue
        match = re.fullmatch(
            r"https://fuel.gazebosim.org/1.0/openrobotics/models/"
            r"([A-Za-z0-9_]+)(?:/(?:tip|[0-9]+)/files/(.+))?",
            text,
        )
        if not match:
            raise ValueError(f"Unsupported Fuel reference: {text}")
        dependency = localize_fuel_asset(match[1], source_root, output_root, resolved)
        path = (dependency / match[2]).resolve() if match[2] else dependency
        if not path.is_relative_to(dependency):
            raise ValueError(f"Unsafe Fuel file reference: {text}")
        original = Path(source_root) / match[1]
        original = original / match[2] if match[2] else original
        if not original.exists() or not path.exists():
            raise FileNotFoundError(original)
        element.text = str(path)
    tree.write(target / "model.sdf", encoding="unicode")
    return target
