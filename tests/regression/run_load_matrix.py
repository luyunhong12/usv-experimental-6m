#!/usr/bin/env python3
"""调度载荷与能源组合工况并保存各自结果。

Sequential reproducible native Gazebo+ROS physical test matrix.
"""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys

# 项目脚本可从任意工作目录启动；建模工具保持在共享库之前。
sys.path.insert(
    0,
    str(
        next(
            p
            for p in _ProjectPath(__file__).resolve().parents
            if (p / "project.toml").is_file()
        )
        / "tools/lib"
    ),
)
sys.path.insert(
    0,
    str(
        next(
            p
            for p in _ProjectPath(__file__).resolve().parents
            if (p / "project.toml").is_file()
        )
        / "tools/modeling"
    ),
)
from project_paths import tool_path
import argparse
import json
import subprocess
import sys
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("--output", type=Path, required=True)
p.add_argument("--spec", type=Path, required=True)
a = p.parse_args()


def _load_runtime_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from concurrent.futures import ThreadPoolExecutor
    import fcntl

    return (
        ThreadPoolExecutor,
        fcntl,
    )


(
    ThreadPoolExecutor,
    fcntl,
) = _load_runtime_dependencies()


def run(case):
    """独占工况锁后执行或复用已完成的载荷工况。

    Args:
        case: 工况名称
    """
    with (a.output / (case + ".lock")).open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        result = a.output / case / "result.json"
        if result.exists() and json.loads(result.read_text()).get("passed"):
            return
        print("START", case, flush=True)
        with (a.output / (case + "_console.log")).open("w") as log:
            proc = subprocess.run(
                [
                    sys.executable,
                    str(tool_path("validate_load_matrix_case.py")),
                    "--case",
                    case,
                    "--spec",
                    str(a.spec.resolve()),
                    "--output",
                    str((a.output / case).resolve()),
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
            )
        print("END", case, proc.returncode, flush=True)


with ThreadPoolExecutor(max_workers=3) as pool:
    list(pool.map(run, json.loads(a.spec.read_text())))
results = {
    k: (
        json.loads((a.output / k / "result.json").read_text())
        if (a.output / k / "result.json").exists()
        else {"passed": False, "error": "missing result"}
    )
    for k in json.loads(a.spec.read_text())
}
(a.output / "summary.json").write_text(
    json.dumps(
        {"passed": all(x["passed"] for x in results.values()), "cases": results},
        indent=2,
    )
)

raise SystemExit(0 if all(x["passed"] for x in results.values()) else 1)
