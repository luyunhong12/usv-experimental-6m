#!/usr/bin/env python3
"""检查公开平台启动入口与必需进程退出处理。

Smoke-test the real coupled launcher, including bounded child cleanup.
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
from project_paths import WORKSPACE, tool_path
import os
import time
import json
import signal
import subprocess
import argparse
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path)
parser.add_argument("--kill-controller", action="store_true")
parser.add_argument("--control", choices=["ros", "ardupilot"], default="ros")
args = parser.parse_args()
stem = "launcher_smoke" if args.control == "ros" else "launcher_ardupilot_smoke"
if args.kill_controller:
    stem = "launcher_controller_exit"
w = WORKSPACE
out = args.output or w / "log/platform_validation"
out.mkdir(parents=True, exist_ok=True)
os.environ.update(
    GZ_PARTITION="usv_performance_launcher_test", GZ_IP="127.0.0.1", ROS_DOMAIN_ID="90"
)


def _load_runtime_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from gz.transport13 import Node
    from gz.msgs10.double_v_pb2 import Double_V

    return (
        Node,
        Double_V,
    )


(
    Node,
    Double_V,
) = _load_runtime_dependencies()

node = Node()
rows = []


def record_message_1(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return rows.append(list(m.data))


node.subscribe(Double_V, "/usv/propulsion/state", record_message_1)
started_epoch = time.time()
f = (out / (stem + ".log")).open("w")
p = subprocess.Popen(
    [
        "bash",
        str(tool_path("run_platform.sh")),
        "--control",
        args.control,
        "--wave-gain",
        "0",
        "headless:=True",
    ],
    stdout=f,
    stderr=subprocess.STDOUT,
    start_new_session=True,
)
children = set()
start = time.monotonic()
injected = False
automatic_exit = False


def descend(pid):
    """递归收集测试进程的子进程标识。

    Args:
        pid: 需要枚举后代的 Linux 进程编号
    """
    try:
        ids = [
            int(x) for x in Path(f"/proc/{pid}/task/{pid}/children").read_text().split()
        ]
    except FileNotFoundError:
        return
    for child in ids:
        children.add(child)
        descend(child)


try:
    while time.monotonic() - start < 60 and p.poll() is None:
        descend(p.pid)
        if rows and rows[-1][0] > 5:
            rf = w / "log/platform-ready-usv_performance_launcher_test.json"
            if args.control != "ros" or (
                rf.exists() and rf.stat().st_mtime >= started_epoch
            ):
                break
        time.sleep(0.1)
    if args.kill_controller:
        descend(p.pid)
        for child in children:
            try:
                command = Path(f"/proc/{child}/cmdline").read_bytes()
            except FileNotFoundError:
                continue
            if b"/boat_control.py" in command:
                os.kill(child, signal.SIGTERM)
                injected = True
                break
        if injected:
            try:
                p.wait(timeout=15)
                automatic_exit = True
            except subprocess.TimeoutExpired:
                pass
finally:
    if p.poll() is None:
        os.kill(p.pid, signal.SIGINT)
        try:
            p.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait()
    f.close()
live = []
for child in children:
    try:
        status = Path(f"/proc/{child}/status").read_text()
        if "\nState:\tZ" not in status:
            live.append(child)
    except FileNotFoundError:
        pass
ready_file = w / "log/platform-ready-usv_performance_launcher_test.json"
checks = {
    "startup_ready": (
        bool(
            ready_file.exists()
            and ready_file.stat().st_mtime >= started_epoch
            and json.loads(ready_file.read_text())["ready"]
        )
        if args.control == "ros"
        else True
    ),
    "physics_stream": bool(rows and rows[-1][0] > 5),
    "no_uncommanded_propulsion": bool(rows and max(abs(r[1]) for r in rows) < 1e-9),
    "bounded_cleanup": p.returncode in [0, 130],
    "no_live_children": not live,
}
if args.kill_controller:
    checks.update(
        injected_controller_exit=injected, stack_exits_automatically=automatic_exit
    )
r = {
    "passed": all(checks.values()),
    "checks": checks,
    "exit_code": p.returncode,
    "observed_children": len(children),
    "remaining_children": live,
}
(out / (stem + ".json")).write_text(json.dumps(r, indent=2))
print(json.dumps(r, indent=2))
raise SystemExit(0 if r["passed"] else 1)
