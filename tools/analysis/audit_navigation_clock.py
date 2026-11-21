#!/usr/bin/env python3
"""审计飞控与物理仿真时间推进记录。

Audit recorded ArduPilot disarm causes and bridge time steps after navigation.
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
from project_paths import WORKSPACE
import sys
import json
import argparse

w = WORKSPACE
sys.path.insert(0, str(w / "../dependencies/vendor/python"))


def _load_runtime_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from pymavlink import mavutil

    return (mavutil,)


(mavutil,) = _load_runtime_dependencies()

p = argparse.ArgumentParser()
p.add_argument("label")
a = p.parse_args()
if not a.label.isidentifier():
    p.error("Invalid label")
out = w / "log/platform_validation" / a.label
m = mavutil.mavlink_connection(str(out / "flight_control.BIN"))
arm = []
while True:
    d = m.recv_match(type="ARM")
    if d is None:
        break
    arm.append(d.to_dict())
t = json.loads((out / "json_frames.jsonl.summary.json").read_text())
checks = {
    "armed_event_recorded": any(x["ArmState"] == 1 for x in arm),
    "no_cpu_failsafe_disarm": not any(
        x["ArmState"] == 0 and x["Method"] == 6 for x in arm
    ),
    "bounded_steps": 0 < t["max_time_step_s"] <= 0.020001,
    "physics_frames": t["frames_sent"] > 100,
}
r = {
    "passed": all(checks.values()),
    "checks": checks,
    "arm_events": arm,
    "timing": t,
    "basis": "ArduPilot AP_Arming::Method::CPUFAILSAFE=6, local pinned source",
}
(out / "flight_log_audit.json").write_text(json.dumps(r, indent=2))
print(json.dumps(r, indent=2))
raise SystemExit(0 if r["passed"] else 1)
