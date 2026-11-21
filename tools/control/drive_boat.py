#!/usr/bin/env python3
"""通过 Gazebo 控制接口发送有时限的推进与转向请求。

Maintain throttle heartbeat; steering is physical azimuth angle in FLU.
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
import argparse
import math
import os
import time

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--throttle", type=float, default=0)
p.add_argument("--steering-deg", type=float, default=0)
p.add_argument("--duration", type=float, default=30)
p.add_argument("--name", default="usv")
a = p.parse_args()
if not a.name.isidentifier():
    p.error("Invalid boat name")
prefix = "/" + a.name
if (
    not all(map(math.isfinite, [a.throttle, a.steering_deg, a.duration]))
    or not -1 <= a.throttle <= 1
    or not -35 <= a.steering_deg <= 35
    or a.duration <= 0
):
    p.error("throttle [-1,1], steering [-35,35], duration > 0 required")
os.environ.setdefault("GZ_PARTITION", "usv_hull_trial")
os.environ.setdefault("GZ_IP", "127.0.0.1")


def _load_runtime_dependencies_1():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from gz.transport13 import Node
    from usv_control.ownership import ControlLease

    return (
        Node,
        ControlLease,
    )


(
    Node,
    ControlLease,
) = _load_runtime_dependencies_1()

lease = ControlLease(a.name, "drive_boat")


def _load_runtime_dependencies_2():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from gz.msgs10.double_pb2 import Double

    return (Double,)


(Double,) = _load_runtime_dependencies_2()

n = Node()
t = n.advertise(prefix + "/propulsion/throttle", Double)
s = n.advertise(prefix + "/steering/angle", Double)


def send(throttle, angle):
    """发布归一化油门和转向角请求。

    Args:
        throttle: 有符号归一化油门
        angle: 转向角，度，发送前转为 rad
    """
    t.publish(Double(data=throttle))
    s.publish(Double(data=math.radians(angle)))


try:
    start = time.monotonic()
    while time.monotonic() - start < a.duration:
        send(a.throttle, a.steering_deg)
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    for _ in range(10):
        send(0, 0)
        time.sleep(0.05)

lease.close()
