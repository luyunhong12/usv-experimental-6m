#!/usr/bin/env python3
"""验证能源状态到 ROS 消息的桥接。

Run alongside charge_sensor test; verify ROS conversion and invalid readings.
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
import subprocess
import json
import math

W = WORKSPACE
out = W / "log/energy_validation/charge_sensor"
os.environ["GZ_PARTITION"] = (out / "partition.txt").read_text().strip()
os.environ["GZ_IP"] = "127.0.0.1"
os.environ["ROS_DOMAIN_ID"] = "177"


def _load_runtime_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    import rclpy
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import Float64
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

    return (
        rclpy,
        BatteryState,
        Float64,
        DiagnosticArray,
        DiagnosticStatus,
    )


(
    rclpy,
    BatteryState,
    Float64,
    DiagnosticArray,
    DiagnosticStatus,
) = _load_runtime_dependencies()

rclpy.init()
node = rclpy.create_node("test_energy_bridge")
battery = []
fuel = []
diagnostic = []


def record_message_1(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return battery.append(m)


node.create_subscription(BatteryState, "/usv/battery", record_message_1, 10)


def record_message_2(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return fuel.append(m.data)


node.create_subscription(Float64, "/usv/fuel_l", record_message_2, 10)


def record_message_3(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return diagnostic.extend(m.status)


node.create_subscription(
    DiagnosticArray,
    "/usv/energy/diagnostics",
    record_message_3,
    10,
)
log = (out / "ros_bridge.log").open("w")
proc = subprocess.Popen(
    ["python3", str(tool_path("energy_console.py")), "--ros", "--duration", "36"],
    stdout=log,
    stderr=subprocess.STDOUT,
)
try:
    start = time.monotonic()
    while time.monotonic() - start < 38:
        rclpy.spin_once(node, timeout_sec=0.2)
finally:
    if proc.poll() is None:
        proc.terminate()
    proc.wait(timeout=5)
    log.close()
    node.destroy_node()
    rclpy.shutdown()
valid = [m for m in battery if math.isfinite(m.voltage)]
checks = {
    "clean_exit": proc.returncode == 0,
    "battery_received": len(valid) > 2,
    "fuel_received": len(fuel) > 2,
    "charging_sign": any(m.current > 0.1 for m in valid),
    "soc_range": all(0 <= m.percentage <= 1 for m in valid),
    "invalid_nan": any(not math.isfinite(m.voltage) for m in battery),
    "invalid_diagnostic": any(m.level == DiagnosticStatus.ERROR for m in diagnostic),
    "valid_after_dropout": any(
        math.isfinite(m.voltage) and m.header.stamp.sec >= 33 for m in battery
    ),
}
r = {
    "passed": all(checks.values()),
    "checks": checks,
    "battery_messages": len(battery),
    "bridge_exit": proc.returncode,
}
(out / "bridge_result.json").write_text(json.dumps(r, indent=2))
print(json.dumps(r, indent=2))
raise SystemExit(0 if r["passed"] else 1)
