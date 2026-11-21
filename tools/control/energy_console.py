#!/usr/bin/env python3
"""显示能源状态并提供显式控制入口。

Monitor sensor readings and issue explicit generator / test-fault commands.
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
import os
import time
import json
import math

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--generator", choices=["on", "off"])
p.add_argument(
    "--cooling", choices=["ok", "fault"], help="simulation cooling-permission fault"
)
p.add_argument("--sensors", choices=["ok", "fault"], help="simulation sensor dropout")
p.add_argument("--duration", type=float, default=30)
p.add_argument(
    "--ros",
    action="store_true",
    help="publish ROS 2 BatteryState, fuel Float64 and diagnostics",
)
p.add_argument("--name", default="usv")
a = p.parse_args()
if not a.name.isidentifier():
    p.error("Invalid boat name")
prefix = "/" + a.name
if not math.isfinite(a.duration) or a.duration <= 0:
    p.error("duration must be finite and positive")
os.environ.setdefault("GZ_PARTITION", "usv_hull_trial")
os.environ.setdefault("GZ_IP", "127.0.0.1")


def _load_runtime_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from gz.transport13 import Node
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.boolean_pb2 import Boolean
    from energy_fields import TRUTH, SENSORS, GENERATOR, MASS

    return (
        Node,
        Double_V,
        Boolean,
        TRUTH,
        SENSORS,
        GENERATOR,
        MASS,
    )


(
    Node,
    Double_V,
    Boolean,
    TRUTH,
    SENSORS,
    GENERATOR,
    MASS,
) = _load_runtime_dependencies()

n = Node()
latest = {}


def cache_telemetry_1(m):
    """更新最近的遥测状态，字段顺序沿用对应消息定义。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return latest.update(truth=dict(zip(TRUTH, m.data)))


n.subscribe(
    Double_V,
    prefix + "/energy/truth",
    cache_telemetry_1,
)


def cache_telemetry_2(m):
    """更新最近的遥测状态，字段顺序沿用对应消息定义。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return latest.update(sensors=dict(zip(SENSORS, m.data)))


n.subscribe(
    Double_V,
    prefix + "/energy/sensors",
    cache_telemetry_2,
)


def cache_telemetry_3(m):
    """更新最近的遥测状态，字段顺序沿用对应消息定义。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return latest.update(mass=dict(zip(MASS, m.data)))


n.subscribe(
    Double_V,
    prefix + "/fuel_mass/state",
    cache_telemetry_3,
)
commands = []
for topic, value in [
    ("generator_enable", None if a.generator is None else a.generator == "on"),
    ("cooling_ok", None if a.cooling is None else a.cooling == "ok"),
    ("sensors_ok", None if a.sensors is None else a.sensors == "ok"),
]:
    if value is not None:
        commands.append((n.advertise(prefix + "/energy/" + topic, Boolean), value))
if a.ros:
    import rclpy
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import Float64
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    rclpy.init()
    ros = rclpy.create_node("usv_energy_bridge")
    bp = ros.create_publisher(BatteryState, prefix + "/battery", 10)
    fp = ros.create_publisher(Float64, prefix + "/fuel_l", 10)
    dp = ros.create_publisher(DiagnosticArray, prefix + "/energy/diagnostics", 10)


def stamp(header, sim):
    """按仿真时间填写 ROS 消息头时间戳。

    Args:
        header: 需要原地更新的消息头
        sim: 仿真时间，s
    """
    header.stamp.sec = int(sim)
    header.stamp.nanosec = int((sim - int(sim)) * 1e9)
    header.frame_id = "hull"


try:
    start = time.monotonic()
    last = -1
    last_rx_wall = time.monotonic()
    seen_time = None
    while time.monotonic() - start < a.duration:
        for pub, value in commands:
            pub.publish(Boolean(data=value))
        sensor = latest.get("sensors")
        truth = latest.get("truth")
        if sensor and sensor["time"] != seen_time:
            seen_time = sensor["time"]
            last_rx_wall = time.monotonic()
        if time.monotonic() - last >= 1:
            last = time.monotonic()
            if sensor and truth:
                fresh = time.monotonic() - last_rx_wall < 2
                valid = bool(sensor["valid"] and 0 <= sensor["age_s"] < 2.5 and fresh)
                view = dict(
                    sensor,
                    valid=valid,
                    generator=GENERATOR.get(int(truth["generator_state"])),
                    low_soc=bool(truth["low_soc"]),
                    low_fuel=bool(truth["low_fuel"]),
                    fault=int(truth["fault"]),
                )
                mass = latest.get("mass")
                if mass and abs(mass["time"] - sensor["time"]) < 0.5 and fresh:
                    view.update(
                        sim_ship_mass_kg=mass["ship_mass"],
                        sim_ship_cg_m=[
                            mass[k] for k in ["ship_cgx", "ship_cgy", "ship_cgz"]
                        ],
                    )
                print(json.dumps(view), flush=True)
                if a.ros:
                    b = BatteryState()
                    stamp(
                        b.header,
                        (
                            sensor["sample_time"]
                            if sensor["sample_time"] >= 0
                            else sensor["time"]
                        ),
                    )
                    b.present = True
                    b.voltage = sensor["voltage_v"] if valid else float("nan")
                    b.current = (
                        -sensor["current_a"] if valid else float("nan")
                    )  # ROS: negative discharging
                    b.percentage = sensor["soc"] if valid else float("nan")
                    b.temperature = sensor["temperature_c"] if valid else float("nan")
                    b.capacity = b.design_capacity = float("nan")
                    b.charge = float("nan")
                    b.power_supply_technology = (
                        BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
                    )
                    b.power_supply_status = (
                        (
                            BatteryState.POWER_SUPPLY_STATUS_CHARGING
                            if sensor["current_a"] < -0.1
                            else (
                                BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                                if sensor["current_a"] > 0.1
                                else BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
                            )
                        )
                        if valid
                        else BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
                    )
                    bp.publish(b)
                    fp.publish(
                        Float64(data=sensor["fuel_l"] if valid else float("nan"))
                    )
                    status = DiagnosticStatus()
                    status.name = "usv_energy"
                    status.hardware_id = "SIMULATION_ESTIMATE"
                    status.level = (
                        DiagnosticStatus.ERROR
                        if not valid or truth["discharge_open"] or truth["fault"]
                        else (
                            DiagnosticStatus.WARN
                            if truth["low_soc"] or truth["low_fuel"]
                            else DiagnosticStatus.OK
                        )
                    )
                    status.message = (
                        "sensor invalid/stale"
                        if not valid
                        else GENERATOR.get(int(truth["generator_state"]), "UNKNOWN")
                    )
                    status.values = [
                        KeyValue(key=k, value=str(v)) for k, v in view.items()
                    ]
                    d = DiagnosticArray()
                    stamp(d.header, sensor["time"])
                    d.status = [status]
                    dp.publish(d)
            else:
                print("Waiting for energy telemetry...", flush=True)
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    # Stop native transport callbacks before Python / ROS interpreter teardown.
    n.unsubscribe(prefix + "/energy/truth")
    n.unsubscribe(prefix + "/energy/sensors")
    n.unsubscribe(prefix + "/fuel_mass/state")
    commands.clear()
    if a.ros:
        ros.destroy_node()
        rclpy.shutdown()
    del n
    import gc

    gc.collect()
