#!/usr/bin/env python3
"""执行载荷与能源组合矩阵中的单个工况。

Bounded engineering-envelope tests; strict nominal target, honest saturation under uncertainty.
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
import argparse
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np

p = argparse.ArgumentParser()
p.add_argument("--case", required=True)
p.add_argument("--output", type=Path, required=True)
p.add_argument("--spec", type=Path, required=True)
a = p.parse_args()
spec = json.loads(a.spec.read_text())[a.case]
w = WORKSPACE
out = a.output if a.output else w / "log/platform_validation" / a.case
out.mkdir(parents=True, exist_ok=True)
os.environ.update(
    GZ_PARTITION="usv_ledger_" + a.case,
    GZ_IP="127.0.0.1",
    ROS_DOMAIN_ID=str(40 + list(json.loads(a.spec.read_text())).index(a.case)),
)


def _load_runtime_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from gz.transport13 import Node
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.vector3d_pb2 import Vector3d
    from gz.msgs10.boolean_pb2 import Boolean
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float64MultiArray

    return (
        Node,
        Double_V,
        Vector3d,
        Boolean,
        rclpy,
        Twist,
        Float64MultiArray,
    )


(
    Node,
    Double_V,
    Vector3d,
    Boolean,
    rclpy,
    Twist,
    Float64MultiArray,
) = _load_runtime_dependencies()

node = Node()
rows = {
    k: []
    for k in [
        "propulsion/state",
        "energy/truth",
        "actuators/state",
        "environment/state",
        "fuel_mass/state",
    ]
}
for topic in rows:

    def record_message_1(m, k=topic):
        """记录收到的消息，沿用当前订阅所绑定的状态容器。

        Args:
            m: 当前订阅接收到的消息，类型由订阅接口确定。
            k: 回调创建时绑定的主题或状态类别键。
        """
        return rows[k].append(list(m.data))

    node.subscribe(Double_V, "/usv/" + topic, record_message_1)
flow = node.advertise("/vrx/current/velocity", Vector3d)
wind = node.advertise("/vrx/wind/velocity", Vector3d)
gen = node.advertise("/usv/energy/generator_enable", Boolean)
world = out / "world.sdf"
subprocess.run(
    [
        "python3",
        str(tool_path("prepare_sydney.py")),
        "--profile",
        "speed15_concept",
        "--payload-kg",
        str(spec["payload_kg"]),
        "--fuel-l",
        str(spec["fuel_l"]),
        "--payload-center",
        *map(str, spec.get("payload_center", [0, 0, 0.6])),
        "--mass-case",
        spec.get("mass_case", "nominal"),
        "--power-mode",
        spec.get("power_mode", "normal"),
        "--wave-gain",
        str(spec["wave_gain"]),
        "--output",
        str(world),
    ],
    check=True,
    stdout=subprocess.DEVNULL,
)
tree = E.parse(world)
inc = tree.find("world/include[name='usv']")
bt = E.parse(inc.findtext("uri"))
bm = bt.find("model")
# Physics/energy matrix: retain IMU/GNSS; omit GPU image generation only.
# Device mass and electrical demand remain present in every case.
for link in bm.findall("link"):
    for sensor in list(link.findall("sensor")):
        if sensor.get("type") in ["camera", "gpu_lidar"]:
            link.remove(sensor)
bf = out / "boat.sdf"
bt.write(bf, encoding="utf-8", xml_declaration=True)
inc.find("uri").text = str(bf)
tree.write(world, encoding="utf-8", xml_declaration=True)
rclpy.init()
ros = rclpy.create_node("platform_validator")
pub = ros.create_publisher(Twist, "/usv/cmd_vel", 10)
control = []


def record_message_2(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return control.append(list(m.data))


ros.create_subscription(Float64MultiArray, "/usv/control/state", record_message_2, 10)
procs = []
files = []
for name, args in [
    ("gazebo", ["bash", str(tool_path("run_hull_trial.sh")), "world:=" + str(world)]),
    ("interfaces", ["bash", str(tool_path("run_interfaces.sh")), "speed_scale:=4.3"]),
]:
    f = (out / (name + ".log")).open("w")
    files.append(f)
    procs.append(
        subprocess.Popen(
            args, stdout=f, stderr=subprocess.STDOUT, start_new_session=True
        )
    )
duration = 65
begin = time.monotonic()
failure = None
try:
    while time.monotonic() - begin < duration * 3 + 60:
        if any(x.poll() is not None for x in procs):
            failure = "child exited"
            break
        d = rows["propulsion/state"]
        t = d[-1][0] if d else 0
        (out / "progress.json").write_text(
            json.dumps({"sim_s": t, "wall_s": time.monotonic() - begin})
        )
        if t >= duration:
            break
        if t > 8:
            msg = Twist()
            msg.linear.x = 15 / 3.6 if t < 50 else 0.0
            msg.angular.z = 0.0
            pub.publish(msg)
        if t > 5:
            flow.publish(Vector3d(**dict(zip(["x", "y", "z"], spec["current"]))))
            wind.publish(Vector3d(**dict(zip(["x", "y", "z"], spec["wind"]))))
            if spec.get("generator_on"):
                gen.publish(Boolean(data=True))
        rclpy.spin_once(ros, timeout_sec=0.01)
        time.sleep(0.02)
finally:
    for x in reversed(procs):
        if x.poll() is None:
            os.killpg(x.pid, signal.SIGINT)
            try:
                x.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(x.pid, signal.SIGKILL)
                x.wait()
    for f in files:
        f.close()
    ros.destroy_node()
    rclpy.shutdown()
for k, v in rows.items():
    np.savetxt(out / (k.replace("/", "_") + ".csv"), v, delimiter=",")
np.savetxt(out / "controller.csv", control, delimiter=",")
d = np.array(rows["propulsion/state"])
e = np.array(rows["energy/truth"])
c = np.array(control)
m = np.array(rows["fuel_mass/state"])
checks = {
    "completed": bool(len(d) and d[-1, 0] >= duration - 0.2),
    "finite": all(len(v) > 0 and np.isfinite(v).all() for v in rows.values()),
    "processes_alive": failure is None,
    "controller_telemetry_received": len(control) > 100,
}
r = {
    "case": a.case,
    "spec": spec,
    "checks": checks,
    "rendering": "GPU sensor image generation omitted; their physical mass and power retained; GNSS/IMU active",
}
if all(len(x) for x in [d, e, m]):
    tail = d[(d[:, 0] > 40) & (d[:, 0] < 49)]
    speed = float(np.mean(tail[:, 10])) * 3.6
    dry = float(
        bm.find("plugin[@name='usv::AzimuthPropulsion']/fuel_mass/dry_mass").text
    )
    expected = float(
        json.loads((out / "scene.json").read_text())["profile_details"]["usv"][
            "ship_mass_kg"
        ]
    )
    checks.update(
        attitude_bounded=bool(np.max(abs(d[:, 15:17])) < math.radians(20)),
        power_limited=bool(e[:, 7].max() <= 40000.01),
        energy_balance=bool(np.max(abs(e[:, 21])) < 1e-6),
        soc_decreases=bool(e[-1, 1] < e[0, 1]),
        mass_tracks_fuel=bool(np.max(abs(m[:, 2] - dry - 0.84 * m[:, 1])) < 1e-7),
        initial_total_mass=bool(abs(m[0, 12] - expected) < 1e-5),
        fuel_agrees_energy=bool(abs(m[-1, 1] - e[-1, 5]) < 1e-4),
        bounded_forward_operation=bool(5 < speed < 18),
    )
    if spec["wave_gain"] == 0 and not any(spec["wind"]) and not any(spec["current"]):
        checks["target_speed"] = abs(speed - 15) < 0.5
    if spec.get("generator_on"):
        checks["fuel_consumed"] = bool(m[0, 1] - m[-1, 1] > 0.001)
    r.update(
        speed_40_49_s_km_h=speed,
        max_roll_pitch_deg=np.rad2deg(np.max(abs(d[:, 15:17]), axis=0)).tolist(),
        peak_power_w=float(e[:, 7].max()),
        final_soc=float(e[-1, 1]),
        max_temperature_c=float(e[:, 4].max()),
        final_sim_s=float(d[-1, 0]),
        initial_ship_mass_kg=float(m[0, 12]),
        initial_ship_cg_m=m[0, 13:16].tolist(),
        fuel_consumed_l=float(m[0, 1] - m[-1, 1]),
        coast_speed_65_s_km_h=float(d[-1, 10]) * 3.6,
        environment_last=rows["environment/state"][-1],
    )
r["passed"] = all(checks.values())
(out / "result.json").write_text(json.dumps(r, indent=2))
print(json.dumps(r, indent=2))
raise SystemExit(0 if r["passed"] else 1)
