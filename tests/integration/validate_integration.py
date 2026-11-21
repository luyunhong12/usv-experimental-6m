#!/usr/bin/env python3
"""运行指定的整船接口与环境集成工况。

Bounded integration checks for the six-item upgrade, with raw evidence.
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
import xml.etree.ElementTree as E
import numpy as np

p = argparse.ArgumentParser()
p.add_argument(
    "--case",
    required=True,
    choices=[
        "full_speed",
        "speed15",
        "heading",
        "sensors",
        "waves",
        "current",
        "wind",
        "power_loss",
        "multi",
        "collision",
        "task",
    ],
)
a = p.parse_args()
root = WORKSPACE
out = root / "log/integration_upgrade" / a.case
out.mkdir(parents=True, exist_ok=True)
os.environ["GZ_PARTITION"] = "usv_upgrade_" + a.case
os.environ["GZ_IP"] = "127.0.0.1"
os.environ["ROS_DOMAIN_ID"] = "73"


def _load_runtime_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from gz.transport13 import Node
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.vector3d_pb2 import Vector3d
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.navsat_pb2 import NavSat
    from gz.msgs10.magnetometer_pb2 import Magnetometer
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.camera_info_pb2 import CameraInfo
    from gz.msgs10.param_pb2 import Param
    from gz.msgs10.float_pb2 import Float

    return (
        Node,
        Double,
        Double_V,
        Vector3d,
        IMU,
        NavSat,
        Magnetometer,
        Image,
        LaserScan,
        Contacts,
        CameraInfo,
        Param,
        Float,
    )


(
    Node,
    Double,
    Double_V,
    Vector3d,
    IMU,
    NavSat,
    Magnetometer,
    Image,
    LaserScan,
    Contacts,
    CameraInfo,
    Param,
    Float,
) = _load_runtime_dependencies()

node = Node()
series = {}
counts = {}
contacts = []
task_info = []
task_errors = []


def record_message_1(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return task_info.append(str(m))


node.subscribe(Param, "/vrx/task/info", record_message_1)


def record_message_2(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return task_errors.append(m.data)


node.subscribe(Float, "/vrx/stationkeeping/pose_error", record_message_2)
for name in [
    "propulsion/state",
    "energy/truth",
    "actuators/state",
    "hydrostatics",
    "environment/state",
    "fuel_mass/state",
]:
    series[name] = []

    def record_message_3(m, key=name):
        """记录收到的消息，沿用当前订阅所绑定的状态容器。

        Args:
            m: 当前订阅接收到的消息，类型由订阅接口确定。
            key: 回调创建时绑定的主题或状态类别键。
        """
        return series[key].append(list(m.data))

    node.subscribe(Double_V, "/usv/" + name, record_message_3)
escort = []


def record_message_4(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return escort.append(list(m.data))


node.subscribe(Double_V, "/escort/propulsion/state", record_message_4)
for name, kind in [
    ("imu", IMU),
    ("gps", NavSat),
    ("magnetometer", Magnetometer),
    ("camera", Image),
    ("lidar", LaserScan),
    ("camera_info", CameraInfo),
]:
    counts[name] = 0

    def received(m, key=name):
        """累加指定传感器的接收次数。

        Args:
            m: 当前接收的消息
            key: 传感器统计键
        """
        counts[key] += 1

    node.subscribe(kind, "/usv/sensors/" + name, received)


def record_message_5(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return contacts.append(str(m)) if m.contact else None


node.subscribe(
    Contacts,
    "/usv/validation/contacts",
    record_message_5,
)
throttle = node.advertise("/usv/propulsion/throttle", Double)
steer = node.advertise("/usv/steering/angle", Double)
water = node.advertise("/vrx/current/velocity", Vector3d)
wind = node.advertise("/vrx/wind/velocity", Vector3d)
args = [
    "python3",
    str(tool_path("prepare_sydney.py")),
    "--output",
    str(out / "source_world.sdf"),
    "--wave-gain",
    str(0.3 if a.case == "waves" else 0),
]
if a.case == "speed15":
    args += ["--profile", "speed15_concept"]
if a.case == "task":
    args += ["--task", "stationkeeping"]
if a.case == "multi":
    args += ["--second-name", "escort"]
subprocess.run(args, check=True, stdout=subprocess.DEVNULL)
tree = E.parse(out / "source_world.sdf")
world = tree.getroot().find("world")
# Snapshot the exact boat and world used for this run.
for inc in world.findall("include"):
    if inc.findtext("name") == "usv":
        bt = E.parse(inc.findtext("uri"))
        bm = bt.getroot().find("model")
        if a.case == "power_loss":
            bm.find(
                "plugin[@name='usv::AzimuthPropulsion']/energy/initial_soc"
            ).text = ".02"
        if a.case == "collision":
            hull = bm.find("link[@name='hull']")
            se = E.SubElement(hull, "sensor", name="validation", type="contact")
            E.SubElement(se, "always_on").text = "true"
            E.SubElement(se, "topic").text = "/usv/validation/contacts"
            ct = E.SubElement(se, "contact")
            E.SubElement(ct, "collision").text = "hull_detailed_external"
            E.SubElement(ct, "topic").text = "/usv/validation/contacts"
        bf = out / "boat.sdf"
        bt.write(bf, encoding="utf-8", xml_declaration=True)
        inc.find("uri").text = str(bf)
if a.case == "collision":
    ball = E.SubElement(world, "model", name="probe")
    E.SubElement(ball, "pose").text = "-480 180 2.5 0 0 0"
    link_element = E.SubElement(ball, "link", name="ball")
    it = E.SubElement(link_element, "inertial")
    E.SubElement(it, "mass").text = "1"
    ii = E.SubElement(it, "inertia")
    for k in ["ixx", "iyy", "izz"]:
        E.SubElement(ii, k).text = ".004"
    for kind in ["collision", "visual"]:
        g = E.SubElement(E.SubElement(link_element, kind, name=kind), "geometry")
        E.SubElement(E.SubElement(g, "sphere"), "radius").text = ".1"
f = out / "world.sdf"
tree.write(f, encoding="utf-8", xml_declaration=True)
log = (out / "launch.log").open("w")
procs = []
procs.append(
    subprocess.Popen(
        ["bash", str(tool_path("run_hull_trial.sh")), "world:=" + str(f)],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
)
ros = None
roscounts = {}
cmd = None
rosstates = []
roslogs = None
tf_edges = set()
odom_samples = []
if a.case in ["heading", "speed15", "sensors", "power_loss"]:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import (
        Imu,
        NavSatFix,
        Image as RosImage,
        LaserScan as RosScan,
        CameraInfo as RosInfo,
        MagneticField,
    )
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float64MultiArray
    from tf2_msgs.msg import TFMessage
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    ros = rclpy.create_node("upgrade_validator")
    for topic, kind in [
        ("sensors/imu/imu/data", Imu),
        ("sensors/gps/gps/fix", NavSatFix),
        ("sensors/cameras/camera/image_raw", RosImage),
        ("sensors/cameras/camera/camera_info", RosInfo),
        ("sensors/lidars/lidar/scan", RosScan),
        ("sensors/magnetometer/data", MagneticField),
    ]:
        roscounts[topic] = 0

        def cb(m, key=topic):
            """累加指定 ROS 主题的接收次数。

            Args:
                m: 当前消息
                key: 主题统计键
            """
            roscounts[key] += 1

        ros.create_subscription(kind, "/usv/" + topic, cb, qos_profile_sensor_data)

    def tf(m):
        """记录消息中的父子坐标系连接。

        Args:
            m: ROS 坐标变换数组消息
        """
        for tr in m.transforms:
            tf_edges.add((tr.header.frame_id, tr.child_frame_id))

    ros.create_subscription(TFMessage, "/tf", tf, 10)
    ros.create_subscription(
        TFMessage,
        "/tf_static",
        tf,
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )

    def record_message_6(m):
        """记录收到的消息，沿用当前订阅所绑定的状态容器。

        Args:
            m: 当前订阅接收到的消息，类型由订阅接口确定。
        """
        return odom_samples.append(
            [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
        )

    ros.create_subscription(
        Odometry,
        "/usv/ground_truth/odometry",
        record_message_6,
        10,
    )

    def record_message_7(m):
        """记录收到的消息，沿用当前订阅所绑定的状态容器。

        Args:
            m: 当前订阅接收到的消息，类型由订阅接口确定。
        """
        return rosstates.append(list(m.data))

    ros.create_subscription(
        Float64MultiArray,
        "/usv/control/state",
        record_message_7,
        10,
    )
    roscounts["clock"] = 0

    def clock(m):
        """累加仿真时钟消息数量。

        Args:
            m: ROS 仿真时钟消息
        """
        roscounts["clock"] += 1

    ros.create_subscription(Clock, "/clock", clock, 10)
    cmd = ros.create_publisher(Twist, "/usv/cmd_vel", 10)
    roslogs = (out / "interfaces.log").open("w")
    procs.append(
        subprocess.Popen(
            [
                "bash",
                str(tool_path("run_interfaces.sh")),
                "speed_scale:=" + ("4.3" if a.case == "speed15" else "1.63"),
            ],
            stdout=roslogs,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    )
duration = (
    70 if a.case in ["full_speed", "speed15"] else 55 if a.case == "heading" else 40
)
start = time.monotonic()
try:
    while time.monotonic() - start < 300 and procs[0].poll() is None:
        data = series["propulsion/state"]
        t = data[-1][0] if data else 0
        if t >= duration:
            break
        if data:
            value = (
                1.0
                if a.case in ["full_speed", "speed15"] and 2 < t < 60
                else (
                    0.6
                    if a.case == "multi" and t > 2
                    else 0.8 if a.case == "power_loss" else 0.0
                )
            )
            angle = (
                math.radians(35 if t < 15 else -35 if t < 30 else 0)
                if a.case in ["collision", "power_loss"]
                else 0.0
            )
            if a.case not in ["heading", "speed15"]:
                throttle.publish(Double(data=value))
                steer.publish(Double(data=angle))
            if a.case == "current" and t > 2:
                water.publish(Vector3d(x=0.3, y=0.2, z=0))
            if a.case == "wind" and t > 2:
                wind.publish(Vector3d(x=0, y=8, z=0))
            if cmd is not None and t > 3:
                msg = Twist()
                msg.linear.x = (
                    1.0
                    if a.case == "heading"
                    else 15 / 3.6 if a.case == "speed15" else 0.0
                )
                cmd.publish(msg)
        if ros:
            rclpy.spin_once(ros, timeout_sec=0.01)
        time.sleep(0.02)
finally:
    for proc in reversed(procs):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
    log.close()
    if roslogs:
        roslogs.close()
    if ros:
        ros.destroy_node()
        rclpy.shutdown()
for name, rows in series.items():
    np.savetxt(out / (name.replace("/", "_") + ".csv"), np.array(rows), delimiter=",")
np.savetxt(out / "escort.csv", np.array(escort), delimiter=",")
np.savetxt(out / "controller.csv", np.array(rosstates), delimiter=",")
(out / "contacts.txt").write_text("\n".join(contacts))
d = np.array(series["propulsion/state"])
en = np.array(series["energy/truth"])
ac = np.array(series["actuators/state"])
hyd = np.array(series["hydrostatics"])
ev = np.array(series["environment/state"])
checks = {
    "completed": bool(len(d) and d[-1, 0] >= duration - 0.2),
    "finite": all(np.isfinite(np.array(v)).all() for v in series.values()),
    "sensor_streams": all(v > 0 for v in counts.values()),
}
r = {
    "case": a.case,
    "checks": checks,
    "native_sensor_counts": counts,
    "ros_counts": roscounts,
}
if len(d):
    r["final_sim_s"] = float(d[-1, 0])
    r["max_roll_pitch_deg"] = np.rad2deg(np.max(abs(d[:, 15:17]), axis=0)).tolist()
    checks["attitude_bounded"] = bool(np.max(abs(d[:, 15:17])) < 0.35)
    if a.case in ["full_speed", "speed15"]:
        steady = d[(d[:, 0] > 50) & (d[:, 0] < 59)]
        r["speed_m_s"] = float(np.mean(steady[:, 10]))
        r["speed_km_h"] = 3.6 * r["speed_m_s"]
        r["rpm"] = float(np.mean(steady[:, 2]))
        r["max_motor_electrical_w"] = float(en[:, 7].max())
        checks.update(
            speed=(
                bool(1.4 < r["speed_m_s"] < 1.9)
                if a.case == "full_speed"
                else bool(14.5 <= r["speed_km_h"] <= 15.5)
            ),
            power=bool(
                en[:, 7].max() <= (6000.01 if a.case == "full_speed" else 40000.01)
            ),
        )
    if a.case == "heading":
        tail = d[d[:, 0] > 40]
        r["final_heading_deg"] = math.degrees(d[-1, 14])
        r["tail_max_heading_deg"] = float(np.max(abs(np.rad2deg(tail[:, 14]))))
        r["tail_speed_m_s"] = float(np.mean(tail[:, 10]))
        checks.update(
            heading=r["tail_max_heading_deg"] < 2,
            speed=abs(r["tail_speed_m_s"] - 1) < 0.12,
            controller=bool(rosstates),
        )
    if a.case == "waves":
        checks.update(
            wave_signal=bool(np.ptp(ev[:, 1]) > 0.03),
            heave_response=bool(np.ptp(hyd[:, 8]) > 0.015),
        )
        r["wave_height_range_m"] = float(np.ptp(ev[:, 1]))
        r["heave_range_m"] = float(np.ptp(hyd[:, 8]))
    if a.case == "current":
        checks["current_drift"] = bool(
            d[-1, 12] - d[0, 12] > 3 and d[-1, 13] - d[0, 13] > 2
        )
    if a.case == "wind":
        checks["wind_drift"] = bool(d[-1, 13] - d[0, 13] > 0.5 and ev[-1, 14] > 0)
    if a.case == "multi":
        checks["isolated"] = bool(
            escort
            and np.linalg.norm(np.array(escort)[-1, 12:14] - np.array(escort)[0, 12:14])
            < 0.05
            and np.linalg.norm(d[-1, 12:14] - d[0, 12:14]) > 5
        )
    if a.case == "power_loss":
        checks.update(
            motor_off=bool(np.max(abs(d[:, 5])) < 1e-9),
            steering_off=bool(len(ac) and np.max(abs(ac[:, 4:6])) < 1e-9),
            sensor_gate=all(v == 0 for k, v in roscounts.items() if k != "clock"),
        )
    if a.case == "task":
        checks["vrx_task"] = bool(
            task_info and task_errors and np.isfinite(task_errors).all()
        )
        r["task_messages"] = len(task_info)
        r["task_error_samples"] = len(task_errors)
        (out / "task_info.txt").write_text("\n".join(task_info))
        np.savetxt(out / "task_error.csv", task_errors)
    if a.case == "collision":
        checks.update(
            external_contact=bool(contacts),
            self_collision_enabled=E.parse(out / "boat.sdf").findtext(
                "model/self_collide"
            )
            == "true",
            steering_limits=bool(
                len(ac) and ac[:, 1].min() < -0.55 and ac[:, 1].max() > 0.55
            ),
        )
    if a.case in ["sensors", "heading"]:
        checks["ros_payloads"] = all(v > 0 for v in roscounts.values())
        checks["tf_tree"] = (
            ("world", "usv/base_link") in tf_edges
            and ("usv/base_link", "usv/imu") in tf_edges
            and ("usv/camera", "usv/camera_optical") in tf_edges
        )
        checks["ground_truth_separate"] = bool(
            odom_samples and np.isfinite(odom_samples).all()
        )
        r["tf_edges"] = sorted(tf_edges)
r["passed"] = all(checks.values())
(out / "result.json").write_text(json.dumps(r, indent=2))
print(json.dumps(r, indent=2))
raise SystemExit(0 if r["passed"] else 1)
