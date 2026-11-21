#!/usr/bin/env python3
"""验证载荷设备与传感器任务接口。

ROS end-to-end payload and upstream VRX scoring interface acceptance.
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
import subprocess
import signal
import argparse
import math
import struct
import shutil
import xml.etree.ElementTree as E
import numpy as np
from PIL import Image as PILImage

p = argparse.ArgumentParser()
p.add_argument(
    "--case",
    choices=["sensors", "stationkeeping", "wayfinding", "navigation"],
    required=True,
)
a = p.parse_args()
w = WORKSPACE
out = w / "../artifacts/runs/WAMV差距完善_2026-11-20" / a.case
out.mkdir(parents=True, exist_ok=True)
os.environ.update(
    GZ_PARTITION="usv_payload_" + a.case,
    GZ_IP="127.0.0.1",
    ROS_DOMAIN_ID="81",
    VRX_EXIT_ON_COMPLETION="false",
)


def _load_runtime_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2, LaserScan
    from geometry_msgs.msg import PoseArray, Twist
    from ros_gz_interfaces.msg import ParamVec
    from std_msgs.msg import Float32
    from gz.transport13 import Node
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.pose_v_pb2 import Pose_V

    return (
        rclpy,
        qos_profile_sensor_data,
        Image,
        CameraInfo,
        PointCloud2,
        LaserScan,
        PoseArray,
        Twist,
        ParamVec,
        Float32,
        Node,
        Double_V,
        Pose_V,
    )


(
    rclpy,
    qos_profile_sensor_data,
    Image,
    CameraInfo,
    PointCloud2,
    LaserScan,
    PoseArray,
    Twist,
    ParamVec,
    Float32,
    Node,
    Double_V,
    Pose_V,
) = _load_runtime_dependencies()

node = Node()
truth = []


def record_message_1(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return truth.append(list(m.data))


node.subscribe(Double_V, "/usv/propulsion/state", record_message_1)
poses = []


def pose_cb(m):
    """记录船舶、航道及标记物位姿。

    Args:
        m: 多物体位姿消息
    """
    for p in m.pose:
        if p.name in ["usv", "short_navigation_course_0"] or p.name.startswith(
            ("red_bound_", "green_bound_")
        ):
            poses.append(
                [
                    m.header.stamp.sec + m.header.stamp.nsec * 1e-9,
                    p.name,
                    p.position.x,
                    p.position.y,
                    p.position.z,
                    p.orientation.x,
                    p.orientation.y,
                    p.orientation.z,
                    p.orientation.w,
                ]
            )


node.subscribe(Pose_V, "/world/usv_sydney/pose/info", pose_cb)
rclpy.init()
ros = rclpy.create_node("payload_task_validator")
drive = ros.create_publisher(Twist, "/usv/cmd_vel", 10)
received = {}
counts = {}


def cb(m, k):
    """缓存指定载荷的最近消息并统计接收次数。

    Args:
        m: 载荷消息
        k: 载荷统计键
    """
    received[k] = m
    counts[k] = counts.get(k, 0) + 1


for n in ["camera", "camera_right", "camera_side"]:

    def receive_payload_2(m, k=n):
        """按绑定的载荷类别缓存消息并更新统计。

        Args:
            m: 当前订阅接收到的消息，类型由订阅接口确定。
            k: 回调创建时绑定的主题或状态类别键。
        """
        return cb(m, k)

    ros.create_subscription(
        Image,
        "/usv/sensors/cameras/" + n + "/image_raw",
        receive_payload_2,
        qos_profile_sensor_data,
    )

    def receive_payload_3(m, k=n + "_info"):
        """按绑定的载荷类别缓存消息并更新统计。

        Args:
            m: 当前订阅接收到的消息，类型由订阅接口确定。
            k: 回调创建时绑定的主题或状态类别键。
        """
        return cb(m, k)

    ros.create_subscription(
        CameraInfo,
        "/usv/sensors/cameras/" + n + "/camera_info",
        receive_payload_3,
        qos_profile_sensor_data,
    )


def receive_payload_4(m):
    """按绑定的载荷类别缓存消息并更新统计。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return cb(m, "cloud")


ros.create_subscription(
    PointCloud2,
    "/usv/sensors/lidars/lidar/points",
    receive_payload_4,
    qos_profile_sensor_data,
)


def receive_payload_5(m):
    """按绑定的载荷类别缓存消息并更新统计。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return cb(m, "scan")


ros.create_subscription(
    LaserScan,
    "/usv/sensors/lidars/lidar/scan",
    receive_payload_5,
    qos_profile_sensor_data,
)


def receive_payload_6(m):
    """按绑定的载荷类别缓存消息并更新统计。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return cb(m, "task")


ros.create_subscription(ParamVec, "/vrx/task/info", receive_payload_6, 10)


def receive_payload_7(m):
    """按绑定的载荷类别缓存消息并更新统计。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return cb(m, "waypoints")


ros.create_subscription(PoseArray, "/vrx/wayfinding/waypoints", receive_payload_7, 10)


def receive_payload_8(m):
    """按绑定的载荷类别缓存消息并更新统计。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return cb(m, "way_error")


ros.create_subscription(Float32, "/vrx/wayfinding/mean_error", receive_payload_8, 10)


def receive_payload_9(m):
    """按绑定的载荷类别缓存消息并更新统计。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return cb(m, "station_error")


ros.create_subscription(
    Float32, "/vrx/stationkeeping/pose_error", receive_payload_9, 10
)
world = out / "world.sdf"
subprocess.run(
    [
        "python3",
        str(tool_path("prepare_sydney.py")),
        "--profile",
        "speed15_concept",
        "--wave-gain",
        "0",
        "--task",
        "none" if a.case == "sensors" else a.case,
        "--output",
        str(world),
    ],
    check=True,
    stdout=subprocess.DEVNULL,
)
t = E.parse(world)
inc = t.find("world/include[name='usv']")
shutil.copy2(inc.findtext("uri"), out / "boat.sdf")
inc.find("uri").text = str(out / "boat.sdf")
if a.case == "sensors":
    for name, pose, size, color in [
        ("front_target", "-468 180 1.5 0 0 0", ".25 4 3", "1 0 0 1"),
        ("side_target", "-480 170 1.5 0 0 0", "4 .25 3", "0 1 0 1"),
    ]:
        m = E.SubElement(t.find("world"), "model", name=name)
        E.SubElement(m, "static").text = "true"
        E.SubElement(m, "pose").text = pose
        link_element = E.SubElement(m, "link", name="target")
        v = E.SubElement(link_element, "visual", name="panel")
        b = E.SubElement(E.SubElement(v, "geometry"), "box")
        E.SubElement(b, "size").text = size
        mat = E.SubElement(v, "material")
        E.SubElement(mat, "ambient").text = color
        E.SubElement(mat, "diffuse").text = color
else:
    task = t.find(
        "world/plugin[@name='vrx::"
        + {
            "stationkeeping": "Stationkeeping",
            "wayfinding": "Wayfinding",
            "navigation": "Navigation",
        }[a.case]
        + "ScoringPlugin']"
    )
    E.SubElement(task, "per_plugin_exit_on_completion").text = "false"
    task.find("initial_state_duration").text = "1"
    task.find("ready_state_duration").text = "1"
    task.find("running_state_duration").text = "25"
if a.case == "navigation":
    angle = -1.44
    local = np.array([8.0, -10.0])
    rotation = np.array(
        [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
    )
    xy = np.array([-524.0, 198.0]) + rotation @ local
    pose = list(map(float, inc.findtext("pose").split()))
    pose[:2] = xy
    pose[5] = angle + math.pi
    inc.find("pose").text = " ".join(map(str, pose))
t.write(world, encoding="utf-8", xml_declaration=True)
procs = []
logs = []
failure = None
for n, args in [
    ("gazebo", ["bash", str(tool_path("run_hull_trial.sh")), "world:=" + str(world)]),
    ("ros", ["bash", str(tool_path("run_interfaces.sh")), "speed_scale:=4.3"]),
]:
    f = (out / (n + ".log")).open("w")
    logs.append(f)
    procs.append(
        subprocess.Popen(
            args, stdout=f, stderr=subprocess.STDOUT, start_new_session=True
        )
    )
begin = time.monotonic()
states = []
try:
    while time.monotonic() - begin < 180:
        rclpy.spin_once(ros, timeout_sec=0.01)
        if "task" in received:
            for x in received["task"].params:
                if x.name == "state" and (
                    not states or states[-1] != x.value.string_value
                ):
                    states.append(x.value.string_value)
        if a.case == "navigation" and truth and 3 < truth[-1][0] < 26:
            cmd = Twist()
            cmd.linear.x = 1.5
            drive.publish(cmd)
        if truth and truth[-1][0] > 32:
            break
        if any(x.poll() is not None for x in procs):
            if a.case != "sensors" and "finished" in states:
                break
            raise RuntimeError("Child process exited")
except Exception as e:
    failure = str(e)
finally:
    for proc in reversed(procs):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
    for f in logs:
        f.close()
    ros.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
np.savetxt(out / "propulsion.csv", truth, delimiter=",")
(out / "poses.json").write_text(json.dumps(poses))
checks = {"no_failure": failure is None}
metrics = {}
if a.case == "sensors":
    checks["all_payloads_received"] = all(
        counts.get(k, 0) > 5
        for k in [
            "camera",
            "camera_right",
            "camera_side",
            "camera_info",
            "camera_right_info",
            "camera_side_info",
            "cloud",
            "scan",
        ]
    )
    masks = {}
    for n in ["camera", "camera_right", "camera_side"]:
        if n not in received:
            continue
        m = received[n]
        im = (
            np.frombuffer(bytes(m.data), dtype=np.uint8)
            .reshape(m.height, m.step)[:, : m.width * 3]
            .reshape(m.height, m.width, 3)
        )
        PILImage.fromarray(im).save(out / (n + ".png"))
        color = 1 if n == "camera_side" else 0
        mask = (
            (im[:, :, color] > 80)
            & (im[:, :, color] > 1.5 * im[:, :, (color + 1) % 3])
            & (im[:, :, color] > 1.5 * im[:, :, (color + 2) % 3])
        )
        checks[n + "_target_visible"] = int(mask.sum()) > 100
        checks[n + "_optical_frame"] = m.header.frame_id == "usv/" + n + "_optical"
        masks[n] = float(np.nonzero(mask)[1].mean()) if mask.any() else 0
        metrics[n + "_target_pixels"] = int(mask.sum())
    if "camera_right_info" in received:
        m = received["camera_right_info"]
        checks["stereo_projection_baseline"] = abs(-m.p[3] / m.k[0] - 0.2) < 1e-6
    if all(k in masks for k in ["camera", "camera_right"]):
        metrics["stereo_pixel_disparity"] = masks["camera"] - masks["camera_right"]
        checks["nonzero_stereo_parallax"] = abs(metrics["stereo_pixel_disparity"]) > 0.5
    if "cloud" in received:
        m = received["cloud"]
        offsets = {f.name: f.offset for f in m.fields}
        raw = bytes(m.data)
        xyz = np.array(
            [
                [
                    struct.unpack_from(
                        "<f", raw, r * m.row_step + c * m.point_step + offsets[k]
                    )[0]
                    for k in ["x", "y", "z"]
                ]
                for r in range(m.height)
                for c in range(m.width)
            ]
        )
        finite = xyz[np.isfinite(xyz).all(1)]
        metrics["cloud_shape"] = [m.height, m.width]
        metrics["finite_points"] = len(finite)
        checks["sixteen_beams"] = m.height == 16 or m.width == 5760
        checks["three_dimensional_returns"] = (
            len(finite) > 50 and np.ptp(finite[:, 2]) > 0.5
        )
        checks["cloud_frame"] = m.header.frame_id == "usv/lidar"
        np.save(out / "cloud_xyz.npy", xyz)
    if "scan" in received:
        m = received["scan"]
        forward = m.ranges[len(m.ranges) // 2]
        metrics["forward_range_m"] = float(forward)
        checks["front_range_clear"] = 10 < forward < 13
else:
    checks["task_running"] = "running" in states
    checks["task_finished"] = "finished" in states
    errors = [
        line
        for line in (out / "gazebo.log").read_text().splitlines()
        if "[Err]" in line
        and "will not shutdown" not in line
        and ("ScoringPlugin" in line or "Unable to find entity" in line)
    ]
    checks["no_scoring_configuration_errors"] = not errors
    if a.case == "navigation":
        scores = [
            x.value.double_value
            for x in received.get("task", ParamVec()).params
            if x.name == "score"
        ]
        metrics["final_gate_score"] = scores[-1] if scores else None
        checks["actual_gate_crossing_scored"] = bool(scores and scores[-1] == 10)
        data = np.array(truth)
        tail = data[(data[:, 0] > 15) & (data[:, 0] < 25)]
        metrics["commanded_speed_m_s"] = 1.5
        metrics["measured_tail_speed_m_s"] = (
            float(tail[:, 10].mean()) if len(tail) else None
        )
        checks["nonzero_heading_speed_tracking"] = bool(
            len(tail) and abs(tail[:, 10].mean() - 1.5) < 0.2
        )
    if "task" in received:
        (out / "task_final.txt").write_text(str(received["task"]))
    if a.case == "wayfinding":
        checks["waypoint_list"] = (
            len(received["waypoints"].poses) == 3 if "waypoints" in received else False
        )
        checks["finite_score"] = "way_error" in received and math.isfinite(
            received["way_error"].data
        )
    if a.case == "stationkeeping":
        checks["finite_score"] = "station_error" in received and math.isfinite(
            received["station_error"].data
        )
checks = {k: bool(v) for k, v in checks.items()}
r = {
    "passed": all(checks.values()),
    "checks": checks,
    "metrics": metrics,
    "message_counts": counts,
    "states": states,
    "failure": failure,
    "scope": "Payload target visibility and ROS delivery, or official task lifecycle/score interfaces; not autonomous competition completion",
}
(out / "result.json").write_text(json.dumps(r, indent=2))
print(json.dumps(r, indent=2))
raise SystemExit(0 if r["passed"] else 1)
