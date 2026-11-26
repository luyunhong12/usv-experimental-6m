#!/usr/bin/env python3
"""验证本机飞控的航点、保持、返航及链路失效行为。

Actual local ArduRover AUTO waypoints, LOITER, RTL and GCS-loss test.

Independent instance/partition; normal arming; never connects to real hardware.
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
from navigation_acceptance import (
    required_checks,
    passed_checks,
    sustained_stop,
    sustained_motion,
    observed_loiter_target,
    RESPONSE_PLAN,
    response_command,
    response_coverage,
    weather_checks,
)
import os
import sys
import time
import json
import signal
import subprocess
import math
import argparse
import socket
import struct
import numpy as np
import xml.etree.ElementTree as E
import shutil
import hashlib
from itertools import cycle

root = WORKSPACE
sys.path.insert(0, str(root / "../dependencies/vendor/python"))


def _load_runtime_dependencies_1():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from pymavlink import mavutil

    return (mavutil,)


(mavutil,) = _load_runtime_dependencies_1()

p = argparse.ArgumentParser()
p.add_argument("--label", default="navigation")
p.add_argument("--failsafe-only", action="store_true")
p.add_argument("--response-only", action="store_true")
p.add_argument("--mirror-course", action="store_true")
p.add_argument("--external-sitl", action="store_true")
p.add_argument("--isolated-water", action="store_true")
p.add_argument("--no-rendering", action="store_true")
p.add_argument("--current-east", type=float, default=0.15)
p.add_argument("--current-north", type=float, default=0.1)
p.add_argument("--current-navigation", action="store_true")
p.add_argument("--wind-east", type=float, default=0.0)
p.add_argument("--wind-north", type=float, default=0.0)
p.add_argument("--wave-gain", type=float, default=0.0)
p.add_argument("--timeout", type=float, default=900)
p.add_argument("--output", type=_ProjectPath)

a = p.parse_args()
weather_enabled = bool(a.wind_east or a.wind_north or a.wave_gain)
weather_limits_path = root.parent / "tests/scenarios/environment_acceptance.json"
weather_limits = json.loads(weather_limits_path.read_text())
if weather_enabled and (a.response_only or a.failsafe_only):
    p.error("Weather acceptance requires the complete navigation sequence")
if not all(math.isfinite(v) and abs(v) <= 10 for v in (a.wind_east, a.wind_north)):
    p.error("Wind components must be finite and within +/-10 m/s")
if not math.isfinite(a.wave_gain) or not 0 <= a.wave_gain <= 1:
    p.error("Wave gain must be finite and within [0, 1]")
if a.response_only and (a.failsafe_only or a.mirror_course):
    p.error(
        "Response identification cannot be combined with failsafe-only or mirror-course"
    )
if not math.isfinite(a.timeout) or a.timeout <= 0:
    p.error("Timeout must be finite and positive")
if not all(math.isfinite(v) and abs(v) <= 1 for v in (a.current_east, a.current_north)):
    p.error("Current components must be finite and within +/-1 m/s")
if not a.label.isidentifier():
    p.error("Invalid label")
os.environ["GZ_PARTITION"] = "usv_platform_" + a.label
os.environ["GZ_IP"] = "127.0.0.1"


def _load_runtime_dependencies_2():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from gz.transport13 import Node
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.vector3d_pb2 import Vector3d

    return (
        Node,
        Double_V,
        Vector3d,
    )


(
    Node,
    Double_V,
    Vector3d,
) = _load_runtime_dependencies_2()

out = (a.output or root.parent / "artifacts/navigation" / a.label).resolve()
out.mkdir(parents=True, exist_ok=False)
rc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
node = Node()
rows = []
energy = []
actuators = []
environment = []
last_actuator_wall = 0.0
last_physics_wall = 0.0


def record_message_1(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    global last_physics_wall
    last_physics_wall = time.monotonic()
    return rows.append(list(m.data))


node.subscribe(Double_V, "/usv/propulsion/state", record_message_1)


def record_message_2(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return energy.append(list(m.data))


node.subscribe(Double_V, "/usv/energy/truth", record_message_2)


def record_actuator(message):
    """记录实测舵角，用于验证手动输入到物理关节的响应。

    Args:
        message: 执行器遥测消息。
    """
    global last_actuator_wall
    last_actuator_wall = time.monotonic()
    actuators.append(list(message.data))


node.subscribe(Double_V, "/usv/actuators/state", record_actuator)


def record_environment(message):
    """记录物理插件实际采用的风速和局部波面，供输入核验。

    Args:
        message: 环境Double_V遥测，字段布局见HullHydrostatics发布实现。
    """
    environment.append(list(message.data))


node.subscribe(Double_V, "/usv/environment/state", record_environment)
wind = node.advertise("/vrx/wind/velocity", Vector3d)
flow = node.advertise("/vrx/current/velocity", Vector3d)
world = out / "world.sdf"
subprocess.run(
    [
        "python3",
        str(tool_path("prepare_sydney.py")),
        "--profile",
        "speed15_concept",
        "--wave-gain",
        str(a.wave_gain),
        "--output",
        str(world),
    ],
    check=True,
    stdout=subprocess.DEVNULL,
)
# Capture the exact boat and flight-control defaults before launching.
tree = E.parse(world)
for inc in tree.findall("world/include"):
    if a.isolated_water and inc.findtext("name") != "usv":
        tree.find("world").remove(inc)
        continue
    if inc.findtext("name") == "usv":
        snapshot = out / "boat.sdf"
        shutil.copy2(inc.findtext("uri"), snapshot)
        inc.find("uri").text = str(snapshot)
if a.no_rendering:
    boat = E.parse(snapshot)
    for parent in boat.iter():
        for sensor in list(parent.findall("sensor")):
            if sensor.get("type") in {"camera", "gpu_lidar", "gpu_ray"}:
                parent.remove(sensor)
    boat.write(snapshot, encoding="utf-8", xml_declaration=True)
    for plugin in list(tree.findall("world/plugin")):
        if plugin.get("name") == "gz::sim::systems::Sensors":
            tree.find("world").remove(plugin)
tree.write(world, encoding="utf-8", xml_declaration=True)
for parameter_file in ["boat.parm", "platform15.parm", "navigation_low_speed.parm"]:
    shutil.copy2(
        root / "src/usv_ardupilot/config" / parameter_file, out / parameter_file
    )
# Bind the generated scenario and actual executable inputs, not just source defaults.
inputs = [
    world,
    snapshot,
    _ProjectPath(__file__),
    root.parent / "tools/lib/navigation_acceptance.py",
    root / "src/usv_ardupilot/scripts/json_bridge.py",
]
inputs.append(weather_limits_path)
inputs += list(out.glob("*.parm"))
inputs += list((root / "install/usv_simulation/lib").glob("*.so"))
inputs += list((root / "install/usv_physics/lib").glob("*.so"))
input_hashes = {
    str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in inputs
}
(out / "runtime-inputs.json").write_text(json.dumps(input_hashes, indent=2))
procs = []
files = []


def start(args, name):
    """启动导航试验子进程并保留日志文件。

    Args:
        args: 子进程命令参数列表
        name: 输出日志文件名
    """
    f = (out / name).open("w")
    files.append(f)
    p = subprocess.Popen(
        args, cwd=out, stdout=f, stderr=subprocess.STDOUT, start_new_session=True
    )
    procs.append(p)
    return p


start(
    ["bash", str(tool_path("run_hull_trial.sh")), "world:=" + str(world)], "gazebo.log"
)
start(
    [
        "bash",
        str(tool_path("run_ardupilot.sh")),
        "--port",
        "9432",
        "--log",
        str(out / "json_frames.jsonl"),
    ],
    "bridge.log",
)
if not a.external_sitl:
    start(
        [
            "python3",
            str(tool_path("run_sitl.py")),
            "--instance",
            "43",
            "--profile",
            "navigation",
            "--wipe",
        ],
        "sitl.log",
    )
conn = None
status = []
events = []
track = []
home = None
waypoints = None
reached = set()
requested = set()
ack = False
armed = False
mode = None
gps = None
phase = "init"
phase_t = 0.0
last_hb = last_arm = last_upload = last_rc = 0.0
begin = time.monotonic()
checks = required_checks(a.failsafe_only, a.response_only, weather_enabled)
response_samples = []
response_index = -1
parameter_values = {}
parameter_targets = {}
for line in (out / "navigation_low_speed.parm").read_text().splitlines():
    if line.strip() and not line.lstrip().startswith("#"):
        name, value = line.split()
        parameter_targets[name] = float(value)
parameter_targets.update(
    AHRS_EKF_TYPE=3,
    EK3_ENABLE=1,
    FS_GCS_ENABLE=1,
    FS_ACTION=2,
    FS_TIMEOUT=1.5,
    FS_THR_ENABLE=1,
    RC_OVERRIDE_TIME=3,
)
if a.response_only:
    parameter_targets.update(
        RC1_MIN=1100,
        RC1_MAX=1900,
        RC1_TRIM=1500,
        RC1_DZ=30,
        RC3_MIN=1100,
        RC3_MAX=1900,
        RC3_TRIM=1500,
        RC3_DZ=30,
        MANUAL_OPTIONS=0,
        MOT_THST_EXPO=1,
        MOT_THR_MIN=1,
    )
parameter_requests = cycle(parameter_targets)
last_parameter_request_wall = 0.0
current_waypoint = 1
route_samples = []
loss_samples = []
loss_status = []
rc_health_times = []
manual_samples = []
outbound_motion_samples = []
manual_stage = 0
last_heartbeat_wall = 0.0
estimator_ready_since = None
last_estimator_wall = 0.0
estimator_faults = []
estimator_log = []
hold_samples = []
auto_speeds = []
rc_health = []
loiter_center = None
loiter_errors = []
loiter_observations = []
loiter_target = None
last_target_wall = 0.0
last_gps_wall = 0.0
home_errors = []
phase_start = None
M = mavutil.mavlink


def event(kind, **data):
    """记录当前仿真时刻的导航事件并打印进度。

    Args:
        kind: 事件类型
        data: 该事件附加的命名字段
    """
    events.append(dict(time=rows[-1][0] if rows else 0, event=kind, **data))
    print(events[-1], flush=True)


def setmode(value):
    """发送飞控自定义模式切换请求。

    Args:
        value: ArduPilot 自定义模式编号
    """
    conn.mav.set_mode_send(1, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, value)


def transition(value, sim):
    """切换导航试验阶段并记录阶段开始时间。

    Args:
        value: 目标阶段标识
        sim: 当前仿真时间，s
    """
    global phase, phase_t
    phase = value
    phase_t = sim
    event(value)
    # 导航包络试验在AUTO、LOITER和RTL施加同一世界坐标水流，其他阶段清零。
    active = value in {"loiter", "response"} or (
        a.current_navigation and value in {"auto", "rtl"}
    )
    east, north = (a.current_east, a.current_north) if active else (0.0, 0.0)
    flow.publish(Vector3d(x=east, y=north, z=0))
    event("current_command", east_m_s=east, north_m_s=north)


def offset(lat, lon, east, north):
    """按本地小范围近似把东、北位移转换为经纬度。

    Args:
        lat: 基准纬度，度
        lon: 基准经度，度
        east: 向东位移，m
        north: 向北位移，m
    """
    return lat + north / 111319.49, lon + east / (
        111319.49 * math.cos(math.radians(lat))
    )


def send_item(seq):
    """发送指定序号的预设航点。

    Args:
        seq: 航点列表中的序号
    """
    lat, lon = waypoints[seq]
    conn.mav.mission_item_int_send(
        1,
        1,
        seq,
        M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        M.MAV_CMD_NAV_WAYPOINT,
        0,
        1,
        0,
        3,
        0,
        0,
        int(lat * 1e7),
        int(lon * 1e7),
        0,
    )


def manual_channels(phase, stage):
    """返回手动试验的原生 RC 通道；其他阶段保持中位。

    Args:
        phase: 当前试验阶段。
        stage: 手动阶段，前进、左舵、右舵、停推。
    """
    if phase == "response":
        return response_command(max(0.0, sim - phase_t))[1]
    if phase != "manual":
        return (1500, 1500, 1500)
    steering = [1500, 1250, 1750, 1500][stage]
    return (steering, 1500, 1600 if stage < 3 else 1500)


def manual_response(samples):
    """用各阶段后四秒验证推进、实测舵角与停推响应。

    Args:
        samples: elapsed、推进请求、纵向速度、实测舵角、阶段。
    """
    stages = [
        [r for r in samples if int(r[4]) == stage and r[0] % 8 > 4]
        for stage in range(4)
    ]
    if not all(stages):
        return False
    return bool(
        np.median([r[1] for r in stages[0]]) > 0.05
        and max(r[2] for r in stages[0]) > 0.1
        and np.median([r[3] for r in stages[1]]) < -0.1
        and np.median([r[3] for r in stages[2]]) > 0.1
        and all(abs(r[1]) < 0.01 for r in stages[3])
    )


try:
    while time.monotonic() - begin < a.timeout:
        if any(p.poll() is not None for p in procs):
            event("process_exit")
            break
        if conn is None:
            try:
                conn = mavutil.mavlink_connection(
                    "tcp:127.0.0.1:6190", source_system=255, source_component=190
                )
            except (ConnectionError, OSError):
                time.sleep(0.2)
                continue
        now = time.monotonic()
        sim = rows[-1][0] if rows else 0
        if weather_enabled:
            wind.publish(Vector3d(x=a.wind_east, y=a.wind_north, z=0))
        for _ in range(60):
            m = conn.recv_match(blocking=False)
            if m is None:
                break
            kind = m.get_type()
            if kind == "STATUSTEXT":
                status.append(m.text)
                if phase == "gcs_loss":
                    loss_status.append(m.text)
                print("SITL:", m.text, flush=True)
                if armed and "EKF variance" in m.text:
                    estimator_faults.append(m.text)
            elif kind == "EKF_STATUS_REPORT":
                if now - last_estimator_wall >= 1:
                    estimator_ready_since = None
                last_estimator_wall = now
                required = M.EKF_ATTITUDE | M.EKF_VELOCITY_HORIZ | M.EKF_POS_HORIZ_ABS
                valid = (
                    (m.flags & required) == required
                    and math.isfinite(m.velocity_variance)
                    and math.isfinite(m.pos_horiz_variance)
                    and m.velocity_variance < 1
                    and m.pos_horiz_variance < 1
                )
                estimator_log.append(dict(sim_s=sim, **m.to_dict()))
                if not valid:
                    estimator_ready_since = None
                    if armed:
                        estimator_faults.append("invalid EKF flags/variance")
                elif estimator_ready_since is None:
                    estimator_ready_since = now
            elif kind == "PARAM_VALUE":
                parameter_values[m.param_id] = m.param_value
            elif kind == "HEARTBEAT":
                last_heartbeat_wall = now
                armed = bool(m.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
                mode = m.custom_mode
            elif kind == "POSITION_TARGET_GLOBAL_INT":
                loiter_target = (m.lat_int * 1e-7, m.lon_int * 1e-7)
                last_target_wall = now
            elif kind == "MISSION_CURRENT":
                current_waypoint = m.seq
            elif kind == "GLOBAL_POSITION_INT":
                gps = (m.lat * 1e-7, m.lon * 1e-7)
                last_gps_wall = now
            elif kind == "SYS_STATUS" and phase == "gcs_loss":
                rc_health_times.append(now)
                rc_health.append(
                    bool(
                        m.onboard_control_sensors_health
                        & M.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
                    )
                )
            elif (
                kind in ["MISSION_REQUEST_INT", "MISSION_REQUEST"]
                and waypoints is not None
                and m.seq < len(waypoints)
            ):
                requested.add(m.seq)
                send_item(m.seq)
            elif kind == "MISSION_ACK":
                ack = bool(
                    waypoints is not None
                    and requested == set(range(len(waypoints)))
                    and m.type == M.MAV_MISSION_ACCEPTED
                )
            elif kind == "MISSION_ITEM_REACHED":
                reached.add(m.seq)
                event("waypoint_reached", seq=m.seq)
        if phase != "gcs_loss" and now - last_hb > 1:
            conn.mav.heartbeat_send(M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_hb = now
        # Keep an independent neutral RC link alive when removing ONLY GCS heartbeat.
        # Native SITL RC UDP is independent: MAVLink RC overrides also refresh GCS last-seen.
        if now - last_rc > 0.2:
            rc_socket.sendto(
                struct.pack(
                    "<8H",
                    *manual_channels(phase, manual_stage),
                    1500,
                    1800,
                    1000,
                    1000,
                    1800
                ),
                ("127.0.0.1", 5931),
            )
            last_rc = now
        checks["low_speed_parameters"] = all(
            name in parameter_values and abs(parameter_values[name] - value) < 1e-5
            for name, value in parameter_targets.items()
        )
        if phase != "init" and now - last_heartbeat_wall > 2:
            event("heartbeat_stale")
            break
        if phase != "init" and now - last_physics_wall > 2:
            event("physics_telemetry_stale")
            break
        navigation_ready = (
            estimator_ready_since is not None
            and now - estimator_ready_since >= 2
            and now - last_estimator_wall < 1
        )
        if now - last_estimator_wall >= 1:
            estimator_ready_since = None
        if armed and now - last_estimator_wall > 2:
            estimator_faults.append("EKF telemetry stale")
            break
        if phase == "init":
            # 逐项限速读回，避免大批请求挤满飞控参数响应队列；不得绕过读回校验。
            if now - last_parameter_request_wall >= 0.1:
                name = next(parameter_requests)
                if (
                    name not in parameter_values
                    or abs(parameter_values[name] - parameter_targets[name]) >= 1e-5
                ):
                    conn.mav.param_request_read_send(1, 1, name.encode(), -1)
                last_parameter_request_wall = now
            if now - last_arm > 4:
                setmode(15)
                conn.mav.request_data_stream_send(1, 1, M.MAV_DATA_STREAM_ALL, 10, 1)
                conn.mav.command_long_send(
                    1,
                    1,
                    M.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    M.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
                    200000,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                conn.mav.command_long_send(
                    1,
                    1,
                    M.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    M.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT,
                    200000,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                if mode == 15 and navigation_ready and checks["low_speed_parameters"]:
                    conn.mav.command_long_send(
                        1, 1, M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
                    )
                last_arm = now
            if armed and gps and rows:
                checks["normal_arming"] = True
                checks["navigation_ready_before_arming"] = navigation_ready
                home = rows[-1][12:14]
                phase_start = sim
                if a.response_only:
                    setmode(0)
                    transition("response", sim)
                    continue
                if a.failsafe_only:
                    lat, lon = offset(*gps, 40, 0)
                    conn.mav.mission_item_int_send(
                        1,
                        1,
                        0,
                        M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        M.MAV_CMD_NAV_WAYPOINT,
                        2,
                        0,
                        0,
                        0,
                        0,
                        0,
                        int(lat * 1e7),
                        int(lon * 1e7),
                        0,
                    )
                    transition("outbound", sim)
                    continue
                waypoints = [
                    gps,
                    offset(*gps, 40, 0),
                    offset(*gps, 40, -30 if a.mirror_course else 30),
                    offset(*gps, 10, -30 if a.mirror_course else 30),
                ]
                conn.mav.mission_clear_all_send(1, 1)
                transition("upload", sim)
        elif phase == "response":
            elapsed = sim - phase_t
            index, channels = response_command(elapsed)
            if index >= len(RESPONSE_PLAN):
                setmode(4)
                transition("complete", sim)
                break
            if index != response_index:
                response_index = index
                event(
                    "response_stage",
                    stage=index,
                    name=RESPONSE_PLAN[index][0],
                    pwm=channels,
                )
            if (
                rows
                and actuators
                and now - last_actuator_wall < 0.5
                and (not response_samples or sim > response_samples[-1][0])
            ):
                response_samples.append(
                    [
                        sim,
                        elapsed,
                        index,
                        mode,
                        rows[-1][1],
                        rows[-1][10],
                        actuators[-1][1],
                        actuators[-1][3],
                        rows[-1][4],
                        rows[-1][14],
                    ]
                )
        elif phase == "upload":
            if now - last_upload > 5:
                conn.mav.mission_count_send(1, 1, len(waypoints))
                last_upload = now
            if ack:
                checks["mission_upload"] = True
                conn.mav.mission_set_current_send(1, 1, 1)
                setmode(10)
                transition("start_auto", sim)
        elif phase == "start_auto":
            if mode == 10:
                transition("auto", sim)
            elif now - last_arm > 2:
                setmode(10)
                last_arm = now
            if sim - phase_t > 45:
                event("auto_mode_rejected")
                break
        elif phase == "auto":
            if rows and 1 <= current_waypoint <= 3:
                route_samples.append([*rows[-1][12:14], current_waypoint, sim])
            if sim - phase_t > 10:
                auto_speeds.append(abs(rows[-1][10]))
            if 3 in reached:
                checks["auto_waypoints"] = bool({1, 2, 3}.issubset(reached) and armed)
                loiter_center = rows[-1][12:14]
                setmode(5)
                transition("loiter", sim)
            elif sim - phase_t > 400:
                event("auto_timeout")
                break
        elif phase == "loiter":
            flow.publish(Vector3d(x=a.current_east, y=a.current_north, z=0))
            if sim - phase_t > 15:
                loiter_errors.append(
                    float(np.linalg.norm(np.array(rows[-1][12:14]) - loiter_center))
                )
            if (
                gps
                and loiter_target
                and now - last_target_wall < 1
                and now - last_gps_wall < 1
            ):
                north = (gps[0] - loiter_target[0]) * 111319.49
                east = (
                    (gps[1] - loiter_target[1])
                    * 111319.49
                    * math.cos(math.radians(gps[0]))
                )
                loiter_observations.append(
                    [sim - phase_t, math.hypot(east, north), *gps, *loiter_target]
                )
            if sim - phase_t > 120:
                checks["loiter_mode"] = mode == 5
                checks["loiter_target_observed"] = observed_loiter_target(
                    loiter_observations
                )
                stable_errors = [r[1] for r in loiter_observations if r[0] > 15]
                checks["loiter_target_radius"] = bool(
                    stable_errors and max(stable_errors) <= 3.0
                )
                checks["loiter_radius"] = bool(
                    loiter_errors and max(loiter_errors) < 12
                )
                flow.publish(Vector3d(x=0, y=0, z=0))
                setmode(4)
                transition("hold", sim)
        elif phase == "hold":
            if sim - phase_t > 20:
                hold_samples.append(rows[-1])
            if sim - phase_t > 30:
                checks["hold_stops_command"] = bool(
                    hold_samples
                    and mode == 4
                    and all(abs(r[1]) < 0.01 for r in hold_samples)
                )
                checks["hold_slows_vessel"] = bool(
                    hold_samples and max(abs(r[10]) for r in hold_samples) < 0.3
                )
                setmode(0)
                transition("manual", sim)
        elif phase == "manual":
            if mode == 0 and rows and actuators and now - last_actuator_wall < 0.5:
                elapsed = sim - phase_t
                manual_stage = min(int(elapsed // 8), 3)
                manual_samples.append(
                    [elapsed, rows[-1][1], rows[-1][10], actuators[-1][1], manual_stage]
                )
                if elapsed > 32:
                    checks["manual_takeover"] = manual_response(manual_samples)
                    manual_stage = 0
                    setmode(11)
                    transition("rtl", sim)
            elif sim - phase_t > 10:
                break
        elif phase == "rtl":
            dist = float(np.linalg.norm(np.array(rows[-1][12:14]) - home))
            home_errors.append(dist)
            if dist < 6 and sim - phase_t > 5:
                checks["rtl_mode"] = mode == 11
                checks["rtl_home"] = True
                # Head away again in GUIDED, then stop only GCS heartbeats. Keep JSON physics alive.
                setmode(15)
                lat, lon = offset(*gps, 40, 0)
                conn.mav.mission_item_int_send(
                    1,
                    1,
                    0,
                    M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    M.MAV_CMD_NAV_WAYPOINT,
                    2,
                    0,
                    0,
                    0,
                    0,
                    0,
                    int(lat * 1e7),
                    int(lon * 1e7),
                    0,
                )
                transition("outbound", sim)
            elif sim - phase_t > 180:
                event("rtl_timeout")
                break
        elif phase == "outbound":
            sample = [rows[-1][0], rows[-1][10], rows[-1][1], mode]
            if (
                not outbound_motion_samples
                or sample[0] > outbound_motion_samples[-1][0]
            ):
                outbound_motion_samples.append(sample)
            motion_window = [
                row for row in outbound_motion_samples if row[0] >= sample[0] - 3
            ]
            if sim - phase_t > 12 and sustained_motion(motion_window):
                checks["guided_mode"] = mode == 15
                checks["moving_before_loss"] = True
                event(
                    "outbound_motion_ready",
                    speed_m_s=sample[1],
                    command=sample[2],
                    duration_s=motion_window[-1][0] - motion_window[0][0],
                )
                transition("gcs_loss", sim)
            elif sim - phase_t > 60:
                event("outbound_motion_timeout")
                break
        elif phase == "gcs_loss":
            if sim - phase_t >= 10 and (
                not loss_samples or rows[-1][0] > loss_samples[-1][0]
            ):
                loss_samples.append([rows[-1][0], rows[-1][1], mode])
            if sim - phase_t > 18:
                checks["gcs_failsafe_hold"] = mode == 4
                checks["gcs_failsafe_stops_command"] = sustained_stop(loss_samples)
                checks["failsafe_status_reported"] = any(
                    s.endswith(" Failsafe") for s in loss_status
                )
                checks["independent_rc_link_healthy"] = bool(
                    len(rc_health) >= 10
                    and all(rc_health)
                    and rc_health_times[-1] - rc_health_times[0] >= 15
                    and max(np.diff(rc_health_times)) < 1
                )
                transition("complete", sim)
                break
        if rows:
            track.append(
                [
                    sim,
                    rows[-1][12],
                    rows[-1][13],
                    rows[-1][10],
                    mode if mode is not None else -1,
                ]
            )
        time.sleep(0.01)
except KeyboardInterrupt:
    event("interrupted")
    phase = "interrupted"
finally:
    if conn:
        setmode(4)
        conn.mav.rc_channels_override_send(
            1, 1, 1500, 1500, 1500, 1500, 65535, 65535, 65535, 65535
        )
        conn.mav.command_long_send(
            1, 1, M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
        )
    for p in reversed(procs):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
    for f in files:
        f.close()
    rc_socket.close()
if not a.failsafe_only:
    logs = root.parent / "artifacts/sitl/navigation/instance_43/logs"
    if list(logs.glob("*.BIN")):

        def modification_time_3(p):
            """返回文件修改时间，供最新记录排序。

            Args:
                p: 待排序的 pathlib 路径对象。
            """
            return p.stat().st_mtime

        shutil.copy2(
            max(logs.glob("*.BIN"), key=modification_time_3),
            out / "flight_control.BIN",
        )
path_metrics = {}
checks["completed"] = phase == "complete"
if not a.failsafe_only and not a.response_only and home is not None and track:
    tr = np.array(track)
    north = -30.0 if a.mirror_course else 30.0
    points = np.array([[0.0, 0.0], [40.0, 0.0], [40.0, north], [10.0, north]]) + home

    def distance_to_segment(points, start, end):
        """计算平面点到有限线段的距离。

        Args:
            points: 待评估的二维点数组
            start: 线段起点，与点数组同单位
            end: 线段终点，与点数组同单位
        """
        v = end - start
        q = np.clip((points - start) @ v / np.dot(v, v), 0, 1)
        return np.linalg.norm(points - (start + q[:, None] * v), axis=1)

    if route_samples:
        path_error = np.array(
            [
                distance_to_segment(
                    np.array([sample[:2]]),
                    points[int(sample[2]) - 1],
                    points[int(sample[2])],
                )[0]
                for sample in route_samples
            ]
        )
        np.savetxt(
            out / "active_segment.csv",
            np.column_stack((route_samples, path_error)),
            delimiter=",",
        )
        checks["auto_path_corridor"] = bool(np.max(path_error) < 15)
        path_metrics = dict(
            max_m=float(np.max(path_error)),
            p95_m=float(np.percentile(path_error, 95)),
            rms_m=float(np.sqrt(np.mean(path_error**2))),
        )
        events.append(
            {
                "event": "auto_path_audit",
                "max_distance_to_route_m": float(np.max(path_error)),
            }
        )
checks["estimator_healthy_after_arming"] = bool(
    estimator_log and not estimator_faults and phase_start is not None
)
if not a.failsafe_only and not a.response_only:
    checks["low_speed_tracking"] = bool(
        auto_speeds
        and 0.5 <= np.median(auto_speeds) <= 1.3
        and np.percentile(auto_speeds, 95) <= 1.5
    )
(out / "estimator.json").write_text(
    json.dumps(dict(samples=estimator_log, faults=estimator_faults), indent=2)
)
checks["finite"] = bool(rows and np.isfinite(rows).all())
if a.response_only:
    checks["response_coverage"] = response_coverage(response_samples)
    checks["response_manual"] = bool(
        response_samples and all(r[3] == 0 for r in response_samples if r[1] > 2)
    )
    excitation = []
    end = 0
    for index, (_, duration, channels) in enumerate(RESPONSE_PLAN):
        end += duration
        tail = [r for r in response_samples if int(r[2]) == index and r[1] > end - 5]
        if not tail:
            excitation.append(False)
            continue
        command = float(np.median([r[4] for r in tail]))
        angle = float(np.median([r[6] for r in tail]))
        direction = np.sign(channels[2] - 1500)
        helm = np.sign(channels[0] - 1500)
        excitation.append(
            bool(
                (abs(command) < 0.01 if direction == 0 else command * direction > 0.05)
                and (abs(angle) < 0.1 if helm == 0 else angle * helm > 0.1)
            )
        )
    checks["response_excitation"] = all(excitation)
    np.savetxt(out / "response.csv", np.array(response_samples), delimiter=",")
if weather_enabled:
    checks.update(
        weather_checks(
            environment,
            rows,
            (a.wind_east, a.wind_north, 0),
            a.wave_gain,
            weather_limits,
        )
    )
r = dict(
    passed=passed_checks(checks, a.failsafe_only, a.response_only, weather_enabled),
    response_plan=RESPONSE_PLAN if a.response_only else None,
    loiter_target_samples=loiter_observations,
    path_metrics=path_metrics,
    mirrored_course=a.mirror_course,
    actual_parameters=parameter_values,
    required_parameters=parameter_targets,
    manual_response_samples=manual_samples,
    outbound_motion_samples=outbound_motion_samples,
    loss_stop_samples=loss_samples,
    loss_status=loss_status,
    scope=dict(
        kind="open_loop_response" if a.response_only else "navigation_acceptance",
        isolated_water=a.isolated_water,
        rendering=not a.no_rendering,
        target_speed_m_s=None if a.response_only else 1.0,
        current_east_m_s=a.current_east,
        current_north_m_s=a.current_north,
        wind_east_m_s=a.wind_east,
        wind_north_m_s=a.wind_north,
        wave_gain=a.wave_gain,
        wind_scope="constant throughout all phases" if weather_enabled else "zero",
        weather_screen_limits=weather_limits if weather_enabled else None,
        current_phases=(
            ["auto", "loiter", "rtl"] if a.current_navigation else ["loiter"]
        ),
    ),
    auto_speed_median_m_s=float(np.median(auto_speeds)) if auto_speeds else None,
    auto_speed_p95_m_s=float(np.percentile(auto_speeds, 95)) if auto_speeds else None,
    checks=checks,
    final_phase=phase,
    events=events,
    waypoints_lat_lon=waypoints,
    home_xy=home,
    loiter_max_error_m=max(loiter_errors) if loiter_errors else None,
    rtl_min_distance_m=min(home_errors) if home_errors else None,
    max_speed_km_h=max((r[10] for r in rows), default=0) * 3.6,
    status_tail=status[-30:],
    rc_health_samples_during_gcs_loss=rc_health,
    rc_source="native SITL UDP 5931; no MAVLink RC overrides during experiment",
)
for name, data in [
    ("propulsion", rows),
    ("energy", energy),
    ("track", track),
    ("actuators", actuators),
    ("environment", environment),
]:
    np.savetxt(out / (name + ".csv"), np.array(data), delimiter=",")
(out / "result.json").write_text(json.dumps(r, indent=2))
print(json.dumps(r, indent=2))
raise SystemExit(0 if r["passed"] else 1)
