#!/usr/bin/env python3
"""将 Gazebo 物理状态转换为本机 ArduPilot JSON SITL 输入。

Gazebo ENU/FLU physics -> ArduPilot NED/FRD JSON SITL (localhost only).

SERVO1 steering and SERVO3 throttle. Never run alongside another actuator owner.
Uses native IMU and full ground-truth pose/velocity as SITL physics input;
ArduPilot builds its own simulated navigation sensors from that input.
"""
import argparse
import json
import math
import socket
import struct
import time
import threading
import signal
import numpy as np
from scipy.spatial.transform import Rotation
from json_time import StepLimiter, encode_sensor_packet
from gz.transport13 import Node
from usv_control.ownership import ControlLease
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.double_pb2 import Double
from gz.msgs10.double_v_pb2 import Double_V

p = argparse.ArgumentParser()
p.add_argument("--name", default="usv")
p.add_argument("--port", type=int, default=9002)
p.add_argument("--duration", type=float, default=0)
p.add_argument("--log")
a = p.parse_args()
if not a.name.isidentifier() or not 1024 <= a.port <= 65535:
    p.error("Invalid name / localhost UDP port")
lease = ControlLease(a.name, "ArduPilot JSON SITL")
node = Node()
prefix = "/" + a.name
latest = {}
lock = threading.Lock()


def save(key, msg):
    """缓存最新消息及其单调接收时间。

    Args:
        key: 消息类别键
        msg: 需要缓存的 Gazebo 消息
    """
    with lock:
        latest[key] = (msg, time.monotonic())


def cache_input_1(m):
    """缓存桥接输入及其接收时间。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return save("imu", m)


node.subscribe(IMU, prefix + "/sensors/imu", cache_input_1)


def cache_input_2(m):
    """缓存桥接输入及其接收时间。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return save("odom", m)


node.subscribe(Odometry, prefix + "/ground_truth/odometry", cache_input_2)


def cache_input_3(m):
    """缓存桥接输入及其接收时间。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return save("energy", m)


node.subscribe(Double_V, prefix + "/energy/truth", cache_input_3)
throttle = node.advertise(prefix + "/propulsion/throttle", Double)
steering = node.advertise(prefix + "/steering/angle", Double)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", a.port))
sock.settimeout(0.002)
enu_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])
flu_frd = np.diag([1.0, -1.0, -1.0])
th = st = 0.0
limiter = StepLimiter()
pending = None
last_frame = None
last_packet = None
max_lag = 0.0
retries = 0
peer = None
last_time = -1.0
last_pwm_wall = 0.0
count = 0
last_log = 0.0
begin = time.monotonic()
f = open(a.log, "w") if a.log else None


def stop_signal(signum, frame):
    """把终止信号转为中断以进入既有清理路径。

    Args:
        signum: 信号编号
        frame: 信号发生时的 Python 栈帧
    """
    raise KeyboardInterrupt


signal.signal(signal.SIGTERM, stop_signal)
print("Listening JSON SITL on localhost UDP", a.port, flush=True)
try:
    while a.duration <= 0 or time.monotonic() - begin < a.duration:
        try:
            packet, remote = sock.recvfrom(256)
            if remote[0] != "127.0.0.1":
                continue
            if len(packet) == 40:
                magic, rate, frame, *pwm = struct.unpack("<HHI16H", packet)
                valid = magic == 18458
            elif len(packet) == 72:
                magic, rate, frame, *pwm = struct.unpack("<HHI32H", packet)
                valid = magic == 29569
            else:
                continue
            if not valid:
                continue
            peer = remote
            if frame == last_frame and last_packet is not None:
                if time.monotonic() - last_pwm_wall > 1:
                    throttle.publish(Double(data=0.0))
                sock.sendto(last_packet, peer)
                retries += 1
                continue
            last_pwm_wall = time.monotonic()
            pending = frame
            # Disarmed/invalid servo values are neutral, never interpreted as full reverse.
            th = (
                max(-1.0, min(1.0, (pwm[2] - 1500) / 500))
                if 1000 <= pwm[2] <= 2000
                else 0.0
            )
            st = (
                max(-1.0, min(1.0, (pwm[0] - 1500) / 500)) * math.radians(35)
                if 1000 <= pwm[0] <= 2000
                else 0.0
            )
        except socket.timeout:
            pass
        if time.monotonic() - last_pwm_wall > 1:
            throttle.publish(Double(data=0.0))
        if peer is None or pending is None:
            continue
        with lock:
            snapshot = dict(latest)
        if not all(k in snapshot for k in ["imu", "odom", "energy"]):
            throttle.publish(Double(data=0.0))
            continue
        if any(time.monotonic() - snapshot[k][1] > 0.5 for k in snapshot):
            throttle.publish(Double(data=0.0))
            continue
        imu = snapshot["imu"][0]
        od = snapshot["odom"][0]
        energy = snapshot["energy"][0].data
        if len(energy) < 25 or energy[23] > 1e-3:
            throttle.publish(Double(data=0.0))
            continue
        stamp = imu.header.stamp.sec + imu.header.stamp.nsec * 1e-9
        if stamp < last_time:
            raise RuntimeError(
                "Gazebo reset detected: restart SITL and bridge together"
            )
        if stamp == last_time:
            continue
        pos = enu_ned @ np.array(
            [od.pose.position.x, od.pose.position.y, od.pose.position.z]
        )
        # Gazebo OdometryPublisher twist is expressed in the child/body frame.
        body_velocity = np.array(
            [od.twist.linear.x, od.twist.linear.y, od.twist.linear.z]
        )
        oq = od.pose.orientation
        odomR = Rotation.from_quat([oq.x, oq.y, oq.z, oq.w]).as_matrix()
        velocity = enu_ned @ odomR @ body_velocity
        odom_stamp = od.header.stamp.sec + od.header.stamp.nsec * 1e-9
        offset = stamp - odom_stamp
        if abs(offset) > 0.15:
            throttle.publish(Double(data=0.0))
            continue
        pos += velocity * offset
        # IMU quaternion is relative to initial attitude; use world odometry and
        # body gyro to align attitude to the IMU timestamp (bounded extrapolation).
        omega = np.array(
            [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z]
        )
        R = odomR @ Rotation.from_rotvec(omega * offset).as_matrix()
        quat = Rotation.from_matrix(enu_ned @ R @ flu_frd).as_quat()
        gyro = flu_frd @ np.array(
            [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z]
        )
        accel = flu_frd @ np.array(
            [
                imu.linear_acceleration.x,
                imu.linear_acceleration.y,
                imu.linear_acceleration.z,
            ]
        )
        target = {
            "no_time_sync": True,
            "timestamp": stamp,
            "imu": {"gyro": gyro.tolist(), "accel_body": accel.tolist()},
            "position": pos.tolist(),
            "quaternion": [quat[3], *quat[:3]],
            "velocity": velocity.tolist(),
            "battery": {"voltage": energy[2], "current": energy[3]},
        }
        data = limiter.next(target)
        if data is None:
            continue
        lag = stamp - data["timestamp"]
        max_lag = max(max_lag, lag)
        # Do not apply a controller command while catching up through stale physics.
        fresh_pwm = time.monotonic() - last_pwm_wall <= 1 and lag <= 0.1
        throttle.publish(Double(data=th if fresh_pwm else 0.0))
        steering.publish(Double(data=st if fresh_pwm else 0.0))
        last_packet = encode_sensor_packet(data)
        sock.sendto(last_packet, peer)
        last_frame = pending
        pending = None
        last_time = data["timestamp"]
        count += 1
        if f and data["timestamp"] - last_log >= 0.1:
            f.write(
                json.dumps(
                    dict(
                        data,
                        pwm_steering=pwm[0],
                        pwm_throttle=pwm[2],
                        gazebo_timestamp=stamp,
                        bridge_lag_s=lag,
                    )
                )
                + "\n"
            )
            f.flush()
            last_log = data["timestamp"]
except KeyboardInterrupt:
    pass
finally:
    for _ in range(3):
        throttle.publish(Double(data=0.0))
        time.sleep(0.02)
    for topic in ["/sensors/imu", "/ground_truth/odometry", "/energy/truth"]:
        node.unsubscribe(prefix + topic)
    sock.close()
    lease.close()
    if f:
        f.close()
    report = {
        "frames_sent": count,
        "interpolated_frames": limiter.interpolated,
        "max_time_step_s": limiter.max_delta,
        "max_lag_s": max_lag,
        "retransmissions": retries,
        "protocol": "one reply per servo frame; <=20ms steps; Gazebo owns time",
    }
    if a.log:
        with open(a.log + ".summary.json", "w") as fp:
            json.dump(report, fp, indent=2)
    print("JSON bridge summary:", report, flush=True)
