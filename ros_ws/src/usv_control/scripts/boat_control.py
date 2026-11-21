#!/usr/bin/env python3
"""处理船舶控制、能源就绪、消息新鲜度及失联停推。

ROS payload power gating and bounded single-azimuth velocity/heading control.

Navigation feedback uses IMU + GPS velocity, never the ground-truth odometry.
Direct actuator commands override cmd_vel; heartbeat expiry stops propulsion.
"""
import math
import os
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import (
    Imu,
    NavSatFix,
    MagneticField,
    Image,
    CameraInfo,
    LaserScan,
    PointCloud2,
)
from geometry_msgs.msg import Twist, Vector3Stamped, TransformStamped
from std_msgs.msg import Float64, Float64MultiArray, String
from tf2_ros import StaticTransformBroadcaster
from gz.transport13 import Node as GzNode
from usv_control.ownership import ControlLease
from gz.msgs10.double_pb2 import Double
from gz.msgs10.double_v_pb2 import Double_V
from gz.msgs10.navsat_pb2 import NavSat


def wrap(v):
    """将角度归一化到主值区间。

    Args:
        v: 输入角度，单位 rad。
    """
    return math.atan2(math.sin(v), math.cos(v))


class BoatControl(Node):
    """桥接载荷消息并管理带就绪和失联保护的船舶控制。"""

    def __init__(self):
        """创建控制发布订阅、静态坐标变换和看门狗定时器。"""
        super().__init__("boat_control")
        self.declare_parameter("boat_name", "usv")
        self.declare_parameter("control_enabled", True)
        self.control_enabled = self.get_parameter("control_enabled").value
        self.declare_parameter("speed_feedforward_m_s", 1.63)
        self.speed_scale = float(self.get_parameter("speed_feedforward_m_s").value)
        if self.speed_scale <= 0:
            raise ValueError("Invalid speed scale")
        self.boat = self.get_parameter("boat_name").value
        self.prefix = "/" + self.boat
        self.payload_cfg = json.loads(
            (
                Path(get_package_share_directory("usv_description"))
                / "config/payloads.json"
            ).read_text()
        )
        self.lease = (
            ControlLease(self.boat, "ROS cmd_vel/direct actuators")
            if self.control_enabled
            else None
        )
        self.gz = GzNode()
        self.command_transport = GzNode()
        self.throttle = self.command_transport.advertise(
            self.prefix + "/propulsion/throttle", Double
        )
        self.steering = self.command_transport.advertise(
            self.prefix + "/steering/angle", Double
        )
        self.physics_throttle = 0.0
        self.direct_request = 0.0
        self.direct_request_wall = 0.0
        self.command_mismatch_since = None
        self.energy = None
        self.energy_wall = 0
        self.sim = 0.0
        self.actuators = []
        self.actuator_wall = 0.0
        self.physics_wall = 0.0
        self.health_wall = 0.0
        self.command_sequence = 0
        self.command_mode = "idle"
        self.fixed_throttle = None
        self.transport_retry_wall = time.monotonic()
        self.transport_retries = 0
        self.gz.subscribe(Double_V, self.prefix + "/energy/truth", self.on_energy)
        self.gz.subscribe(Double_V, self.prefix + "/actuators/state", self.on_actuators)
        self.gz.subscribe(Double_V, self.prefix + "/propulsion/state", self.on_physics)
        self.yaw = None
        self.rate = 0.0
        self.velocity = None
        self.imu_wall = 0.0
        self.gps_wall = 0.0
        self.imu_stamp = -1e9
        self.gps_stamp = -1e9
        self.desired = None
        self.command_wall = 0.0
        self.hold_yaw = None
        self.integral = 0.0
        self.last_sim = None
        self.subscriptions_keep = []
        self.publishers_keep = []
        self.status = self.create_publisher(
            Float64MultiArray, self.prefix + "/control/state", 10
        )
        self.health = self.create_publisher(String, self.prefix + "/control/health", 10)
        payloads = [
            ("imu", Imu, "sensors/imu/imu/data"),
            ("gps", NavSatFix, "sensors/gps/gps/fix"),
            ("magnetometer", MagneticField, "sensors/magnetometer/data"),
            ("camera", Image, "sensors/cameras/camera/image_raw"),
            ("camera_info", CameraInfo, "sensors/cameras/camera/camera_info"),
            ("lidar", LaserScan, "sensors/lidars/lidar/scan"),
            ("gps_velocity", Vector3Stamped, "sensors/gps/velocity"),
        ]
        cfg = json.loads(
            (
                Path(get_package_share_directory("usv_description"))
                / "config/payloads.json"
            ).read_text()
        )
        if cfg["advanced_sensors"]:
            for n in ["camera_right", "camera_side"]:
                payloads.extend(
                    [
                        (n, Image, "sensors/cameras/" + n + "/image_raw"),
                        (
                            n + "_info",
                            CameraInfo,
                            "sensors/cameras/" + n + "/camera_info",
                        ),
                    ]
                )
            payloads.append(
                ("lidar_points", PointCloud2, "sensors/lidars/lidar/points")
            )
        for name, kind, topic in payloads:
            pub = self.create_publisher(
                kind, self.prefix + "/" + topic, qos_profile_sensor_data
            )
            self.publishers_keep.append(pub)

            def forward_payload_1(msg, pub=pub, name=name):
                """把载荷消息转交给当前绑定的发布器与载荷名称。

                Args:
                    msg: 当前订阅接收到的消息，类型由订阅接口确定。
                    pub: 回调创建时绑定的 ROS 发布器。
                    name: 回调创建时绑定的载荷名称。
                """
                return self.payload(name, msg, pub)

            self.subscriptions_keep.append(
                self.create_subscription(
                    kind,
                    self.prefix + "/raw/" + name,
                    forward_payload_1,
                    qos_profile_sensor_data,
                )
            )
        self.subscriptions_keep.append(
            self.create_subscription(Twist, self.prefix + "/cmd_vel", self.cmd_vel, 10)
        )
        self.subscriptions_keep.append(
            self.create_subscription(
                Float64, self.prefix + "/actuators/throttle", self.direct_throttle, 10
            )
        )
        self.subscriptions_keep.append(
            self.create_subscription(
                Float64, self.prefix + "/actuators/steering", self.direct_steering, 10
            )
        )
        self.subscriptions_keep.append(
            self.create_subscription(
                Float64,
                self.prefix + "/actuators/heading_hold_throttle",
                self.fixed_heading_throttle,
                10,
            )
        )
        self.gps_velocity_pub = self.create_publisher(
            Vector3Stamped, self.prefix + "/raw/gps_velocity", qos_profile_sensor_data
        )
        self.gz.subscribe(NavSat, self.prefix + "/sensors/gps", self.gps_velocity)
        self.static_tf = StaticTransformBroadcaster(self)
        transforms = []
        cfg = json.loads(
            (
                Path(get_package_share_directory("usv_description"))
                / "config/payloads.json"
            ).read_text()
        )
        poses = {
            k: v
            for k, v in cfg["poses"].items()
            if cfg["advanced_sensors"] or k not in ["camera_right", "camera_side"]
        }
        for name, (x, y, z, r, p, yaw) in poses.items():
            t = TransformStamped()
            t.header.frame_id = self.boat + "/base_link"
            t.child_frame_id = self.boat + "/" + name
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = float(z)
            q = Rotation.from_euler("xyz", [r, p, yaw]).as_quat()
            t.transform.rotation.x = float(q[0])
            t.transform.rotation.y = float(q[1])
            t.transform.rotation.z = float(q[2])
            t.transform.rotation.w = float(q[3])
            transforms.append(t)
        t = TransformStamped()
        t.header.frame_id = self.boat + "/camera"
        t.child_frame_id = self.boat + "/camera_optical"
        t.transform.rotation.x = -0.5
        t.transform.rotation.y = 0.5
        t.transform.rotation.z = -0.5
        t.transform.rotation.w = 0.5
        transforms.append(t)
        if cfg["advanced_sensors"]:
            for n in ["camera_right", "camera_side"]:
                t = TransformStamped()
                t.header.frame_id = self.boat + "/" + n
                t.child_frame_id = self.boat + "/" + n + "_optical"
                t.transform.rotation.x = -0.5
                t.transform.rotation.y = 0.5
                t.transform.rotation.z = -0.5
                t.transform.rotation.w = 0.5
                transforms.append(t)
        self.static_tf.sendTransform(transforms)
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.tick, clock=self.watchdog_clock)

    def gps_velocity(self, msg):
        """将 Gazebo 东北天速度转发为 ROS 世界坐标速度。

        Args:
            msg: Gazebo NavSat 消息，携带时间戳和三轴速度。
        """
        out = Vector3Stamped()
        out.header.stamp.sec = msg.header.stamp.sec
        out.header.stamp.nanosec = msg.header.stamp.nsec
        out.header.frame_id = "world"
        out.vector.x = msg.velocity_east
        out.vector.y = msg.velocity_north
        out.vector.z = msg.velocity_up
        self.gps_velocity_pub.publish(out)

    def on_energy(self, msg):
        """接收有限且完整的能源真值并记录墙钟新鲜度。

        Args:
            msg: 能源 Double_V 遥测数组。
        """
        if len(msg.data) >= 25 and all(map(math.isfinite, msg.data)):
            self.energy = list(msg.data)
            self.sim = msg.data[0]
            self.energy_wall = time.monotonic()

    def on_actuators(self, msg):
        """接收执行器状态并记录反馈时间。

        Args:
            msg: 执行器 Double_V 遥测数组。
        """
        if len(msg.data) >= 8 and all(map(math.isfinite, msg.data)):
            self.actuators = list(msg.data)
            self.actuator_wall = time.monotonic()

    def on_physics(self, msg):
        """检查推进状态完整性并更新时间戳。

        Args:
            msg: 推进物理状态 Double_V 消息。
        """
        if len(msg.data) >= 17 and all(map(math.isfinite, msg.data)):
            self.physics_wall = time.monotonic()
            self.physics_throttle = msg.data[1]

    def actuator_ready(self):
        """检查供电与执行器、物理反馈是否同时可用。"""
        now = time.monotonic()
        return (
            self.powered()
            and now - self.actuator_wall < 1
            and now - self.physics_wall < 1
            and self.command_connected()
        )

    def command_connected(self):
        """确认油门与转向发布器均发现物理订阅端。"""
        return self.throttle.has_connections() and self.steering.has_connections()

    def direct_command_confirmed(self, now):
        """允许短暂传输延迟，但拒绝持续未到达物理层的直接油门。

        Args:
            now: 当前单调墙钟时间，秒。
        """
        expected = self.direct_request if now - self.direct_request_wall < 1 else 0.0
        mismatch = (
            self.command_mode == "direct"
            and abs(self.physics_throttle - expected) > 0.02
        )
        if not mismatch:
            self.command_mismatch_since = None
        elif self.command_mismatch_since is None:
            self.command_mismatch_since = now
        return (
            self.command_mismatch_since is None or now - self.command_mismatch_since < 1
        )

    def publish_health(self):
        """限频发布就绪判据、当前控制模式和通信重试次数。"""
        now = time.monotonic()
        if now - self.health_wall < 0.2:
            return
        self.health_wall = now
        checks = {
            "control_enabled": self.control_enabled,
            "energy_available": self.powered(),
            "actuator_feedback": now - self.actuator_wall < 1,
            "physics_feedback": now - self.physics_wall < 1,
            "navigation_fresh": self.navigation_fresh(),
            "command_connected": self.command_connected(),
            "direct_command_confirmed": self.direct_command_confirmed(now),
        }
        data = {
            "gz_partition": os.environ.get("GZ_PARTITION", "default"),
            "sim_s": self.sim,
            "ready": all(checks.values()),
            "checks": checks,
            "missing": [k for k, v in checks.items() if not v],
            "command_sequence": self.command_sequence,
            "command_mode": self.command_mode,
            "transport_retries": self.transport_retries,
            "fixed_throttle_request": (
                self.fixed_throttle if self.command_mode == "fixed_heading" else None
            ),
            "heading_reference_rad": self.hold_yaw,
        }
        self.health.publish(String(data=json.dumps(data)))

    def powered(self):
        """检查能源反馈新鲜度及未满足功率是否处于允许范围。"""
        return (
            self.energy is not None
            and time.monotonic() - self.energy_wall < 2
            and self.energy[23] < 1e-3
        )

    def navigation_fresh(self):
        """检查导航消息的墙钟新鲜度与仿真时间一致性。"""
        return (
            self.yaw is not None
            and self.velocity is not None
            and time.monotonic() - min(self.imu_wall, self.gps_wall) < 1
            and -0.2 <= self.sim - self.imu_stamp < 0.5
            and -0.2 <= self.sim - self.gps_stamp < 0.5
        )

    def payload(self, name, msg, pub):
        """在供电与时间条件满足时转发载荷消息并更新导航状态。

        Args:
            name: 载荷或传感器的逻辑名称。
            msg: 待转发的 ROS 消息，会更新其坐标系或相机投影字段。
            pub: 对应的 ROS 发布器。
        """
        if not self.powered():
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if name in ["imu", "gps_velocity"] and not -0.2 <= self.sim - stamp < 0.5:
            return
        if name == "imu":
            q = msg.orientation
            self.yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            )
            self.rate = msg.angular_velocity.z
            self.imu_wall = time.monotonic()
            self.imu_stamp = stamp
        elif name == "gps_velocity":
            self.velocity = (msg.vector.x, msg.vector.y)
            self.gps_wall = time.monotonic()
            self.gps_stamp = stamp
        if hasattr(msg, "header"):
            frame = (
                (name.removesuffix("_info") + "_optical")
                if name.startswith("camera")
                else ("lidar" if name == "lidar_points" else name)
            )
            msg.header.frame_id = self.boat + "/" + frame
            if name == "gps_velocity":
                msg.header.frame_id = "world"
        if name == "camera_right_info":
            msg.p[3] = -msg.k[0] * self.payload_cfg["camera"]["stereo_baseline_m"]
        pub.publish(msg)

    def cmd_vel(self, msg):
        """接收速度请求；就绪不足时清除目标并停推。

        Args:
            msg: Twist 消息，线速度 x 为 m/s、角速度 z 为 rad/s。
        """
        if not self.control_enabled:
            return
        if not all(map(math.isfinite, [msg.linear.x, msg.angular.z])):
            return
        if not self.navigation_fresh() or not self.actuator_ready():
            self.desired = None
            self.hold_yaw = None
            self.integral = 0.0
            self.throttle.publish(Double(data=0.0))
            return
        first = self.desired is None or time.monotonic() - self.command_wall > 1
        self.desired = (
            max(-4.5, min(4.5, msg.linear.x)),
            max(-0.5, min(0.5, msg.angular.z)),
        )
        if first:
            self.hold_yaw = self.yaw
            self.integral = 0.0
        self.command_wall = time.monotonic()
        self.command_sequence += 1
        self.command_mode = "velocity"
        self.fixed_throttle = None

    def fixed_heading_throttle(self, msg):
        """接收保持当前航向的固定油门请求。

        Args:
            msg: Float64 消息，data 为归一化有符号油门。
        """
        if not self.control_enabled or not math.isfinite(msg.data):
            return
        if not self.navigation_fresh() or not self.actuator_ready():
            self.desired = None
            self.throttle.publish(Double(data=0.0))
            return
        if (
            self.desired is None
            or self.command_mode != "fixed_heading"
            or time.monotonic() - self.command_wall > 1
        ):
            self.hold_yaw = self.yaw
            self.integral = 0.0
        self.fixed_throttle = max(-1.0, min(1.0, msg.data))
        self.desired = (
            math.copysign(1.0, msg.data) if abs(msg.data) > 0.05 else 0.0,
            0.0,
        )
        self.command_wall = time.monotonic()
        self.command_sequence += 1
        self.command_mode = "fixed_heading"

    def reconnect_stale_transport(self):
        """对长期未更新的 Gazebo 状态主题重新订阅。"""
        now = time.monotonic()
        if now - self.transport_retry_wall < 3:
            return
        self.transport_retry_wall = now
        if self.control_enabled and not self.command_connected():
            # Recreate only command publishers; sensor subscriptions remain intact.
            self.throttle = None
            self.steering = None
            self.command_transport = GzNode()
            self.throttle = self.command_transport.advertise(
                self.prefix + "/propulsion/throttle", Double
            )
            self.steering = self.command_transport.advertise(
                self.prefix + "/steering/angle", Double
            )
            self.transport_retries += 1
        for topic, kind, callback, stamp in [
            ("/energy/truth", Double_V, self.on_energy, self.energy_wall),
            ("/actuators/state", Double_V, self.on_actuators, self.actuator_wall),
            ("/propulsion/state", Double_V, self.on_physics, self.physics_wall),
        ]:
            if now - stamp > 3:
                self.gz.unsubscribe(self.prefix + topic)
                self.gz.subscribe(kind, self.prefix + topic, callback)
                self.transport_retries += 1

    def direct_throttle(self, msg):
        """接收直接油门请求并切换控制模式，不就绪时停推。

        Args:
            msg: Float64 消息，data 为归一化有符号油门。
        """
        if not self.control_enabled:
            return
        if not math.isfinite(msg.data):
            return
        self.desired = None
        self.integral = 0.0
        self.command_sequence += 1
        self.command_mode = "direct"
        self.fixed_throttle = None
        self.direct_request = (
            max(-1.0, min(1.0, msg.data)) if self.actuator_ready() else 0.0
        )
        self.direct_request_wall = time.monotonic()
        self.throttle.publish(Double(data=self.direct_request))

    def direct_steering(self, msg):
        """在执行器就绪时发送限幅后的直接转向角。

        Args:
            msg: Float64 消息，data 为转向角，单位 rad。
        """
        if not self.control_enabled:
            return
        if not math.isfinite(msg.data):
            return
        self.desired = None
        if self.actuator_ready():
            self.steering.publish(
                Double(data=max(-math.radians(35), min(math.radians(35), msg.data)))
            )

    def tick(self):
        """执行看门狗、健康发布与当前控制模式的周期更新。"""
        self.reconnect_stale_transport()
        self.publish_health()
        dt = 0 if self.last_sim is None else self.sim - self.last_sim
        self.last_sim = self.sim
        if dt < 0:
            self.desired = None
            self.integral = 0.0
            self.hold_yaw = None
        if self.desired is None:
            return
        if dt > 0.5:
            # A clock/transport gap invalidates the old heading reference.
            self.hold_yaw = None
            self.integral = 0.0
        fresh = self.navigation_fresh()
        if (
            not self.actuator_ready()
            or not fresh
            or time.monotonic() - self.command_wall > 1
        ):
            self.throttle.publish(Double(data=0.0))
            self.desired = None
            self.hold_yaw = None
            self.integral = 0.0
            return
        if dt <= 0:
            return
        dt = min(dt, 0.2)
        speed, turn = self.desired
        if abs(speed) < 0.02:
            self.throttle.publish(Double(data=0.0))
            self.steering.publish(Double(data=0.0))
            self.integral = 0.0
            self.hold_yaw = self.yaw
            return
        actual = self.velocity[0] * math.cos(self.yaw) + self.velocity[1] * math.sin(
            self.yaw
        )
        if self.command_mode == "fixed_heading":
            # Steering closes the heading loop; shaft-speed request stays fixed.
            throttle = self.fixed_throttle
            raw = abs(throttle)
        else:
            error = abs(speed) - math.copysign(1, speed) * actual
            candidate = max(-0.25, min(0.35, self.integral + 0.08 * error * dt))
            raw = 0.05 + 0.95 * abs(speed) / self.speed_scale + 0.35 * error + candidate
            if 0 < raw < 1 or (raw >= 1 and error < 0) or (raw <= 0 and error > 0):
                self.integral = candidate
            throttle = math.copysign(max(0.0, min(1.0, raw)), speed)
        if abs(turn) < 1e-5:
            if self.hold_yaw is None:
                self.hold_yaw = self.yaw
            heading_error = wrap(self.hold_yaw - self.yaw)
            requested = max(-0.3, min(0.3, 0.8 * heading_error))
        else:
            heading_error = 0.0
            requested = turn
            self.hold_yaw = self.yaw
        angle = -math.copysign(1, speed) * (1.5 * (requested - self.rate))
        angle = max(-math.radians(35), min(math.radians(35), angle))
        self.throttle.publish(Double(data=throttle))
        self.steering.publish(Double(data=angle))
        if self.command_mode == "velocity":
            self.status.publish(
                Float64MultiArray(
                    data=[
                        self.sim,
                        speed,
                        actual,
                        heading_error,
                        self.rate,
                        throttle,
                        angle,
                        float(raw >= 1),
                    ]
                )
            )

    def close(self):
        """停止控制输出并释放控制权及节点资源。"""
        if self.control_enabled:
            self.throttle.publish(Double(data=0.0))
        if self.lease:
            self.lease.close()
        self.gz.unsubscribe(self.prefix + "/sensors/gps")
        self.gz.unsubscribe(self.prefix + "/energy/truth")
        self.gz.unsubscribe(self.prefix + "/actuators/state")
        self.gz.unsubscribe(self.prefix + "/propulsion/state")


def main():
    """启动 ROS 控制节点，并在退出时清理控制资源。"""
    rclpy.init()
    node = BoatControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # launch may signal both the process group and the child during cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
