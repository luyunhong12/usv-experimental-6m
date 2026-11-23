"""组织 ROS 与 Gazebo 之间的传感器和控制接口启动。

WAM-V-style payload names; physical single-azimuth actuation is retained.
"""

import json
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    EmitEvent,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    """按启动上下文建立桥接与控制节点。

    Args:
        context: ROS launch 参数求值上下文
    """
    name = LaunchConfiguration("name").perform(context)
    world = LaunchConfiguration("world").perform(context)
    clock = LaunchConfiguration("clock").perform(context).lower() == "true"
    if not name.isidentifier():
        raise ValueError("Invalid boat name")
    prefix = "/" + name
    types = [
        ("imu", "sensor_msgs/msg/Imu", "gz.msgs.IMU"),
        ("gps", "sensor_msgs/msg/NavSatFix", "gz.msgs.NavSat"),
        ("magnetometer", "sensor_msgs/msg/MagneticField", "gz.msgs.Magnetometer"),
        ("camera", "sensor_msgs/msg/Image", "gz.msgs.Image"),
        ("camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
        ("lidar", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
    ]
    cfg = json.loads(
        (
            Path(get_package_share_directory("usv_description"))
            / "config/payloads.json"
        ).read_text()
    )
    if cfg["advanced_sensors"]:
        for n in ["camera_right", "camera_side"]:
            types.extend(
                [
                    (n, "sensor_msgs/msg/Image", "gz.msgs.Image"),
                    (n + "_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
                ]
            )
    args = []
    remaps = []
    for sensor, ros, gz in types:
        topic = prefix + "/sensors/" + sensor
        args.append(topic + "@" + ros + "[" + gz)
        remaps.append((topic, prefix + "/raw/" + sensor))
    if cfg["advanced_sensors"]:
        topic = prefix + "/sensors/lidar3d/points"
        args.append(topic + "@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked")
        remaps.append((topic, prefix + "/raw/lidar_points"))
    args += [
        prefix + "/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        prefix + "/ground_truth/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        prefix + "/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
    ]
    remaps += [
        (prefix + "/ground_truth/tf", "/tf"),
        (prefix + "/joint_state", prefix + "/joint_states"),
    ]
    for collision in [
        "sealed_hull",
        "hull_detailed_external",
        "azimuth_detailed_external",
        "propeller_detailed_external",
    ]:
        args.append(
            prefix
            + "/contacts/"
            + collision
            + "@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
        )
    args += [
        "/vrx/task/info@ros_gz_interfaces/msg/ParamVec[gz.msgs.Param",
        "/vrx/stationkeeping/pose_error@std_msgs/msg/Float32[gz.msgs.Float",
        "/vrx/stationkeeping/goal@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
    ]
    args += [
        "/vrx/wayfinding/waypoints@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
        "/vrx/wayfinding/mean_error@std_msgs/msg/Float32[gz.msgs.Float",
    ]
    if clock:
        args.append("/world/" + world + "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock")
        remaps.append(("/world/" + world + "/clock", "/clock"))
    # 只作用于本次桥接/控制子进程；保留调用方显式指定的 Fast DDS 配置。
    dds_environment = {
        "FASTRTPS_DEFAULT_PROFILES_FILE": os.environ.get(
            "FASTRTPS_DEFAULT_PROFILES_FILE",
            str(
                Path(get_package_share_directory("usv_control"))
                / "config/dds_transport.xml"
            ),
        )
    }
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=name + "_bridge",
        additional_env=dds_environment,
        arguments=args,
        remappings=remaps,
        output="screen",
    )
    controller = Node(
        package="usv_control",
        executable="boat_control.py",
        name=name + "_controller",
        additional_env=dds_environment,
        parameters=[
            {
                "boat_name": name,
                "use_sim_time": True,
                "control_enabled": LaunchConfiguration("control")
                .perform(context)
                .lower()
                == "true",
                "speed_feedforward_m_s": float(
                    LaunchConfiguration("speed_scale").perform(context)
                ),
            }
        ],
        output="screen",
    )
    return [
        bridge,
        controller,
        *[
            RegisterEventHandler(
                OnProcessExit(
                    target_action=child,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="Required USV interface process exited"
                            )
                        )
                    ],
                )
            )
            for child in [bridge, controller]
        ],
    ]


def generate_launch_description():
    """声明平台接口启动参数及配置入口。"""
    return LaunchDescription(
        [
            DeclareLaunchArgument("name", default_value="usv"),
            DeclareLaunchArgument("world", default_value="usv_sydney"),
            DeclareLaunchArgument("clock", default_value="true"),
            DeclareLaunchArgument("control", default_value="true"),
            DeclareLaunchArgument("speed_scale", default_value="1.63"),
            OpaqueFunction(function=setup),
        ]
    )
