#!/usr/bin/env python3
"""通过 ROS 控制接口发送有时限的操船请求。

Send manual commands through the running ROS controller, not around it.
"""
import argparse
import json
import math
import os
import time
import rclpy
from std_msgs.msg import Float64, String


def main():
    """解析操船参数并执行带时限的 ROS 操控。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--throttle", type=float, default=0.0)
    parser.add_argument("--steering-deg", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--name", default="usv")
    args = parser.parse_args()
    if not (
        args.name.isidentifier()
        and all(
            math.isfinite(v) for v in (args.throttle, args.steering_deg, args.duration)
        )
        and -1 <= args.throttle <= 1
        and -35 <= args.steering_deg <= 35
        and args.duration > 0
    ):
        parser.error(
            "name must be an identifier; throttle [-1,1], steering [-35,35], duration > 0"
        )
    rclpy.init()
    node = rclpy.create_node("usv_manual_command")
    prefix = "/" + args.name
    throttle = node.create_publisher(Float64, prefix + "/actuators/throttle", 10)
    steering = node.create_publisher(Float64, prefix + "/actuators/steering", 10)
    health = {}

    def update(msg):
        """缓存与当前分区匹配的控制健康状态。

        Args:
            msg: 包含 JSON 健康状态的 ROS 消息
        """
        value = json.loads(msg.data)
        if value.get("gz_partition") == os.environ.get(
            "GZ_PARTITION", "usv_platform15"
        ):
            health.clear()
            health.update(value, received=time.monotonic())

    node.create_subscription(String, prefix + "/control/health", update, 10)
    sent = False
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if (
                health.get("ready")
                and throttle.get_subscription_count()
                and steering.get_subscription_count()
            ):
                break
        else:
            raise RuntimeError(
                "No ready ROS controller in this Gazebo partition; start ./usv run first"
            )
        start = time.monotonic()
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            if (
                not health.get("ready")
                or time.monotonic() - health.get("received", 0) > 1
            ):
                raise RuntimeError("Controller health lost; stopping manual command")
            steering.publish(Float64(data=math.radians(args.steering_deg)))
            throttle.publish(Float64(data=float(args.throttle)))
            sent = True
    except KeyboardInterrupt:
        pass
    finally:
        if sent and rclpy.ok():
            for _ in range(3):
                throttle.publish(Float64(data=0.0))
                steering.publish(Float64(data=0.0))
                time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
