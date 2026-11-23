"""观测完整场景的 ROS 渲染载荷与进程资源，不保存逐帧大图或改变传感器。"""

import csv
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np
import psutil
from PIL import Image as PillowImage


def point_coordinates(message):
    """按 PointCloud2 的字段偏移、大小端和行步幅读取 XYZ。

    Args:
        message: ROS PointCloud2 消息，XYZ 必须是单个 float32 字段。

    Returns:
        N×3 浮点数组；无回波的非有限点保留，不能假装有效回波。

    Raises:
        ValueError: 数据长度、字段类型或布局不合法。
    """
    fields = {f.name: f for f in message.fields}
    if len(message.data) != message.height * message.row_step:
        raise ValueError("PointCloud2 byte count differs from row layout")
    if message.row_step < message.width * message.point_step:
        raise ValueError("PointCloud2 row stride is too small")
    arrays = []
    for name in ["x", "y", "z"]:
        field = fields.get(name)
        if (
            field is None
            or field.datatype != 7
            or field.count != 1
            or field.offset + 4 > message.point_step
        ):
            raise ValueError("PointCloud2 requires scalar float32 XYZ")
        arrays.append(
            np.ndarray(
                (message.height, message.width),
                dtype=">f4" if message.is_bigendian else "<f4",
                buffer=message.data,
                offset=field.offset,
                strides=(message.row_step, message.point_step),
            ).ravel()
        )
    return np.column_stack(arrays)


def stream_summary(rows, start, end, hz, limits):
    """从仿真时间窗口判断载荷连续性，包含首尾缺帧而不只算相邻频率。

    Args:
        rows: stamp/wall/latency/valid/digest/content 字典列表。
        start: 计时工况起始仿真秒。
        end: 完整工况结束仿真秒。
        hz: 传感器配置频率，单位 Hz（仿真时间）。
        limits: 预声明门槛，含 warmup_s、minimum_delivery_ratio、max_gap_sim_s。

    Returns:
        统计值、独立检查和通过状态；缺数据不能通过。
    """
    begin = start + limits["warmup_s"]
    selected = [r for r in rows if begin <= r["stamp"] <= end]
    if end <= begin or len(selected) < 2:
        return {"passed": False, "failure": "Insufficient measured sensor interval"}
    stamps = np.array([r["stamp"] for r in selected])
    gaps = np.diff(np.r_[begin, stamps, end])
    ratio = len(selected) / ((end - begin) * hz)
    latency = np.array([r["latency"] for r in selected])
    checks = {
        "monotonic_stamps": bool(np.all(np.diff(stamps) > 0)),
        "delivery_ratio": bool(ratio >= limits["minimum_delivery_ratio"]),
        "no_long_gap": bool(gaps.max() <= limits["max_gap_sim_s"]),
        "valid_payloads": all(r["valid"] for r in selected),
        "latency": bool(latency.max() <= limits["max_latency_sim_s"]),
        "not_future": bool(latency.min() >= -0.25),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "frames": len(selected),
        "delivered_sim_hz": len(selected) / (end - begin),
        "delivered_wall_hz": (len(selected) - 1)
        / (selected[-1]["wall"] - selected[0]["wall"]),
        "delivery_ratio": ratio,
        "maximum_gap_sim_s": float(gaps.max()),
        "latency_sim_s_min_p95_max": np.percentile(latency, [0, 95, 100]).tolist(),
        "unique_payloads": len({r["digest"] for r in selected}),
        "content_min_median_max": np.percentile(
            [r["content"] for r in selected], [0, 50, 100]
        ).tolist(),
    }


class RenderingAudit:
    """由测试器的单线程 ROS executor 调用，持有订阅及轻量原始观测记录。"""

    def __init__(self, ros, output, config, limits):
        """订阅公开载荷接口；大型样本每分钟最多保存一次。

        Args:
            ros: 调用方拥有的 rclpy 节点。
            output: 本次独立试验输出目录。
            config: payloads.json 的载荷配置。
            limits: 预声明渲染验收门槛。
        """
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2

        if (
            not all(np.isfinite(value) for value in limits.values())
            or limits["warmup_s"] < 0
            or not 0 < limits["minimum_delivery_ratio"] <= 1
            or not 0 < limits["minimum_mean_rtf"] <= 1
            or any(
                limits[key] <= 0
                for key in [
                    "max_gap_sim_s",
                    "max_latency_sim_s",
                    "maximum_rss_mib",
                    "maximum_rss_growth_mib",
                ]
            )
        ):
            raise ValueError("Invalid declared rendering acceptance limits")
        self.output = Path(output)
        self.config = config
        self.limits = limits
        self.sim = 0.0
        self.begin = time.monotonic()
        self.last_resource = self.begin
        self.last_sample = {}
        self.rows = {}
        self.errors = []
        self.resources = []
        self.processes = {}
        self.subscriptions = []
        self.topics = {}
        for name in ["camera", "camera_right", "camera_side"]:
            self.topics[name] = (Image, f"/usv/sensors/cameras/{name}/image_raw")
            self.topics[name + "_info"] = (
                CameraInfo,
                f"/usv/sensors/cameras/{name}/camera_info",
            )
        self.topics.update(
            lidar=(LaserScan, "/usv/sensors/lidars/lidar/scan"),
            points=(PointCloud2, "/usv/sensors/lidars/lidar/points"),
        )
        for name, (kind, topic) in self.topics.items():
            self.rows[name] = []

            def receive(message, stream=name):
                """记录当前绑定流的一条消息。

                Args:
                    message: 载荷消息。
                    stream: 回调创建时绑定的流名。
                """
                self.record(stream, message)

            self.subscriptions.append(
                ros.create_subscription(kind, topic, receive, qos_profile_sensor_data)
            )

    def observe_source(self, transport, ros):
        """比较 Gazebo 原始图像、ROS 桥接原始图像和公开输出的帧损失位置。

        Args:
            transport: 调用方拥有的 Gazebo Transport 节点。
            ros: 调用方拥有的 ROS 节点。
        """
        from gz.msgs10.image_pb2 import Image as GzImage
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image

        for camera in ["camera", "camera_right", "camera_side"]:
            for prefix in ["native_", "raw_"]:
                self.rows[prefix + camera] = []

            def native(message, name="native_" + camera):
                """记录源帧时间而不重复拷贝图像。

                Args:
                    message: Gazebo Image 消息。
                    name: 绑定的源流名称。
                """
                stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
                self.source_frame(name, stamp)

            def raw(message, name="raw_" + camera):
                """记录桥接原始图像时间。

                Args:
                    message: ROS Image 消息。
                    name: 绑定的原始流名称。
                """
                stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                self.source_frame(name, stamp)

            transport.subscribe(GzImage, "/usv/sensors/" + camera, native)
            self.subscriptions.append(
                ros.create_subscription(
                    Image, "/usv/raw/" + camera, raw, qos_profile_sensor_data
                )
            )

    def source_frame(self, name, stamp):
        """记录链路对照的时间元数据，不当作图像内容验收。

        Args:
            name: 已声明的链路对照流。
            stamp: 源消息仿真秒。
        """
        self.rows[name].append(
            dict(
                stamp=stamp,
                wall=time.monotonic() - self.begin,
                latency=self.sim - stamp,
                valid=True,
                content=0,
                digest="timing-only",
            )
        )

    def record(self, name, message):
        """检查消息与时间，错误保留为失败而不从统计中删除。

        Args:
            name: 已声明的流名称。
            message: 对应 ROS 消息。
        """
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        valid, content, digest = False, 0.0, ""
        try:
            valid, content, digest = self.inspect(name, message, stamp)
        except (ValueError, TypeError, BufferError, IndexError) as exc:
            if len(self.errors) < 20:
                self.errors.append({"stream": name, "stamp": stamp, "error": str(exc)})
        self.rows[name].append(
            dict(
                stamp=stamp,
                wall=time.monotonic() - self.begin,
                latency=self.sim - stamp,
                valid=bool(valid),
                content=float(content),
                digest=digest,
            )
        )

    def inspect(self, name, msg, stamp):
        """核验实际图像、标定和雷达数据，允许无回波点为非有限数。

        Args:
            name: 流名称。
            msg: ROS 载荷消息。
            stamp: 消息仿真秒，用于限制样本写盘频率。

        Returns:
            合法性、内容指标和有效载荷摘要。
        """
        camera = self.config["camera"]
        lidar = self.config["lidar"]
        save = stamp - self.last_sample.get(name, -100) >= 60
        sample = self.output / "sensor_samples"
        sample.mkdir(exist_ok=True)
        if name.endswith("_info"):
            expected_tx = (
                -msg.k[0] * camera["stereo_baseline_m"]
                if name == "camera_right_info"
                else 0
            )
            valid = (
                msg.width == camera["width"]
                and msg.height == camera["height"]
                and np.isfinite([*msg.k, *msg.p]).all()
                and msg.k[0] > 0
                and abs(msg.p[3] - expected_tx) < 1e-6
                and msg.header.frame_id
                == "usv/" + name.removesuffix("_info") + "_optical"
            )
            return valid, msg.k[0], "calibration"
        if name.startswith("camera"):
            if msg.encoding != "rgb8" or msg.step < msg.width * 3:
                raise ValueError("Unexpected RGB image encoding or row stride")
            pixels = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.step
            )
            rgb = pixels[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
            valid = (
                msg.width == camera["width"]
                and msg.height == camera["height"]
                and msg.header.frame_id == "usv/" + name + "_optical"
                and rgb.std() > 1
            )
            if save:
                PillowImage.fromarray(rgb).save(sample / f"{name}_{stamp:.1f}.png")
                self.last_sample[name] = stamp
            return valid, rgb.std(), hashlib.sha256(msg.data).hexdigest()
        if name == "points":
            xyz = point_coordinates(msg)
            finite = xyz[np.isfinite(xyz).all(axis=1)]
            ranges = np.linalg.norm(finite, axis=1)
            valid = (
                len(xyz) == lidar["horizontal_samples"] * lidar["vertical_samples"]
                and msg.header.frame_id == "usv/lidar"
                and len(finite) > 0
                and ranges.min() >= lidar["near_m"] - 0.1
                and ranges.max() <= lidar["far_m"] + 0.1
            )
            if save:
                np.savez_compressed(sample / f"points_{stamp:.1f}.npz", xyz=xyz)
                self.last_sample[name] = stamp
            return valid, len(finite) / len(xyz), hashlib.sha256(msg.data).hexdigest()
        ranges = np.asarray(msg.ranges)
        finite = ranges[np.isfinite(ranges)]
        valid = (
            len(ranges) == lidar["horizontal_samples"]
            and msg.header.frame_id == "usv/lidar"
            and np.all(finite >= lidar["near_m"] - 0.1)
            and np.all(finite <= lidar["far_m"] + 0.1)
        )
        return (
            valid,
            len(finite) / len(ranges),
            hashlib.sha256(ranges.tobytes()).hexdigest(),
        )

    def ready(self):
        """所有渲染接口已产生合法且新鲜消息时才允许开始计时。"""
        return all(
            v and v[-1]["valid"] and abs(self.sim - v[-1]["stamp"]) < 1
            for v in self.rows.values()
        )

    def sample_resources(self, sim, roots):
        """每墙钟秒读取自有进程树与整卡 GPU 状态，不把整卡用量冒充进程独占。

        Args:
            sim: 当前物理遥测中的仿真秒。
            roots: 本测试创建的 Popen 根进程列表。
        """
        self.sim = sim
        now = time.monotonic()
        if now - self.last_resource < 1:
            return
        self.last_resource = now
        owned = {psutil.Process().pid: psutil.Process()}
        for root in roots:
            try:
                process = psutil.Process(root.pid)
                owned.update(
                    {p.pid: p for p in [process, *process.children(recursive=True)]}
                )
            except psutil.NoSuchProcess:
                continue
        rss = cpu = 0.0
        for pid, process in owned.items():
            try:
                old = self.processes.setdefault(pid, process)
                rss += old.memory_info().rss / 2**20
                cpu += old.cpu_percent()
            except psutil.NoSuchProcess:
                continue
        gpu = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=utilization.gpu,memory.used,memory.total",
                "--format=csv,noheader,nounits",
            ],
            capture_output=True,
            text=True,
            timeout=3,
        )
        values = (
            [float(v) for v in gpu.stdout.splitlines()[0].split(",")]
            if gpu.returncode == 0
            else [float("nan")] * 3
        )
        self.resources.append([now - self.begin, sim, rss, cpu, *values])

    def finish(self, start, end):
        """保存观测并按事前门槛判定连续性、资源增长和实时率。

        Args:
            start: 正式工况起始仿真秒；启动失败时为 None。
            end: 最后完成阶段的结束仿真秒；未完成时为当前仿真秒。

        Returns:
            渲染与性能独立检查结果。
        """
        result = {
            "limits": self.limits,
            "errors": self.errors,
            "streams": {},
            "checks": {},
        }
        for name, rows in self.rows.items():
            with (self.output / ("sensor_" + name + ".csv")).open("w") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=[
                        "stamp",
                        "wall",
                        "latency",
                        "valid",
                        "content",
                        "digest",
                    ],
                )
                writer.writeheader()
                writer.writerows(rows)
            hz = self.config["camera" if name.startswith("camera") else "lidar"]["hz"]
            stat = (
                stream_summary(rows, start, end, hz, self.limits)
                if start is not None
                else {"passed": False}
            )
            if (
                name in ["camera", "camera_right", "camera_side", "points"]
                and "checks" in stat
            ):
                stat["checks"]["content_changes"] = stat["unique_payloads"] > 2
                stat["passed"] = all(stat["checks"].values())
            result["streams"][name] = stat
            result["checks"][name] = stat["passed"]
        data = np.array(self.resources)
        np.savetxt(
            self.output / "rendering_resources.csv",
            data,
            delimiter=",",
            header="wall_s,sim_s,process_tree_rss_mib,cpu_percent_of_one_core,gpu_percent,device_vram_mib,device_vram_total_mib",
        )
        selected = (
            data[data[:, 1] >= start + self.limits["warmup_s"]]
            if start is not None and len(data)
            else np.empty((0, 7))
        )
        if len(selected) > 10:
            rtf = np.diff(selected[:, 1]) / np.diff(selected[:, 0])
            mean_rtf = (selected[-1, 1] - selected[0, 1]) / (
                selected[-1, 0] - selected[0, 0]
            )
            growth = float(np.median(selected[-10:, 2]) - np.median(selected[:10, 2]))
            result["performance"] = {
                "rtf_mean": float(mean_rtf),
                "rtf_p10_p50_p90": np.percentile(rtf, [10, 50, 90]).tolist(),
                "rss_growth_mib": growth,
                "rss_peak_mib": float(selected[:, 2].max()),
                "cpu_mean_percent_of_one_core": float(selected[:, 3].mean()),
                "gpu_mean_percent": float(selected[:, 4].mean()),
                "device_vram_peak_mib": float(selected[:, 5].max()),
            }
            result["checks"].update(
                resource_samples_finite=bool(np.isfinite(selected).all()),
                usable_rtf=bool(mean_rtf >= self.limits["minimum_mean_rtf"]),
                bounded_rss=bool(
                    selected[:, 2].max() <= self.limits["maximum_rss_mib"]
                ),
                bounded_growth=growth <= self.limits["maximum_rss_growth_mib"],
                vram_headroom=bool(np.all(selected[:, 5] < 0.95 * selected[:, 6])),
            )
        else:
            result["checks"]["resource_samples"] = False
        result["passed"] = all(result["checks"].values())
        (self.output / "rendering_result.json").write_text(
            json.dumps(result, indent=2) + "\n"
        )
        return result
