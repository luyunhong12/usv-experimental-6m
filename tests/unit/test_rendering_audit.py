"""用独立字节布局和断流反例检查渲染验收，避免仅验证有消息。"""

from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/lib"))
from rendering_audit import point_coordinates, stream_summary


class RenderingAuditTests(unittest.TestCase):
    """检查带行填充的大小端点云，以及窗口两端断流和重复时间戳。"""

    def test_independent_camera_has_no_stereo_translation(self):
        """侧相机不能继承 Gazebo 的伪基线，右相机保留已知 0.2 m 基线。"""
        import importlib.util
        from sensor_msgs.msg import CameraInfo

        source = (
            Path(__file__).resolve().parents[2]
            / "ros_ws/src/usv_control/scripts/boat_control.py"
        )
        spec = importlib.util.spec_from_file_location("tested_boat_control", source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        control = SimpleNamespace(
            powered=lambda: True,
            boat="usv",
            payload_cfg={"camera": {"stereo_baseline_m": 0.2}},
        )
        received = []
        publisher = SimpleNamespace(publish=received.append)
        for name, expected in [
            ("camera_info", 0),
            ("camera_side_info", 0),
            ("camera_right_info", -100),
        ]:
            message = CameraInfo()
            message.k[0] = 500.0
            message.p[3] = -375.0
            module.BoatControl.payload(control, name, message, publisher)
            self.assertEqual(received[-1].p[3], expected)
            self.assertEqual(
                received[-1].header.frame_id,
                "usv/" + name.removesuffix("_info") + "_optical",
            )

    def test_padded_big_endian_points(self):
        """两行点云带填充且字段顺序不同，按明确 XYZ 预期读取。"""
        fields = [
            SimpleNamespace(name=n, offset=o, datatype=7, count=1)
            for n, o in [("x", 8), ("y", 0), ("z", 4)]
        ]
        message = SimpleNamespace(
            width=1,
            height=2,
            point_step=16,
            row_step=24,
            is_bigendian=True,
            fields=fields,
            data=struct.pack(">fff", 2, 3, 1)
            + bytes(12)
            + struct.pack(">fff", 5, 6, 4)
            + bytes(12),
        )
        np.testing.assert_array_equal(
            point_coordinates(message), [[1, 2, 3], [4, 5, 6]]
        )
        message.data = message.data[:-1]
        with self.assertRaises(ValueError):
            point_coordinates(message)

    def test_stream_rejects_missing_tail_and_duplicate_time(self):
        """正常 5 Hz 流通过；尾部无数据和重复时间不能被平均频率隐藏。"""
        limits = {
            "warmup_s": 0,
            "minimum_delivery_ratio": 0.95,
            "max_gap_sim_s": 1,
            "max_latency_sim_s": 1,
        }
        rows = [
            dict(
                stamp=i / 5,
                wall=i / 4,
                latency=0.1,
                valid=True,
                digest=str(i),
                content=20,
            )
            for i in range(50)
        ]
        self.assertTrue(stream_summary(rows, 0, 10, 5, limits)["passed"])
        self.assertFalse(stream_summary(rows[:-10], 0, 10, 5, limits)["passed"])
        rows[10]["stamp"] = rows[9]["stamp"]
        self.assertFalse(
            stream_summary(rows, 0, 10, 5, limits)["checks"]["monotonic_stamps"]
        )

    def test_invalid_content_and_stale_frames_fail(self):
        """无效内容和旧帧必须失败，即使消息数量足够。"""
        limits = {
            "warmup_s": 0,
            "minimum_delivery_ratio": 0.95,
            "max_gap_sim_s": 1,
            "max_latency_sim_s": 1,
        }
        rows = [
            dict(
                stamp=i / 5,
                wall=i / 5,
                latency=2,
                valid=False,
                digest="same",
                content=0,
            )
            for i in range(50)
        ]
        result = stream_summary(rows, 0, 10, 5, limits)
        self.assertFalse(result["checks"]["valid_payloads"])
        self.assertFalse(result["checks"]["latency"])


if __name__ == "__main__":
    unittest.main()
