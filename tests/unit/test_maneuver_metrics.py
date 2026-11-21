"""用解析轨迹检验操纵指标计算。"""

import importlib.util
import unittest
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "maneuver_metrics", ROOT / "tests/regression/assess_maneuvers.py"
)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class ManeuverMetricsTest(unittest.TestCase):
    """验证本模块的解析不变量与失败条件。"""

    def test_circle(self):
        """用解析圆轨迹验证回转指标。"""
        x = np.zeros((1001, 17))
        t = np.linspace(0, 40, 1001)
        w = 0.2
        radius = 5
        x[:, 0] = t
        x[:, 12] = radius * np.sin(w * t)
        x[:, 13] = radius * (1 - np.cos(w * t))
        x[:, 14] = np.arctan2(np.sin(w * t), np.cos(w * t))
        a = m.turning_metrics(x)
        self.assertAlmostEqual(a["radius_m"], radius, places=3)
        self.assertAlmostEqual(a["advance_90_m"], radius, places=3)
        self.assertAlmostEqual(a["tactical_diameter_m"], 2 * radius, places=3)

    def test_coasting_sway_is_not_stop(self):
        """验证仍有横向漂移时不会误报停止。"""
        x = np.zeros((101, 17))
        x[:, 0] = np.linspace(0, 10, 101)
        x[:, 13] = x[:, 0] * 0.2
        self.assertIsNone(m.stop_metrics(x)["stop_time_s"])

    def test_reverse_crossing_reports_sway(self):
        """验证反推过零时保留横向速度。"""
        x = np.zeros((101, 17))
        x[:, 0] = np.linspace(0, 10, 101)
        x[:, 10] = 1 - x[:, 0] / 5
        x[:, 11] = 0.2
        x[:, 12] = x[:, 0] - x[:, 0] ** 2 / 10
        a = m.stop_metrics(x, True)
        self.assertAlmostEqual(a["stop_time_s"], 5)
        self.assertAlmostEqual(a["sway_at_detection_m_s"], 0.2)

    def test_recovery_counts_transient_not_only_tail(self):
        """稳态已恢复也不能忽略此前的大幅超越。"""
        x = np.zeros((401, 17))
        x[:, 0] = np.linspace(0, 40, 401)
        x[:, 14] = np.radians(50 * (1 - np.exp(-x[:, 0])))
        result = m.recovery_metrics(x, 0.172)
        self.assertGreater(result["extra_heading_deg"], 49)
        self.assertLess(result["settling_time_s"], 10)

    def test_duplicate_time_rejected(self):
        """重复采样时间不能进入速度微分或拟合。"""
        x = np.zeros((10, 17))
        self.assertFalse(m.valid_trace(x))

    def test_trigger_requires_measured_heading(self):
        """即使记录触发标记，也必须达到实际航向阈值。"""
        x = np.zeros((10, 17))
        stage = {
            "name": "execute_1",
            "heading_reference_rad": 0,
            "until_heading_deg": 10,
            "heading_triggered": True,
        }
        checks, _ = m.zigzag_checks([stage], [x], {"max_zigzag_overshoot_deg": 25})
        self.assertFalse(checks["execute_1_trigger"])
