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

    def test_force_audit_requires_fin_and_respects_stage(self):
        """新鳍遥测缺失必须失败，短阶段不得混入前一阶段力矩。"""
        import json
        import tempfile

        module_spec = importlib.util.spec_from_file_location(
            "force_audit", ROOT / "tools/analysis/analyze_force_audit.py"
        )
        audit = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(audit)
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            (folder / "boat.sdf").write_text(
                '<sdf><model><plugin name="usv::PassiveStabilizer"/></model></sdf>'
            )
            (folder / "stages.json").write_text(
                json.dumps(
                    [dict(start_sim=0, end_sim=18), dict(start_sim=18, end_sim=20)]
                )
            )
            trace = np.zeros((201, 17))
            trace[:, 0] = np.linspace(0, 20, 201)
            trace[:, 14] = np.maximum(trace[:, 0] - 18, 0)
            np.savetxt(folder / "propulsion_state.csv", trace, delimiter=",")
            for name, columns in [("audit_hull", 28), ("audit_propulsion", 16)]:
                values = np.zeros((201, columns))
                values[:, 0] = trace[:, 0]
                np.savetxt(folder / (name + ".csv"), values, delimiter=",")
            with self.assertRaises(FileNotFoundError):
                audit.analyze(folder, -1)
            fin = np.zeros((201, 10))
            fin[:, 0] = trace[:, 0]
            fin[:, 9] = np.where(trace[:, 0] >= 18, 7, -100)
            np.savetxt(folder / "stabilizer_audit_appendages.csv", fin, delimiter=",")
            result = audit.analyze(folder, -1)
            self.assertAlmostEqual(result["total_external_world_vertical_moment_Nm"], 7)
            self.assertAlmostEqual(result["yaw_rate_deg_s"], np.degrees(1))
            self.assertGreater(result["window_sim_s"][0], 18)

    def test_environment_rejects_missing_motion_and_dry_propeller(self):
        """有波高而无升沉不能通过，推进器浸水不足也必须被识别。"""
        import tempfile

        x = np.zeros((301, 17))
        x[:, 0] = np.linspace(0, 30, len(x))
        x[:, 8] = 1
        wave = np.zeros((301, 7))
        wave[:, 0] = x[:, 0]
        wave[:, 1] = 0.1 * np.sin(x[:, 0])
        criteria = dict(
            environment_max_roll_deg=10,
            environment_max_pitch_deg=10,
            minimum_propeller_wet_fraction=0.8,
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            np.savetxt(path / "wave_response_state.csv", wave, delimiter=",")
            checks, _ = m.environment_quality(path, x, {"wave_gain": 0.3}, criteria)
            self.assertFalse(checks["heave_responds"])
            wave[:, 6] = 0.08 * np.sin(x[:, 0])
            np.savetxt(path / "wave_response_state.csv", wave, delimiter=",")
            x[:, 8] = 0.7
            x[:, 15] = np.radians(12)
            checks, _ = m.environment_quality(path, x, {"wave_gain": 0.3}, criteria)
            self.assertTrue(checks["heave_responds"])
            self.assertFalse(checks["propeller_immersion"])
            self.assertFalse(checks["roll_envelope"])

    def test_reverse_distance_uses_initial_frame(self):
        """旋转坐标系下区分纵向进距、横漂和累计路径。"""
        x = np.zeros((101, 17))
        x[:, 0] = np.linspace(0, 10, len(x))
        x[:, 12] = 3 - 0.2 * x[:, 0]
        x[:, 13] = 4 - x[:, 0]
        x[:, 14] = np.pi / 2
        result = m.reverse_task_metrics(x, [3, 4, np.pi / 2])
        self.assertAlmostEqual(result["along_m"], -10)
        self.assertAlmostEqual(result["max_cross_track_m"], 2)
        stage = {
            "name": "distance_task",
            "stage_origin": [3, 4, np.pi / 2],
            "until_along_m": -30,
            "distance_triggered": True,
        }
        checks, _ = m.reverse_task_checks(
            [stage],
            [x],
            {"reverse_corridors": {"30": {"cross_track_m": 1, "heading_deg": 10}}},
        )
        self.assertFalse(checks["distance_reached"])
        self.assertFalse(checks["cross_track"])

    def test_reverse_recovery_retains_equilibrium_yaw(self):
        """恢复到非零配平偏航不等于回到原航向。"""
        x = np.zeros((501, 17))
        t = np.linspace(0, 50, len(x))
        x[:, 0] = t
        x[:, 14] = np.radians(0.4 * t + 2 * (1 - np.exp(-t)))
        result = m.recovery_metrics(x, 0.05, 0.4)
        self.assertLess(result["settling_time_s"], 5)
        self.assertGreater(result["extra_heading_deg"], 20)
        self.assertIsNone(m.recovery_metrics(x, 0.05)["settling_time_s"])

    def test_short_pulse_excludes_previous_stage(self):
        """短脉冲舵角验收不能混入此前的零舵样本。"""
        trace = np.zeros((51, 17))
        trace[:, 0] = np.linspace(0, 5, len(trace))
        trace[:, 1] = -0.4
        actuators = np.zeros((51, 4))
        actuators[:, 0] = trace[:, 0]
        actuators[actuators[:, 0] > 3.5, 1] = np.radians(5)
        stage = {"start_sim": 3, "end_sim": 5, "throttle": -0.4, "steering_deg": 5}
        self.assertTrue(
            m.held_stage_checks(stage, trace, actuators, {"max_servo_error_deg": 0.5})[
                "servo_tracks"
            ]
        )
        actuators[-1, 1] = 0
        self.assertFalse(
            m.held_stage_checks(stage, trace, actuators, {"max_servo_error_deg": 0.5})[
                "servo_tracks"
            ]
        )

    def test_dropout_window_preserves_real_failures(self):
        """结束停推应排除，阶段内失联或漏停推仍必须失败。"""
        from performance_analysis import command_dropout_checks

        trace = np.zeros((512, 17))
        trace[:, 0] = np.arange(len(trace)) / 10
        trace[:, 1] = 0.5
        trace[(trace[:, 0] > 12) & (trace[:, 0] < 25), 1] = 0
        trace[trace[:, 0] >= 50, 1] = 0
        self.assertTrue(all(command_dropout_checks(trace, 0, 50).values()))
        trace[450, 1] = 0
        self.assertFalse(
            command_dropout_checks(trace, 0, 50)["command_resumes_after_gap"]
        )
        trace[200, 1] = 0.5
        self.assertFalse(
            command_dropout_checks(trace, 0, 50)["command_timeout_stops_throttle"]
        )

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
