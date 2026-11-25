"""确保不完整导航运行、缺失检查和错误结果类型不能伪装通过。"""

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/lib"))
from navigation_acceptance import passed_checks, required_checks


class NavigationAcceptanceTests(unittest.TestCase):
    """验证提前退出和分阶段验收的失败语义。"""

    def test_empty_and_partial_runs_fail(self):
        """空结果、仅连接成功或未完成任务不得通过。"""
        self.assertFalse(passed_checks({}))
        self.assertFalse(passed_checks({"normal_arming": True, "finite": True}))
        self.assertFalse(passed_checks(required_checks()))

    def test_every_required_stage_is_enforced(self):
        """逐项丢失或失败都应使完整运行失败。"""
        checks = dict.fromkeys(required_checks(), True)
        self.assertTrue(passed_checks(checks))
        for name in checks:
            with self.subTest(name=name):
                self.assertFalse(
                    passed_checks({k: v for k, v in checks.items() if k != name})
                )
                self.assertFalse(passed_checks({**checks, name: False}))
                self.assertFalse(passed_checks({**checks, name: "False"}))

    def test_failsafe_cannot_certify_navigation(self):
        """仅失联试验不能充当 AUTO、返航与接管验收。"""
        checks = dict.fromkeys(required_checks(True), True)
        self.assertTrue(passed_checks(checks, True))
        self.assertFalse(passed_checks(checks))


class SustainedStopTests(unittest.TestCase):
    """拒绝单点停推、遥测缺口和中途重新推进。"""

    def test_stop_requires_continuous_window(self):
        """窗口内每个样本均须处于 HOLD 且停推。"""
        from navigation_acceptance import sustained_stop

        samples = [(i / 10, 0.0, 4) for i in range(61)]
        self.assertTrue(sustained_stop(samples))
        self.assertFalse(sustained_stop(samples[-1:]))
        self.assertFalse(sustained_stop(samples[:10] + samples[25:]))
        self.assertFalse(sustained_stop(samples[:30] + [(3.0, 0.2, 4)] + samples[31:]))
        self.assertFalse(sustained_stop([(t, c, 0) for t, c, _ in samples]))


class LoiterTargetTests(unittest.TestCase):
    """防止缺失目标或移动目标伪装保持成功。"""

    def test_target_coverage_and_stationarity(self):
        """不足时长、缺测、漂移目标和非有限数值必须拒绝。"""
        from navigation_acceptance import observed_loiter_target

        samples = [[16 + i / 5, 2.0, -33.7, 150.6, -33.7, 150.6] for i in range(521)]
        self.assertTrue(observed_loiter_target(samples))
        self.assertFalse(observed_loiter_target([]))
        self.assertFalse(observed_loiter_target(samples[:100]))
        self.assertFalse(observed_loiter_target(samples[:100] + samples[120:]))
        drift = [row.copy() for row in samples]
        drift[-1][4] += 0.001
        self.assertFalse(observed_loiter_target(drift))
        drift[-1][4] = float("nan")
        self.assertFalse(observed_loiter_target(drift))
        invalid_time = [row.copy() for row in samples]
        invalid_time[100][0] = float("nan")
        self.assertFalse(observed_loiter_target(invalid_time))
        self.assertFalse(observed_loiter_target(samples + [[121, 0]]))


class SustainedMotionTests(unittest.TestCase):
    """确保失联故障仅在真实持续运动时注入。"""

    def test_motion_requires_speed_thrust_and_coverage(self):
        """拒绝只有指令、短时移动、模式错误和中途缺测。"""
        from navigation_acceptance import sustained_motion

        samples = [[i / 10, 0.6, 0.2, 15] for i in range(31)]
        self.assertTrue(sustained_motion(samples))
        self.assertFalse(sustained_motion(samples[:10]))
        self.assertFalse(sustained_motion(samples[:10] + samples[20:]))
        for column, value in [(1, 0.0), (2, 0.0), (3, 4), (1, float("nan"))]:
            invalid = [row.copy() for row in samples]
            invalid[15][column] = value
            self.assertFalse(sustained_motion(invalid))
