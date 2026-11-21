#!/usr/bin/env python3
"""验证飞控时间插值，测试导入时不执行断言或覆盖历史结果。"""

import copy
import sys
import unittest
from pathlib import Path

import numpy as np

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT / "ros_ws/src/usv_ardupilot/scripts"))


def _load_runtime_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from json_time import StepLimiter

    return (StepLimiter,)


(StepLimiter,) = _load_runtime_dependencies()


class StepLimiterTests(unittest.TestCase):
    """验证步长、姿态归一化与时钟异常处理。"""

    def setUp(self):
        """为每项测试创建独立状态。"""
        self.base = {
            "timestamp": 1.0,
            "position": [0, 0, 0],
            "velocity": [0, 0, 0],
            "quaternion": [1, 0, 0, 0],
            "imu": {"gyro": [0, 0, 0], "accel_body": [0, 0, -9.81]},
            "battery": {"voltage": 50.0, "current": 0.0},
            "no_time_sync": True,
        }
        self.limiter = StepLimiter()
        self.assertEqual(self.limiter.next(self.base), self.base)

    def test_bounded_steps_and_quaternions(self):
        """一秒间隔分成五十步，保持单位四元数并到达目标位置。"""
        target = copy.deepcopy(self.base)
        target.update(timestamp=2.0, position=[10, 0, 0], quaternion=[0, 0, 0, 1])
        states = [self.limiter.next(target) for _ in range(50)]
        self.assertTrue(all(state is not None for state in states))
        self.assertLessEqual(self.limiter.max_delta, 0.020000001)
        self.assertAlmostEqual(states[-1]["timestamp"], 2.0)
        np.testing.assert_allclose(states[-1]["position"], [10, 0, 0])
        for state in states:
            self.assertAlmostEqual(np.linalg.norm(state["quaternion"]), 1.0)
        self.assertIsNone(self.limiter.next(target))

    def test_duplicate_suppressed(self):
        """相同时间戳不重复发送状态。"""
        self.assertIsNone(self.limiter.next(copy.deepcopy(self.base)))

    def test_clock_reset_rejected(self):
        """时间回退必须明确报错。"""
        target = copy.deepcopy(self.base)
        target["timestamp"] = 0.5
        with self.assertRaises(RuntimeError):
            self.limiter.next(target)


if __name__ == "__main__":
    unittest.main()
