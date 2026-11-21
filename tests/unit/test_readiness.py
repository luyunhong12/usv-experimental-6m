# Resolve the checkout independently of the current working directory.
"""验证持续就绪时间窗口与过期消息处理。"""

from pathlib import Path as _ProjectPath
import sys

# 项目脚本可从任意工作目录启动；建模工具保持在共享库之前。
sys.path.insert(
    0,
    str(
        next(
            p
            for p in _ProjectPath(__file__).resolve().parents
            if (p / "project.toml").is_file()
        )
        / "tools/lib"
    ),
)
sys.path.insert(
    0,
    str(
        next(
            p
            for p in _ProjectPath(__file__).resolve().parents
            if (p / "project.toml").is_file()
        )
        / "tools/modeling"
    ),
)
import unittest
from readiness import ReadinessWindow


class TestReadiness(unittest.TestCase):
    """验证持续就绪与健康消息新鲜度。"""

    def test_window(self):
        """验证完整就绪时间窗口。"""
        g = ReadinessWindow()
        self.assertFalse(g.update({"ready": True, "sim_s": 1}, 0))
        self.assertFalse(g.update({"ready": True, "sim_s": 1.5}, 0.5))
        self.assertTrue(g.update({"ready": True, "sim_s": 2}, 1.1))

    def test_missing_and_stale(self):
        """验证缺失和过期健康状态会清空窗口。"""
        g = ReadinessWindow()
        g.update({"ready": True, "sim_s": 1}, 0)
        self.assertFalse(g.update({"ready": True, "sim_s": 4}, 3))
        self.assertFalse(g.update({"ready": False, "sim_s": 4.2}, 3.2))

    def test_paused_and_reset(self):
        """验证暂停和时钟回退不被视为持续就绪。"""
        g = ReadinessWindow()
        g.update({"ready": True, "sim_s": 10}, 0)
        g.update({"ready": True, "sim_s": 10}, 0.5)
        self.assertFalse(g.update({"ready": True, "sim_s": 10}, 1.1))
        self.assertFalse(g.update({"ready": True, "sim_s": 0}, 1.2))


class TestCommandReadiness(unittest.TestCase):
    """验证控制器不会把仅传感器在线当成指令已送达。"""

    @classmethod
    def setUpClass(cls):
        """在项目测试环境中加载实际控制器类，不启动 ROS 节点。"""
        import importlib.util
        from pathlib import Path

        path = (
            Path(__file__).resolve().parents[2]
            / "ros_ws/src/usv_control/scripts/boat_control.py"
        )
        spec = importlib.util.spec_from_file_location("tested_boat_control", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cls.controller = module.BoatControl

    def test_missing_physical_subscriber(self):
        """只有两个物理订阅端都连接才能声明指令通道可用。"""
        from types import SimpleNamespace
        from unittest.mock import Mock

        state = SimpleNamespace(throttle=Mock(), steering=Mock())
        state.throttle.has_connections.return_value = True
        state.steering.has_connections.return_value = False
        self.assertFalse(self.controller.command_connected(state))
        state.steering.has_connections.return_value = True
        self.assertTrue(self.controller.command_connected(state))

    def test_unacknowledged_command_and_recovery(self):
        """持续丢失油门反馈会失败，实际收到后才恢复。"""
        from types import SimpleNamespace

        state = SimpleNamespace(
            command_mode="direct",
            direct_request=0.4,
            direct_request_wall=10,
            physics_throttle=0,
            command_mismatch_since=None,
        )
        self.assertTrue(self.controller.direct_command_confirmed(state, 10))
        state.direct_request_wall = 11.1
        self.assertFalse(self.controller.direct_command_confirmed(state, 11.1))
        state.physics_throttle = 0.4
        self.assertTrue(self.controller.direct_command_confirmed(state, 11.2))
        # Expired commands should fall to zero in the native watchdog.
        self.assertTrue(self.controller.direct_command_confirmed(state, 12.2))
        state.physics_throttle = 0
        self.assertTrue(self.controller.direct_command_confirmed(state, 12.3))


if __name__ == "__main__":
    unittest.main()
