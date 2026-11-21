"""维护连续就绪窗口，拒绝缺失、过期或回退的健康状态。

A continuous readiness window; stale/missing health must restart the window.
"""

# Resolve the checkout independently of the current working directory.
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


class ReadinessWindow:
    """同时跟踪墙钟与仿真时钟的连续就绪窗口。"""

    def __init__(self, hold_s=1.0):
        """初始化连续就绪状态。

        Args:
            hold_s: 要求持续就绪的墙钟时间，单位 s。
        """
        self.hold_s = hold_s
        self.start = None
        self.sim_start = None
        self.last_received = None
        self.last_health = None

    def update(self, health, wall):
        """接收健康状态并判定就绪窗口是否满足；回退或失效会重置窗口。

        Args:
            health: 含 ready 和 sim_s 的健康状态映射。
            wall: 接收时的单调墙钟时间，单位 s。
        """
        self.fresh(wall)
        self.last_received = wall
        self.last_health = health
        if not health.get("ready"):
            self.start = None
            self.sim_start = None
            return False
        if self.start is None:
            self.start = wall
            self.sim_start = health["sim_s"]
        if health["sim_s"] < self.sim_start:
            self.start = wall
            self.sim_start = health["sim_s"]
        return (
            wall - self.start >= self.hold_s and health["sim_s"] - self.sim_start >= 0.3
        )

    def fresh(self, wall):
        """检查最近健康消息是否新鲜，过期时清空就绪窗口。

        Args:
            wall: 当前单调墙钟时间，单位 s。
        """
        ok = self.last_received is not None and wall - self.last_received < 1.0
        if not ok:
            self.start = None
            self.sim_start = None
        return ok
