"""验证阻力预算分配守恒及异常处理。"""

import sys
import unittest
import json
import copy
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/lib"))


def _load_additional_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from resistance_budget import allocate_resistance

    return (allocate_resistance,)


(allocate_resistance,) = _load_additional_dependencies()


class ResistanceBudgetTest(unittest.TestCase):
    """验证本模块的解析不变量与失败条件。"""

    def setUp(self):
        """读取阻力和附体配置作为独立测试输入。"""
        root = ROOT / "ros_ws/src/usv_description/config"
        self.cfg = json.loads((root / "resistance.json").read_text())
        self.app = json.loads((root / "appendage_hydrodynamics.json").read_text())

    def test_reference_conservation(self):
        """验证原轴向阻力总预算守恒。"""
        b = allocate_resistance(self.cfg, self.app)
        self.assertAlmostEqual(
            b["hull_quadratic_drag_N_s2_m2"][0] + b["allocated_axial_N_s2_m2"], 180
        )
        self.assertGreater(b["hull_quadratic_drag_N_s2_m2"][0], 0)
        # Independent rectangle has rho/2 * area * Cd = 100.
        a = copy.deepcopy(self.app)
        a["surfaces"] = [
            dict(
                name="rectangle",
                outline_xz_m=[[0, 0], [2, 0], [2, 1], [0, 1]],
                profile_drag_coefficient=0.1,
            )
        ]
        self.assertAlmostEqual(
            allocate_resistance(self.cfg, a)["allocated_axial_N_s2_m2"], 100
        )

    def test_overallocation_rejected(self):
        """验证附体分配超过预算时被拒绝。"""
        self.cfg["legacy_quadratic_drag_N_s2_m2"][0] = 0
        with self.assertRaises(ValueError):
            allocate_resistance(self.cfg, self.app)

    def test_geometry_is_authoritative(self):
        """验证轮廓几何优先于缓存面积。"""
        a = copy.deepcopy(self.app)
        for s in a["surfaces"]:
            s["area_m2"] = 999
        self.assertEqual(
            allocate_resistance(self.cfg, a), allocate_resistance(self.cfg, self.app)
        )
