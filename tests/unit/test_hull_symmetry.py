"""验证船体浮力几何的中线与横倾镜像性质。

Mirror symmetry must hold for submerged geometry, not only CAD vertices.
"""

import sys
import unittest
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/lib"))


def _load_additional_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from fuel_equilibrium import displaced

    return (displaced,)


(displaced,) = _load_additional_dependencies()


class HullSymmetryTest(unittest.TestCase):
    """验证本模块的解析不变量与失败条件。"""

    def test_centerline_buoyancy(self):
        """验证中线姿态下浮心横向对称。"""
        for depth in [-0.15, -0.21, -0.30]:
            for pitch in [-0.05, 0, 0.05]:
                vol, cb, _ = displaced([depth, 0, pitch])
                self.assertGreater(vol, 0)
                self.assertLess(abs(cb[1]), 1e-10)

    def test_heeled_mirror(self):
        """验证左右横倾的体积与浮心镜像关系。"""
        for roll in [0.02, 0.1]:
            va, ca, _ = displaced([-0.24, roll, -0.02])
            vb, cb, _ = displaced([-0.24, -roll, -0.02])
            self.assertAlmostEqual(va, vb, places=10)
            np.testing.assert_allclose(ca, cb * [1, -1, 1], atol=1e-10)
