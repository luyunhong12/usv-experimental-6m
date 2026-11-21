# Resolve the checkout independently of the current working directory.
"""验证质量、重心与惯性汇总的解析性质和非法输入。"""

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
import numpy as np
from mass_properties import aggregate


class MassProperties(unittest.TestCase):
    """验证质量属性汇总的解析结果与输入约束。"""

    def test_parallel_axis(self):
        """验证平行轴定理的惯性组合结果。"""
        p = [
            {
                "name": str(x),
                "mass_kg": 2,
                "cg_m": [x, 0, 0],
                "inertia_at_cg_kg_m2": np.eye(3).tolist(),
            }
            for x in [-1, 1]
        ]
        r = aggregate(p)
        self.assertEqual(r["mass_kg"], 4)
        np.testing.assert_allclose(r["cg_m"], [0, 0, 0])
        np.testing.assert_allclose(r["inertia_at_cg_kg_m2"], np.diag([2, 6, 6]))

    def test_plate_mass(self):
        """验证板面积、厚度与密度给出的质量。"""
        r = aggregate(
            [
                {
                    "name": "test plate",
                    "density_kg_m3": 1000,
                    "plate_area_m2": 2,
                    "thickness_m": 0.005,
                    "cg_m": [0, 0, 0],
                    "inertia_at_cg_kg_m2": np.eye(3).tolist(),
                }
            ]
        )
        self.assertEqual(r["mass_kg"], 10)

    def test_missing_and_invalid(self):
        """验证缺失与非物理的质量属性被拒绝。"""
        with self.assertRaises(ValueError):
            aggregate([{"name": "unknown"}])
        with self.assertRaises(ValueError):
            aggregate(
                [
                    {
                        "name": "bad",
                        "mass_kg": 1,
                        "cg_m": [0, 0, 0],
                        "inertia_at_cg_kg_m2": np.diag([1, 1, 5]).tolist(),
                    }
                ]
            )


if __name__ == "__main__":
    unittest.main()
