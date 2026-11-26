"""用可解析长方体检验湿剖面、流体惯性积分及参考点变换。"""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/lib"))
from section_hydrodynamics import lateral_inertia, wet_section  # noqa: E402


def box_mesh():
    """返回长6米、宽2米、高1米的封闭长方体网格。"""
    vertices = np.array([[x, y, z] for x in [-3, 3] for y in [-1, 1] for z in [0, 1]])
    faces = [
        [0, 1, 3],
        [0, 3, 2],
        [4, 6, 7],
        [4, 7, 5],
        [0, 4, 5],
        [0, 5, 1],
        [2, 3, 7],
        [2, 7, 6],
        [0, 2, 6],
        [0, 6, 4],
        [1, 5, 7],
        [1, 7, 3],
    ]
    return vertices[faces]


class SectionHydrodynamicsTest(unittest.TestCase):
    """解析几何验证，不以实际船的预期恢复结果调整公式。"""

    def test_wet_and_dry_section(self):
        """半米吃水时湿宽两米，水面低于船底时无浸水。"""
        self.assertEqual(wet_section(box_mesh(), 0, 0.5), (0.5, 2.0))
        self.assertEqual(wet_section(box_mesh(), 0, -0.1), (0.0, 0.0))
        self.assertEqual(wet_section(box_mesh(), 0, 2), (1.0, 2.0))

    def test_uniform_strip_integrals_and_shift(self):
        """均匀剖面满足A66=A22 L²/12及平行轴变换。"""
        result = lateral_inertia(box_mesh(), 0.5, 0, strips=120)
        matrix = np.array(result["sway_yaw_matrix"])
        expected_mass = 750 * np.pi
        self.assertAlmostEqual(matrix[0, 0], expected_mass, places=8)
        self.assertAlmostEqual(matrix[0, 1], 0, places=8)
        self.assertLess(abs(matrix[1, 1] / expected_mass - 3), 0.0003)
        shifted = np.array(
            lateral_inertia(box_mesh(), 0.5, 1, strips=120)["sway_yaw_matrix"]
        )
        self.assertAlmostEqual(shifted[0, 1], -expected_mass, places=8)
        self.assertAlmostEqual(shifted[1, 1], matrix[1, 1] + expected_mass, places=8)
        self.assertTrue(np.all(np.linalg.eigvalsh(shifted) > 0))

    def test_invalid_inputs(self):
        """拒绝非有限网格、零密度、非法剖分和无浸水工况。"""
        for options in [{"density": 0}, {"strips": 0}, {"strips": 2.5}]:
            with self.assertRaises(ValueError):
                lateral_inertia(box_mesh(), 0.5, 0, **options)
        with self.assertRaises(ValueError):
            lateral_inertia(box_mesh(), -0.1, 0)
        with self.assertRaises(ValueError):
            lateral_inertia(box_mesh(), 2, 0)
        with self.assertRaises(ValueError):
            wet_section(box_mesh() * np.nan, 0, 0.5)
