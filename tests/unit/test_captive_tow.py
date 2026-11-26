"""固定来流台架的符号、遥测拒绝及重心力矩换算测试。"""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/analysis"))
from captive_tow import reduce_stage, tail_samples, tow_matrix  # noqa: E402


class CaptiveTowTests(unittest.TestCase):
    """用独立解析值验证试验数据的物理含义。"""

    def test_matrix_sign(self):
        """倒车纵速反号，但同侧斜流仍取相同横速。"""
        cases = tow_matrix([2], [-30, 0, 30])
        self.assertEqual(len(cases), 6)
        self.assertEqual(cases[0]["u_m_s"], 2)
        self.assertEqual(cases[3]["u_m_s"], -2)
        self.assertAlmostEqual(cases[2]["v_m_s"], 2 / np.sqrt(3))
        self.assertEqual(cases[2]["v_m_s"], cases[5]["v_m_s"])

    def test_invalid_matrix(self):
        """拒绝零速、非有限值及超出试验定义的角度。"""
        for speeds, angles in [([0], [0]), ([1], [45]), ([np.nan], [0])]:
            with self.assertRaises(ValueError):
                tow_matrix(speeds, angles)

    def test_incomplete_window(self):
        """断流数据不得当作恒定力测量。"""
        rows = np.zeros((10, 2))
        rows[:, 0] = np.linspace(2, 3, 10)
        self.assertEqual(len(tail_samples(rows, 2, 3, 2)), 10)
        rows[5:, 0] += 0.5
        with self.assertRaises(ValueError):
            tail_samples(rows, 2, 3.5, 2)

    def test_reference_shift_and_separation(self):
        """负侧力乘负纵向力臂产生正偏航矩，不能重复计分离项。"""
        widths = {
            "hull": 28,
            "appendages": 10,
            "separation": 6,
            "pose": 12,
            "environment": 16,
        }
        data = {k: np.zeros((11, n)) for k, n in widths.items()}
        for a in data.values():
            a[:, 0] = np.linspace(2, 3, 11)
        data["hull"][:, 1] = -0.5
        data["hull"][:, 11] = -7
        data["hull"][:, 15] = -11
        data["hull"][:, 21] = 2
        data["appendages"][:, 5] = -3
        data["appendages"][:, 9] = 1
        data["separation"][:, 1:6] = [-2, 0.1, 0, -2, 3]
        data["environment"][:, 4:7] = [2, -0.1, 0]
        data["pose"][:, 8] = -0.25
        result = reduce_stage(
            data, {"u_m_s": -2, "v_m_s": 0.1, "beta_deg": 0}, 0, 3, [-0.25, 0, 0], -1
        )
        self.assertTrue(result["valid"])
        self.assertEqual(result["separation_N_Nm"], 4)
        self.assertEqual(result["hull_drag_without_separation_N_Nm"], -15)
        self.assertEqual(result["total_N_Nm"][5], -8)
        data["environment"][:, 4] = -2
        self.assertFalse(
            reduce_stage(data, {"u_m_s": -2, "v_m_s": 0.1}, 0, 3, [-0.25, 0, 0], -1)[
                "valid"
            ]
        )


if __name__ == "__main__":
    unittest.main()
