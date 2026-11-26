"""用独立矩形面积和重叠并集检验风压几何工具。"""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/modeling"))
from derive_windage import clip_air_polygon, projected_union  # noqa: E402


class WindageTests(unittest.TestCase):
    """核对遮挡、面积、形心、裁切与分辨率误差。"""

    def test_overlapping_rectangles(self):
        """两个重叠矩形并集宽3高1，不得重复计面积。"""
        first = np.array([[0, 0, 0], [2, 0, 0], [2, 0, 1], [0, 0, 1]])
        second = first + [1, 0, 0]
        coarse = projected_union([first, second], 1, 0.01)
        fine = projected_union([first, second], 1, 0.002)
        self.assertLess(abs(fine["area_m2"] - 3), 0.01)
        self.assertLess(abs(fine["area_m2"] - 3), abs(coarse["area_m2"] - 3))
        np.testing.assert_allclose(fine["center_m"], [1.5, 0, 0.5], atol=0.002)

    def test_clip_triangle(self):
        """横切等腰三角形下半部后，上部面积为原来的四分之一。"""
        triangle = np.array([[-1, 0, -1], [1, 0, -1], [0, 0, 1]])
        polygon = clip_air_polygon(triangle, triangle[:, 2])
        result = projected_union([polygon], 1, 0.002)
        self.assertLess(abs(result["area_m2"] - 0.5), 0.006)
        np.testing.assert_allclose(result["center_m"], [0, 0, 1 / 3], atol=0.003)

    def test_invalid_spacing(self):
        """拒绝无效网格间距，而不是产生空的可信面积。"""
        with self.assertRaises(ValueError):
            projected_union([], 1, 0)


if __name__ == "__main__":
    unittest.main()
