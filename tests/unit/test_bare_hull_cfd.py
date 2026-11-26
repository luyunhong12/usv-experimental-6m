"""验证 CFD 共面合并保留实体几何，并拒绝不可靠输入。"""

from pathlib import Path
import sys
import unittest

import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/analysis"))
from bare_hull_cfd import boundary_loop, planar_polygons  # noqa: E402


class BareHullCfdTests(unittest.TestCase):
    """使用解析长方体体积和表面积，而非重复网格算法作为预期。"""

    def test_box_shape_and_orientation(self):
        """合并十二个三角形为六个面，保留面积22与正体积6。"""
        mesh = trimesh.creation.box(extents=[1, 2, 3])
        polygons = planar_polygons(mesh)
        self.assertEqual(len(polygons), 6)
        faces = []
        for face in polygons:
            self.assertEqual(len(face), 4)
            faces.extend([[face[0], face[i], face[i + 1]] for i in range(1, 3)])
        result = trimesh.Trimesh(mesh.vertices, faces, process=False)
        self.assertTrue(result.is_watertight)
        self.assertAlmostEqual(result.area, 22)
        self.assertAlmostEqual(result.volume, 6)

    def test_crease_is_preserved(self):
        """非共面三角形不能因为相邻而被合并。"""
        mesh = trimesh.creation.icosphere(subdivisions=0)
        self.assertEqual(len(planar_polygons(mesh)), 20)

    def test_invalid_surface_rejected(self):
        """开放面、反向实体及非有限坐标必须拒绝。"""
        mesh = trimesh.creation.box()
        open_mesh = trimesh.Trimesh(mesh.vertices, mesh.faces[:-1], process=False)
        inverted = mesh.copy()
        inverted.invert()
        invalid = mesh.copy()
        invalid.vertices[0, 0] = np.nan
        for candidate in (open_mesh, inverted, invalid):
            with self.assertRaises(ValueError):
                planar_polygons(candidate)

    def test_disconnected_boundary_rejected(self):
        """不能静默丢弃第二个边界环。"""
        with self.assertRaises(ValueError):
            boundary_loop(np.array([[0, 1, 2], [3, 4, 5]]))
        with self.assertRaises(ValueError):
            boundary_loop(np.array([[0, 1, 2], [0, 1, 3]]))


if __name__ == "__main__":
    unittest.main()
