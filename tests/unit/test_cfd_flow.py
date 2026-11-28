"""CFD 来流方向、边界选择和输入拒绝的独立检查。"""

from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/analysis"))
from bare_hull_cfd import foam_list, cell_vertices, select_sliver_faces  # noqa: E402
from cfd_flow import flow_state, patch_fields, initial_water_phase  # noqa: E402


class CfdFlowTests(unittest.TestCase):
    """验证固定船体与移动船体参考系的反号关系。"""

    def test_reverse_and_mirror(self):
        """倒车来流向船艏，左右角改变横向符号但不改变湍流强度。"""
        positive, k, omega = flow_state(-2, 30)
        negative, km, om = flow_state(-2, -30)
        np.testing.assert_allclose(positive, [2, -2 / np.sqrt(3), 0])
        np.testing.assert_allclose(negative, [2, 2 / np.sqrt(3), 0])
        self.assertAlmostEqual(k, km)
        self.assertAlmostEqual(omega, om)

    def test_side_inlet_switches(self):
        """侧边界必须允许横向流入，不能统一设成滑移壁。"""
        water = np.array([0.0, 1.0])
        flow, k, omega = flow_state(-2, 30)
        inlet = patch_fields("ymax", water, flow, k, omega)
        outlet = patch_fields("ymin", water, flow, k, omega)
        self.assertIn("type fixedValue", inlet["U"])
        self.assertIn("type fixedFluxPressure", inlet["p_rgh"])
        self.assertIn("type inletOutlet", outlet["U"])
        self.assertIn("type fixedValue", outlet["p_rgh"])

    def test_static_and_invalid_inputs(self):
        """静水来流为零；非法角度和非有限输入必须拒绝。"""
        flow, k, omega = flow_state(0, 0)
        np.testing.assert_array_equal(flow, [0, 0, 0])
        self.assertGreater(k, 0)
        self.assertGreater(omega, 0)
        for u, angle in [(0, 2), (1, 45), (np.nan, 0), (1, np.inf)]:
            with self.assertRaises(ValueError):
                flow_state(u, angle)

    def test_polygon_water_classification(self):
        """合并后的四边形仍按水面位置分类，跨界面必须拒绝。"""
        vertices = np.array([[0, 0, -1], [1, 0, -1], [1, 1, 0], [0, 1, 0]])
        self.assertEqual(initial_water_phase(vertices), 1)
        self.assertEqual(initial_water_phase(vertices + [0, 0, 2]), 0)
        with self.assertRaises(ValueError):
            initial_water_phase(vertices + [0, 0, 0.5])

    def test_ascii_list_formats(self):
        """小集合紧凑格式与嵌套向量列表均不能被截断。"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "values"
            path.write_text("FoamFile {format ascii;}\n2(4 9)\n")
            self.assertEqual(foam_list(path), (2, "4 9"))
            path.write_text("FoamFile {format ascii;}\n2\n(\n(1 2 3)\n(4 5 6)\n)\n")
            count, text = foam_list(path)
            self.assertEqual(count, 2)
            self.assertIn("(4 5 6)", text)
            path.write_text("FoamFile {format binary;}\n2(4 9)\n")
            with self.assertRaises(ValueError):
                foam_list(path)

    def test_cell_membership(self):
        """从三角面的关联恢复四面体，缺面时拒绝。"""
        faces = np.array([[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]])
        np.testing.assert_array_equal(
            cell_vertices(0, faces, np.zeros(4, dtype=int), np.array([], dtype=int)),
            [0, 1, 2, 3],
        )
        with self.assertRaises(ValueError):
            cell_vertices(0, faces[:3], np.zeros(3, dtype=int), np.array([], dtype=int))

    def merge_fixture(self, case, offset):
        """写入两个共享水平面的四面体，解析总体积为1/3。

        Args:
            case: 临时 case 目录 Path。
            offset: 全部顶点的竖直平移，m。
        """
        mesh = case / "constant/polyMesh"
        (mesh / "sets").mkdir(parents=True)
        points = np.array(
            [[0, 0, 2], [1, 0, 2], [0, 1, 2], [0.2, 0.2, 3], [0.2, 0.2, 1]]
        )
        points[:, 2] += offset
        rows = "\n".join("(" + " ".join(map(str, p)) + ")" for p in points)
        (mesh / "points").write_text("FoamFile {format ascii;}\n5\n(\n" + rows + "\n)")
        (mesh / "faces").write_text(
            "FoamFile {format ascii;}\n7\n(\n3(0 1 2)\n3(0 1 3)\n"
            "3(0 2 3)\n3(1 2 3)\n3(0 1 4)\n3(0 2 4)\n3(1 2 4)\n)"
        )
        for name, values in {
            "owner": [0, 0, 0, 0, 1, 1, 1],
            "neighbour": [1],
            "sets/underdeterminedCells": [0],
        }.items():
            (mesh / name).write_text(
                "FoamFile {format ascii;}\n"
                + str(len(values))
                + "("
                + " ".join(map(str, values))
                + ")"
            )

    def test_convex_merge_selection_and_no_overwrite(self):
        """只选择内部公共面，且重复运行不得覆盖原证据。"""
        with tempfile.TemporaryDirectory() as directory:
            case = Path(directory)
            self.merge_fixture(case, 0)
            select_sliver_faces(case)
            count, faces = foam_list(case / "constant/polyMesh/sets/sliverInterfaces")
            self.assertEqual(count, 1)
            self.assertEqual(faces.strip(), "0")
            with self.assertRaises(FileExistsError):
                select_sliver_faces(case)

    def test_water_interface_is_not_removed(self):
        """即使体积和凸性正确，也不能合并水面两侧的单元。"""
        with tempfile.TemporaryDirectory() as directory:
            case = Path(directory)
            self.merge_fixture(case, -2)
            with self.assertRaises(ValueError):
                select_sliver_faces(case)
            self.assertFalse(
                (case / "constant/polyMesh/sets/sliverInterfaces").exists()
            )


if __name__ == "__main__":
    unittest.main()
