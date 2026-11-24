"""验证离线场景包含递归依赖、保留原文件并拒绝缺失资源。"""

from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/lib"))
from fuel_assets import localize_fuel_asset


class TestFuelAssets(unittest.TestCase):
    """从场景加载者视角检查生成副本的资源闭包。"""

    def test_recursive_references_preserve_sources(self):
        """递归模型和跨模型网格可离线访问，原SDF保持不变。"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source"
            for name in ("station", "chair"):
                (source / name).mkdir(parents=True)
            original = "<sdf><model><include><uri>https://fuel.gazebosim.org/1.0/openrobotics/models/chair</uri></include></model></sdf>"
            (source / "station/model.sdf").write_text(original)
            (source / "chair/model.sdf").write_text(
                "<sdf><model><uri>https://fuel.gazebosim.org/1.0/openrobotics/models/chair/tip/files/mesh.obj</uri></model></sdf>"
            )
            (source / "chair/mesh.obj").write_text("v 0 0 0\n")
            result = localize_fuel_asset("station", source, root / "output")
            chair = Path(ET.parse(result / "model.sdf").find(".//uri").text)
            mesh = Path(ET.parse(chair / "model.sdf").find(".//uri").text)
            self.assertEqual(mesh.read_text(), "v 0 0 0\n")
            self.assertEqual((source / "station/model.sdf").read_text(), original)
            (source / "chair/mesh.obj").unlink()
            with self.assertRaises(FileNotFoundError):
                localize_fuel_asset("station", source, root / "output")

    def test_missing_or_unsafe_model_fails(self):
        """缺模型和越界名称不得静默联网或继续启动。"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaises(FileNotFoundError):
                localize_fuel_asset("missing", root, root / "out")
            with self.assertRaises(ValueError):
                localize_fuel_asset("../escape", root, root / "out")


if __name__ == "__main__":
    unittest.main()
