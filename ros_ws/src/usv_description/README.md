# 自定义船体模型

从第二船体 Blender 文件导出船身、转向单元、螺旋桨三个视觉网格。`models/usv_azimuth_trial.sdf` 是试验模型；`config/trial_load.json` 的质量、惯性、阻尼和执行器性能均为未标定试验值。

`hull_sealed.json` / `hull_triangles.txt` 为基于原船体站位的封闭排水包络。原船壳的孔洞不参与进水仿真。附件排水体积尚未包含，转向单元及桨叶使用导出几何，固定外部突出件按边界选择导出；内部及柔性接触未完整覆盖。

原文件和导出文件不自动双向同步。若原船模变化，重新运行 `scripts/export_boat.py`（需 Blender），再运行 `scripts/build.sh`。生成的 SDF 使用当前工作区绝对路径；搬迁工作区后重新构建会更新路径。

精细碰撞导出：设置 USV_COLLISION_EXPORT=1 运行同一个 Blender 导出脚本。物理覆盖明细见 docs/参数与数据/零件物理覆盖台账.csv。

当前默认为 `engineering_mass_estimate.json` 派生的约 1443 kg 工程估算。实心 CAD 材料假设与聚合残余预算均有标注，不是逐零件实测。`collision_exclusions.json` 明确内部关节和柔性线束的范围边界。
