# 模型与导出

- 双固定推进器原始设计：`design/twin_thruster/01_模型/`。
- 桨舵一体原始设计：`design/azimuth_thruster/模型/`。
- 原船独立备份：`design/backups/`。
- 当前运行模型：`ros_ws/src/usv_description/`。
- 导出、网格处理、扫碰工具：`tools/modeling/`。

本次没有重新保存任何 Blender 模型，模型字节校验保存在迁移报告。设计文件夹内部保留中文原名及对应附件，避免拆散交付材料。

重新导出时显式指定可用 Blender：

```bash
USV_COLLISION_EXPORT=1 blender -b --python-exit-code 1 --python tools/modeling/export_boat.py
./usv build
```

导出会更新运行网格和参考输入，不应覆盖历史测试目录。可选 Blender 连接工具单独保留，不是仿真运行依赖。

原船模型中的支腿、底鳍和水下电机的水动力轮廓由 `tools/modeling/export_appendages.py` 导出；更新原模型后，先完成外观/碰撞导出，再用 Blender 运行此脚本，最后重新构建。运行时按波面裁切，不能直接用固定全浸面积代替。
