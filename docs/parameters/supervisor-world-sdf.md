# 基础试验世界 SDF 参数说明

对应 `ros_ws/src/usv_simulation/worlds/usv_hull_trial.sdf`。表格只解释字段，不保存参数数值；生成文件的值以权威 JSON、生成脚本及当前 SDF 为准。
同名兄弟序号使用通配符，未改变任何生成规则。注释存在不代表参数已标定。

| 参数路径 | 中文说明 |
| --- | --- |
| `/sdf[*]` | SDF 文档根节点，格式版本由 version 属性指定。 |
| `/sdf[*]/@version` | SDF 格式版本。 |
| `/sdf[*]/world[*]` | 仿真世界定义。 |
| `/sdf[*]/world[*]/@name` | 当前对象或插件的名称，用于引用和识别。 |
| `/sdf[*]/world[*]/physics[*]` | 物理步进与求解器配置。 |
| `/sdf[*]/world[*]/physics[*]/@name` | 当前对象或插件的名称，用于引用和识别。 |
| `/sdf[*]/world[*]/physics[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/world[*]/physics[*]/max_step_size[*]` | 最大物理仿真步长，单位 s。 |
| `/sdf[*]/world[*]/physics[*]/real_time_factor[*]` | 目标仿真时间与实际时间比。 |
| `/sdf[*]/world[*]/physics[*]/dart[*]` | DART 物理后端配置。 |
| `/sdf[*]/world[*]/physics[*]/dart[*]/collision_detector[*]` | 碰撞检测后端标识。 |
| `/sdf[*]/world[*]/gui[*]` | 图形界面配置。 |
| `/sdf[*]/world[*]/gui[*]/@fullscreen` | 是否使用全屏窗口。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]` | 加载的插件及其配置，filename 为库或插件标识，name 为实例或类标识。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/@filename` | 加载库或插件文件标识。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/@name` | 当前对象或插件的名称，用于引用和识别。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/gz-gui[*]` | Gazebo GUI 插件布局属性。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/gz-gui[*]/title[*]` | GUI 面板标题。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/gz-gui[*]/property[*]` | GUI 属性值，key 指定属性名，type 指定值类型。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/gz-gui[*]/property[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/gz-gui[*]/property[*]/@key` | GUI 属性键名。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/engine[*]` | GUI 渲染引擎标识。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/scene[*]` | 场景显示配置。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/ambient_light[*]` | 渲染环境光设置。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/background_color[*]` | 视图背景颜色。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/camera_pose[*]` | 观察相机的位置和姿态。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/camera_clip[*]` | 观察相机近远裁切范围。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/camera_clip[*]/near[*]` | 近裁切距离，单位 m。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/camera_clip[*]/far[*]` | 远裁切距离，单位 m。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/gz-gui[*]/anchors[*]` | GUI 锚定布局关系。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/gz-gui[*]/anchors[*]/@target` | 目标控件的锚定边。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/gz-gui[*]/anchors[*]/line[*]` | GUI 布局对齐关系。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/gz-gui[*]/anchors[*]/line[*]/@own` | 当前控件的锚定边。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/gz-gui[*]/anchors[*]/line[*]/@target` | 目标控件的锚定边。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/play_pause[*]` | 是否显示播放暂停控制。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/step[*]` | 是否显示单步控制。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/start_paused[*]` | 是否以暂停状态启动。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/use_event[*]` | GUI 控件是否通过事件控制仿真。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/sim_time[*]` | 是否显示仿真时间信息。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/real_time[*]` | 是否显示实际时间信息。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/real_time_factor[*]` | 目标仿真时间与实际时间比。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/iterations[*]` | 求解迭代次数。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/legacy[*]` | 相关 GUI 控件的兼容模式。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/record_video[*]` | 视频记录设置。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/record_video[*]/use_sim_time[*]` | 视频或界面是否使用仿真时间。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/record_video[*]/lockstep[*]` | 是否使用同步步进模式。 |
| `/sdf[*]/world[*]/gui[*]/plugin[*]/record_video[*]/bitrate[*]` | 视频编码码率。 |
| `/sdf[*]/world[*]/plugin[*]` | 加载的插件及其配置，filename 为库或插件标识，name 为实例或类标识。 |
| `/sdf[*]/world[*]/plugin[*]/@filename` | 加载库或插件文件标识。 |
| `/sdf[*]/world[*]/plugin[*]/@name` | 当前对象或插件的名称，用于引用和识别。 |
| `/sdf[*]/world[*]/plugin[*]/render_engine[*]` | 传感器或视图的渲染引擎标识。 |
| `/sdf[*]/world[*]/plugin[*]/disable_on_drained_battery[*]` | 是否在电池耗尽时禁用相关系统。 |
| `/sdf[*]/world[*]/scene[*]` | 场景显示配置。 |
| `/sdf[*]/world[*]/scene[*]/sky[*]` | 天空显示配置。 |
| `/sdf[*]/world[*]/scene[*]/grid[*]` | 是否显示网格。 |
| `/sdf[*]/world[*]/scene[*]/ambient[*]` | 环境颜色分量。 |
| `/sdf[*]/world[*]/scene[*]/background[*]` | 背景颜色分量。 |
| `/sdf[*]/world[*]/spherical_coordinates[*]` | 地理坐标参考原点配置。 |
| `/sdf[*]/world[*]/spherical_coordinates[*]/surface_model[*]` | 地理参考地表模型标识。 |
| `/sdf[*]/world[*]/spherical_coordinates[*]/world_frame_orientation[*]` | 世界坐标轴方向约定。 |
| `/sdf[*]/world[*]/spherical_coordinates[*]/latitude_deg[*]` | 参考原点纬度，单位度。 |
| `/sdf[*]/world[*]/spherical_coordinates[*]/longitude_deg[*]` | 参考原点经度，单位度。 |
| `/sdf[*]/world[*]/spherical_coordinates[*]/elevation[*]` | 参考原点高程，单位 m。 |
| `/sdf[*]/world[*]/spherical_coordinates[*]/heading_deg[*]` | 地理参考朝向，单位度。 |
| `/sdf[*]/world[*]/light[*]` | 场景光源。 |
| `/sdf[*]/world[*]/light[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/world[*]/light[*]/@name` | 当前对象或插件的名称，用于引用和识别。 |
| `/sdf[*]/world[*]/light[*]/cast_shadows[*]` | 是否投射阴影。 |
| `/sdf[*]/world[*]/light[*]/pose[*]` | 位置与欧拉姿态，依次为 x、y、z（m）及滚转、俯仰、偏航（rad）；参考系见 relative_to。 |
| `/sdf[*]/world[*]/light[*]/diffuse[*]` | 漫反射颜色分量。 |
| `/sdf[*]/world[*]/light[*]/specular[*]` | 镜面反射颜色分量。 |
| `/sdf[*]/world[*]/light[*]/attenuation[*]` | 光强随距离衰减的设置。 |
| `/sdf[*]/world[*]/light[*]/attenuation[*]/range[*]` | 所属传感器量程或光源作用范围。 |
| `/sdf[*]/world[*]/light[*]/attenuation[*]/constant[*]` | 光照衰减的常数项。 |
| `/sdf[*]/world[*]/light[*]/attenuation[*]/linear[*]` | 光照衰减的一次项。 |
| `/sdf[*]/world[*]/light[*]/attenuation[*]/quadratic[*]` | 光照衰减的二次项。 |
| `/sdf[*]/world[*]/light[*]/direction[*]` | 光源或波的传播方向，按所属节点解释。 |
| `/sdf[*]/world[*]/gravity[*]` | 世界重力加速度向量，单位 m/s²。 |
| `/sdf[*]/world[*]/model[*]` | 模型定义，包含刚体、关节和插件。 |
| `/sdf[*]/world[*]/model[*]/@name` | 当前对象或插件的名称，用于引用和识别。 |
| `/sdf[*]/world[*]/model[*]/static[*]` | 模型是否为静态模型。 |
| `/sdf[*]/world[*]/model[*]/link[*]` | 刚体定义，其惯性、几何和传感器位于子节点。 |
| `/sdf[*]/world[*]/model[*]/link[*]/@name` | 当前对象或插件的名称，用于引用和识别。 |
| `/sdf[*]/world[*]/model[*]/link[*]/visual[*]` | 显示几何与材质，不自动代表碰撞或浮力几何。 |
| `/sdf[*]/world[*]/model[*]/link[*]/visual[*]/@name` | 当前对象或插件的名称，用于引用和识别。 |
| `/sdf[*]/world[*]/model[*]/link[*]/visual[*]/cast_shadows[*]` | 是否投射阴影。 |
| `/sdf[*]/world[*]/model[*]/link[*]/visual[*]/geometry[*]` | 几何形状容器。 |
| `/sdf[*]/world[*]/model[*]/link[*]/visual[*]/geometry[*]/plane[*]` | 平面几何。 |
| `/sdf[*]/world[*]/model[*]/link[*]/visual[*]/geometry[*]/plane[*]/normal[*]` | 平面法线向量。 |
| `/sdf[*]/world[*]/model[*]/link[*]/visual[*]/geometry[*]/plane[*]/size[*]` | 所属几何的尺寸，单位 m；向量分量数由形状决定。 |
| `/sdf[*]/world[*]/model[*]/link[*]/visual[*]/material[*]` | 视觉材质配置。 |
| `/sdf[*]/world[*]/model[*]/link[*]/visual[*]/material[*]/ambient[*]` | 环境颜色分量。 |
| `/sdf[*]/world[*]/model[*]/link[*]/visual[*]/material[*]/diffuse[*]` | 漫反射颜色分量。 |
| `/sdf[*]/world[*]/model[*]/link[*]/visual[*]/transparency[*]` | 视觉材质透明度。 |
| `/sdf[*]/world[*]/include[*]` | 包含外部模型资源。 |
| `/sdf[*]/world[*]/include[*]/uri[*]` | 当前资源的 URI 或路径。 |
| `/sdf[*]/world[*]/include[*]/pose[*]` | 位置与欧拉姿态，依次为 x、y、z（m）及滚转、俯仰、偏航（rad）；参考系见 relative_to。 |
