# 主船模 SDF 参数说明

对应 `ros_ws/src/usv_description/models/usv_azimuth_trial.sdf`。表格只解释字段，不保存参数数值；生成文件的值以权威 JSON、生成脚本及当前 SDF 为准。
同名兄弟序号使用通配符，未改变任何生成规则。注释存在不代表参数已标定。

| 参数路径 | 中文说明 |
| --- | --- |
| `/sdf[*]` | SDF 文档根节点，格式版本由 version 属性指定。 |
| `/sdf[*]/@version` | SDF 格式版本。 |
| `/sdf[*]/model[*]` | 模型定义，包含刚体、关节和插件。 |
| `/sdf[*]/model[*]/@name` | 当前对象或插件的名称，用于引用和识别。 |
| `/sdf[*]/model[*]/self_collide[*]` | 是否启用模型内部刚体间碰撞。 |
| `/sdf[*]/model[*]/link[*]` | 刚体定义，其惯性、几何和传感器位于子节点。 |
| `/sdf[*]/model[*]/link[*]/@name` | 当前对象或插件的名称，用于引用和识别。 |
| `/sdf[*]/model[*]/link[*]/pose[*]` | 位置与欧拉姿态，依次为 x、y、z（m）及滚转、俯仰、偏航（rad）；参考系见 relative_to。 |
| `/sdf[*]/model[*]/link[*]/inertial[*]` | 刚体质量、质心姿态及惯性张量配置。 |
| `/sdf[*]/model[*]/link[*]/inertial[*]/pose[*]` | 位置与欧拉姿态，依次为 x、y、z（m）及滚转、俯仰、偏航（rad）；参考系见 relative_to。 |
| `/sdf[*]/model[*]/link[*]/inertial[*]/mass[*]` | 刚体质量，单位 kg。 |
| `/sdf[*]/model[*]/link[*]/inertial[*]/inertia[*]` | 惯性张量分量，单位 kg·m²，参考惯性坐标系。 |
| `/sdf[*]/model[*]/link[*]/inertial[*]/inertia[*]/ixx[*]` | 惯性张量 xx 分量，单位 kg·m²。 |
| `/sdf[*]/model[*]/link[*]/inertial[*]/inertia[*]/iyy[*]` | 惯性张量 yy 分量，单位 kg·m²。 |
| `/sdf[*]/model[*]/link[*]/inertial[*]/inertia[*]/izz[*]` | 惯性张量 zz 分量，单位 kg·m²。 |
| `/sdf[*]/model[*]/link[*]/inertial[*]/inertia[*]/ixy[*]` | 惯性张量 xy 对称分量，单位 kg·m²。 |
| `/sdf[*]/model[*]/link[*]/inertial[*]/inertia[*]/ixz[*]` | 惯性张量 xz 对称分量，单位 kg·m²。 |
| `/sdf[*]/model[*]/link[*]/inertial[*]/inertia[*]/iyz[*]` | 惯性张量 yz 对称分量，单位 kg·m²。 |
| `/sdf[*]/model[*]/link[*]/visual[*]` | 显示几何与材质，不自动代表碰撞或浮力几何。 |
| `/sdf[*]/model[*]/link[*]/visual[*]/@name` | 当前对象或插件的名称，用于引用和识别。 |
| `/sdf[*]/model[*]/link[*]/visual[*]/geometry[*]` | 几何形状容器。 |
| `/sdf[*]/model[*]/link[*]/visual[*]/geometry[*]/mesh[*]` | 外部网格几何。 |
| `/sdf[*]/model[*]/link[*]/visual[*]/geometry[*]/mesh[*]/uri[*]` | 当前资源的 URI 或路径。 |
| `/sdf[*]/model[*]/link[*]/visual[*]/material[*]` | 视觉材质配置。 |
| `/sdf[*]/model[*]/link[*]/visual[*]/material[*]/diffuse[*]` | 漫反射颜色分量。 |
| `/sdf[*]/model[*]/link[*]/visual[*]/material[*]/pbr[*]` | 基于物理的渲染材质配置，不参与船体受力。 |
| `/sdf[*]/model[*]/link[*]/visual[*]/material[*]/pbr[*]/metal[*]` | 金属度工作流材质。 |
| `/sdf[*]/model[*]/link[*]/visual[*]/material[*]/pbr[*]/metal[*]/albedo_map[*]` | 基础颜色纹理路径。 |
| `/sdf[*]/model[*]/link[*]/visual[*]/material[*]/pbr[*]/metal[*]/roughness_map[*]` | 粗糙度纹理路径。 |
| `/sdf[*]/model[*]/link[*]/visual[*]/material[*]/pbr[*]/metal[*]/normal_map[*]` | 法线纹理路径。 |
| `/sdf[*]/model[*]/link[*]/visual[*]/material[*]/pbr[*]/metal[*]/metalness[*]` | 材质金属度。 |
| `/sdf[*]/model[*]/link[*]/visual[*]/material[*]/pbr[*]/metal[*]/roughness[*]` | 材质粗糙度。 |
| `/sdf[*]/model[*]/link[*]/collision[*]` | 碰撞几何定义。 |
| `/sdf[*]/model[*]/link[*]/collision[*]/@name` | 当前对象或插件的名称，用于引用和识别。 |
| `/sdf[*]/model[*]/link[*]/collision[*]/geometry[*]` | 几何形状容器。 |
| `/sdf[*]/model[*]/link[*]/collision[*]/geometry[*]/mesh[*]` | 外部网格几何。 |
| `/sdf[*]/model[*]/link[*]/collision[*]/geometry[*]/mesh[*]/uri[*]` | 当前资源的 URI 或路径。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]` | 传感器定义。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/@name` | 当前对象或插件的名称，用于引用和识别。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/always_on[*]` | 是否持续启用传感器。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/update_rate[*]` | 传感器更新频率，单位 Hz。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/topic[*]` | 所属传感器或插件使用的消息主题。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/contact[*]` | 接触传感器或接触参数配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/contact[*]/collision[*]` | 碰撞几何定义。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/contact[*]/topic[*]` | 所属传感器或插件使用的消息主题。 |
| `/sdf[*]/model[*]/link[*]/visual[*]/pose[*]` | 位置与欧拉姿态，依次为 x、y、z（m）及滚转、俯仰、偏航（rad）；参考系见 relative_to。 |
| `/sdf[*]/model[*]/link[*]/visual[*]/geometry[*]/box[*]` | 长方体几何。 |
| `/sdf[*]/model[*]/link[*]/visual[*]/geometry[*]/box[*]/size[*]` | 所属几何的尺寸，单位 m；向量分量数由形状决定。 |
| `/sdf[*]/model[*]/link[*]/collision[*]/pose[*]` | 位置与欧拉姿态，依次为 x、y、z（m）及滚转、俯仰、偏航（rad）；参考系见 relative_to。 |
| `/sdf[*]/model[*]/link[*]/collision[*]/geometry[*]/box[*]` | 长方体几何。 |
| `/sdf[*]/model[*]/link[*]/collision[*]/geometry[*]/box[*]/size[*]` | 所属几何的尺寸，单位 m；向量分量数由形状决定。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/pose[*]` | 位置与欧拉姿态，依次为 x、y、z（m）及滚转、俯仰、偏航（rad）；参考系见 relative_to。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/gz_frame_id[*]` | Gazebo 消息的坐标系标识。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]` | 惯性测量传感器配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/enable_orientation[*]` | 是否输出姿态估计。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/orientation_reference_frame[*]` | 姿态测量参考系设置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/orientation_reference_frame[*]/localization[*]` | 姿态参考系约定。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/angular_velocity[*]` | 角速度测量配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/angular_velocity[*]/x[*]` | 所属三轴测量的 x 分量配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/angular_velocity[*]/x[*]/noise[*]` | 测量噪声模型及参数。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/angular_velocity[*]/x[*]/noise[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/angular_velocity[*]/x[*]/noise[*]/mean[*]` | 噪声均值，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/angular_velocity[*]/x[*]/noise[*]/stddev[*]` | 噪声标准差，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/angular_velocity[*]/y[*]` | 所属三轴测量的 y 分量配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/angular_velocity[*]/y[*]/noise[*]` | 测量噪声模型及参数。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/angular_velocity[*]/y[*]/noise[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/angular_velocity[*]/y[*]/noise[*]/mean[*]` | 噪声均值，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/angular_velocity[*]/y[*]/noise[*]/stddev[*]` | 噪声标准差，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/angular_velocity[*]/z[*]` | 所属三轴测量的 z 分量配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/angular_velocity[*]/z[*]/noise[*]` | 测量噪声模型及参数。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/angular_velocity[*]/z[*]/noise[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/angular_velocity[*]/z[*]/noise[*]/mean[*]` | 噪声均值，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/angular_velocity[*]/z[*]/noise[*]/stddev[*]` | 噪声标准差，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/linear_acceleration[*]` | 线加速度测量配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/linear_acceleration[*]/x[*]` | 所属三轴测量的 x 分量配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/linear_acceleration[*]/x[*]/noise[*]` | 测量噪声模型及参数。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/linear_acceleration[*]/x[*]/noise[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/linear_acceleration[*]/x[*]/noise[*]/mean[*]` | 噪声均值，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/linear_acceleration[*]/x[*]/noise[*]/stddev[*]` | 噪声标准差，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/linear_acceleration[*]/y[*]` | 所属三轴测量的 y 分量配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/linear_acceleration[*]/y[*]/noise[*]` | 测量噪声模型及参数。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/linear_acceleration[*]/y[*]/noise[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/linear_acceleration[*]/y[*]/noise[*]/mean[*]` | 噪声均值，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/linear_acceleration[*]/y[*]/noise[*]/stddev[*]` | 噪声标准差，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/linear_acceleration[*]/z[*]` | 所属三轴测量的 z 分量配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/linear_acceleration[*]/z[*]/noise[*]` | 测量噪声模型及参数。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/linear_acceleration[*]/z[*]/noise[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/linear_acceleration[*]/z[*]/noise[*]/mean[*]` | 噪声均值，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/imu[*]/linear_acceleration[*]/z[*]/noise[*]/stddev[*]` | 噪声标准差，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]` | 卫星定位传感器配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/position_sensing[*]` | 定位位置测量噪声。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/position_sensing[*]/horizontal[*]` | 水平扫描或水平测量分量的配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/position_sensing[*]/horizontal[*]/noise[*]` | 测量噪声模型及参数。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/position_sensing[*]/horizontal[*]/noise[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/position_sensing[*]/horizontal[*]/noise[*]/mean[*]` | 噪声均值，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/position_sensing[*]/horizontal[*]/noise[*]/stddev[*]` | 噪声标准差，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/position_sensing[*]/vertical[*]` | 垂直扫描或垂直测量分量的配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/position_sensing[*]/vertical[*]/noise[*]` | 测量噪声模型及参数。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/position_sensing[*]/vertical[*]/noise[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/position_sensing[*]/vertical[*]/noise[*]/mean[*]` | 噪声均值，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/position_sensing[*]/vertical[*]/noise[*]/stddev[*]` | 噪声标准差，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/velocity_sensing[*]` | 定位速度测量噪声。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/velocity_sensing[*]/horizontal[*]` | 水平扫描或水平测量分量的配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/velocity_sensing[*]/horizontal[*]/noise[*]` | 测量噪声模型及参数。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/velocity_sensing[*]/horizontal[*]/noise[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/velocity_sensing[*]/horizontal[*]/noise[*]/mean[*]` | 噪声均值，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/velocity_sensing[*]/horizontal[*]/noise[*]/stddev[*]` | 噪声标准差，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/velocity_sensing[*]/vertical[*]` | 垂直扫描或垂直测量分量的配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/velocity_sensing[*]/vertical[*]/noise[*]` | 测量噪声模型及参数。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/velocity_sensing[*]/vertical[*]/noise[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/velocity_sensing[*]/vertical[*]/noise[*]/mean[*]` | 噪声均值，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/navsat[*]/velocity_sensing[*]/vertical[*]/noise[*]/stddev[*]` | 噪声标准差，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/magnetometer[*]` | 磁场传感器配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/magnetometer[*]/x[*]` | 所属三轴测量的 x 分量配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/magnetometer[*]/x[*]/noise[*]` | 测量噪声模型及参数。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/magnetometer[*]/x[*]/noise[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/magnetometer[*]/x[*]/noise[*]/mean[*]` | 噪声均值，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/magnetometer[*]/x[*]/noise[*]/stddev[*]` | 噪声标准差，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/magnetometer[*]/y[*]` | 所属三轴测量的 y 分量配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/magnetometer[*]/y[*]/noise[*]` | 测量噪声模型及参数。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/magnetometer[*]/y[*]/noise[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/magnetometer[*]/y[*]/noise[*]/mean[*]` | 噪声均值，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/magnetometer[*]/y[*]/noise[*]/stddev[*]` | 噪声标准差，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/magnetometer[*]/z[*]` | 所属三轴测量的 z 分量配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/magnetometer[*]/z[*]/noise[*]` | 测量噪声模型及参数。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/magnetometer[*]/z[*]/noise[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/magnetometer[*]/z[*]/noise[*]/mean[*]` | 噪声均值，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/magnetometer[*]/z[*]/noise[*]/stddev[*]` | 噪声标准差，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/camera[*]` | 相机成像配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/camera[*]/camera_info_topic[*]` | 相机标定信息发布主题。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/camera[*]/horizontal_fov[*]` | 水平视场角，单位 rad。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/camera[*]/image[*]` | 输出图像规格。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/camera[*]/image[*]/width[*]` | 图像宽度，单位像素。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/camera[*]/image[*]/height[*]` | 图像高度，单位像素。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/camera[*]/image[*]/format[*]` | 输出图像的像素编码。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/camera[*]/clip[*]` | 相机近远裁切范围。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/camera[*]/clip[*]/near[*]` | 近裁切距离，单位 m。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/camera[*]/clip[*]/far[*]` | 远裁切距离，单位 m。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]` | 激光扫描传感器配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/scan[*]` | 扫描角度与采样配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/scan[*]/horizontal[*]` | 水平扫描或水平测量分量的配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/scan[*]/horizontal[*]/samples[*]` | 扫描样本数。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/scan[*]/horizontal[*]/resolution[*]` | 所属扫描或距离测量的分辨率设置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/scan[*]/horizontal[*]/min_angle[*]` | 扫描起始角，单位 rad。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/scan[*]/horizontal[*]/max_angle[*]` | 扫描终止角，单位 rad。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/range[*]` | 所属传感器量程或光源作用范围。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/range[*]/min[*]` | 所属量程下限，单位跟随父配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/range[*]/max[*]` | 所属量程上限，单位跟随父配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/range[*]/resolution[*]` | 所属扫描或距离测量的分辨率设置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/noise[*]` | 测量噪声模型及参数。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/noise[*]/type[*]` | 所属插件配置的类型选择。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/noise[*]/mean[*]` | 噪声均值，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/noise[*]/stddev[*]` | 噪声标准差，单位跟随所属测量量。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/scan[*]/vertical[*]` | 垂直扫描或垂直测量分量的配置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/scan[*]/vertical[*]/samples[*]` | 扫描样本数。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/scan[*]/vertical[*]/resolution[*]` | 所属扫描或距离测量的分辨率设置。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/scan[*]/vertical[*]/min_angle[*]` | 扫描起始角，单位 rad。 |
| `/sdf[*]/model[*]/link[*]/sensor[*]/lidar[*]/scan[*]/vertical[*]/max_angle[*]` | 扫描终止角，单位 rad。 |
| `/sdf[*]/model[*]/joint[*]` | 连接父子刚体的关节。 |
| `/sdf[*]/model[*]/joint[*]/@name` | 当前对象或插件的名称，用于引用和识别。 |
| `/sdf[*]/model[*]/joint[*]/@type` | 当前元素的类型标识。 |
| `/sdf[*]/model[*]/joint[*]/parent[*]` | 关节父刚体名称。 |
| `/sdf[*]/model[*]/joint[*]/child[*]` | 关节子刚体名称。 |
| `/sdf[*]/model[*]/joint[*]/pose[*]` | 位置与欧拉姿态，依次为 x、y、z（m）及滚转、俯仰、偏航（rad）；参考系见 relative_to。 |
| `/sdf[*]/model[*]/joint[*]/pose[*]/@relative_to` | 位姿表达所用的参考坐标系。 |
| `/sdf[*]/model[*]/joint[*]/axis[*]` | 关节轴定义。 |
| `/sdf[*]/model[*]/joint[*]/axis[*]/xyz[*]` | 轴向三维向量。 |
| `/sdf[*]/model[*]/joint[*]/axis[*]/limit[*]` | 关节限位和能力约束。 |
| `/sdf[*]/model[*]/joint[*]/axis[*]/limit[*]/lower[*]` | 关节位置下限；转动关节单位 rad。 |
| `/sdf[*]/model[*]/joint[*]/axis[*]/limit[*]/upper[*]` | 关节位置上限；转动关节单位 rad。 |
| `/sdf[*]/model[*]/joint[*]/axis[*]/limit[*]/effort[*]` | 关节作用力或转矩上限；转动关节单位 N·m。 |
| `/sdf[*]/model[*]/joint[*]/axis[*]/limit[*]/velocity[*]` | 所属关节速度上限或传感器速度分量；转动关节为 rad/s。 |
| `/sdf[*]/model[*]/joint[*]/axis[*]/dynamics[*]` | 关节动力学参数。 |
| `/sdf[*]/model[*]/joint[*]/axis[*]/dynamics[*]/damping[*]` | 关节阻尼系数，按当前关节类型解释。 |
| `/sdf[*]/model[*]/plugin[*]` | 加载的插件及其配置，filename 为库或插件标识，name 为实例或类标识。 |
| `/sdf[*]/model[*]/plugin[*]/@filename` | 加载库或插件文件标识。 |
| `/sdf[*]/model[*]/plugin[*]/@name` | 当前对象或插件的名称，用于引用和识别。 |
| `/sdf[*]/model[*]/plugin[*]/link_name[*]` | 插件作用的刚体名称。 |
| `/sdf[*]/model[*]/plugin[*]/density[*]` | 水动力使用的流体密度，单位 kg/m³。 |
| `/sdf[*]/model[*]/plugin[*]/water_level[*]` | 静水面参考高度，单位 m。 |
| `/sdf[*]/model[*]/plugin[*]/linear_damping[*]` | 平动线性阻尼各分量。 |
| `/sdf[*]/model[*]/plugin[*]/angular_damping[*]` | 转动线性阻尼各分量。 |
| `/sdf[*]/model[*]/plugin[*]/quadratic_damping[*]` | 平动二次阻尼各分量。 |
| `/sdf[*]/model[*]/plugin[*]/angular_quadratic_damping[*]` | 转动二次阻尼各分量。 |
| `/sdf[*]/model[*]/plugin[*]/triangles_file[*]` | 水动力积分使用的三角面片数据文件。 |
| `/sdf[*]/model[*]/plugin[*]/distributed_lateral_drag[*]` | 按浸水侧投影面积积分局部横向速度 v+x*r 的阻力及偏航力矩；替代集中 Y/N 阻尼，诊断关闭时恢复旧公式。 |
| `/sdf[*]/model[*]/plugin[*]/geometry_resistance[*]` | 按浸水几何缩放阻力的配置。 |
| `/sdf[*]/model[*]/plugin[*]/reference_wet_area[*]` | 阻力缩放使用的参考浸水面积，单位 m²。 |
| `/sdf[*]/model[*]/plugin[*]/reference_projected_area[*]` | 各轴参考投影面积，单位 m²。 |
| `/sdf[*]/model[*]/plugin[*]/added_linear[*]` | 平动附加质量各分量，单位 kg。 |
| `/sdf[*]/model[*]/plugin[*]/added_angular[*]` | 转动附加惯性各分量，单位 kg·m²。 |
| `/sdf[*]/model[*]/plugin[*]/pressure_max_edge[*]` | 分布式压力几何离散的最大边长，单位 m。 |
| `/sdf[*]/model[*]/plugin[*]/drag_center[*]` | 阻力作用参考点，单位 m，坐标系由水动力插件定义。 |
| `/sdf[*]/model[*]/plugin[*]/air_center[*]` | 空气阻力作用参考点，单位 m。 |
| `/sdf[*]/model[*]/plugin[*]/air_cd_area[*]` | 气动阻力系数与参考面积的乘积，各轴单位 m²。 |
| `/sdf[*]/model[*]/plugin[*]/topic_namespace[*]` | 插件消息主题的命名空间。 |
| `/sdf[*]/model[*]/plugin[*]/wavefield[*]` | 波场配置。 |
| `/sdf[*]/model[*]/plugin[*]/wavefield[*]/topic[*]` | 所属传感器或插件使用的消息主题。 |
| `/sdf[*]/model[*]/plugin[*]/wavefield[*]/wave[*]` | 波分量配置。 |
| `/sdf[*]/model[*]/plugin[*]/wavefield[*]/wave[*]/gain[*]` | 所属控制或受力配置的增益。 |
| `/sdf[*]/model[*]/plugin[*]/wavefield[*]/wave[*]/steepness[*]` | 波陡度设置。 |
| `/sdf[*]/model[*]/plugin[*]/reference_rpm[*]` | 推力模型参考桨轴转速，单位 rpm。 |
| `/sdf[*]/model[*]/plugin[*]/static_thrust[*]` | 参考静推力，单位 N；当前工程代理值并非全工况标定曲线。 |
| `/sdf[*]/model[*]/plugin[*]/reverse_ratio[*]` | 反推相对于正推的比例。 |
| `/sdf[*]/model[*]/plugin[*]/shaft_power[*]` | 桨轴机械功率上限，单位 W。 |
| `/sdf[*]/model[*]/plugin[*]/stall_torque[*]` | 电机堵转转矩限制，单位 N·m。 |
| `/sdf[*]/model[*]/plugin[*]/speed_gain[*]` | 轴速控制的转矩增益，单位 N·m/(rad/s)。 |
| `/sdf[*]/model[*]/plugin[*]/reference_tau[*]` | 轴速请求滤波时间常数，单位 s。 |
| `/sdf[*]/model[*]/plugin[*]/deadband[*]` | 归一化油门死区。 |
| `/sdf[*]/model[*]/plugin[*]/advance_speed[*]` | 推力模型的零推力前进速度，单位 m/s。 |
| `/sdf[*]/model[*]/plugin[*]/command_timeout[*]` | 控制指令失效时间，单位 s。 |
| `/sdf[*]/model[*]/plugin[*]/radius[*]` | 所属几何或推进配置的半径，单位 m。 |
| `/sdf[*]/model[*]/plugin[*]/fuel_mass[*]` | 燃油质量与惯性联动配置。 |
| `/sdf[*]/model[*]/plugin[*]/fuel_mass[*]/dry_mass[*]` | 不含所联动燃油的质量，单位 kg。 |
| `/sdf[*]/model[*]/plugin[*]/fuel_mass[*]/dry_cg[*]` | 不含所联动燃油的重心位置，单位 m。 |
| `/sdf[*]/model[*]/plugin[*]/fuel_mass[*]/dry_inertia[*]` | 不含所联动燃油的惯性张量，单位 kg·m²。 |
| `/sdf[*]/model[*]/plugin[*]/fuel_mass[*]/tank_size[*]` | 油箱内尺寸，单位 m。 |
| `/sdf[*]/model[*]/plugin[*]/fuel_mass[*]/tank_center[*]` | 油箱中心位置，单位 m。 |
| `/sdf[*]/model[*]/plugin[*]/fuel_mass[*]/density[*]` | 水动力使用的流体密度，单位 kg/m³。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]` | 能源系统参数，来源与假设见 energy.json。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/capacity_ah[*]` | 等效并联电池容量，单位 Ah。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/nominal_voltage_v[*]` | 电池标称电压，单位 V。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/initial_soc[*]` | 初始荷电状态，取值为容量比例。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/resistance_ohm[*]` | 等效电池内阻，单位 Ω。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/max_discharge_a[*]` | 最大放电电流，单位 A。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/cutoff_voltage_v[*]` | 放电截止电压，单位 V。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/charge_voltage_v[*]` | 充电电压，单位 V。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/reserve_soc[*]` | 停止放电的保留荷电状态阈值。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/restart_soc[*]` | 重新允许放电的荷电状态阈值。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/derate_soc[*]` | 低荷电状态降额阈值。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/low_soc[*]` | 低电量告警阈值。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/charge_efficiency[*]` | 充电电量转换效率。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/ambient_c[*]` | 环境温度，单位 °C。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/initial_temperature_c[*]` | 初始电池温度，单位 °C。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/thermal_capacity_j_k[*]` | 等效热容量，单位 J/K。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/thermal_conductance_w_k[*]` | 向环境散热的等效热导，单位 W/K。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/derate_temperature_c[*]` | 热降额温度，单位 °C。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/cutoff_temperature_c[*]` | 过温截止温度，单位 °C。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/aux_output_w[*]` | 辅助负载输出功率，单位 W；生成模型按设备台账所选模式计算。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/aux_efficiency[*]` | 辅助电源转换效率。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/motor_limit_w[*]` | 电机电输入功率限制，单位 W。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/motor_efficiency[*]` | 电机效率工程假设。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/motor_idle_w[*]` | 电机空载损耗功率，单位 W。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/motor_copper_w_per_nm2[*]` | 电机转矩平方铜耗系数，单位 W/(N·m)²。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/tank_capacity_l[*]` | 油箱容量，单位 L。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/initial_fuel_l[*]` | 初始燃油体积，单位 L。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/unusable_fuel_l[*]` | 无法使用的保留燃油体积，单位 L。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/low_fuel_l[*]` | 低油量告警体积，单位 L。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/diesel_kg_l[*]` | 燃油密度，单位 kg/L。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/generator_ac_limit_w[*]` | 发电机交流输出功率限制，单位 W。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/fuel_idle_l_h[*]` | 发电机空载耗油率，单位 L/h。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/fuel_l_kwh[*]` | 按发电量估算的耗油系数，单位 L/kWh。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/crank_s[*]` | 启动机拖动时间，单位 s。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/warmup_s[*]` | 发电机预热时间，单位 s。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/generator_ramp_s[*]` | 发电机功率爬升时间，单位 s。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/charger_current_a[*]` | 充电器输出电流限制，单位 A。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/charger_efficiency[*]` | 充电器效率。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/starter_capacity_ah[*]` | 启动电池容量，单位 Ah。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/starter_initial_soc[*]` | 启动电池初始荷电比例。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/starter_crank_a[*]` | 启动过程电流，单位 A。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/starter_min_soc[*]` | 允许启动的最小荷电状态。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/starter_charge_a[*]` | 启动电池充电电流，单位 A。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/sensor_period_s[*]` | 能源传感器采样周期，单位 s。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/sensor_filter_s[*]` | 能源测量滤波时间常数，单位 s。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/fuel_filter_s[*]` | 燃油测量滤波时间常数，单位 s。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/soc_bias[*]` | 荷电状态测量偏置，按比例表示。 |
| `/sdf[*]/model[*]/plugin[*]/energy[*]/fuel_bias_l[*]` | 油量测量偏置，单位 L。 |
| `/sdf[*]/model[*]/plugin[*]/steering[*]` | 转向执行器配置。 |
| `/sdf[*]/model[*]/plugin[*]/steering[*]/kp[*]` | 所属控制器的比例增益。 |
| `/sdf[*]/model[*]/plugin[*]/steering[*]/ki[*]` | 所属控制器的积分增益。 |
| `/sdf[*]/model[*]/plugin[*]/steering[*]/kd[*]` | 所属控制器的微分增益。 |
| `/sdf[*]/model[*]/plugin[*]/steering[*]/limit[*]` | 关节限位和能力约束。 |
| `/sdf[*]/model[*]/plugin[*]/steering[*]/rate[*]` | 转向角速度限制，按插件输入单位解释。 |
| `/sdf[*]/model[*]/plugin[*]/steering[*]/reference_velocity_feedforward[*]` | 是否使用参考关节速度：启用时 KD 作用于参考速度减实际速度；用于执行器轨迹跟踪，不读取船体航向。默认 true；false 仅用于旧实现对照。 |
| `/sdf[*]/model[*]/plugin[*]/steering[*]/acceleration[*]` | 转向角加速度限制，按插件输入单位解释。 |
| `/sdf[*]/model[*]/plugin[*]/steering[*]/effort[*]` | 关节作用力或转矩上限；转动关节单位 N·m。 |
| `/sdf[*]/model[*]/plugin[*]/steering[*]/efficiency[*]` | 所属执行器的效率假设。 |
| `/sdf[*]/model[*]/plugin[*]/steering[*]/idle_w[*]` | 执行器空载功率，单位 W。 |
| `/sdf[*]/model[*]/plugin[*]/steering[*]/copper_w_per_nm2[*]` | 转矩平方对应的铜耗系数，单位 W/(N·m)²。 |
| `/sdf[*]/model[*]/plugin[*]/steering[*]/max_w[*]` | 转向执行器功率上限，单位 W。 |
| `/sdf[*]/model[*]/plugin[*]/surface[*]` | 附体表面或几何表面配置，按父插件解释。 |
| `/sdf[*]/model[*]/plugin[*]/surface[*]/@name` | 当前对象或插件的名称，用于引用和识别。 |
| `/sdf[*]/model[*]/plugin[*]/surface[*]/cd0[*]` | 附体零升力型阻系数的工程假设。 |
| `/sdf[*]/model[*]/plugin[*]/surface[*]/efficiency[*]` | 所属执行器的效率假设。 |
| `/sdf[*]/model[*]/plugin[*]/surface[*]/point[*]` | 附体轮廓点，坐标与单位沿用所属表面配置。 |
| `/sdf[*]/model[*]/plugin[*]/publish_link_pose[*]` | 是否发布刚体位姿。 |
| `/sdf[*]/model[*]/plugin[*]/use_pose_vector_msg[*]` | 是否使用位姿数组消息。 |
| `/sdf[*]/model[*]/plugin[*]/update_frequency[*]` | 插件更新频率，单位 Hz。 |
| `/sdf[*]/model[*]/plugin[*]/topic[*]` | 所属传感器或插件使用的消息主题。 |
| `/sdf[*]/model[*]/plugin[*]/odom_frame[*]` | 里程计参考坐标系名称。 |
| `/sdf[*]/model[*]/plugin[*]/robot_base_frame[*]` | 机器人基座坐标系名称。 |
| `/sdf[*]/model[*]/plugin[*]/odom_topic[*]` | 里程计发布主题。 |
| `/sdf[*]/model[*]/plugin[*]/tf_topic[*]` | 坐标变换发布主题。 |
| `/sdf[*]/model[*]/plugin[*]/dimensions[*]` | 里程计处理的空间维度。 |
| `/sdf[*]/model[*]/plugin[*]/odom_publish_frequency[*]` | 里程计发布频率，单位 Hz。 |
| `/sdf[*]/model[*]/plugin[*]/buoyancy_triangles_file[*]` | 可选闭合浮力网格，坐标为所属 link 的 m；只有新增实体鳍使用，原推进附体不重复添加浮力。 |
| `/sdf[*]/model[*]/plugin[*]/telemetry_group[*]` | 附体遥测可选子组，仅允许英文字母、数字及下划线；不改变船名命名空间。 |
