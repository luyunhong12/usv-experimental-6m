/** @file HullHydrostatics.cc
 * @brief 无人艇HullHydrostatics接口与实现；工程模型边界见项目文档。
 */
#include "AddedMass.hh"
#include "Environment.hh"
#include "Hydrostatics.hh"
#include "HullSeparation.hh"
#include "Resistance.hh"
#include "WavePressure.hh"
#include "WrenchAudit.hh"
#include <fstream>
#include <gz/msgs/double_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Gravity.hh>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
namespace usv {

/** @brief 计算船体分布式压力、阻尼、附加质量及环境作用。
 */
class HullHydrostatics : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPreUpdate {
  gz::sim::Entity modelEntity;
  gz::transport::Node::Publisher auditPub;
  gz::sim::Link link;
  std::vector<Triangle> mesh;
  double rho = 1000, level = 0, last = -1;
  V linear, angular, quadratic, angularQuadratic, gravity, dragCenter,
      airCenter, airCdArea;
  Environment environment;
  gz::transport::Node node;
  gz::transport::Node::Publisher pub, envPub, wavePub;
  std::vector<V> vertices, rotated;
  std::vector<double> depths;
  std::vector<std::array<size_t, 3>> faces;
  V addedM, addedJ, previousV, previousW;
  double addedSwayYaw = 0; ///< 横移/偏航流体惯性耦合，kg·m。
  std::array<V, 3> airCenters; ///< 各受力轴的水上投影形心，船体坐标m。
  std::optional<HullSeparationProfile> separation; ///< 可选固定载荷分离修正。
  gz::transport::Node::Publisher separationPub;
  double previousTime = -1;
  gz::transport::Node::Publisher addedPub;
  bool geometryResistance = false;
  double referenceArea = 1;
  V referenceProjected{1, 1, 1};
  std::vector<V> normals;
  gz::transport::Node::Publisher resistancePub;
  bool distributedLateralDrag = false;
  std::vector<LateralPatch> lateralPatches;
  V minimum{1e9, 1e9, 1e9}, maximum{-1e9, -1e9, -1e9};

public:
  /** @brief 读取插件配置并建立所需实体引用与通信接口。
   * @param e 当前插件所属实体。
   * @param s 插件 SDF 配置元素。
   * @param ecm Gazebo 实体组件管理器。
   */
  void Configure(const gz::sim::Entity &e,
                 const std::shared_ptr<const sdf::Element> &s,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    modelEntity = e;
    auditPub =
        node.Advertise<gz::msgs::Double_V>(TopicPrefix(s) + "/audit/hull");
    link = gz::sim::Link(
        gz::sim::Model(e).LinkByName(ecm, s->Get<std::string>("link_name")));
    if (!link.Valid(ecm))
      throw std::runtime_error("Invalid hydrostatic link");
    LoadFluid(s);
    LoadGravity(ecm);
    LoadMesh(s);
    geometryResistance = s->Get<bool>("geometry_resistance", false).first;
    distributedLateralDrag =
        s->Get<bool>("distributed_lateral_drag", false).first;
    referenceArea = s->Get<double>("reference_wet_area", 1).first;
    referenceProjected =
        s->Get<V>("reference_projected_area", V(1, 1, 1)).first;
    GeometryResistanceScales(0, V::Zero, referenceArea, referenceProjected);
    resistancePub = node.Advertise<gz::msgs::Double_V>(
        TopicPrefix(s) + "/hydrodynamics/resistance");
    rotated.resize(vertices.size());
    depths.resize(vertices.size());
    addedM = s->Get<V>("added_linear", V::Zero).first;
    addedJ = s->Get<V>("added_angular", V::Zero).first;
    addedSwayYaw = s->Get<double>("added_sway_yaw", 0).first;
    if (!addedM.IsFinite() || !addedJ.IsFinite() ||
        !std::isfinite(addedSwayYaw) || addedM.Min() < 0 || addedJ.Min() < 0 ||
        addedSwayYaw * addedSwayYaw > addedM.Y() * addedJ.Z())
      throw std::runtime_error("Added inertia must be finite positive semidefinite");
    LoadSeparation(s);
    addedPub = node.Advertise<gz::msgs::Double_V>(TopicPrefix(s) +
                                                  "/hydrodynamics/added_mass");
    environment.Configure(s);
    link.EnableVelocityChecks(ecm, true);
    wavePub = node.Advertise<gz::msgs::Double_V>(TopicPrefix(s) +
                                                 "/wave_response/state");
    pub = node.Advertise<gz::msgs::Double_V>(TopicPrefix(s) + "/hydrostatics");
    envPub = node.Advertise<gz::msgs::Double_V>(TopicPrefix(s) +
                                                "/environment/state");
  }

public:
  /** @brief 在物理步进前更新该系统的状态或外力请求。
   * @param info 当前物理步的时间、步长及暂停状态。
   * @param ecm Gazebo 实体组件管理器。
   */
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused)
      return;
    auto pose = link.WorldPose(ecm);
    if (!pose)
      return;
    double now = std::chrono::duration<double>(info.simTime).count();
    V dragPoint = pose->Pos() + pose->Rot() * dragCenter;
    auto frame = environment.Snapshot(now, level);
    auto env = frame.At(dragPoint);
    V projected = V::Zero;
    auto pressure = IntegratePressure(*pose, frame, projected);
    auto scales =
        geometryResistance
            ? GeometryResistanceScales(pressure.wetArea, projected,
                                       referenceArea, referenceProjected)
            : ResistanceScales{1, V(1, 1, 1)};
    double dissipation = 0;
    Volume vol;
    vol.volume = std::max(0., pressure.volume);
    vol.centroid = pose->Pos() + pressure.Centroid();
    V force = pressure.force;
    if (pressure.wetArea > 0)
      link.AddWorldWrench(ecm, force, pressure.torque);
    auto vel = link.WorldLinearVelocity(ecm),
         omega = link.WorldAngularVelocity(ecm);
    V drag = V::Zero, dragTorque = V::Zero;
    AirDragWrench air;
    AddedWrench aw{V::Zero, V::Zero};
    const bool movingWet = HasWetMotion(vel, omega, vol.volume);
    if (movingWet) {
      auto velocities = LocalVelocities(*pose, *vel, *omega, env.waterVelocity);
      const auto &local = velocities.first;
      const auto &w = velocities.second;
      V torque = V::Zero;
      aw = UpdateAddedMass(now, local, w, *pose, ecm);
      PublishAddedMass(now, aw, local, w);
      drag = GeometryDrag(linear, quadratic, local, scales);
      torque = AngularDrag(w, scales);
      ApplyDistributedLateral(now, local, w, drag, torque);
      dragTorque = torque;
      dissipation = -(drag.Dot(local) + torque.Dot(w));
      V worldDrag = pose->Rot() * drag;
      link.AddWorldWrench(ecm, worldDrag,
                          (dragPoint - pose->Pos()).Cross(worldDrag) +
                              pose->Rot() * torque);
      const V airRelative = pose->Rot().Inverse() * (*vel - env.airVelocity);
      air = DirectionalAirDrag(airCdArea, airCenters, airRelative, w);
      link.AddWorldWrench(ecm, pose->Rot() * air.force,
                          pose->Rot() * air.torque);
    }
    if (!movingWet)
      previousTime = -1;
    if (now < last) {
      last = -1;
      previousTime = -1;
    }
    PublishState(now, *pose, frame, env, pressure, projected, scales, vol,
                 force, drag, dragTorque, dragPoint, aw, air, dissipation, ecm);
  }

private:
  /** @brief 读取可选分离剖面并核对与附加惯性来自同一几何积分。
   * @param s 船体插件配置；separation_sections_file为可选绝对数据路径。
   */
  void LoadSeparation(const std::shared_ptr<const sdf::Element> &s) {
    if (!s->HasElement("separation_sections_file"))
      return;
    if (!distributedLateralDrag)
      throw std::runtime_error("Separation candidate requires distributed drag");
    std::ifstream input(s->Get<std::string>("separation_sections_file"));
    separation = LoadHullSeparation(input, dragCenter.X());
    const V expected(addedM.Y(), addedSwayYaw, addedJ.Z());
    const V integrated(separation->swayMass, separation->swayYaw,
                       separation->yawInertia);
    for (int i = 0; i < 3; ++i)
      if (std::abs(expected[i] - integrated[i]) >
          .001 * std::max(1.0, std::abs(expected[i])))
        throw std::runtime_error("Separation and added inertia must share a profile");
    separationPub = node.Advertise<gz::msgs::Double_V>(
        TopicPrefix(s) + "/hydrodynamics/separation");
  }
  /** @brief 读取流体、阻尼和空气阻力配置。
   * @param s 插件 SDF 配置元素。
   */
  void LoadFluid(const std::shared_ptr<const sdf::Element> &s) {
    rho = s->Get<double>("density");
    level = s->Get<double>("water_level");
    linear = s->Get<V>("linear_damping");
    angular = s->Get<V>("angular_damping");
    quadratic = s->Get<V>("quadratic_damping", V::Zero).first;
    angularQuadratic = s->Get<V>("angular_quadratic_damping", V::Zero).first;
    dragCenter = s->Get<V>("drag_center", V::Zero).first;
    airCenter = s->Get<V>("air_center", V(0, 0, .5)).first;
    airCdArea = s->Get<V>("air_cd_area", V(1, 3, 0)).first;
    LoadAirCenters(s);
    if (rho <= 0 || linear.Min() < 0 || angular.Min() < 0 ||
        quadratic.Min() < 0 || angularQuadratic.Min() < 0 ||
        airCdArea.Min() < 0)
      throw std::runtime_error("Invalid fluid/damping parameters");
  }
  /** @brief 读取可选分轴风压形心；未配置时逐轴回退到原air_center。
   * @param s 当前船体插件的SDF配置。
   */
  void LoadAirCenters(const std::shared_ptr<const sdf::Element> &s) {
    const std::array<std::string, 3> keys{
        "air_center_x", "air_center_y", "air_center_z"};
    for (size_t i = 0; i < keys.size(); ++i) {
      airCenters[i] = s->Get<V>(keys[i], airCenter).first;
      if (!airCenters[i].IsFinite())
        throw std::runtime_error("Air pressure centers must be finite");
    }
    if (!airCdArea.IsFinite())
      throw std::runtime_error("Air drag areas must be finite");
  }
  /** @brief 检查并读取竖直向下的重力。
   * @param ecm Gazebo 实体组件管理器。
   */
  void LoadGravity(gz::sim::EntityComponentManager &ecm) {
    auto g =
        ecm.Component<gz::sim::components::Gravity>(gz::sim::worldEntity(ecm));
    if (!g || std::abs(g->Data().X()) > 1e-9 ||
        std::abs(g->Data().Y()) > 1e-9 || g->Data().Z() >= 0)
      throw std::runtime_error("Requires vertical downward gravity");
    gravity = g->Data();
  }
  /** @brief 建立去重顶点、面索引和表面法向。
   * @param refined 按原有边长阈值细分的三角网格。
   */
  void IndexMesh(std::vector<Triangle> &refined) {
    std::map<std::array<long long, 3>, size_t> index;
    for (auto &tri : refined) {
      std::array<size_t, 3> face;
      for (size_t j = 0; j < 3; ++j) {
        auto &v = tri[j];
        std::array<long long, 3> key{std::llround(v.X() * 1e9),
                                     std::llround(v.Y() * 1e9),
                                     std::llround(v.Z() * 1e9)};
        auto found = index.find(key);
        if (found == index.end()) {
          face[j] = vertices.size();
          index[key] = face[j];
          vertices.push_back(v);
          for (int i = 0; i < 3; ++i) {
            minimum[i] = std::min(minimum[i], v[i]);
            maximum[i] = std::max(maximum[i], v[i]);
          }
        } else
          face[j] = found->second;
      }
      faces.push_back(face);
    }
    for (auto &f : faces)
      normals.push_back((vertices[f[1]] - vertices[f[0]])
                            .Cross(vertices[f[2]] - vertices[f[0]])
                            .Normalized());
  }
  /** @brief 读取并细分压力积分网格。
   * @param s 插件 SDF 配置元素。
   */
  void LoadMesh(const std::shared_ptr<const sdf::Element> &s) {
    std::ifstream input(s->Get<std::string>("triangles_file"));
    Triangle t;
    while (input >> t[0].X() >> t[0].Y() >> t[0].Z() >> t[1].X() >> t[1].Y() >>
           t[1].Z() >> t[2].X() >> t[2].Y() >> t[2].Z())
      mesh.push_back(t);
    if (mesh.empty() || !input.eof())
      throw std::runtime_error("Invalid hull triangles file");
    double maxEdge = s->Get<double>("pressure_max_edge", .35).first;
    if (!(maxEdge > 0 && maxEdge <= 1))
      throw std::runtime_error("pressure_max_edge must be in (0,1] m");
    std::vector<Triangle> refined;
    for (auto &tri : mesh)
      RefineTriangle(tri, maxEdge, refined);
    IndexMesh(refined);
  }
  /** @brief 积分当前湿表面压力并累计方向投影面积。
   * @param pose 船体世界位姿。
   * @param frame 当前波场快照。
   * @param projected 返回各轴浸水投影面积，m²。
   */
  PressureResult IntegratePressure(const gz::math::Pose3d &pose,
                                   const WaveFrame &frame, V &projected) {
    for (size_t i = 0; i < vertices.size(); ++i) {
      rotated[i] = pose.Rot() * vertices[i];
      V world = pose.Pos() + rotated[i];
      depths[i] = level + frame.Height(world) - world.Z();
    }
    PressureResult pressure;
    lateralPatches.clear();
    for (size_t i = 0; i < faces.size(); ++i) {
      auto &f = faces[i];
      double before = pressure.wetArea;
      const V momentBefore = pressure.wetFirstMoment;
      AccumulatePressure({rotated[f[0]], rotated[f[1]], rotated[f[2]]},
                         {depths[f[0]], depths[f[1]], depths[f[2]]},
                         rho * (-gravity.Z()), pressure);
      double da = pressure.wetArea - before;
      for (int j = 0; j < 3; ++j)
        projected[j] += .5 * da * std::abs(normals[i][j]);
      if (da > 1e-12) {
        const V center = pose.Rot().Inverse() *
                         ((pressure.wetFirstMoment - momentBefore) / da);
        lateralPatches.push_back({center.X() - dragCenter.X(),
                                  .5 * da * std::abs(normals[i].Y()), da});
      }
    }
    return pressure;
  }
  /** @brief 按原有步长限制更新附加质量力并保存速度历史。
   * @param now 仿真时间，s。
   * @param local 作用点局部相对线速度，m/s。
   * @param w 局部角速度，rad/s。
   * @param pose 船体世界位姿。
   * @param ecm Gazebo 实体组件管理器。
   */
  AddedWrench UpdateAddedMass(double now, const V &local, const V &w,
                              const gz::math::Pose3d &pose,
                              gz::sim::EntityComponentManager &ecm) {
    AddedWrench aw{V::Zero, V::Zero};
    double dt = now - previousTime;
    if (previousTime >= 0 && dt > 0 && dt <= .004001) {
      aw = AddedMassWrench(addedM, addedJ, local, w, (local - previousV) / dt,
                           (w - previousW) / dt, addedSwayYaw);
      V f = pose.Rot() * aw.force;
      link.AddWorldWrench(
          ecm, f, (pose.Rot() * dragCenter).Cross(f) + pose.Rot() * aw.torque);
    } else if (previousTime >= 0 && dt > .004001 &&
               (addedM.Length() > 0 || addedJ.Length() > 0))
      throw std::runtime_error("Added mass requires timestep <=4 ms");
    previousV = local;
    previousW = w;
    previousTime = now;
    return aw;
  }
  /** @brief 按原有频率和字段顺序发布船体受力及环境遥测。
   * @param now 仿真时间，s。
   * @param pose 船体世界位姿。
   * @param frame 波场快照。
   * @param env 作用点环境采样。
   * @param pressure 压力积分结果。
   * @param projected 方向浸水投影面积。
   * @param scales 阻力面积比例。
   * @param vol 浸水体积和形心。
   * @param force 世界坐标压力合力。
   * @param drag 局部阻力。
   * @param dragTorque 局部阻力矩。
   * @param dragPoint 世界坐标阻力作用点。
   * @param aw 附加质量力与力矩。
   * @param air 局部空气阻力及船体原点力矩。
   * @param dissipation 耗散功率，W。
   * @param ecm Gazebo 实体组件管理器。
   */
  void PublishState(double now, const gz::math::Pose3d &pose,
                    const WaveFrame &frame, const WaveFrame::Sample &env,
                    const PressureResult &pressure, const V &projected,
                    const ResistanceScales &scales, const Volume &vol,
                    const V &force, const V &drag, const V &dragTorque,
                    const V &dragPoint, const AddedWrench &aw, const AirDragWrench &air,
                    double dissipation, gz::sim::EntityComponentManager &ecm) {
    if (now - last >= .1) {
      auto af = WholeVesselFrame(modelEntity, ecm);
      auto audit = AuditMessage(now, af);
      AuditWrench(audit, af, force, pressure.torque, pose.Pos());
      AuditWrench(audit, af, pose.Rot() * drag, pose.Rot() * dragTorque,
                  dragPoint);
      AuditWrench(audit, af, pose.Rot() * aw.force, pose.Rot() * aw.torque,
                  dragPoint);
      AuditWrench(audit, af, pose.Rot() * air.force,
                  pose.Rot() * air.torque, pose.Pos());
      auditPub.Publish(audit);
      gz::msgs::Double_V dm;
      for (double x :
           {now, pressure.wetArea, projected.X(), projected.Y(), projected.Z(),
            scales.surface, scales.projected.X(), scales.projected.Y(),
            scales.projected.Z(), dissipation, drag.X(), drag.Y(), drag.Z()})
        dm.add_data(x);
      resistancePub.Publish(dm);
      gz::msgs::Double_V msg;
      for (double x : {now, vol.volume, vol.centroid.X(), vol.centroid.Y(),
                       vol.centroid.Z(), force.Z(), pose.Pos().X(),
                       pose.Pos().Y(), pose.Pos().Z(), pose.Rot().Roll(),
                       pose.Rot().Pitch(), pose.Rot().Yaw()})
        msg.add_data(x);
      pub.Publish(msg);
      auto heightAt = [&](const V &v) {
        return frame.Height(pose.Pos() + pose.Rot() * v);
      };
      gz::msgs::Double_V wm;
      for (double x :
           {now, heightAt(V::Zero), heightAt(V(maximum.X(), 0, 0)),
            heightAt(V(minimum.X(), 0, 0)), heightAt(V(0, maximum.Y(), 0)),
            heightAt(V(0, minimum.Y(), 0)), pose.Pos().Z(), pose.Rot().Roll(),
            pose.Rot().Pitch(), force.X(), force.Y(), force.Z(),
            pressure.torque.X(), pressure.torque.Y(), pressure.torque.Z(),
            vol.volume, pressure.wetArea, double(faces.size())})
        wm.add_data(x);
      wavePub.Publish(wm);
      gz::msgs::Double_V em;
      for (double x :
           {now, env.height, env.dx, env.dy, env.waterVelocity.X(),
            env.waterVelocity.Y(), env.waterVelocity.Z(), env.airVelocity.X(),
            env.airVelocity.Y(), env.airVelocity.Z(), drag.X(), drag.Y(),
            drag.Z(), air.force.X(), air.force.Y(), air.force.Z()})
        em.add_data(x);
      envPub.Publish(em);
      last = now;
    }
  }
  /** @brief 发布附加质量力与输入速度的诊断量。
   * @param now 仿真时间，s。
   * @param aw 局部附加质量力与力矩。
   * @param local 局部相对线速度，m/s。
   * @param w 局部角速度，rad/s。
   */
  void PublishAddedMass(double now, const AddedWrench &aw, const V &local,
                        const V &w) {
    if (now - last >= .1) {
      gz::msgs::Double_V am;
      for (double x : {now, aw.force.X(), aw.force.Y(), aw.force.Z(),
                       aw.torque.X(), aw.torque.Y(), aw.torque.Z(), local.X(),
                       local.Y(), local.Z(), w.X(), w.Y(), w.Z()})
        am.add_data(x);
      addedPub.Publish(am);
    }
  }
  /** @brief 判断是否具有可用运动状态且存在正的浸水体积。
   * @param vel 世界线速度可选值。
   * @param omega 世界角速度可选值。
   * @param volume 浸水体积，m³。
   */
  static bool HasWetMotion(const std::optional<V> &vel,
                           const std::optional<V> &omega, double volume) {
    return vel && omega && volume > 0;
  }
  /** @brief 把作用点相对速度和角速度转换到船体坐标。
   * @param pose 船体世界位姿。
   * @param vel 船体世界线速度。
   * @param omega 船体世界角速度。
   * @param water 世界坐标水流速度。
   */
  std::pair<V, V> LocalVelocities(const gz::math::Pose3d &pose, const V &vel,
                                  const V &omega, const V &water) {
    V pointVelocity = vel + omega.Cross(pose.Rot() * dragCenter);
    V local = pose.Rot().Inverse() * (pointVelocity - water),
      w = pose.Rot().Inverse() * omega;
    return {local, w};
  }
  /** @brief 用几何分布的横向阻力替代集中侧向及偏航项。
   * @param now 当前仿真时间，s，用于同步分离修正遥测。
   * @param local 阻力参考点的局部相对速度，m/s。
   * @param w 体坐标角速度，rad/s。
   * @param drag 需要更新侧向分量的体坐标阻力，N。
   * @param torque 需要更新偏航分量的参考点阻力矩，N·m。
   */
  void ApplyDistributedLateral(double now, const V &local, const V &w, V &drag,
                               V &torque) {
    if (distributedLateralDrag) {
      const auto lateral = DistributedLateralDrag(
          lateralPatches, local.Y(), w.Z(), linear.Y() / referenceArea,
          quadratic.Y() / referenceProjected.Y());
      drag.Y(lateral.force);
      torque.Z(lateral.moment);
    }
    if (separation) {
      const auto correction =
          HullSeparationCorrection(*separation, local.X(), local.Y(), w.Z());
      drag.Y() += correction.force;
      torque.Z() += correction.moment;
      if (now - last >= .1) {
        gz::msgs::Double_V msg;
        for (double value : {now, local.X(), local.Y(), w.Z(), correction.force,
                             correction.moment})
          msg.add_data(value);
        separationPub.Publish(msg);
      }
    }
  }
  /** @brief 按现有角阻尼公式计算局部阻力矩。
   * @param w 局部角速度，rad/s。
   * @param scales 浸水面积相对基准的比例。
   */
  V AngularDrag(const V &w, const ResistanceScales &scales) const {
    V torque = V::Zero;
    for (int i = 0; i < 3; ++i)
      torque[i] =
          scales.surface *
          (-angular[i] * w[i] - angularQuadratic[i] * w[i] * std::abs(w[i]));
    return torque;
  }
};
} // namespace usv
GZ_ADD_PLUGIN(usv::HullHydrostatics, gz::sim::System,
              usv::HullHydrostatics::ISystemConfigure,
              usv::HullHydrostatics::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(usv::HullHydrostatics, "usv::HullHydrostatics")
