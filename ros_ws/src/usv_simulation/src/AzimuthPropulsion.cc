/** @file AzimuthPropulsion.cc
 * @brief 无人艇AzimuthPropulsion接口与实现；工程模型边界见项目文档。
 */
#include "Environment.hh"
#include "FuelMass.hh"
#include "PropulsionModel.hh"
#include "SteeringIntegral.hh"
#include "SteeringReference.hh"
#include "WrenchAudit.hh"
#include <atomic>
#include <optional>
#include <utility>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sstream>
#include <stdexcept>
#include <usv_energy/EnergyModel.hh>
#include <usv_energy/Sensors.hh>
#include <usv_physics/RuntimeInertia.hh>
namespace usv {

/** @brief 耦合推进、转向、能源及燃油惯性的 Gazebo 系统。
 */
class AzimuthPropulsion : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate,
                          public gz::sim::ISystemReset,
                          public gz::sim::ISystemPostUpdate,
                          public gz::sim::ISystemConfigurePriority {
  gz::sim::Entity modelEntity;
  gz::transport::Node::Publisher auditPub;
  double diagnosticWaterTorqueScale = 1;
  gz::sim::Joint steering;
  SteeringReference steeringRef;
  std::atomic<double> steeringTarget{0};
  SteeringIntegral steeringIntegral;
  double steeringKi = 0;
  bool steeringVelocityFeedforward = true;
  double steeringKp = 300, steeringKd = 80, steeringLimit = .610865238,
         steeringRate = .15, steeringAccel = .3, steeringEffort = 100;
  double steeringEfficiency = .7, steeringIdle = 8, steeringCopper = .002,
         steeringMaxW = 150;
  Environment environment;
  std::string prefix = "/usv";
  gz::transport::Node::Publisher actuatorPub;
  gz::sim::Joint shaft;
  gz::sim::Link prop, hull;
  PropulsionParameters p;
  fuel::Tank tank;
  fuel::Props dryHull;
  std::vector<gz::sim::Entity> massLinks;
  gz::transport::Node::Publisher massPub;
  double massLast = -1;
  energy::Model energyModel;
  energy::Sensors sensors;
  std::atomic<bool> generatorEnabled{false}, coolingOk{true}, sensorsOk{true};
  gz::transport::Node::Publisher energyPub, sensorPub;
  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
  std::atomic<double> target{0}, commandTime{-1e9}, clock{0};
  double ref = 0, last = -1, timeout = 1, level = 0, radius = .15;

public:
  /** @brief 接收并校验推进或转向控制指令。
   * @param m 归一化控制量或转向请求消息，按当前插件约定。
   */
  void OnCommand(const gz::msgs::Double &m) {
    if (std::isfinite(m.data())) {
      target = std::clamp(m.data(), -1., 1.);
      commandTime = clock.load();
    }
  }

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
    diagnosticWaterTorqueScale =
        s->Get<double>("diagnostic_water_torque_scale", 1).first;
    if (!std::isfinite(diagnosticWaterTorqueScale) ||
        std::abs(diagnosticWaterTorqueScale) > 1)
      throw std::runtime_error("Invalid diagnostic water torque scale");
    prefix = TopicPrefix(s);
    auditPub = node.Advertise<gz::msgs::Double_V>(prefix + "/audit/propulsion");
    environment.Configure(s);
    auto model = gz::sim::Model(e);
    steering = gz::sim::Joint(model.JointByName(ecm, "steering"));
    steering.EnableVelocityCheck(ecm);
    steering.EnablePositionCheck(ecm);
    LoadSteering(s);
    node.Subscribe(prefix + "/steering/angle", &AzimuthPropulsion::OnSteering,
                   this);
    actuatorPub =
        node.Advertise<gz::msgs::Double_V>(prefix + "/actuators/state");
    shaft = gz::sim::Joint(model.JointByName(ecm, "propeller_spin"));
    prop = gz::sim::Link(model.LinkByName(ecm, "propeller"));
    hull = gz::sim::Link(model.LinkByName(ecm, "hull"));
    if (!shaft.Valid(ecm) || !prop.Valid(ecm) || !hull.Valid(ecm))
      throw std::runtime_error("Missing propulsion links/joint");
    LoadPropulsion(s);
    LoadEnergyAndTank(s);
    massLinks = {hull.Entity(), model.LinkByName(ecm, "azimuth"),
                 prop.Entity()};
    massPub = node.Advertise<gz::msgs::Double_V>(prefix + "/fuel_mass/state");
    node.Subscribe(prefix + "/energy/generator_enable",
                   &AzimuthPropulsion::OnGenerator, this);
    node.Subscribe(prefix + "/energy/cooling_ok", &AzimuthPropulsion::OnCooling,
                   this);
    node.Subscribe(prefix + "/energy/sensors_ok", &AzimuthPropulsion::OnSensors,
                   this);
    energyPub = node.Advertise<gz::msgs::Double_V>(prefix + "/energy/truth");
    sensorPub = node.Advertise<gz::msgs::Double_V>(prefix + "/energy/sensors");
    shaft.EnableVelocityCheck(ecm);
    prop.EnableVelocityChecks(ecm);
    hull.EnableVelocityChecks(ecm);
    node.Subscribe(prefix + "/propulsion/throttle",
                   &AzimuthPropulsion::OnCommand, this);
    pub = node.Advertise<gz::msgs::Double_V>(prefix + "/propulsion/state");
  }

  /** @brief 接收并校验转向角请求。
   * @param m 转向角请求消息，rad。
   */
  void OnSteering(const gz::msgs::Double &m) {
    if (std::isfinite(m.data()))
      steeringTarget = std::clamp(m.data(), -steeringLimit, steeringLimit);
  }

  /** @brief 更新发电机启停请求。
   * @param m 发电机启停布尔请求。
   */
  void OnGenerator(const gz::msgs::Boolean &m) { generatorEnabled = m.data(); }

  /** @brief 更新冷却系统状态输入。
   * @param m 冷却系统健康布尔输入。
   */
  void OnCooling(const gz::msgs::Boolean &m) { coolingOk = m.data(); }

  /** @brief 更新能源传感器健康状态输入。
   * @param m 传感器健康布尔输入。
   */
  void OnSensors(const gz::msgs::Boolean &m) { sensorsOk = m.data(); }

public:
  /** @brief 将模型或插件状态恢复到配置规定的初始状态。
   * @param ecm Gazebo 实体组件管理器。
   */
  void Reset(const gz::sim::UpdateInfo &,
             gz::sim::EntityComponentManager &ecm) override {
    steeringTarget = 0;
    steeringRef = {};
    steeringIntegral = {};
    target = 0;
    commandTime = -1e9;
    clock = 0;
    ref = 0;
    last = -1;
    energyModel.Reset();
    sensors.Reset();
    generatorEnabled = false;
    coolingOk = true;
    sensorsOk = true;
    massLast = -1;
    ApplyFuel(ecm);
  }

public:
  /** @brief 在物理步进前更新该系统的状态或外力请求。
   * @param info 当前物理步的时间、步长及暂停状态。
   * @param ecm Gazebo 实体组件管理器。
   */
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    double now = std::chrono::duration<double>(info.simTime).count();
    clock = now;
    if (info.paused)
      return;
    double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0)
      return;
    auto w = shaft.Velocity(ecm);
    auto pose = prop.WorldPose(ecm), com = prop.WorldInertialPose(ecm);
    auto vel = prop.WorldLinearVelocity(ecm);
    auto hp = hull.WorldPose(ecm);
    auto hv = hull.WorldLinearVelocity(ecm);
    if (!ValidMotionSnapshot(w,pose,com,vel,hp,hv))
      return;
    double cmd = now - commandTime.load() <= timeout ? target.load() : 0.;
    double normalized =
        std::abs(cmd) <= p.deadband
            ? 0.
            : std::copysign((std::abs(cmd) - p.deadband) / (1 - p.deadband),
                            cmd);
    ref += (normalized * p.maxOmega - ref) * (-std::expm1(-dt / p.tau));
    auto axis = pose->Rot() * gz::math::Vector3d::UnitX;
    auto env = environment.At(pose->Pos(), now, level);
    double wet =
        WetFraction(level + env.height - pose->Pos().Z(),
                    radius * std::sqrt(std::max(0., 1 - axis.Z() * axis.Z())));
    auto output = PropellerForces(p, cmd, (*w)[0], ref,
                                  (*vel - env.waterVelocity).Dot(axis), wet);
    energyModel.Prepare(dt, generatorEnabled.load(), coolingOk.load());
    auto sp = steering.Position(ecm), sv = steering.Velocity(ecm);
    if (!ValidSteeringSnapshot(sp,sv))
      return;
    auto steeringOutput=UpdateSteering(dt,*sp,*sv,ecm);
    double steeringTorque=steeringOutput.first,steeringW=steeringOutput.second;
    output.motorTorque =
        energy::LimitTorque(energyModel.p, output.motorTorque, (*w)[0],
                            energyModel.MotorBudget(dt, steeringW));
    output.shaftPower = output.motorTorque * (*w)[0];
    energyModel.Step(
        dt, energy::MotorElectrical(energyModel.p, output.motorTorque, (*w)[0]),
        steeringW);
    ApplyFuel(ecm);
    sensors.Update(now, energyModel, hp->Rot().Roll(), hp->Rot().Pitch(),
                   sensorsOk.load() && energyModel.s.unservedW < 1e-3);
    shaft.SetForce(ecm, {output.motorTorque});
    prop.AddWorldWrench(ecm, axis * output.thrust, gz::math::Vector3d::Zero);
    prop.AddWorldWrench(ecm, gz::math::Vector3d::Zero,
                        axis * output.waterTorque);
    // Diagnostic only: retain rotor load/RPM while cancelling or reversing the
    // net external water couple. Not a physical operating configuration.
    if (diagnosticWaterTorqueScale != 1)
      hull.AddWorldWrench(
          ecm, gz::math::Vector3d::Zero,
          axis * (output.waterTorque * (diagnosticWaterTorqueScale - 1)));
    PublishState({now,dt,cmd,(*w)[0],output,wet,*vel,env,axis,*pose,*hp,*hv,(*sp)[0],(*sv)[0],steeringTorque,steeringW},ecm);
  }

  /** @brief 将当前燃油质量属性传递给物理后端更新路径。
   * @param ecm Gazebo 实体组件管理器。
   */
  void ApplyFuel(gz::sim::EntityComponentManager &ecm) {
    auto pose = hull.WorldPose(ecm);
    if (!pose)
      return;
    auto up = pose->Rot().Inverse() * gz::math::Vector3d::UnitZ;
    auto value = fuel::ToInertial(fuel::Combine(
        dryHull, tank.Fuel(energyModel.s.fuel, gz::math::eigen3::convert(up))));
    // ECM supplies consistent COM offsets to other PreUpdate force systems;
    // UsvPhysics commits the same value to DART before integration.
    ecm.SetComponentData<gz::sim::components::Inertial>(hull.Entity(), value);
    if (!ecm.Component<usv::physics::RuntimeInertiaCmd>(hull.Entity()))
      ecm.CreateComponent(hull.Entity(),
                          usv::physics::RuntimeInertiaCmd(value));
    else
      ecm.SetComponentData<usv::physics::RuntimeInertiaCmd>(hull.Entity(),
                                                            value);
  }

public:
  /** @brief 返回系统初始化优先级。
   */
  gz::sim::System::PriorityType ConfigurePriority() override { return -10; }

public:
  /** @brief 在物理步进后读取实际状态并发布验证或遥测结果。
   * @param info 当前物理步的时间、步长及暂停状态。
   * @param ecm Gazebo 实体组件管理器。
   */
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    if (info.paused)
      return;
    double now = std::chrono::duration<double>(info.simTime).count();
    if (now - massLast < .1)
      return;
    auto applied =
        ecm.Component<usv::physics::RuntimeInertiaApplied>(hull.Entity());
    if (!applied) {
      if (now > 1.)
        throw std::runtime_error(
            "No native inertia acknowledgement: launch with libUsvPhysics.so");
      return;
    }
    auto hp = hull.WorldPose(ecm);
    if (!hp)
      return;
    const auto &actual = applied->Data();
    auto hm = actual.MassMatrix();
    auto hc = actual.Pose().Pos();
    auto hi = actual.Moi();
    fuel::Props ship;
    if(!AggregateShipMass(*hp,ecm,ship))return;
    gz::msgs::Double_V msg;
    for (double x : {now,
                     energyModel.s.fuel,
                     hm.Mass(),
                     hc.X(),
                     hc.Y(),
                     hc.Z(),
                     hi(0, 0),
                     hi(1, 1),
                     hi(2, 2),
                     hi(0, 1),
                     hi(0, 2),
                     hi(1, 2),
                     ship.mass,
                     ship.cg.x(),
                     ship.cg.y(),
                     ship.cg.z(),
                     ship.inertia(0, 0),
                     ship.inertia(1, 1),
                     ship.inertia(2, 2),
                     ship.inertia(0, 1),
                     ship.inertia(0, 2),
                     ship.inertia(1, 2)})
      msg.add_data(x);
    massPub.Publish(msg);
    massLast = now;
  }
/** @brief 把所有质量刚体合并到船体坐标系。
 * @param hp 船体世界位姿。
 * @param ecm Gazebo 实体组件管理器。
 * @param ship 返回合并后的质量属性；缺失组件时返回假。
 */
bool AggregateShipMass(const gz::math::Pose3d&hp,const gz::sim::EntityComponentManager&ecm,fuel::Props&ship) {
    for (auto id : massLinks) {
      gz::sim::Link l(id);
      auto ip = l.WorldInertialPose(ecm);
      auto cp = ecm.Component<gz::sim::components::Inertial>(id);
      if (!ip || !cp)
        return false;
      auto rel = hp.Inverse() * (*ip);
      fuel::Props part;
      part.mass = cp->Data().MassMatrix().Mass();
      part.cg = gz::math::eigen3::convert(rel.Pos());
      auto R = gz::math::eigen3::convert(gz::math::Matrix3d(rel.Rot()));
      part.inertia = R *
                     gz::math::eigen3::convert(cp->Data().MassMatrix().Moi()) *
                     R.transpose();
      ship = fuel::Combine(ship, part);
    }
    return true;
}

/** @brief 判断转向关节是否同时提供角度和角速度。
 * @param sp 转向关节角度可选值。
 * @param sv 转向关节角速度可选值。
 */
static bool ValidSteeringSnapshot(const std::optional<std::vector<double>>&sp,const std::optional<std::vector<double>>&sv) {
 return sp && !sp->empty() && sv && !sv->empty();
}

private:
/** @brief 检查转向执行器效率与电功率约束。
 */
bool ValidSteeringPower() const {
 return steeringEfficiency > 0 && steeringEfficiency <= 1 && steeringIdle >= 0 && steeringCopper >= 0 && steeringMaxW > 0;
}
/** @brief 检查转向控制增益与运动限幅约束。
 */
bool ValidSteeringMotion() const {
 return steeringKp > 0 && steeringKd >= 0 && std::isfinite(steeringKi) && steeringKi >= 0 && steeringLimit > 0 && steeringRate > 0 && steeringAccel > 0 && steeringEffort > 0;
}
/** @brief 读取并校验转向配置。
 * @param s 插件 SDF 配置元素。
 */
void LoadSteering(const std::shared_ptr<const sdf::Element>&s) {
    auto sc = s->FindElement("steering");
    if (!sc)
      throw std::runtime_error("Missing steering configuration");
    steeringVelocityFeedforward =
        sc->Get<bool>("reference_velocity_feedforward", true).first;
    steeringKp = sc->Get<double>("kp");
    steeringKd = sc->Get<double>("kd");
    steeringKi = sc->Get<double>("ki", 0).first;
    steeringLimit = sc->Get<double>("limit");
    steeringRate = sc->Get<double>("rate");
    steeringAccel = sc->Get<double>("acceleration");
    steeringEffort = sc->Get<double>("effort");
    steeringEfficiency = sc->Get<double>("efficiency");
    steeringIdle = sc->Get<double>("idle_w");
    steeringCopper = sc->Get<double>("copper_w_per_nm2");
    steeringMaxW = sc->Get<double>("max_w");
    if (!(ValidSteeringPower() && ValidSteeringMotion()))
      throw std::runtime_error("Invalid steering configuration");
}
/** @brief 检查推进器转速、推力与转矩功率约束。
 */
bool ValidPropellerLimits() const {
 return p.maxOmega > 0 && p.staticThrust > 0 && p.reverseRatio > 0 && p.reverseRatio <= 1 && p.shaftPower > 0 && p.stallTorque > 0;
}
/** @brief 检查推进控制和几何输入约束。
 */
bool ValidPropellerControl() const {
 return p.gain > 0 && p.tau > 0 && p.deadband >= 0 && p.deadband < 1 && timeout > 0 && radius > 0;
}
/** @brief 读取并校验推进器配置。
 * @param s 插件 SDF 配置元素。
 */
void LoadPropulsion(const std::shared_ptr<const sdf::Element>&s) {
    p.maxOmega = s->Get<double>("reference_rpm") * 2 * M_PI / 60;
    p.staticThrust = s->Get<double>("static_thrust");
    p.reverseRatio = s->Get<double>("reverse_ratio");
    p.shaftPower = s->Get<double>("shaft_power");
    p.stallTorque = s->Get<double>("stall_torque");
    p.gain = s->Get<double>("speed_gain");
    p.tau = s->Get<double>("reference_tau");
    p.deadband = s->Get<double>("deadband");
    p.advanceSpeed = s->Get<double>("advance_speed");
    timeout = s->Get<double>("command_timeout");
    level = s->Get<double>("water_level");
    radius = s->Get<double>("radius");
    if (!(ValidPropellerLimits() && ValidPropellerControl()))
      throw std::runtime_error("Invalid propulsion configuration");
}
/** @brief 读取能源与油箱配置并检查二者一致性。
 * @param s 插件 SDF 配置元素。
 */
void LoadEnergyAndTank(const std::shared_ptr<const sdf::Element>&s) {
    auto ep = s->FindElement("energy");
    if (!ep)
      throw std::runtime_error(
          "Missing energy configuration: regenerate the boat SDF");
    energy::Parameters config;
#define P(name, value) config.name = ep->Get<double>(#name);
#include <usv_energy/Parameters.def>
#undef P
    energyModel = energy::Model(config);
    auto fc = s->FindElement("fuel_mass");
    if (!fc)
      throw std::runtime_error("Missing fuel mass configuration");
    dryHull.mass = fc->Get<double>("dry_mass");
    dryHull.cg =
        gz::math::eigen3::convert(fc->Get<gz::math::Vector3d>("dry_cg"));
    std::istringstream inertiaInput(fc->Get<std::string>("dry_inertia"));
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        if (!(inertiaInput >> dryHull.inertia(i, j)))
          throw std::runtime_error("Invalid dry inertia matrix");
    fuel::ToInertial(dryHull);
    tank.size =
        gz::math::eigen3::convert(fc->Get<gz::math::Vector3d>("tank_size"));
    tank.center =
        gz::math::eigen3::convert(fc->Get<gz::math::Vector3d>("tank_center"));
    tank.density = fc->Get<double>("density");
    if (std::abs(tank.size.prod() * 1000 - config.tank_capacity_l) > 1e-8 ||
        std::abs(tank.density - 1000 * config.diesel_kg_l) > 1e-8)
      throw std::runtime_error("Energy and tank geometry disagree");
}
/** @brief 检查推进受力所需的关节和刚体状态是否齐备。
 * @param w 轴角速度。
 * @param pose 推进器世界位姿。
 * @param com 推进器惯性世界位姿。
 * @param vel 推进器世界线速度。
 * @param hp 船体世界位姿。
 * @param hv 船体世界线速度。
 */
static bool ValidMotionSnapshot(const std::optional<std::vector<double>>&w,const std::optional<gz::math::Pose3d>&pose,const std::optional<gz::math::Pose3d>&com,const std::optional<V>&vel,const std::optional<gz::math::Pose3d>&hp,const std::optional<V>&hv) {
 return w && !w->empty() && pose && com && vel && hp && hv;
}
/** @brief 推进转向参考并按原有电功率预算施加关节转矩。
 * @param dt 仿真步长，s。
 * @param sp 转向关节角度，rad。
 * @param sv 转向关节角速度，rad/s。
 * @param ecm Gazebo 实体组件管理器。
 */
std::pair<double,double> UpdateSteering(double dt,const std::vector<double>&sp,const std::vector<double>&sv,gz::sim::EntityComponentManager&ecm) {
    steeringRef.Step(steeringTarget.load(), dt, steeringRate, steeringAccel,
                     steeringLimit);
    double steeringError = steeringRef.position - sp[0];
    // 关节伺服跟踪角速度参考，避免把移动参考本身当成应抵消的扰动。
    // 此项只使用关节参考轨迹，不读取船舶航向或偏航角速度。
    const double desiredVelocity =
        steeringVelocityFeedforward ? steeringRef.velocity : 0.0;
    const double requestedSteering = steeringIntegral.Request(
        steeringError, desiredVelocity - sv[0], steeringKp, steeringKd);
    double steeringTorque =
        std::clamp(requestedSteering, -steeringEffort, steeringEffort);
    auto servoPower = [&](double torque) {
      return steeringIdle + std::abs(torque * sv[0]) / steeringEfficiency +
             steeringCopper * torque * torque;
    };
    double servoBudget = std::min(steeringMaxW, energyModel.ActuatorBudget(dt));
    double steeringW = 0;
    if (servoBudget < steeringIdle) {
      steeringIntegral = {};
      steeringTorque = 0;
      steeringRef.position = sp[0];
      steeringRef.velocity = 0;
    } else {
      double lo = 0, hi = 1;
      for (int k = 0; k < 40; ++k) {
        double mid = (lo + hi) * .5;
        if (servoPower(steeringTorque * mid) > servoBudget)
          hi = mid;
        else
          lo = mid;
      }
      steeringTorque *= lo;
      steeringW = servoPower(steeringTorque);
    }
    if (servoBudget >= steeringIdle)
      steeringIntegral.Update(steeringError, dt, steeringKi, steeringEffort,
                              requestedSteering, steeringTorque);
    steering.SetForce(ecm, {steeringTorque});
    return {steeringTorque,steeringW};
}
/** @brief 汇集单步遥测输入，引用仅在同步发布调用期间有效。
 */
struct Telemetry {
 double now;
 double dt;
 double cmd;
 double omega;
 const PropulsionOutput& output;
 double wet;
 const V& velocity;
 const Environment::Sample& env;
 const V& axis;
 const gz::math::Pose3d& pose;
 const gz::math::Pose3d& hullPose;
 const V& hullVelocity;
 double steeringPosition;
 double steeringVelocity;
 double steeringTorque;
 double steeringW;
};
/** @brief 按原有频率与字段顺序发布推进和能源遥测。
 * @param state 当前步遥测输入。
 * @param ecm Gazebo 实体组件管理器。
 */
void PublishState(const Telemetry&state,gz::sim::EntityComponentManager&ecm) {
 const auto& [now,dt,cmd,omega,output,wet,velocity,env,axis,pose,hullPose,hullVelocity,steeringPosition,steeringVelocity,steeringTorque,steeringW]=state;
    if (now - last >= .1) {
      auto af = WholeVesselFrame(modelEntity, ecm);
      auto audit = AuditMessage(now, af);
      AuditWrench(audit, af, axis * output.thrust, gz::math::Vector3d::Zero,
                  pose.Pos());
      AuditWrench(audit, af, gz::math::Vector3d::Zero,
                  axis * (output.waterTorque * diagnosticWaterTorqueScale),
                  pose.Pos());
      auditPub.Publish(audit);
      auto bodyv = hullPose.Rot().Inverse() * hullVelocity;
      gz::msgs::Double_V msg;
      for (double x :
           {now, cmd, omega * 60 / (2 * M_PI), ref * 60 / (2 * M_PI),
            output.thrust, output.motorTorque, output.waterTorque,
            output.shaftPower, wet, (velocity - env.waterVelocity).Dot(axis),
            bodyv.X(), bodyv.Y(), hullPose.Pos().X(), hullPose.Pos().Y(), hullPose.Rot().Yaw(),
            hullPose.Rot().Roll(), hullPose.Rot().Pitch()})
        msg.add_data(x);
      pub.Publish(msg);
      gz::msgs::Double_V am;
      for (double x : {now, steeringPosition, steeringVelocity, steeringRef.position,
                       steeringTorque, steeringW, energyModel.s.steeringWh,
                       double(energyModel.s.unservedW < 1e-3)})
        am.add_data(x);
      actuatorPub.Publish(am);
      const auto &es = energyModel.s;
      const auto &ep = energyModel.p;
      gz::msgs::Double_V em;
      for (double x : {now,
                       es.soc,
                       es.voltage,
                       es.current,
                       es.temperature,
                       es.fuel,
                       es.fuelRate,
                       es.motorW,
                       es.auxW,
                       es.chargerW,
                       es.generatorW,
                       double(es.generator),
                       es.starterSoc,
                       double(es.dischargeOpen),
                       double(es.soc < ep.low_soc),
                       double(es.fuel < ep.low_fuel_l),
                       double(es.fault),
                       es.batteryWh,
                       es.motorWh,
                       es.chargerWh,
                       es.lossWh,
                       es.balanceW,
                       es.removedFuelKg,
                       es.unservedW,
                       energyModel.MotorBudget(dt)})
        em.add_data(x);
      energyPub.Publish(em);
      gz::msgs::Double_V sm;
      for (double x :
           {now, sensors.time, double(sensors.valid), sensors.voltage,
            sensors.current, sensors.soc, sensors.fuel, sensors.temperature,
            sensors.time < 0 ? -1. : now - sensors.time})
        sm.add_data(x);
      sensorPub.Publish(sm);
      last = now;
    }
}

};
} // namespace usv
GZ_ADD_PLUGIN(usv::AzimuthPropulsion, gz::sim::System,
              usv::AzimuthPropulsion::ISystemConfigure,
              usv::AzimuthPropulsion::ISystemPreUpdate,
              usv::AzimuthPropulsion::ISystemReset,
              usv::AzimuthPropulsion::ISystemPostUpdate,
              usv::AzimuthPropulsion::ISystemConfigurePriority)
GZ_ADD_PLUGIN_ALIAS(usv::AzimuthPropulsion, "usv::AzimuthPropulsion")
