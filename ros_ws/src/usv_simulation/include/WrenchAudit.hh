/** @file WrenchAudit.hh
 * @brief 无人艇WrenchAudit接口与实现；工程模型边界见项目文档。
 */
#pragma once
#include <stdexcept>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/msgs/double_v.pb.h>
namespace usv {
// All external wrenches are expressed in hull axes about the instantaneous
// whole-vessel COM. Joint actuator torques are internal and must not be summed.

/** @brief 保存船体姿态与瞬时整船重心，用于外力矩统一表达。
 */
struct AuditFrame {
 gz::math::Pose3d hull;
 gz::math::Vector3d center;
};

/** @brief 从实际刚体质量与姿态计算整船重心审计参考系。
 * @param model 被审计船舶的模型实体。
 * @param ecm Gazebo 实体组件管理器。
 */
inline AuditFrame WholeVesselFrame(gz::sim::Entity model,const gz::sim::EntityComponentManager&ecm){
 gz::sim::Model m(model);AuditFrame a;double mass=0;
 a.hull=*gz::sim::Link(m.LinkByName(ecm,"hull")).WorldPose(ecm);
 for(const auto&name:{"hull","azimuth","propeller"}){
  gz::sim::Link l(m.LinkByName(ecm,name));auto p=l.WorldInertialPose(ecm);
  auto i=ecm.Component<gz::sim::components::Inertial>(l.Entity());
  if(p&&i){double v=i->Data().MassMatrix().Mass();mass+=v;a.center+=v*p->Pos();}
 }
 if(mass<=0)throw std::runtime_error("Missing vessel mass for wrench audit");
 a.center/=mass;return a;
}

/** @brief 把外力和力矩换算到整船重心及船体轴后追加到消息。
 * @param msg 追加审计数值的 Double_V 消息。
 * @param a 含整船重心和船体姿态的审计参考系。
 * @param forceWorld 世界坐标表达的外力，N。
 * @param momentAtOriginWorld 关于给定作用原点的外力矩，N·m，世界坐标。
 * @param originWorld 力矩参考原点的世界坐标，m。
 */
inline void AuditWrench(gz::msgs::Double_V&msg,const AuditFrame&a,
 const gz::math::Vector3d&forceWorld,const gz::math::Vector3d&momentAtOriginWorld,
 const gz::math::Vector3d&originWorld){
 auto f=a.hull.Rot().Inverse()*forceWorld;
 auto m=a.hull.Rot().Inverse()*(momentAtOriginWorld+(originWorld-a.center).Cross(forceWorld));
 for(double v:{f.X(),f.Y(),f.Z(),m.X(),m.Y(),m.Z()})msg.add_data(v);
}

/** @brief 创建包含时刻和整船重心坐标的受力审计消息。
 * @param t 当前仿真时间，s。
 * @param a 含整船重心和船体姿态的审计参考系。
 */
inline gz::msgs::Double_V AuditMessage(double t,const AuditFrame&a){
 gz::msgs::Double_V msg;auto c=a.hull.Rot().Inverse()*(a.center-a.hull.Pos());
 for(double v:{t,c.X(),c.Y(),c.Z()})msg.add_data(v);return msg;
}
}
