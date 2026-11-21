/** @file SteeringCommandFilter.cc
 * @brief 无人艇SteeringCommandFilter接口与实现；工程模型边界见项目文档。
 */
#include "SteeringReference.hh"
#include <atomic>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <stdexcept>
namespace usv {

/** @brief 对转向请求实施现有速率和限位约束。
 */
class SteeringCommandFilter: public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 gz::transport::Node node;gz::transport::Node::Publisher pub;std::atomic<double> target{0};SteeringReference ref;double rate=.15,accel=.3,limit=.610865238;
 public:
/** @brief 读取插件配置并建立所需实体引用与通信接口。
 * @param s 插件 SDF 配置元素。
 */
void Configure(const gz::sim::Entity&,const std::shared_ptr<const sdf::Element>&s,gz::sim::EntityComponentManager&,gz::sim::EventManager&)override {
  rate=s->Get<double>("reference_rate");accel=s->Get<double>("reference_acceleration");limit=s->Get<double>("angle_limit");
  if(!(rate>0&&accel>0&&limit>0)||!std::isfinite(rate+accel+limit))throw std::runtime_error("Invalid steering reference limits");
  pub=node.Advertise<gz::msgs::Double>("/usv/steering/filtered_angle");
  node.Subscribe("/usv/steering/angle",&SteeringCommandFilter::OnCommand,this);
 }

/** @brief 接收并校验推进或转向控制指令。
 * @param m 归一化控制量或转向请求消息，按当前插件约定。
 */
void OnCommand(const gz::msgs::Double&m){if(std::isfinite(m.data()))target=std::clamp(m.data(),-limit,limit);}
 public:
/** @brief 在物理步进前更新该系统的状态或外力请求。
 * @param info 当前物理步的时间、步长及暂停状态。
 */
void PreUpdate(const gz::sim::UpdateInfo&info,gz::sim::EntityComponentManager&)override {
  if(info.paused)return;
  double dt=std::chrono::duration<double>(info.dt).count();
  if(dt<0){ref={};target=0;return;}
  ref.Step(target.load(),dt,rate,accel,limit);gz::msgs::Double msg;msg.set_data(ref.position);pub.Publish(msg);
 }
};
}
GZ_ADD_PLUGIN(usv::SteeringCommandFilter,gz::sim::System,usv::SteeringCommandFilter::ISystemConfigure,usv::SteeringCommandFilter::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(usv::SteeringCommandFilter,"usv::SteeringCommandFilter")
