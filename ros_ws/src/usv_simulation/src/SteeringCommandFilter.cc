#include "SteeringReference.hh"
#include <atomic>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <stdexcept>
namespace usv {
class SteeringCommandFilter: public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 gz::transport::Node node;gz::transport::Node::Publisher pub;std::atomic<double> target{0};SteeringReference ref;double rate=.15,accel=.3,limit=.610865238;
 public:void Configure(const gz::sim::Entity&,const std::shared_ptr<const sdf::Element>&s,gz::sim::EntityComponentManager&,gz::sim::EventManager&)override {
  rate=s->Get<double>("reference_rate");accel=s->Get<double>("reference_acceleration");limit=s->Get<double>("angle_limit");
  if(!(rate>0&&accel>0&&limit>0)||!std::isfinite(rate+accel+limit))throw std::runtime_error("Invalid steering reference limits");
  pub=node.Advertise<gz::msgs::Double>("/usv/steering/filtered_angle");
  node.Subscribe("/usv/steering/angle",&SteeringCommandFilter::OnCommand,this);
 }
 void OnCommand(const gz::msgs::Double&m){if(std::isfinite(m.data()))target=std::clamp(m.data(),-limit,limit);}
 public:void PreUpdate(const gz::sim::UpdateInfo&info,gz::sim::EntityComponentManager&)override {
  if(info.paused)return;
  double dt=std::chrono::duration<double>(info.dt).count();
  if(dt<0){ref={};target=0;return;}
  ref.Step(target.load(),dt,rate,accel,limit);gz::msgs::Double msg;msg.set_data(ref.position);pub.Publish(msg);
 }
};
}
GZ_ADD_PLUGIN(usv::SteeringCommandFilter,gz::sim::System,usv::SteeringCommandFilter::ISystemConfigure,usv::SteeringCommandFilter::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(usv::SteeringCommandFilter,"usv::SteeringCommandFilter")
