#include <usv_physics/RuntimeInertia.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double_v.pb.h>
namespace usv::physics {
class InertiaResponseProbe:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate,public gz::sim::ISystemPostUpdate {
 gz::sim::Link link;std::string mode;int stage=0;double last=-1;gz::transport::Node node;gz::transport::Node::Publisher pub;
 public:void Configure(const gz::sim::Entity&e,const std::shared_ptr<const sdf::Element>&s,gz::sim::EntityComponentManager&ecm,gz::sim::EventManager&)override{
  link=gz::sim::Link(gz::sim::Model(e).LinkByName(ecm,"body"));mode=s->Get<std::string>("mode");link.EnableVelocityChecks(ecm);pub=node.Advertise<gz::msgs::Double_V>("/usv/inertia_probe");}
 public:void PreUpdate(const gz::sim::UpdateInfo&i,gz::sim::EntityComponentManager&ecm)override {
  if(i.paused)return;double t=std::chrono::duration<double>(i.simTime).count();int next=t>=2?2:1;
  if(next!=stage){double factor=next==1?1:.5;gz::math::MassMatrix3d m(10*factor,{factor,2*factor,3*factor},{0,0,0});gz::math::Inertiald v(m,gz::math::Pose3d(next==1?gz::math::Vector3d::Zero:gz::math::Vector3d(.1,.2,.3),gz::math::Quaterniond::Identity));
   ecm.SetComponentData<gz::sim::components::Inertial>(link.Entity(),v);ecm.CreateComponent(link.Entity(),RuntimeInertiaCmd(v));stage=next;}
  if(mode=="force")link.AddWorldForce(ecm,{10,0,0});else link.AddWorldWrench(ecm,{0,0,0},{0,0,1});
 }
 public:void PostUpdate(const gz::sim::UpdateInfo&i,const gz::sim::EntityComponentManager&ecm)override {
  double t=std::chrono::duration<double>(i.simTime).count();if(i.paused||t-last<.02)return;
  auto a=ecm.Component<RuntimeInertiaApplied>(link.Entity());auto v=link.WorldLinearVelocity(ecm),w=link.WorldAngularVelocity(ecm);if(!a||!v||!w)return;
  gz::msgs::Double_V msg;auto c=a->Data().Pose().Pos();for(double x:{t,v->X(),w->Z(),a->Data().MassMatrix().Mass(),c.X(),c.Y(),c.Z(),a->Data().Moi()(2,2)})msg.add_data(x);pub.Publish(msg);last=t;
 }
};
}
GZ_ADD_PLUGIN(usv::physics::InertiaResponseProbe,gz::sim::System,usv::physics::InertiaResponseProbe::ISystemConfigure,usv::physics::InertiaResponseProbe::ISystemPreUpdate,usv::physics::InertiaResponseProbe::ISystemPostUpdate)
