#include "AppendageHydrodynamics.hh"
#include "Environment.hh"
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/double_v.pb.h>
#include <stdexcept>
namespace usv {
class AppendageHydrodynamics:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate {
 struct Surface {std::vector<V> points;double cd=0,e=.7;};
 gz::sim::Link link;Environment environment;std::vector<Surface> surfaces;
 double rho=1000,level=0,last=-1,coefficientScale=1;gz::transport::Node node;gz::transport::Node::Publisher pub;
 public:void Configure(const gz::sim::Entity&entity,const std::shared_ptr<const sdf::Element>&s,gz::sim::EntityComponentManager&ecm,gz::sim::EventManager&)override {
  link=gz::sim::Link(gz::sim::Model(entity).LinkByName(ecm,s->Get<std::string>("link_name")));if(!link.Valid(ecm))throw std::runtime_error("Invalid appendage link");
  rho=s->Get<double>("density",1000).first;level=s->Get<double>("water_level",0).first;
  coefficientScale=s->Get<double>("coefficient_scale",1).first;
  if(!(coefficientScale>0 && std::isfinite(coefficientScale)))throw std::runtime_error("Invalid appendage coefficient scale");
  if(!(rho>0))throw std::runtime_error("Invalid appendage density");
  auto cfg=s->Clone();auto el=cfg->HasElement("surface")?cfg->GetElement("surface"):nullptr;
  for(;el;el=el->GetNextElement("surface")){
   Surface surface;surface.cd=el->Get<double>("cd0");surface.e=el->Get<double>("efficiency");
   if(!(surface.cd>=0 && surface.e>0 && surface.e<=1))throw std::runtime_error("Invalid appendage coefficients");
   for(auto p=el->GetElement("point");p;p=p->GetNextElement("point"))surface.points.push_back(p->Get<V>());
   if(surface.points.size()<3)throw std::runtime_error("Missing appendage outline");surfaces.push_back(surface);
  }
  environment.Configure(s);link.EnableVelocityChecks(ecm,true);pub=node.Advertise<gz::msgs::Double_V>(TopicPrefix(s)+"/hydrodynamics/appendages");
 }
 public:void PreUpdate(const gz::sim::UpdateInfo&info,gz::sim::EntityComponentManager&ecm)override {
  if(info.paused)return;auto pose=link.WorldPose(ecm);auto v=link.WorldLinearVelocity(ecm);auto w=link.WorldAngularVelocity(ecm);if(!pose||!v||!w)return;
  double now=std::chrono::duration<double>(info.simTime).count();auto frame=environment.Snapshot(now,level);V total=V::Zero,moment=V::Zero;double area=0,dissipation=0;
  for(auto&s:surfaces){std::vector<double> depths;
   for(auto&p:s.points){auto world=pose->Pos()+pose->Rot()*p;depths.push_back(level+frame.Height(world)-world.Z());}
   auto wet=ClipAppendage(s.points,depths);if(wet.area<=1e-12)continue;
   auto offset=pose->Rot()*wet.center;auto env=frame.At(pose->Pos()+offset);
   V local=pose->Rot().Inverse()*(*v+w->Cross(offset)-env.waterVelocity);
   V force=coefficientScale*AppendageForce(rho,wet.area,wet.span,s.cd,s.e,local);V worldForce=pose->Rot()*force;
   link.AddWorldWrench(ecm,worldForce,offset.Cross(worldForce));total+=force;moment+=wet.center.Cross(force);area+=wet.area;dissipation-=force.Dot(local);
  }
  if(now<last)last=-1;
  if(now-last>=.1){gz::msgs::Double_V msg;for(double x:{now,area,total.X(),total.Y(),total.Z(),moment.X(),moment.Y(),moment.Z(),dissipation})msg.add_data(x);pub.Publish(msg);last=now;}
 }
};
}
GZ_ADD_PLUGIN(usv::AppendageHydrodynamics,gz::sim::System,usv::AppendageHydrodynamics::ISystemConfigure,usv::AppendageHydrodynamics::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(usv::AppendageHydrodynamics,"usv::AppendageHydrodynamics")
