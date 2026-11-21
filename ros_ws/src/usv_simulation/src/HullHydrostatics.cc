#include "Hydrostatics.hh"
#include "WavePressure.hh"
#include "AddedMass.hh"
#include "Resistance.hh"
#include <map>
#include "Environment.hh"
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/double_v.pb.h>
#include <fstream>
#include <stdexcept>
namespace usv {
class HullHydrostatics:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate {
 gz::sim::Link link;std::vector<Triangle> mesh;double rho=1000,level=0,last=-1;
 V linear,angular,quadratic,angularQuadratic,gravity,dragCenter,airCenter,airCdArea;
 Environment environment;gz::transport::Node node;gz::transport::Node::Publisher pub,envPub,wavePub;
 std::vector<V> vertices,rotated;std::vector<double> depths;std::vector<std::array<size_t,3>> faces;
 V addedM,addedJ,previousV,previousW;double previousTime=-1;gz::transport::Node::Publisher addedPub;
 bool geometryResistance=false;double referenceArea=1;V referenceProjected{1,1,1};std::vector<V> normals;gz::transport::Node::Publisher resistancePub;
 V minimum{1e9,1e9,1e9},maximum{-1e9,-1e9,-1e9};
 public:void Configure(const gz::sim::Entity&e,const std::shared_ptr<const sdf::Element>&s,gz::sim::EntityComponentManager&ecm,gz::sim::EventManager&)override {
  link=gz::sim::Link(gz::sim::Model(e).LinkByName(ecm,s->Get<std::string>("link_name")));
  if(!link.Valid(ecm))throw std::runtime_error("Invalid hydrostatic link");
  rho=s->Get<double>("density");level=s->Get<double>("water_level");linear=s->Get<V>("linear_damping");angular=s->Get<V>("angular_damping");
  quadratic=s->Get<V>("quadratic_damping",V::Zero).first;angularQuadratic=s->Get<V>("angular_quadratic_damping",V::Zero).first;
  dragCenter=s->Get<V>("drag_center",V::Zero).first;airCenter=s->Get<V>("air_center",V(0,0,.5)).first;airCdArea=s->Get<V>("air_cd_area",V(1,3,0)).first;
  if(rho<=0||linear.Min()<0||angular.Min()<0||quadratic.Min()<0||angularQuadratic.Min()<0||airCdArea.Min()<0)throw std::runtime_error("Invalid fluid/damping parameters");
  auto g=ecm.Component<gz::sim::components::Gravity>(gz::sim::worldEntity(ecm));
  if(!g || std::abs(g->Data().X())>1e-9 || std::abs(g->Data().Y())>1e-9 || g->Data().Z()>=0)throw std::runtime_error("Requires vertical downward gravity");gravity=g->Data();
  std::ifstream input(s->Get<std::string>("triangles_file"));Triangle t;
  while(input>>t[0].X()>>t[0].Y()>>t[0].Z()>>t[1].X()>>t[1].Y()>>t[1].Z()>>t[2].X()>>t[2].Y()>>t[2].Z())mesh.push_back(t);
  if(mesh.empty()||!input.eof())throw std::runtime_error("Invalid hull triangles file");
  double maxEdge=s->Get<double>("pressure_max_edge",.35).first;
  if(!(maxEdge>0 && maxEdge<=1))throw std::runtime_error("pressure_max_edge must be in (0,1] m");
  std::vector<Triangle> refined;for(auto&tri:mesh)RefineTriangle(tri,maxEdge,refined);
  std::map<std::array<long long,3>,size_t> index;
  for(auto&tri:refined){std::array<size_t,3> face;
   for(size_t j=0;j<3;++j){auto&v=tri[j];std::array<long long,3> key{std::llround(v.X()*1e9),std::llround(v.Y()*1e9),std::llround(v.Z()*1e9)};
    auto found=index.find(key);if(found==index.end()){face[j]=vertices.size();index[key]=face[j];vertices.push_back(v);for(int i=0;i<3;++i){minimum[i]=std::min(minimum[i],v[i]);maximum[i]=std::max(maximum[i],v[i]);}}else face[j]=found->second;
   }faces.push_back(face);
  }
  for(auto&f:faces)normals.push_back((vertices[f[1]]-vertices[f[0]]).Cross(vertices[f[2]]-vertices[f[0]]).Normalized());
  geometryResistance=s->Get<bool>("geometry_resistance",false).first;
  referenceArea=s->Get<double>("reference_wet_area",1).first;referenceProjected=s->Get<V>("reference_projected_area",V(1,1,1)).first;
  GeometryResistanceScales(0,V::Zero,referenceArea,referenceProjected);
  resistancePub=node.Advertise<gz::msgs::Double_V>(TopicPrefix(s)+"/hydrodynamics/resistance");
  rotated.resize(vertices.size());depths.resize(vertices.size());
  addedM=s->Get<V>("added_linear",V::Zero).first;addedJ=s->Get<V>("added_angular",V::Zero).first;
  if(addedM.Min()<0 || addedJ.Min()<0)throw std::runtime_error("Negative added inertia");
  addedPub=node.Advertise<gz::msgs::Double_V>(TopicPrefix(s)+"/hydrodynamics/added_mass");
  environment.Configure(s);link.EnableVelocityChecks(ecm,true);
  wavePub=node.Advertise<gz::msgs::Double_V>(TopicPrefix(s)+"/wave_response/state");
  pub=node.Advertise<gz::msgs::Double_V>(TopicPrefix(s)+"/hydrostatics");envPub=node.Advertise<gz::msgs::Double_V>(TopicPrefix(s)+"/environment/state");
 }
 public:void PreUpdate(const gz::sim::UpdateInfo&info,gz::sim::EntityComponentManager&ecm)override {
  if(info.paused)return;
  auto pose=link.WorldPose(ecm);if(!pose)return;
  double now=std::chrono::duration<double>(info.simTime).count();
  V dragPoint=pose->Pos()+pose->Rot()*dragCenter;auto frame=environment.Snapshot(now,level);auto env=frame.At(dragPoint);
  for(size_t i=0;i<vertices.size();++i){rotated[i]=pose->Rot()*vertices[i];V world=pose->Pos()+rotated[i];depths[i]=level+frame.Height(world)-world.Z();}
  PressureResult pressure;
  V projected=V::Zero;
  for(size_t i=0;i<faces.size();++i){auto&f=faces[i];double before=pressure.wetArea;
   AccumulatePressure({rotated[f[0]],rotated[f[1]],rotated[f[2]]},{depths[f[0]],depths[f[1]],depths[f[2]]},rho*(-gravity.Z()),pressure);
   double da=pressure.wetArea-before;for(int j=0;j<3;++j)projected[j]+=.5*da*std::abs(normals[i][j]);
  }
  auto scales=geometryResistance?GeometryResistanceScales(pressure.wetArea,projected,referenceArea,referenceProjected):ResistanceScales{1,V(1,1,1)};
  double dissipation=0;
  Volume vol;vol.volume=std::max(0.,pressure.volume);vol.centroid=pose->Pos()+pressure.Centroid();V force=pressure.force;
  if(pressure.wetArea>0)link.AddWorldWrench(ecm,force,pressure.torque);
  auto vel=link.WorldLinearVelocity(ecm),omega=link.WorldAngularVelocity(ecm);
  V drag=V::Zero,air=V::Zero;
  if(vel&&omega&&vol.volume>0){
    V pointVelocity=*vel+omega->Cross(pose->Rot()*dragCenter);
    V local=pose->Rot().Inverse()*(pointVelocity-env.waterVelocity),w=pose->Rot().Inverse()*(*omega);
    V torque=V::Zero;
    double dt=now-previousTime;AddedWrench aw{V::Zero,V::Zero};
    if(previousTime>=0 && dt>0 && dt<=.004001){aw=AddedMassWrench(addedM,addedJ,local,w,(local-previousV)/dt,(w-previousW)/dt);V f=pose->Rot()*aw.force;link.AddWorldWrench(ecm,f,(pose->Rot()*dragCenter).Cross(f)+pose->Rot()*aw.torque);}
    else if(previousTime>=0 && dt>.004001 && (addedM.Length()>0||addedJ.Length()>0))throw std::runtime_error("Added mass requires timestep <=4 ms");
    previousV=local;previousW=w;previousTime=now;
    if(now-last>=.1){gz::msgs::Double_V am;for(double x:{now,aw.force.X(),aw.force.Y(),aw.force.Z(),aw.torque.X(),aw.torque.Y(),aw.torque.Z(),local.X(),local.Y(),local.Z(),w.X(),w.Y(),w.Z()})am.add_data(x);addedPub.Publish(am);}
    drag=GeometryDrag(linear,quadratic,local,scales);
    for(int i=0;i<3;++i)torque[i]=scales.surface*(-angular[i]*w[i]-angularQuadratic[i]*w[i]*std::abs(w[i]));
    dissipation=-(drag.Dot(local)+torque.Dot(w));
    V worldDrag=pose->Rot()*drag;
    link.AddWorldWrench(ecm,worldDrag,(dragPoint-pose->Pos()).Cross(worldDrag)+pose->Rot()*torque);
    V airRelative=pose->Rot().Inverse()*(*vel+omega->Cross(pose->Rot()*airCenter)-env.airVelocity);
    for(int i=0;i<3;++i)air[i]=-.5*1.225*airCdArea[i]*airRelative[i]*std::abs(airRelative[i]);
    V airWorld=pose->Rot()*air;link.AddWorldWrench(ecm,airWorld,(pose->Rot()*airCenter).Cross(airWorld));
  }
  if(!(vel&&omega&&vol.volume>0))previousTime=-1;
  if(now<last){last=-1;previousTime=-1;}
  if(now-last>=.1){gz::msgs::Double_V dm;for(double x:{now,pressure.wetArea,projected.X(),projected.Y(),projected.Z(),scales.surface,scales.projected.X(),scales.projected.Y(),scales.projected.Z(),dissipation,drag.X(),drag.Y(),drag.Z()})dm.add_data(x);resistancePub.Publish(dm);
   gz::msgs::Double_V msg;for(double x:{now,vol.volume,vol.centroid.X(),vol.centroid.Y(),vol.centroid.Z(),force.Z(),pose->Pos().X(),pose->Pos().Y(),pose->Pos().Z(),pose->Rot().Roll(),pose->Rot().Pitch(),pose->Rot().Yaw()})msg.add_data(x);pub.Publish(msg);
   auto heightAt=[&](const V&v){return frame.Height(pose->Pos()+pose->Rot()*v);};
   gz::msgs::Double_V wm;for(double x:{now,heightAt(V::Zero),heightAt(V(maximum.X(),0,0)),heightAt(V(minimum.X(),0,0)),heightAt(V(0,maximum.Y(),0)),heightAt(V(0,minimum.Y(),0)),pose->Pos().Z(),pose->Rot().Roll(),pose->Rot().Pitch(),force.X(),force.Y(),force.Z(),pressure.torque.X(),pressure.torque.Y(),pressure.torque.Z(),vol.volume,pressure.wetArea,double(faces.size())})wm.add_data(x);wavePub.Publish(wm);
   gz::msgs::Double_V em;for(double x:{now,env.height,env.dx,env.dy,env.waterVelocity.X(),env.waterVelocity.Y(),env.waterVelocity.Z(),env.airVelocity.X(),env.airVelocity.Y(),env.airVelocity.Z(),drag.X(),drag.Y(),drag.Z(),air.X(),air.Y(),air.Z()})em.add_data(x);envPub.Publish(em);last=now;}
 }
};
}
GZ_ADD_PLUGIN(usv::HullHydrostatics,gz::sim::System,usv::HullHydrostatics::ISystemConfigure,usv::HullHydrostatics::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(usv::HullHydrostatics,"usv::HullHydrostatics")
