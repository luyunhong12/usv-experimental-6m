#include "PropulsionModel.hh"
#include "FuelMass.hh"
#include "Environment.hh"
#include "SteeringReference.hh"
#include "SteeringIntegral.hh"
#include <usv_physics/RuntimeInertia.hh>
#include <sstream>
#include <atomic>
#include <usv_energy/EnergyModel.hh>
#include <usv_energy/Sensors.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/plugin/Register.hh>
#include <stdexcept>
namespace usv {
class AzimuthPropulsion:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate,public gz::sim::ISystemReset,public gz::sim::ISystemPostUpdate,public gz::sim::ISystemConfigurePriority {
 gz::sim::Joint steering;SteeringReference steeringRef;std::atomic<double> steeringTarget{0};
 SteeringIntegral steeringIntegral;double steeringKi=0;
 double steeringKp=300,steeringKd=80,steeringLimit=.610865238,steeringRate=.15,steeringAccel=.3,steeringEffort=100;
 double steeringEfficiency=.7,steeringIdle=8,steeringCopper=.002,steeringMaxW=150;
 Environment environment;std::string prefix="/usv";gz::transport::Node::Publisher actuatorPub;
 gz::sim::Joint shaft;gz::sim::Link prop,hull;PropulsionParameters p;
 fuel::Tank tank;fuel::Props dryHull;std::vector<gz::sim::Entity> massLinks;
 gz::transport::Node::Publisher massPub;double massLast=-1;
 energy::Model energyModel;energy::Sensors sensors;
 std::atomic<bool> generatorEnabled{false},coolingOk{true},sensorsOk{true};
 gz::transport::Node::Publisher energyPub,sensorPub;
 gz::transport::Node node;gz::transport::Node::Publisher pub;
 std::atomic<double> target{0},commandTime{-1e9},clock{0};double ref=0,last=-1,timeout=1,level=0,radius=.15;
 public:void OnCommand(const gz::msgs::Double&m){if(std::isfinite(m.data())){target=std::clamp(m.data(),-1.,1.);commandTime=clock.load();}}
 public:void Configure(const gz::sim::Entity&e,const std::shared_ptr<const sdf::Element>&s,gz::sim::EntityComponentManager&ecm,gz::sim::EventManager&)override {
  prefix=TopicPrefix(s);environment.Configure(s);auto model=gz::sim::Model(e);
  steering=gz::sim::Joint(model.JointByName(ecm,"steering"));steering.EnableVelocityCheck(ecm);steering.EnablePositionCheck(ecm);
  auto sc=s->FindElement("steering");if(!sc)throw std::runtime_error("Missing steering configuration");
  steeringKp=sc->Get<double>("kp");steeringKd=sc->Get<double>("kd");steeringKi=sc->Get<double>("ki",0).first;steeringLimit=sc->Get<double>("limit");steeringRate=sc->Get<double>("rate");steeringAccel=sc->Get<double>("acceleration");steeringEffort=sc->Get<double>("effort");
  steeringEfficiency=sc->Get<double>("efficiency");steeringIdle=sc->Get<double>("idle_w");steeringCopper=sc->Get<double>("copper_w_per_nm2");steeringMaxW=sc->Get<double>("max_w");
  if(!(steeringEfficiency>0&&steeringEfficiency<=1&&steeringIdle>=0&&steeringCopper>=0&&steeringMaxW>0&&steeringKp>0&&steeringKd>=0&&std::isfinite(steeringKi)&&steeringKi>=0&&steeringLimit>0&&steeringRate>0&&steeringAccel>0&&steeringEffort>0))throw std::runtime_error("Invalid steering configuration");
  node.Subscribe(prefix+"/steering/angle",&AzimuthPropulsion::OnSteering,this);actuatorPub=node.Advertise<gz::msgs::Double_V>(prefix+"/actuators/state");
  shaft=gz::sim::Joint(model.JointByName(ecm,"propeller_spin"));prop=gz::sim::Link(model.LinkByName(ecm,"propeller"));hull=gz::sim::Link(model.LinkByName(ecm,"hull"));
  if(!shaft.Valid(ecm)||!prop.Valid(ecm)||!hull.Valid(ecm))throw std::runtime_error("Missing propulsion links/joint");
  p.maxOmega=s->Get<double>("reference_rpm")*2*M_PI/60;p.staticThrust=s->Get<double>("static_thrust");p.reverseRatio=s->Get<double>("reverse_ratio");p.shaftPower=s->Get<double>("shaft_power");p.stallTorque=s->Get<double>("stall_torque");p.gain=s->Get<double>("speed_gain");p.tau=s->Get<double>("reference_tau");p.deadband=s->Get<double>("deadband");p.advanceSpeed=s->Get<double>("advance_speed");timeout=s->Get<double>("command_timeout");level=s->Get<double>("water_level");radius=s->Get<double>("radius");
  if(!(p.maxOmega>0&&p.staticThrust>0&&p.reverseRatio>0&&p.reverseRatio<=1&&p.shaftPower>0&&p.stallTorque>0&&p.gain>0&&p.tau>0&&p.deadband>=0&&p.deadband<1&&timeout>0&&radius>0))throw std::runtime_error("Invalid propulsion configuration");
  auto ep=s->FindElement("energy");if(!ep)throw std::runtime_error("Missing energy configuration: regenerate the boat SDF");energy::Parameters config;
#define P(name,value) config.name=ep->Get<double>(#name);
#include <usv_energy/Parameters.def>
#undef P
  energyModel=energy::Model(config);
  auto fc=s->FindElement("fuel_mass");if(!fc)throw std::runtime_error("Missing fuel mass configuration");
  dryHull.mass=fc->Get<double>("dry_mass");dryHull.cg=gz::math::eigen3::convert(fc->Get<gz::math::Vector3d>("dry_cg"));
  std::istringstream inertiaInput(fc->Get<std::string>("dry_inertia"));for(int i=0;i<3;++i)for(int j=0;j<3;++j)if(!(inertiaInput>>dryHull.inertia(i,j)))throw std::runtime_error("Invalid dry inertia matrix");
  fuel::ToInertial(dryHull);tank.size=gz::math::eigen3::convert(fc->Get<gz::math::Vector3d>("tank_size"));tank.center=gz::math::eigen3::convert(fc->Get<gz::math::Vector3d>("tank_center"));tank.density=fc->Get<double>("density");
  if(std::abs(tank.size.prod()*1000-config.tank_capacity_l)>1e-8 || std::abs(tank.density-1000*config.diesel_kg_l)>1e-8)throw std::runtime_error("Energy and tank geometry disagree");
  massLinks={hull.Entity(),model.LinkByName(ecm,"azimuth"),prop.Entity()};
  massPub=node.Advertise<gz::msgs::Double_V>(prefix+"/fuel_mass/state");
  node.Subscribe(prefix+"/energy/generator_enable",&AzimuthPropulsion::OnGenerator,this);
  node.Subscribe(prefix+"/energy/cooling_ok",&AzimuthPropulsion::OnCooling,this);
  node.Subscribe(prefix+"/energy/sensors_ok",&AzimuthPropulsion::OnSensors,this);
  energyPub=node.Advertise<gz::msgs::Double_V>(prefix+"/energy/truth");sensorPub=node.Advertise<gz::msgs::Double_V>(prefix+"/energy/sensors");
  shaft.EnableVelocityCheck(ecm);prop.EnableVelocityChecks(ecm);hull.EnableVelocityChecks(ecm);
  node.Subscribe(prefix+"/propulsion/throttle",&AzimuthPropulsion::OnCommand,this);pub=node.Advertise<gz::msgs::Double_V>(prefix+"/propulsion/state");
 }
 void OnSteering(const gz::msgs::Double&m){if(std::isfinite(m.data()))steeringTarget=std::clamp(m.data(),-steeringLimit,steeringLimit);}
 void OnGenerator(const gz::msgs::Boolean&m){generatorEnabled=m.data();}
 void OnCooling(const gz::msgs::Boolean&m){coolingOk=m.data();}
 void OnSensors(const gz::msgs::Boolean&m){sensorsOk=m.data();}
 public:void Reset(const gz::sim::UpdateInfo&,gz::sim::EntityComponentManager&ecm)override {steeringTarget=0;steeringRef={};steeringIntegral={};target=0;commandTime=-1e9;clock=0;ref=0;last=-1;energyModel.Reset();sensors.Reset();generatorEnabled=false;coolingOk=true;sensorsOk=true;massLast=-1;ApplyFuel(ecm);}
 public:void PreUpdate(const gz::sim::UpdateInfo&info,gz::sim::EntityComponentManager&ecm)override {
  double now=std::chrono::duration<double>(info.simTime).count();clock=now;if(info.paused)return;
  double dt=std::chrono::duration<double>(info.dt).count();if(dt<=0)return;
  auto w=shaft.Velocity(ecm);auto pose=prop.WorldPose(ecm),com=prop.WorldInertialPose(ecm);auto vel=prop.WorldLinearVelocity(ecm);auto hp=hull.WorldPose(ecm);auto hv=hull.WorldLinearVelocity(ecm);
  if(!w||w->empty()||!pose||!com||!vel||!hp||!hv)return;
  double cmd=now-commandTime.load()<=timeout?target.load():0.;
  double normalized=std::abs(cmd)<=p.deadband?0.:std::copysign((std::abs(cmd)-p.deadband)/(1-p.deadband),cmd);
  ref+=(normalized*p.maxOmega-ref)*(-std::expm1(-dt/p.tau));
  auto axis=pose->Rot()*gz::math::Vector3d::UnitX;
  auto env=environment.At(pose->Pos(),now,level);
  double wet=WetFraction(level+env.height-pose->Pos().Z(),radius*std::sqrt(std::max(0.,1-axis.Z()*axis.Z())));
  auto output=PropellerForces(p,cmd,(*w)[0],ref,(*vel-env.waterVelocity).Dot(axis),wet);
  energyModel.Prepare(dt,generatorEnabled.load(),coolingOk.load());
  auto sp=steering.Position(ecm),sv=steering.Velocity(ecm);
  if(!sp||sp->empty()||!sv||sv->empty())return;
  steeringRef.Step(steeringTarget.load(),dt,steeringRate,steeringAccel,steeringLimit);
  double steeringError=steeringRef.position-(*sp)[0];
  double requestedSteering=steeringKp*steeringError-steeringKd*(*sv)[0]+steeringIntegral.torque;
  double steeringTorque=std::clamp(requestedSteering,-steeringEffort,steeringEffort);
  auto servoPower=[&](double torque){return steeringIdle+std::abs(torque*(*sv)[0])/steeringEfficiency+steeringCopper*torque*torque;};
  double servoBudget=std::min(steeringMaxW,energyModel.ActuatorBudget(dt));
  double steeringW=0;
  if(servoBudget<steeringIdle){steeringIntegral={};steeringTorque=0;steeringRef.position=(*sp)[0];steeringRef.velocity=0;}
  else {
   double lo=0,hi=1;
   for(int k=0;k<40;++k){double mid=(lo+hi)*.5;if(servoPower(steeringTorque*mid)>servoBudget)hi=mid;else lo=mid;}
   steeringTorque*=lo;steeringW=servoPower(steeringTorque);
  }
  if(servoBudget>=steeringIdle)steeringIntegral.Update(steeringError,dt,steeringKi,steeringEffort,requestedSteering,steeringTorque);
  steering.SetForce(ecm,{steeringTorque});
  output.motorTorque=energy::LimitTorque(energyModel.p,output.motorTorque,(*w)[0],energyModel.MotorBudget(dt,steeringW));
  output.shaftPower=output.motorTorque*(*w)[0];
  energyModel.Step(dt,energy::MotorElectrical(energyModel.p,output.motorTorque,(*w)[0]),steeringW);
  ApplyFuel(ecm);
  sensors.Update(now,energyModel,hp->Rot().Roll(),hp->Rot().Pitch(),sensorsOk.load() && energyModel.s.unservedW<1e-3);
  shaft.SetForce(ecm,{output.motorTorque});
  prop.AddWorldWrench(ecm,axis*output.thrust,gz::math::Vector3d::Zero);
  prop.AddWorldWrench(ecm,gz::math::Vector3d::Zero,axis*output.waterTorque);
  if(now-last>=.1){auto bodyv=hp->Rot().Inverse()*(*hv);gz::msgs::Double_V msg;
   for(double x:{now,cmd,(*w)[0]*60/(2*M_PI),ref*60/(2*M_PI),output.thrust,output.motorTorque,output.waterTorque,output.shaftPower,wet,(*vel-env.waterVelocity).Dot(axis),bodyv.X(),bodyv.Y(),hp->Pos().X(),hp->Pos().Y(),hp->Rot().Yaw(),hp->Rot().Roll(),hp->Rot().Pitch()})msg.add_data(x);
   pub.Publish(msg);
   gz::msgs::Double_V am;for(double x:{now,(*sp)[0],(*sv)[0],steeringRef.position,steeringTorque,steeringW,energyModel.s.steeringWh,double(energyModel.s.unservedW<1e-3)})am.add_data(x);actuatorPub.Publish(am);
   const auto&es=energyModel.s;const auto&ep=energyModel.p;gz::msgs::Double_V em;
   for(double x:{now,es.soc,es.voltage,es.current,es.temperature,es.fuel,es.fuelRate,es.motorW,es.auxW,es.chargerW,es.generatorW,double(es.generator),es.starterSoc,double(es.dischargeOpen),double(es.soc<ep.low_soc),double(es.fuel<ep.low_fuel_l),double(es.fault),es.batteryWh,es.motorWh,es.chargerWh,es.lossWh,es.balanceW,es.removedFuelKg,es.unservedW,energyModel.MotorBudget(dt)})em.add_data(x);
   energyPub.Publish(em);gz::msgs::Double_V sm;
   for(double x:{now,sensors.time,double(sensors.valid),sensors.voltage,sensors.current,sensors.soc,sensors.fuel,sensors.temperature,sensors.time<0?-1.:now-sensors.time})sm.add_data(x);
   sensorPub.Publish(sm);last=now;
  }
 }
 void ApplyFuel(gz::sim::EntityComponentManager&ecm){
  auto pose=hull.WorldPose(ecm);if(!pose)return;
  auto up=pose->Rot().Inverse()*gz::math::Vector3d::UnitZ;
  auto value=fuel::ToInertial(fuel::Combine(dryHull,tank.Fuel(energyModel.s.fuel,gz::math::eigen3::convert(up))));
  // ECM supplies consistent COM offsets to other PreUpdate force systems;
  // UsvPhysics commits the same value to DART before integration.
  ecm.SetComponentData<gz::sim::components::Inertial>(hull.Entity(),value);
  if(!ecm.Component<usv::physics::RuntimeInertiaCmd>(hull.Entity()))ecm.CreateComponent(hull.Entity(),usv::physics::RuntimeInertiaCmd(value));
  else ecm.SetComponentData<usv::physics::RuntimeInertiaCmd>(hull.Entity(),value);
 }
 public:gz::sim::System::PriorityType ConfigurePriority()override{return -10;}
 public:void PostUpdate(const gz::sim::UpdateInfo&info,const gz::sim::EntityComponentManager&ecm)override {
  if(info.paused)return;double now=std::chrono::duration<double>(info.simTime).count();if(now-massLast<.1)return;
  auto applied=ecm.Component<usv::physics::RuntimeInertiaApplied>(hull.Entity());
  if(!applied){if(now>1.)throw std::runtime_error("No native inertia acknowledgement: launch with libUsvPhysics.so");return;}
  auto hp=hull.WorldPose(ecm);if(!hp)return;
  const auto&actual=applied->Data();auto hm=actual.MassMatrix();auto hc=actual.Pose().Pos();auto hi=actual.Moi();
  fuel::Props ship;
  for(auto id:massLinks){gz::sim::Link l(id);auto ip=l.WorldInertialPose(ecm);auto cp=ecm.Component<gz::sim::components::Inertial>(id);if(!ip||!cp)return;
   auto rel=hp->Inverse()*(*ip);fuel::Props part;part.mass=cp->Data().MassMatrix().Mass();part.cg=gz::math::eigen3::convert(rel.Pos());
   auto R=gz::math::eigen3::convert(gz::math::Matrix3d(rel.Rot()));part.inertia=R*gz::math::eigen3::convert(cp->Data().MassMatrix().Moi())*R.transpose();ship=fuel::Combine(ship,part);}
  gz::msgs::Double_V msg;
  for(double x:{now,energyModel.s.fuel,hm.Mass(),hc.X(),hc.Y(),hc.Z(),hi(0,0),hi(1,1),hi(2,2),hi(0,1),hi(0,2),hi(1,2),ship.mass,ship.cg.x(),ship.cg.y(),ship.cg.z(),ship.inertia(0,0),ship.inertia(1,1),ship.inertia(2,2),ship.inertia(0,1),ship.inertia(0,2),ship.inertia(1,2)})msg.add_data(x);
  massPub.Publish(msg);massLast=now;
 }

};
}
GZ_ADD_PLUGIN(usv::AzimuthPropulsion,gz::sim::System,usv::AzimuthPropulsion::ISystemConfigure,usv::AzimuthPropulsion::ISystemPreUpdate,usv::AzimuthPropulsion::ISystemReset,usv::AzimuthPropulsion::ISystemPostUpdate,usv::AzimuthPropulsion::ISystemConfigurePriority)
GZ_ADD_PLUGIN_ALIAS(usv::AzimuthPropulsion,"usv::AzimuthPropulsion")
