#pragma once
#include "Wavefield.hh"
#include "WaveKinematics.hh"
#include <gz/transport/Node.hh>
#include <gz/msgs/vector3d.pb.h>
#include <mutex>
#include <stdexcept>
namespace usv {
// Reuse the SAME VRX spectrum and phase as the visual water. Only zero
// steepness is supported by the distributed-pressure wave approximation.
class Environment {
 vrx::Wavefield waves;
 gz::transport::Node node;
 std::mutex mutex;bool unsupported=false;
 gz::math::Vector3d current{0,0,0},wind{0,0,0};
 public:
 void Configure(const std::shared_ptr<const sdf::Element>&s) {
  waves.Load(s);
  if(waves.Steepness()!=0 || waves.Phase()!=0)throw std::runtime_error("Only zero-steepness, zero-phase VRX waves are supported");
  node.Subscribe(waves.Topic(),&Environment::OnWave,this);
  node.Subscribe("/vrx/current/velocity",&Environment::OnCurrent,this);
  node.Subscribe("/vrx/wind/velocity",&Environment::OnWind,this);
 }
 void OnWave(const gz::msgs::Param&m) {
  std::lock_guard<std::mutex> lock(mutex);
  const auto& p=m.params();
  if((p.count("steepness") && p.at("steepness").double_value()!=0) || (p.count("phase") && p.at("phase").double_value()!=0)){unsupported=true;return;}
  unsupported=false;
  for(const auto&kv:p) if(kv.second.type()==gz::msgs::Any::DOUBLE && !std::isfinite(kv.second.double_value()))return;
  if(p.count("period") && p.at("period").double_value()<=0)return;
  if(p.count("gain") && p.at("gain").double_value()<0)return;
  waves.Load(m);
 }
 void OnCurrent(const gz::msgs::Vector3d&m) {std::lock_guard<std::mutex> lock(mutex);if(std::isfinite(m.x()+m.y()+m.z()))current.Set(m.x(),m.y(),m.z());}
 void OnWind(const gz::msgs::Vector3d&m) {std::lock_guard<std::mutex> lock(mutex);if(std::isfinite(m.x()+m.y()+m.z()))wind.Set(m.x(),m.y(),m.z());}
 using Sample=WaveFrame::Sample;
 WaveFrame Snapshot(double t,double level=0) {
  std::lock_guard<std::mutex> lock(mutex);
  if(unsupported)throw std::runtime_error("Unsupported nonzero steepness/phase: refusing visual/physical mismatch");
  WaveFrame f;f.time=t;f.level=level;f.current=current;f.wind=wind;
  if(!waves.Active())return f;
  double tau=waves.Tau();if(!(tau>0))throw std::runtime_error("Wave startup time must be positive");
  double decay=std::exp(-std::max(0.,t)/tau);f.ramp=1-decay;f.rampRate=decay/tau;
  for(size_t i=0;i<waves.Amplitude_V().size();++i)f.components.push_back({waves.Amplitude_V()[i],waves.Wavenumber_V()[i],waves.AngularFrequency_V()[i],waves.Direction_V()[i].X(),waves.Direction_V()[i].Y()});
  return f;
 }
 Sample At(const V&p,double t,double level=0) {return Snapshot(t,level).At(p);}

};
inline std::string TopicPrefix(const std::shared_ptr<const sdf::Element>&s) {
 auto value=s->Get<std::string>("topic_namespace","usv").first;
 if(value.empty()||value.front()=='/'||value.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_")!=std::string::npos)throw std::runtime_error("Invalid boat topic_namespace");
 return "/"+value;
}
}
