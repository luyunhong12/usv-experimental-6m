/** @file Environment.hh
 * @brief 无人艇Environment接口与实现；工程模型边界见项目文档。
 */
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

/** @brief 复用 VRX 波谱并缓存流场和风场状态。
 */
class Environment {
 vrx::Wavefield waves;
 gz::transport::Node node;
 std::mutex mutex;bool unsupported=false;
 gz::math::Vector3d current{0,0,0},wind{0,0,0};
 public:

/** @brief 读取插件配置并建立所需实体引用与通信接口。
 * @param s 插件 SDF 配置元素。
 */
void Configure(const std::shared_ptr<const sdf::Element>&s) {
  waves.Load(s);
  if(waves.Steepness()!=0 || waves.Phase()!=0)throw std::runtime_error("Only zero-steepness, zero-phase VRX waves are supported");
  node.Subscribe(waves.Topic(),&Environment::OnWave,this);
  node.Subscribe("/vrx/current/velocity",&Environment::OnCurrent,this);
  node.Subscribe("/vrx/wind/velocity",&Environment::OnWind,this);
 }

/** @brief 校验并更新波场参数，拒绝当前模型不支持的设置。
 * @param m VRX 波场参数消息。
 */
void OnWave(const gz::msgs::Param&m) {
  std::lock_guard<std::mutex> lock(mutex);
  if(UnsupportedWaveShape(m)){unsupported=true;return;}
  unsupported=false;
  if(!ValidWaveValues(m))return;
  waves.Load(m);
 }

/** @brief 接收有限的流速向量并更新缓存。
 * @param m 世界坐标水流速度消息，m/s。
 */
void OnCurrent(const gz::msgs::Vector3d&m) {std::lock_guard<std::mutex> lock(mutex);if(std::isfinite(m.x()+m.y()+m.z()))current.Set(m.x(),m.y(),m.z());}

/** @brief 接收有限的风速向量并更新缓存。
 * @param m 世界坐标风速消息，m/s。
 */
void OnWind(const gz::msgs::Vector3d&m) {std::lock_guard<std::mutex> lock(mutex);if(std::isfinite(m.x()+m.y()+m.z()))wind.Set(m.x(),m.y(),m.z());}
 using Sample=WaveFrame::Sample;

/** @brief 创建给定时刻的波场、风流与启动渐变快照。
 * @param t 采样时刻，s。
 * @param level 参考静水面高度，m。
 */
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

/** @brief 在给定空间位置采样波面与水气流速度。
 * @param p 世界坐标采样位置，m。
 * @param t 采样时刻，s。
 * @param level 参考静水面高度，m。
 */
Sample At(const V&p,double t,double level=0) {return Snapshot(t,level).At(p);}

 private:
/** @brief 判断消息是否要求当前近似不支持的波陡或相位。
 * @param m VRX 波场参数消息。
 */
static bool UnsupportedWaveShape(const gz::msgs::Param&m) {
 const auto& p=m.params();
 return (p.count("steepness") && p.at("steepness").double_value()!=0) || (p.count("phase") && p.at("phase").double_value()!=0);
}
/** @brief 检查消息中的浮点值、周期和波增益范围。
 * @param m VRX 波场参数消息。
 */
static bool ValidWaveValues(const gz::msgs::Param&m) {
 const auto& p=m.params();
 for(const auto&kv:p) if(kv.second.type()==gz::msgs::Any::DOUBLE && !std::isfinite(kv.second.double_value()))return false;
 if(p.count("period") && p.at("period").double_value()<=0)return false;
 if(p.count("gain") && p.at("gain").double_value()<0)return false;
 return true;
}

};

/** @brief 校验并构造船舶消息主题前缀。
 * @param s 包含 topic_namespace 的插件配置。
 */
inline std::string TopicPrefix(const std::shared_ptr<const sdf::Element>&s) {
 auto value=s->Get<std::string>("topic_namespace","usv").first;
 if(value.empty()||value.front()=='/'||value.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_")!=std::string::npos)throw std::runtime_error("Invalid boat topic_namespace");
 return "/"+value;
}
}
