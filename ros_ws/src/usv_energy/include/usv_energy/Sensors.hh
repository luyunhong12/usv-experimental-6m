/** @file Sensors.hh
 * @brief 无人艇Sensors接口与实现；工程模型边界见项目文档。
 */
#pragma once
#include "EnergyModel.hh"
#include <random>
namespace usv::energy {

/** @brief 模拟能源传感器采样、滤波与显式误差假设。
 */
class Sensors {
 std::mt19937 random{20261119};std::normal_distribution<double> normal{0,1};double lastAttempt=-1,lastFilter=0;
 public:double time=-1,voltage=0,current=0,soc=0,fuel=0,temperature=0;bool valid=false,initialized=false;

/** @brief 将模型或插件状态恢复到配置规定的初始状态。
 */
void Reset(){*this=Sensors{};}

/** @brief 按采样周期更新能源传感器状态。
 * @param now 当前仿真时间，s。
 * @param m 用于读取真值与参数的能源模型。
 * @param roll 横摇角，rad。
 * @param pitch 纵摇角，rad。
 * @param healthy 传感器健康输入。
 */
void Update(double now,const Model&m,double roll,double pitch,bool healthy){
  if(lastAttempt>=0 && now-lastAttempt<m.p.sensor_period_s)return;
  lastAttempt=now;valid=healthy && m.s.auxW>=1;
  if(!valid)return;
  double dt=initialized?now-lastFilter:m.p.sensor_period_s;
  double a=initialized?-std::expm1(-dt/m.p.sensor_filter_s):1.;
  double af=initialized?-std::expm1(-dt/m.p.fuel_filter_s):1.;
  // Attitude sensitivity is an explicitly uncalibrated sender proxy, not fluid slosh CFD.
  double targetFuel=std::clamp(m.s.fuel+m.p.fuel_bias_l+.5*std::sin(roll)+.8*std::sin(pitch)+.1*normal(random),0.,m.p.tank_capacity_l);
  voltage+=a*(m.s.voltage+.05*normal(random)-voltage);
  current+=a*(m.s.current+.2*normal(random)-current);
  soc=std::clamp(soc+a*(m.s.soc+m.p.soc_bias+.002*normal(random)-soc),0.,1.);
  fuel+=af*(targetFuel-fuel);temperature+=a*(m.s.temperature+.1*normal(random)-temperature);
  time=lastFilter=now;initialized=true;
 }
};
}
