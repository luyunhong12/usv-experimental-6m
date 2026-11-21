/** @file EnergyModel.hh
 * @brief 无人艇EnergyModel接口与实现；工程模型边界见项目文档。
 */
#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace usv::energy {

/** @brief 能源模型参数与输入约束。
 */
struct Parameters {
#define P(name,value) double name=value;
#include "Parameters.def"
#undef P

/** @brief 检查能源参数是否有限并满足现有取值约束。
 */
void Validate() const {
#define P(name,value) if(!std::isfinite(name))throw std::runtime_error("Nonfinite energy parameter: " #name);
#include "Parameters.def"
#undef P
 if (!(ValidBatterySupply() && ValidChargeLevels() && ValidThermalAuxiliary() && ValidMotor() && ValidFuel() && ValidGeneratorCharger() && ValidStarter() && ValidSensors()))
  throw std::runtime_error("Invalid energy parameters");
 }
 private:
 /** @brief 检查容量、电压、电流与初始电量约束。 */
 bool ValidBatterySupply() const {
  return capacity_ah>0 && nominal_voltage_v>0 && resistance_ohm>0 && max_discharge_a>0 && cutoff_voltage_v>0 && charge_voltage_v>cutoff_voltage_v && initial_soc>=0 && initial_soc<=1;
 }
 /** @brief 检查电量阈值顺序及充电效率。 */
 bool ValidChargeLevels() const {
  return reserve_soc>=0 && reserve_soc<restart_soc && restart_soc<derate_soc && derate_soc<low_soc && low_soc<1 && charge_efficiency>0 && charge_efficiency<=1;
 }
 /** @brief 检查热保护与辅助电源约束。 */
 bool ValidThermalAuxiliary() const {
  return thermal_capacity_j_k>0 && thermal_conductance_w_k>=0 && derate_temperature_c<cutoff_temperature_c && aux_output_w>=0 && aux_efficiency>0 && aux_efficiency<=1;
 }
 /** @brief 检查电机功率、效率及损耗约束。 */
 bool ValidMotor() const {
  return motor_limit_w>0 && motor_efficiency>0 && motor_efficiency<=1 && motor_idle_w>=0 && motor_copper_w_per_nm2>=0;
 }
 /** @brief 检查油箱容量、油量阈值及密度。 */
 bool ValidFuel() const {
  return tank_capacity_l>0 && initial_fuel_l>=0 && initial_fuel_l<=tank_capacity_l && unusable_fuel_l>=0 && unusable_fuel_l<low_fuel_l && low_fuel_l<tank_capacity_l && diesel_kg_l>0;
 }
 /** @brief 检查发电机阶段与充电器约束。 */
 bool ValidGeneratorCharger() const {
  return generator_ac_limit_w>0 && fuel_idle_l_h>0 && fuel_l_kwh>0 && crank_s>0 && warmup_s>0 && generator_ramp_s>0 && charger_current_a>0 && charger_efficiency>0 && charger_efficiency<=1;
 }
 /** @brief 检查启动电池的容量、电流和电量范围。 */
 bool ValidStarter() const {
  return starter_capacity_ah>0 && starter_initial_soc>=0 && starter_initial_soc<=1 && starter_crank_a>0 && starter_min_soc>=0 && starter_min_soc<1 && starter_charge_a>0;
 }
 /** @brief 检查各采样与滤波周期为正。 */
 bool ValidSensors() const {
  return sensor_period_s>0 && sensor_filter_s>0 && fuel_filter_s>0;
 }

};

/** @brief 发电机状态枚举。
 */
enum Generator {Off=0,Starting=1,Warming=2,Running=3,Fault=4};

/** @brief 能源模型的当前状态与累计能量。
 */
struct State {
 double soc=0,fuel=0,temperature=0,starterSoc=0,voltage=0,current=0;
 double steeringW=0,steeringWh=0;
 double motorW=0,auxW=0,unservedW=0,chargerW=0,generatorW=0,fuelRate=0;
 double batteryWh=0,motorWh=0,chargerWh=0,lossWh=0,balanceW=0,removedFuelKg=0;
 bool dischargeOpen=false;int generator=Off,fault=0;
};

/** @brief 耦合电池、负载、充电与燃油消耗的工程模型。
 */
class Model {
 public:Parameters p;State s;double stageTime=0,runFraction=0;

/** @brief 耦合电池、负载、充电与燃油消耗的工程模型。
 * @param config 经过约束校验的能源参数。
 */
explicit Model(Parameters config={}):p(config){p.Validate();Reset();}

/** @brief 将模型或插件状态恢复到配置规定的初始状态。
 */
void Reset(){s={};s.soc=p.initial_soc;s.fuel=p.initial_fuel_l;s.temperature=p.initial_temperature_c;s.starterSoc=p.starter_initial_soc;s.voltage=OCV();stageTime=runFraction=0;}
 // Illustrative LiFePO4 OCV curve, not a measured E163 cell curve.

/** @brief 按现有分段曲线计算开路电压，不代表厂家实测曲线。
 */
double OCV()const {
  constexpr double x[]={0,.05,.10,.20,.50,.90,1.0},y[]={44.8,48,50,51.2,52.4,53.6,54.4};
  for(int i=1;i<7;++i)if(s.soc<=x[i])return y[i-1]+(y[i]-y[i-1])*(s.soc-x[i-1])/(x[i]-x[i-1]);
  return y[6];
 }

/** @brief 更新发电机阶段、供电保护与当前输出比例。
 * @param dt 仿真步长，s。
 * @param enable 是否请求启用发电机。
 * @param cooling 冷却系统是否正常。
 */
void Prepare(double dt,bool enable,bool cooling) {
  if(!(dt>0 && std::isfinite(dt)))throw std::runtime_error("Invalid energy timestep");
  UpdateDischargeProtection();
  UpdateGeneratorRequest(enable,cooling);
  AdvanceGeneratorStage(dt);
  runFraction=s.generator==Running?std::clamp(stageTime/p.generator_ramp_s,0.,1.):0.;
  // Conservative last fuel step prevents sustained generation below the pickup reserve.
  double maxRate=std::max(p.fuel_idle_l_h,p.fuel_l_kwh*p.generator_ac_limit_w/1000);
  runFraction*=std::clamp((s.fuel-p.unusable_fuel_l)/(maxRate*dt/3600),0.,1.);
 }

/** @brief 计算当前步长可用的最大放电电流。
 * @param dt 仿真步长，s。
 */
double MaxDischarge(double dt)const {
  if(s.dischargeOpen)return 0;
  return std::max(0.,std::min({p.max_discharge_a,(OCV()-p.cutoff_voltage_v)/p.resistance_ohm,OCV()/(2*p.resistance_ohm), (s.soc-p.reserve_soc)*p.capacity_ah*3600/dt}));
 }

/** @brief 计算当前发电条件下的充电器电流。
 */
double ChargerCurrent()const {return runFraction*std::min(p.charger_current_a,std::max(0.,p.generator_ac_limit_w-120.)*p.charger_efficiency/p.charge_voltage_v);}

/** @brief 计算扣除辅助负载后的执行器电功率预算。
 * @param dt 仿真步长，s。
 */
double ActuatorBudget(double dt)const {
  if(s.dischargeOpen)return 0;
  double i=MaxDischarge(dt),v=OCV()-i*p.resistance_ohm;
  return std::max(0.,v*(i+ChargerCurrent())-p.aux_output_w/p.aux_efficiency);
 }

/** @brief 计算扣除转向功率后的电机功率预算。
 * @param dt 仿真步长，s。
 * @param steeringW 转向执行器电功率，W。
 */
double MotorBudget(double dt,double steeringW=0)const {
  if(s.dischargeOpen)return 0; // Engineering policy: propulsion stays off until SOC recovery.
  double i=MaxDischarge(dt),v=OCV()-i*p.resistance_ohm;
  double socScale=std::clamp((s.soc-p.reserve_soc)/(p.derate_soc-p.reserve_soc),0.,1.);
  double thermal=std::clamp((p.cutoff_temperature_c-s.temperature)/(p.cutoff_temperature_c-p.derate_temperature_c),0.,1.);
  return std::max(0.,std::min({p.motor_limit_w,p.motor_limit_w*socScale*thermal,v*(i+ChargerCurrent())-p.aux_output_w/p.aux_efficiency-steeringW}));
 }

/** @brief 推进能源状态并累计能量和燃油消耗。
 * @param dt 仿真步长，s。
 * @param motorW 电机电输入功率请求，W。
 * @param steeringW 转向执行器电功率，W。
 */
void Step(double dt,double motorW,double steeringW=0) {
  ValidateDemands(dt,motorW,steeringW);
  double e=OCV(),r=p.resistance_ohm,imax=MaxDischarge(dt),ic=ChargerCurrent();
  double demand=motorW+steeringW+p.aux_output_w/p.aux_efficiency;
  s.steeringW=steeringW;s.steeringWh+=steeringW*dt/3600;
  double vmin=e-r*imax;
  double load=std::min(demand,vmin*(imax+ic));
  double b=e+r*ic,v=.5*(b+std::sqrt(std::max(0.,b*b-4*r*load)));
  double i=(e-v)/r;
  double chargeAccept=std::min({p.max_discharge_a,std::max(0.,(p.charge_voltage_v-e)/r),std::max(0.,(1-s.soc)*p.capacity_ah*3600/(dt*p.charge_efficiency)),p.charger_current_a*std::clamp((1-s.soc)/.10,0.,1.)});
  if(s.temperature<0 || s.temperature>=55)chargeAccept=0;
  if(i< -chargeAccept){i=-chargeAccept;v=e-r*i;ic=(load/v)-i;}
  s.voltage=v;s.current=i;s.motorW=motorW;s.auxW=std::max(0.,load-motorW);s.unservedW=demand-load;s.chargerW=v*ic;
  double starterI=s.generator==Running?runFraction*p.starter_charge_a*std::clamp((1-s.starterSoc)/.1,0.,1.):0;
  starterI=std::min(starterI,(1-s.starterSoc)*p.starter_capacity_ah*3600/dt);
  s.starterSoc=std::clamp(s.starterSoc+starterI*dt/(3600*p.starter_capacity_ah),0.,1.);
  s.generatorW=s.chargerW/p.charger_efficiency+12.8*starterI/.85;
  s.fuelRate=(s.generator==Warming || s.generator==Running)?std::max(p.fuel_idle_l_h,p.fuel_l_kwh*s.generatorW/1000):0;
  double used=std::min(std::max(0.,s.fuel-p.unusable_fuel_l),s.fuelRate*dt/3600);
  s.fuel=std::max(0.,s.fuel-used);s.removedFuelKg=(p.initial_fuel_l-s.fuel)*p.diesel_kg_l;
  s.balanceW=v*i+s.chargerW-s.motorW-s.auxW;
  s.batteryWh+=v*i*dt/3600;s.motorWh+=motorW*dt/3600;s.chargerWh+=s.chargerW*dt/3600;s.lossWh+=i*i*r*dt/3600;
  s.soc=std::clamp(s.soc-i*(i<0?p.charge_efficiency:1.)*dt/(3600*p.capacity_ah),0.,1.);
  s.temperature+=(i*i*r-p.thermal_conductance_w_k*(s.temperature-p.ambient_c))*dt/p.thermal_capacity_j_k;
 }
 private:
/** @brief 按当前电量与温度更新放电保护的滞回状态。 */
void UpdateDischargeProtection() {
  if(s.soc<=p.reserve_soc+1e-12 || s.temperature>=p.cutoff_temperature_c || s.temperature< -10)s.dischargeOpen=true;
  else if(s.soc>=p.restart_soc && s.temperature<p.derate_temperature_c && s.temperature>=0)s.dischargeOpen=false;
}
/** @brief 处理发电机启停请求及冷却、燃油和启动电池故障。
 * @param enable 是否请求启用发电机。
 * @param cooling 冷却系统是否正常。
 */
void UpdateGeneratorRequest(bool enable,bool cooling) {
  if(!enable){s.generator=Off;s.fault=0;stageTime=0;}
  else if(!cooling || s.fuel<=p.unusable_fuel_l){s.generator=Fault;s.fault=!cooling?2:1;}
  else if(s.generator==Off){
   if(s.starterSoc<p.starter_min_soc){s.generator=Fault;s.fault=3;}
   else{s.generator=Starting;stageTime=0;}
  }
}
/** @brief 推进启动、预热和运行阶段，保持原有阶段切换顺序。
 * @param dt 仿真步长，s。
 */
void AdvanceGeneratorStage(double dt) {
  if(s.generator==Starting){
   double used=std::min(dt,std::max(0.,p.crank_s-stageTime));
   s.starterSoc=std::max(0.,s.starterSoc-p.starter_crank_a*used/(3600*p.starter_capacity_ah));
   stageTime+=dt;if(s.starterSoc<p.starter_min_soc){s.generator=Fault;s.fault=3;}
   else if(stageTime>=p.crank_s){s.generator=Warming;stageTime=0;}
  }else if(s.generator==Warming){stageTime+=dt;if(stageTime>=p.warmup_s){s.generator=Running;stageTime=0;}}
  else if(s.generator==Running)stageTime+=dt;
}
/** @brief 在更新能源状态前校验执行器需求是否超出预算。
 * @param dt 仿真步长，s。
 * @param motorW 电机电输入功率请求，W。
 * @param steeringW 转向执行器电功率请求，W。
 */
void ValidateDemands(double dt,double motorW,double steeringW) const {
  if(!std::isfinite(steeringW)||steeringW<0||steeringW>ActuatorBudget(dt)+1e-5)throw std::runtime_error("Steering demand exceeds allocated energy");
  if(!std::isfinite(motorW)||motorW<0||motorW>MotorBudget(dt,steeringW)+1e-5)throw std::runtime_error("Motor demand exceeds allocated energy");
}

};
// Input losses are positive even during shaft braking; no unvalidated regeneration.

/** @brief 按转矩和转速计算工程电机电功率需求。
 * @param p 能源模型的电机参数。
 * @param torque 转矩请求，N·m。
 * @param omega 轴角速度，rad/s。
 */
inline double MotorElectrical(const Parameters&p,double torque,double omega) {
 if(std::abs(torque)<1e-12)return 0;
 return std::max(0.,torque*omega)/p.motor_efficiency+p.motor_idle_w+p.motor_copper_w_per_nm2*torque*torque;
}

/** @brief 将转矩请求约束到可用电功率预算。
 * @param p 能源模型的电机参数。
 * @param torque 转矩请求，N·m。
 * @param omega 轴角速度，rad/s。
 * @param budget 可用电功率预算，W。
 */
inline double LimitTorque(const Parameters&p,double torque,double omega,double budget) {
 if(MotorElectrical(p,torque,omega)<=budget)return torque;
 if(budget<=p.motor_idle_w)return 0;
 double lo=0,hi=1;
 for(int k=0;k<45;++k){double mid=(lo+hi)/2;if(MotorElectrical(p,torque*mid,omega)>budget)hi=mid;else lo=mid;}
 return torque*lo;
}
}
