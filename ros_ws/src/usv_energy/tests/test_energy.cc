/** @file test_energy.cc
 * @brief 当前模块的解析回归验证。
 */
#include <usv_energy/EnergyModel.hh>
#include <usv_energy/Sensors.hh>
#include <iostream>
using namespace usv::energy;

/** @brief 校验测试判据，失败时报告错误。
 * @param b 应为真的测试判据。
 * @param m 测试失败时的说明文本。
 */
void require(bool b,const char*m){if(!b)throw std::runtime_error(m);}

/** @brief 按现有负载与发电条件推进能源测试模型。
 * @param m 需要推进的能源模型。
 * @param dt 仿真步长，s。
 * @param request 测试请求的电机电功率，W。
 * @param gen 是否启用发电机。
 * @param cooling 冷却系统是否正常。
 */
void tick(Model&m,double dt,double request,bool gen=false,bool cooling=true){m.Prepare(dt,gen,cooling);m.Step(dt,std::min(request,m.MotorBudget(dt)));require(std::abs(m.s.balanceW)<1e-7,"DC bus conservation");require(m.s.chargerW/m.s.voltage<=m.p.charger_current_a+1e-7,"charger current");require(m.s.current<=m.p.max_discharge_a+1e-7,"battery current");require(m.s.generatorW<=m.p.generator_ac_limit_w+1e-7,"generator cap");require(m.s.soc>=0 && m.s.soc<=1 && m.s.fuel>=0,"state bounds");}

/** @brief 验证传感器过期状态和能源复位。
 * @param drive 已运行的能源模型。
 * @param initial 初始电量比例。
 */
void CheckSensorAndReset(Model&drive,double initial) {
 Sensors sensor;sensor.Update(0,drive,0,0,true);double before=sensor.soc;sensor.Update(2,drive,0,0,false);require(!sensor.valid && sensor.time==0 && sensor.soc==before,"stale sensor validity");sensor.Reset();require(!sensor.initialized,"sensor reset");
 drive.Reset();require(drive.s.soc==initial && drive.s.batteryWh==0 && drive.s.generator==Off,"reset clears state");
}

/** @brief 验证充放电、累计能量、传感器失效和状态复位。 */
void CheckEnergyAndSensors(){
 Model idle;double initial=idle.s.soc;for(int i=0;i<3600;++i)tick(idle,1,0);
 require(idle.s.soc<initial && idle.s.fuel==60,"idle draws electricity, no fuel");
 Model drive,hybrid,charge;for(int i=0;i<3600;++i){tick(drive,1,3000);tick(hybrid,1,3000,true);tick(charge,1,0,true);}
 require(drive.s.soc<hybrid.s.soc && hybrid.s.soc<initial,"charger cannot sustain propulsion demand");
 require(charge.s.soc>initial && charge.s.fuel<60 && charge.s.starterSoc>.899,"charging and starter maintenance");
 require(std::abs(drive.s.batteryWh-drive.s.motorWh-idle.s.batteryWh)<2,"energy integrated in simulation seconds");
 CheckSensorAndReset(drive,initial);
}

/** @brief 验证储备电量保护及充电恢复滞回。 */
void CheckReserveRecovery(){
 Parameters low;low.initial_soc=.0201;Model depleted(low);for(int i=0;i<3600;++i)tick(depleted,1,6000);
 require(depleted.s.dischargeOpen && depleted.MotorBudget(1)==0,"low SOC cuts propulsion");
 for(int i=0;i<10000;++i)tick(depleted,1,0,true);
 require(depleted.s.soc>low.restart_soc && !depleted.s.dischargeOpen,"charge recovery hysteresis");
}

/** @brief 验证满电时不会过充。 */
void CheckFullCharge(){
 Parameters full;full.initial_soc=1;Model topped(full);for(int i=0;i<200;++i)tick(topped,1,0,true);require(topped.s.soc<=1,"no overcharge");
}

/** @brief 验证过热时推进功率被切断。 */
void CheckThermalCutoff(){
 Parameters hot;hot.initial_temperature_c=56;Model overheated(hot);tick(overheated,.004,6000);require(overheated.s.motorW==0,"thermal cutoff");
}

/** @brief 验证燃油吸取储备触发故障。 */
void CheckFuelPickup(){
 Parameters fuel;fuel.initial_fuel_l=1.001;Model empty(fuel);for(int i=0;i<120;++i)tick(empty,1,0,true);require(empty.s.generator==Fault && empty.s.fault==1 && empty.s.chargerW==0,"fuel pickup fault");
}

/** @brief 验证冷却失效触发停机。 */
void CheckCoolingFault(){
 Model cooling;for(int i=0;i<30;++i)tick(cooling,1,0,true);tick(cooling,1,0,true,false);require(cooling.s.generator==Fault && cooling.s.generatorW==0,"cooling interlock");
}

/** @brief 验证启动电池电量不足时拒绝启动。 */
void CheckStarterFault(){
 Parameters starter;starter.starter_initial_soc=.1;Model noStart(starter);tick(noStart,1,0,true);require(noStart.s.generator==Fault && noStart.s.fault==3,"weak starter");
}

/** @brief 验证转向电功率计入直流母线收支。 */
void CheckSteeringAccounting(){
 Model servo;servo.Prepare(.004,false,true);double budget=servo.MotorBudget(.004,150);servo.Step(.004,std::min(6000.,budget),150);
 require(servo.s.steeringW==150 && servo.s.steeringWh>0 && std::abs(servo.s.balanceW)<1e-7,"steering DC bus accounting");
}

/** @brief 验证放电保护切断转向功率。 */
void CheckSteeringCutoff(){
 Parameters off;off.initial_soc=.02;Model dead(off);dead.Prepare(.004,false,true);require(dead.ActuatorBudget(.004)==0,"steering off after BMS cutoff");
}

/** @brief 验证制动损耗与各转速转矩下的功率预算。 */
void CheckTorqueBudget(){
 Model brake;require(MotorElectrical(brake.p,-30,100)>0,"braking is not free charge");
 for(double w:{-160.,0.,160.})for(double t:{-60.,60.})for(double budget:{0.,20.,100.,6000.})require(MotorElectrical(brake.p,LimitTorque(brake.p,t,w,budget),w)<=budget+1e-7,"torque electrical budget");
}

/** @brief 执行全部能源守恒与故障场景。 */
int main(){
 CheckEnergyAndSensors();
 CheckReserveRecovery();
 CheckFullCharge();
 CheckThermalCutoff();
 CheckFuelPickup();
 CheckCoolingFault();
 CheckStarterFault();
 CheckSteeringAccounting();
 CheckSteeringCutoff();
 CheckTorqueBudget();
 std::cout<<"PASS conservation, charge/discharge, reserve, thermal, fuel, starter, cooling, sensor, reset\n";
}
