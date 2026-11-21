#include <usv_energy/EnergyModel.hh>
#include <usv_energy/Sensors.hh>
#include <iostream>
using namespace usv::energy;
void require(bool b,const char*m){if(!b)throw std::runtime_error(m);}
void tick(Model&m,double dt,double request,bool gen=false,bool cooling=true){m.Prepare(dt,gen,cooling);m.Step(dt,std::min(request,m.MotorBudget(dt)));require(std::abs(m.s.balanceW)<1e-7,"DC bus conservation");require(m.s.chargerW/m.s.voltage<=m.p.charger_current_a+1e-7,"charger current");require(m.s.current<=m.p.max_discharge_a+1e-7,"battery current");require(m.s.generatorW<=m.p.generator_ac_limit_w+1e-7,"generator cap");require(m.s.soc>=0 && m.s.soc<=1 && m.s.fuel>=0,"state bounds");}
int main(){
 Model idle;double initial=idle.s.soc;for(int i=0;i<3600;++i)tick(idle,1,0);
 require(idle.s.soc<initial && idle.s.fuel==60,"idle draws electricity, no fuel");
 Model drive,hybrid,charge;for(int i=0;i<3600;++i){tick(drive,1,3000);tick(hybrid,1,3000,true);tick(charge,1,0,true);}
 require(drive.s.soc<hybrid.s.soc && hybrid.s.soc<initial,"charger cannot sustain propulsion demand");
 require(charge.s.soc>initial && charge.s.fuel<60 && charge.s.starterSoc>.899,"charging and starter maintenance");
 require(std::abs(drive.s.batteryWh-drive.s.motorWh-idle.s.batteryWh)<2,"energy integrated in simulation seconds");
 Parameters low;low.initial_soc=.0201;Model depleted(low);for(int i=0;i<3600;++i)tick(depleted,1,6000);
 require(depleted.s.dischargeOpen && depleted.MotorBudget(1)==0,"low SOC cuts propulsion");
 for(int i=0;i<10000;++i)tick(depleted,1,0,true);
 require(depleted.s.soc>low.restart_soc && !depleted.s.dischargeOpen,"charge recovery hysteresis");
 Parameters full;full.initial_soc=1;Model topped(full);for(int i=0;i<200;++i)tick(topped,1,0,true);require(topped.s.soc<=1,"no overcharge");
 Parameters hot;hot.initial_temperature_c=56;Model overheated(hot);tick(overheated,.004,6000);require(overheated.s.motorW==0,"thermal cutoff");
 Parameters fuel;fuel.initial_fuel_l=1.001;Model empty(fuel);for(int i=0;i<120;++i)tick(empty,1,0,true);require(empty.s.generator==Fault && empty.s.fault==1 && empty.s.chargerW==0,"fuel pickup fault");
 Model cooling;for(int i=0;i<30;++i)tick(cooling,1,0,true);tick(cooling,1,0,true,false);require(cooling.s.generator==Fault && cooling.s.generatorW==0,"cooling interlock");
 Parameters starter;starter.starter_initial_soc=.1;Model noStart(starter);tick(noStart,1,0,true);require(noStart.s.generator==Fault && noStart.s.fault==3,"weak starter");
 Model servo;servo.Prepare(.004,false,true);double budget=servo.MotorBudget(.004,150);servo.Step(.004,std::min(6000.,budget),150);
 require(servo.s.steeringW==150 && servo.s.steeringWh>0 && std::abs(servo.s.balanceW)<1e-7,"steering DC bus accounting");
 Parameters off;off.initial_soc=.02;Model dead(off);dead.Prepare(.004,false,true);require(dead.ActuatorBudget(.004)==0,"steering off after BMS cutoff");
 Model brake;require(MotorElectrical(brake.p,-30,100)>0,"braking is not free charge");
 for(double w:{-160.,0.,160.})for(double t:{-60.,60.})for(double budget:{0.,20.,100.,6000.})require(MotorElectrical(brake.p,LimitTorque(brake.p,t,w,budget),w)<=budget+1e-7,"torque electrical budget");
 Sensors sensor;sensor.Update(0,drive,0,0,true);double before=sensor.soc;sensor.Update(2,drive,0,0,false);require(!sensor.valid && sensor.time==0 && sensor.soc==before,"stale sensor validity");sensor.Reset();require(!sensor.initialized,"sensor reset");
 drive.Reset();require(drive.s.soc==initial && drive.s.batteryWh==0 && drive.s.generator==Off,"reset clears state");
 std::cout<<"PASS conservation, charge/discharge, reserve, thermal, fuel, starter, cooling, sensor, reset\n";
}
