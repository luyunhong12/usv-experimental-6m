#include <usv_energy/EnergyModel.hh>
#include <iostream>
#include <iomanip>
using namespace usv::energy;
int main(){std::cout<<"case,time_s,soc,fuel_l,current_a,voltage_v,motor_w,charger_w,generator_w,temperature_c,battery_wh,loss_wh\n"<<std::setprecision(12);
 for(int scenario=0;scenario<4;++scenario){Model m;const char*names[]={"standby","motor_3000W","motor_3000W_generator","charging"};
  for(int t=0;t<=3600;++t){if(t%60==0)std::cout<<names[scenario]<<","<<t<<","<<m.s.soc<<","<<m.s.fuel<<","<<m.s.current<<","<<m.s.voltage<<","<<m.s.motorW<<","<<m.s.chargerW<<","<<m.s.generatorW<<","<<m.s.temperature<<","<<m.s.batteryWh<<","<<m.s.lossWh<<"\n";
   if(t==3600)break;m.Prepare(1,scenario>=2,true);double request=(scenario==1||scenario==2)?3000.:0.;m.Step(1,std::min(request,m.MotorBudget(1)));}
 }
}
