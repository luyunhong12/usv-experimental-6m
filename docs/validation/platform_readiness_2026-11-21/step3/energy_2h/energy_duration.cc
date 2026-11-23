#include <usv_energy/EnergyModel.hh>
#include <iostream>
#include <iomanip>
int main(){usv::energy::Parameters p;
p.capacity_ah=978.0;
p.nominal_voltage_v=51.2;
p.initial_soc=0.9;
p.resistance_ohm=0.004333333333333333;
p.max_discharge_a=900.0;
p.cutoff_voltage_v=41.6;
p.charge_voltage_v=57.6;
p.reserve_soc=0.02;
p.restart_soc=0.07;
p.derate_soc=0.15;
p.low_soc=0.2;
p.charge_efficiency=0.995;
p.ambient_c=25.0;
p.initial_temperature_c=25.0;
p.thermal_capacity_j_k=410400.0;
p.thermal_conductance_w_k=45.0;
p.derate_temperature_c=45.0;
p.cutoff_temperature_c=55.0;
p.aux_output_w=186.64999999999998;
p.aux_efficiency=0.87;
p.motor_limit_w=40000.0;
p.motor_efficiency=0.84;
p.motor_idle_w=30.0;
p.motor_copper_w_per_nm2=0.05;
p.tank_capacity_l=60.0;
p.initial_fuel_l=60.0;
p.unusable_fuel_l=1.0;
p.low_fuel_l=6.0;
p.diesel_kg_l=0.84;
p.generator_ac_limit_w=4000.0;
p.fuel_idle_l_h=0.25;
p.fuel_l_kwh=0.35;
p.crank_s=3.0;
p.warmup_s=10.0;
p.generator_ramp_s=5.0;
p.charger_current_a=25.0;
p.charger_efficiency=0.9;
p.starter_capacity_ah=60.0;
p.starter_initial_soc=0.9;
p.starter_crank_a=180.0;
p.starter_min_soc=0.2;
p.starter_charge_a=7.0;
p.sensor_period_s=1.0;
p.sensor_filter_s=2.0;
p.fuel_filter_s=5.0;
p.soc_bias=0.003;
p.fuel_bias_l=0.2;
std::cout<<"generator,time_s,soc,temperature_c,motor_w,budget_w,voltage_v,current_a,fuel_l,balance_w\n"<<std::setprecision(12);
for(int gen=0;gen<2;++gen){usv::energy::Model m(p);for(int i=0;i<=72000;++i){double dt=.1;m.Prepare(dt,gen,true);double servo=std::min(8.,m.ActuatorBudget(dt));double budget=m.MotorBudget(dt,servo);m.Step(dt,std::min(40000.0,budget),servo);if(i%100==0)std::cout<<gen<<","<<i*dt<<","<<m.s.soc<<","<<m.s.temperature<<","<<m.s.motorW<<","<<budget<<","<<m.s.voltage<<","<<m.s.current<<","<<m.s.fuel<<","<<m.s.balanceW<<"\n";}}
}
