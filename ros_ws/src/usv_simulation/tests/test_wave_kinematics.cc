/** @file test_wave_kinematics.cc
 * @brief 当前模块的解析回归验证。
 */
#include "WaveKinematics.hh"
#include <stdexcept>
#include <iostream>
using namespace usv;

/** @brief 验证两个解析结果在规定容差内一致。
 * @param a 实际计算结果。
 * @param b 解析参考结果。
 * @param tol 比较允许的绝对误差。
 */
void close(double a,double b,double tol=1e-7){if(std::abs(a-b)>tol)throw std::runtime_error("Wave kinematics consistency failed");}

/** @brief 执行该文件定义的解析或场景回归验证。
 */
int main(){
 WaveFrame f;f.time=1.7;f.ramp=.8;f.rampRate=.1;f.components.push_back({.2,.5,std::sqrt(9.81*.5),.6,.8});V p(2,-1,0);auto s=f.At(p);double h=1e-5;
 close(s.height,f.Height(p));close(s.dx,(f.Height(p+V(h,0,0))-f.Height(p-V(h,0,0)))/(2*h));
 auto before=f,after=f;before.time-=h;after.time+=h;before.ramp-=h*f.rampRate;after.ramp+=h*f.rampRate;
 close(s.waterVelocity.Z(),(after.Height(p)-before.Height(p))/(2*h));
 auto deep=f.At(p+V(0,0,-2));close(deep.waterVelocity.Length()/s.waterVelocity.Length(),std::exp(-1));
 p.Z()=-1;
 double div=(f.At(p+V(h,0,0)).waterVelocity.X()-f.At(p-V(h,0,0)).waterVelocity.X()+f.At(p+V(0,h,0)).waterVelocity.Y()-f.At(p-V(0,h,0)).waterVelocity.Y()+f.At(p+V(0,0,h)).waterVelocity.Z()-f.At(p-V(0,0,h)).waterVelocity.Z())/(2*h);close(div,0);
 WaveFrame calm;calm.current=V(.2,.1,0);calm.wind=V(0,8,0);auto c=calm.At(p);close(c.height,0);close((c.waterVelocity-calm.current).Length(),0);
 std::cout<<"Shared surface height/slopes, startup derivative, orbital depth decay, divergence and calm current passed\n";
}
