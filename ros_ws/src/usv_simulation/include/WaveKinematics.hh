/** @file WaveKinematics.hh
 * @brief 无人艇WaveKinematics接口与实现；工程模型边界见项目文档。
 */
#pragma once
#include <algorithm>
#include "Hydrostatics.hh"
#include <cmath>
namespace usv {

/** @brief 保存单个波分量的振幅、波数、频率和方向。
 */
struct WaveComponent {double amplitude,k,omega,dx,dy;};

/** @brief 保存同一时刻用于受力计算的波场快照。
 */
struct WaveFrame {

/** @brief 保存波面高度、坡度以及水和空气速度。
 */
struct Sample {double height=0,dx=0,dy=0;V waterVelocity{0,0,0},airVelocity{0,0,0};};
 double time=0,level=0,ramp=1,rampRate=0;
 V current{0,0,0},wind{0,0,0};std::vector<WaveComponent> components;

/** @brief 计算位置处相对于参考水位的波面高度。
 * @param p 世界坐标采样位置，m。
 */
double Height(const V&p)const{
  double z=0;for(auto&w:components)z+=w.amplitude*ramp*std::cos(w.k*(p.X()*w.dx+p.Y()*w.dy)-w.omega*time);return z;
 }

/** @brief 在给定空间位置采样波面与水气流速度。
 * @param p 世界坐标采样位置，m。
 */
Sample At(const V&p)const{
  Sample s;s.waterVelocity=current;s.airVelocity=wind;
  for(auto&w:components){
   double theta=w.k*(p.X()*w.dx+p.Y()*w.dy)-w.omega*time,c=std::cos(theta),sn=std::sin(theta),a=w.amplitude;
   s.height+=a*ramp*c;s.dx-=a*ramp*w.k*w.dx*sn;s.dy-=a*ramp*w.k*w.dy*sn;
   // Linear deep-water orbital velocity, with depth attenuation. The ramp
   // derivative preserves the free-surface kinematic condition during startup.
   double decay=std::exp(w.k*std::min(0.,p.Z()-level));
   double horizontal=a*decay*(w.omega*ramp*c-rampRate*sn);
   s.waterVelocity+=V(w.dx*horizontal,w.dy*horizontal,a*decay*(w.omega*ramp*sn+rampRate*c));
  }
  return s;
 }
};
}
