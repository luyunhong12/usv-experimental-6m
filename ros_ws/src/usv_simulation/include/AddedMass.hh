/** @file AddedMass.hh
 * @brief 正对角附加质量与斜对称耦合力矩，不将流体惯性计入刚体重力。
 */
#pragma once
#include "Hydrostatics.hh"
namespace usv {
/** @brief 保存附加质量产生的力与力矩。
 */
struct AddedWrench {V force,torque;};
// Positive diagonal fluid inertia at a fixed body point. C_A is skew-symmetric:
// nu dot C_A(nu) nu = 0. Coefficients are NOT rigid mass or gravity loads.
/** @brief 计算固定体坐标参考点上的附加质量广义力。
 * @param m 三轴平动附加质量，kg。
 * @param j 三轴转动附加惯性，kg·m²。
 * @param v 体坐标相对线速度，m/s。
 * @param w 体坐标角速度，rad/s。
 * @param dv 体坐标线加速度分量，m/s²。
 * @param dw 体坐标角加速度分量，rad/s²。
 */
inline AddedWrench AddedMassWrench(const V&m,const V&j,const V&v,const V&w,const V&dv,const V&dw){
 V p(m.X()*v.X(),m.Y()*v.Y(),m.Z()*v.Z()),h(j.X()*w.X(),j.Y()*w.Y(),j.Z()*w.Z());
 return {-V(m.X()*dv.X(),m.Y()*dv.Y(),m.Z()*dv.Z())-w.Cross(p),
         -V(j.X()*dw.X(),j.Y()*dw.Y(),j.Z()*dw.Z())-v.Cross(p)-w.Cross(h)};
}
}
