/** @file AddedMass.hh
 * @brief 对称附加惯性与斜对称科氏力矩，不将流体惯性计入刚体重力。
 */
#pragma once
#include "Hydrostatics.hh"
namespace usv {
/** @brief 保存附加质量产生的力与力矩。
 */
struct AddedWrench {
  V force, torque;
};
// 固定体坐标点上的对称流体惯性，允许横移/偏航的一阶矩。
// 调用入口校验正半定性；科氏项保持功率为零，不作为刚体重力载荷。
/** @brief 计算固定体坐标参考点上的附加质量广义力。
 * @param m 三轴平动附加质量，kg。
 * @param j 三轴转动附加惯性，kg·m²。
 * @param v 体坐标相对线速度，m/s。
 * @param w 体坐标角速度，rad/s。
 * @param dv 体坐标线加速度分量，m/s²。
 * @param dw 体坐标角加速度分量，rad/s²。
 * @param swayYaw 横移/偏航对称附加惯性耦合项，kg·m；默认零兼容对角模型。
 */
inline AddedWrench AddedMassWrench(const V &m, const V &j, const V &v,
                                  const V &w, const V &dv, const V &dw,
                                  double swayYaw = 0) {
  V p(m.X() * v.X(), m.Y() * v.Y(), m.Z() * v.Z());
  V h(j.X() * w.X(), j.Y() * w.Y(), j.Z() * w.Z());
  p.Y() += swayYaw * w.Z();
  h.Z() += swayYaw * v.Y();
  return {-V(m.X() * dv.X(), m.Y() * dv.Y() + swayYaw * dw.Z(),
             m.Z() * dv.Z()) - w.Cross(p),
          -V(j.X() * dw.X(), j.Y() * dw.Y(),
             j.Z() * dw.Z() + swayYaw * dv.Y()) - v.Cross(p) - w.Cross(h)};
}
} // namespace usv
