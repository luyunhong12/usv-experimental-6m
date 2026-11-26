/** @file Resistance.hh
 * @brief 按照浸水面积和方向投影面积缩放阻力。
 */
#pragma once
#include "Hydrostatics.hh"
#include <stdexcept>
#include <array>
namespace usv {
/** @brief 空气阻力合力及关于船体原点的力矩，单位N和N·m。 */
struct AirDragWrench {
  V force = V::Zero;
  V torque = V::Zero;
};

/** @brief 按各轴投影形心计算法向空气阻力，包含转动引起的局部风速。
 * @param cdArea 各轴阻力系数乘投影面积，m²，有限且非负。
 * @param centers 各轴投影形心，船体坐标m；沿受力轴的分量不影响结果。
 * @param velocity 船体原点相对均匀空气的线速度，船体轴m/s。
 * @param angular 船体角速度，船体轴rad/s。
 * @return 合力及原点力矩；空气密度沿用1.225 kg/m³。
 * @note 各方向法向平板近似，不代表三维绕流压力求解。
 */
inline AirDragWrench DirectionalAirDrag(const V &cdArea,
                                       const std::array<V, 3> &centers,
                                       const V &velocity, const V &angular) {
  AirDragWrench result;
  for (size_t axis = 0; axis < centers.size(); ++axis) {
    const double normal = (velocity + angular.Cross(centers[axis]))[axis];
    V force = V::Zero;
    force[axis] = -.5 * 1.225 * cdArea[axis] * normal * std::abs(normal);
    result.force += force;
    result.torque += centers[axis].Cross(force);
  }
  return result;
}

/** @brief 保存表面积比例与各轴投影面积比例。
 */
struct ResistanceScales {
  double surface;
  V projected;
};
/** @brief 判断当前面积非负且参考面积为正，拒绝非有限输入。
 * @param area 当前面积，m²。
 * @param reference 基准面积，m²。
 */
inline bool ValidResistanceArea(double area, double reference) {
  return std::isfinite(area) && area >= 0 && std::isfinite(reference) &&
         reference > 0;
}
/** @brief 检查参考面积有效性并计算无量纲阻力缩放比例。
 * @param area 当前浸水面积，m²。
 * @param projected 当前各轴投影面积，m²。
 * @param reference 正的基准浸水面积，m²。
 * @param referenceProjected 正的基准各轴投影面积，m²。
 */
inline ResistanceScales GeometryResistanceScales(double area,
                                                 const V &projected,
                                                 double reference,
                                                 const V &referenceProjected) {
  if (!ValidResistanceArea(area, reference))
    throw std::runtime_error("Invalid wet area reference");
  ResistanceScales s{area / reference, V::Zero};
  for (int i = 0; i < 3; ++i) {
    if (!ValidResistanceArea(projected[i], referenceProjected[i]))
      throw std::runtime_error("Invalid projected wet area");
    s.projected[i] = projected[i] / referenceProjected[i];
  }
  return s;
}
/** @brief 按线性和二次阻尼系数计算与相对速度反向的阻力。
 * @param linear 三轴线性阻尼系数。
 * @param quadratic 三轴二次阻尼系数。
 * @param velocity 体坐标相对线速度，m/s。
 * @param s 浸水与投影面积相对固定基准的缩放比例。
 */
inline V GeometryDrag(const V &linear, const V &quadratic, const V &velocity,
                      const ResistanceScales &s) {
  V f;
  for (int i = 0; i < 3; ++i)
    f[i] = -linear[i] * s.surface * velocity[i] -
           quadratic[i] * s.projected[i] * velocity[i] * std::abs(velocity[i]);
  return f;
}
/** @brief 保存侧向浸水面片相对阻力参考点的纵坐标、浸水面积及投影面积。 */
struct LateralPatch {
  double x = 0;             ///< 纵向力臂，m。
  double projectedArea = 0; ///< 单侧等效浸水投影面积，m²。
  double wetArea = 0;       ///< 浸水表面积，m²。
};
/** @brief 保存分布式侧向力及其关于参考点的偏航力矩。 */
struct LateralWrench {
  double force = 0;  ///< 侧向力，N。
  double moment = 0; ///< 偏航力矩，N·m。
};
/** @brief 按各面片局部横向水速积分阻力，偏航力矩由真实力臂产生。
 * @param patches 浸水面片面积、投影面积及纵向力臂。
 * @param sway 参考点相对横向水速，m/s。
 * @param yawRate 船体偏航角速度，rad/s。
 * @param linearPerArea 单位浸水表面积线性阻力系数，N·s/m³。
 * @param quadraticPerArea 单位侧向面积二次阻力系数，N·s²/m⁴。
 */
inline LateralWrench
DistributedLateralDrag(const std::vector<LateralPatch> &patches, double sway,
                       double yawRate, double linearPerArea,
                       double quadraticPerArea) {
  LateralWrench result;
  for (const auto &patch : patches) {
    const double velocity = sway + patch.x * yawRate;
    const double force =
        -linearPerArea * patch.wetArea * velocity -
        quadraticPerArea * patch.projectedArea * velocity * std::abs(velocity);
    result.force += force;
    result.moment += patch.x * force;
  }
  return result;
}

} // namespace usv
