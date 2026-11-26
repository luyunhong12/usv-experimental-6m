/** @file HullSeparation.hh
 * @brief 从截面附加质量梯度计算分离区斜流修正，不重复加入Munk力矩。
 */
#pragma once
#include "Resistance.hh"
#include <algorithm>
#include <istream>
#include <utility>
#include <vector>

namespace usv {
/** @brief 保存势流恢复段的一阶、二阶力臂及附加质量变化。 */
struct RecoveryInterval {
  double meanX;     ///< 平均纵向力臂，m。
  double meanX2;    ///< 纵向力臂平方的区间平均，m²。
  double massDrop;  ///< 沿来流方向减少的截面附加质量，kg/m。
};

/** @brief 保存前进/倒车分离区，以及相同剖面的流体惯性积分。 */
struct HullSeparationProfile {
  std::vector<RecoveryInterval> ahead, astern;
  double swayMass = 0;       ///< 横向附加质量，kg。
  double swayYaw = 0;        ///< 横移/偏航耦合，kg·m。
  double yawInertia = 0;     ///< 偏航附加惯性，kg·m²。
};

/** @brief 校验剖面端点闭合和非零长度。
 * @param sections 已检查每行数值与顺序的(x,μ)，单位m及kg/m。
 * @throws std::runtime_error 点数不足、端点不闭合或无长度。
 */
inline void ValidateSeparationEnvelope(
    const std::vector<std::pair<double, double>> &sections) {
  if (sections.size() < 3 || sections.front().second != 0 ||
      sections.back().second != 0 ||
      sections.front().first >= sections.back().first)
    throw std::runtime_error("Separation profile must close at both ends");
}

/** @brief 校验并读取封闭、有序的截面附加质量曲线。
 * @param input 每行船体纵坐标x(m)和截面附加质量μ(kg/m)，无表头。
 * @return 按x非递减排列的(x,μ)；重复x允许表示端部跳变。
 * @throws std::runtime_error 输入截断、非有限、无正峰或端部不闭合。
 */
inline std::vector<std::pair<double, double>> ReadSeparationSections(
    std::istream &input) {
  std::vector<std::pair<double, double>> sections;
  while (input >> std::ws && input.peek() != std::char_traits<char>::eof()) {
    double x, mass;
    if (!(input >> x >> mass) || !std::isfinite(x) || !std::isfinite(mass) ||
        mass < 0 || (!sections.empty() && x < sections.back().first))
      throw std::runtime_error("Invalid separation section row");
    sections.emplace_back(x, mass);
  }
  if (input.bad())
    throw std::runtime_error("Cannot read separation profile");
  ValidateSeparationEnvelope(sections);
  return sections;
}

/** @brief 校验单峰拓扑并寻找平坦峰区的首尾索引。
 * @param sections 已通过数值、顺序和端点校验的(x,μ)剖面。
 * @return 最大附加质量区的首尾索引。
 * @throws std::runtime_error 无正峰，或下降后重新上升。
 */
inline std::pair<size_t, size_t> SeparationPeakRange(
    const std::vector<std::pair<double, double>> &sections) {
  double peak = 0;
  size_t first = 0, last = 0;
  bool falling = false;
  for (size_t i = 1; i < sections.size(); ++i) {
    const double mass = sections[i].second;
    const double previous = sections[i - 1].second;
    if (mass < previous)
      falling = true;
    if (falling && mass > previous)
      throw std::runtime_error("Separation profile must be single-peaked");
    if (mass > peak) {
      peak = mass;
      first = i;
    }
    if (mass == peak)
      last = i;
  }
  if (peak <= 0)
    throw std::runtime_error("Separation profile has no wet section");
  return {first, last};
}

/** @brief 以最大截面附加质量区的下游边界确定候选分离位置。
 * @param input 静水固定载荷剖面，格式见ReadSeparationSections。
 * @param referenceX 力与附加惯性使用的船体参考点纵坐标，m。
 * @return 不带自由增益的前进/倒车修正区间和惯性积分。
 * @throws std::runtime_error 参考点非法、剖面非单峰或无正附加质量。
 * @note 这是小斜流、低弗劳德数的候选近似，不是分离位置的实测值。
 */
inline HullSeparationProfile LoadHullSeparation(std::istream &input,
                                                double referenceX) {
  if (!std::isfinite(referenceX))
    throw std::runtime_error("Invalid separation reference");
  const auto sections = ReadSeparationSections(input);
  const auto [firstPeak, lastPeak] = SeparationPeakRange(sections);
  HullSeparationProfile result;
  for (size_t i = 0; i + 1 < sections.size(); ++i) {
    const double a = sections[i].first - referenceX;
    const double b = sections[i + 1].first - referenceX;
    const double ma = sections[i].second, mb = sections[i + 1].second;
    const double dx = b - a, delta = mb - ma;
    result.swayMass += dx * (ma + mb) / 2;
    result.swayYaw += dx * ((2 * a + b) * ma + (a + 2 * b) * mb) / 6;
    result.yawInertia += dx * ((3 * a * a + 2 * a * b + b * b) * ma +
                              (a * a + 2 * a * b + 3 * b * b) * mb) / 12;
    const RecoveryInterval interval{(a + b) / 2,
                                    (a * a + a * b + b * b) / 3,
                                    std::abs(delta)};
    if (i < firstPeak && delta > 0)
      result.ahead.push_back(interval);
    if (i >= lastPeak && delta < 0)
      result.astern.push_back(interval);
  }
  return result;
}

/** @brief 去掉下游势流压力恢复项，仅返回与现有惯性模型的差额。
 * @param profile 与附加惯性同源、同参考点的固定载荷剖面。
 * @param surge 参考点纵向相对水速，m/s；符号决定下游方向。
 * @param sway 同参考点横向相对水速，m/s。
 * @param yawRate 偏航角速度，rad/s。
 * @return 侧向力N及参考点偏航力矩N·m；零纵速时为零。
 * @note ΔY=-u∫下游μ′(v+xr)dx；ΔN多乘x。线性μ区间解析积分。
 */
inline LateralWrench HullSeparationCorrection(
    const HullSeparationProfile &profile, double surge, double sway,
    double yawRate) {
  LateralWrench result;
  const auto &intervals = surge >= 0 ? profile.ahead : profile.astern;
  for (const auto &interval : intervals) {
    const double scale = std::abs(surge) * interval.massDrop;
    result.force -= scale * (sway + interval.meanX * yawRate);
    result.moment -= scale * (interval.meanX * sway + interval.meanX2 * yawRate);
  }
  return result;
}
} // namespace usv
