#pragma once

#ifndef GEO_UTILS_H
#define GEO_UTILS_H

#include <Eigen/Core>
#include <iostream>

namespace geo {

#define C_PI (double)3.141592653589793
const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

const double KRegularPentagonPos[5][2] = {
  {0.0, 0.0},
  {-std::cos(54 * deg2rad), std::sin(54 * deg2rad)},
  {-std::cos(54 * deg2rad) - std::sin(72 * deg2rad), std::sin(54 * deg2rad) - std::cos(72 * deg2rad)},
  {-std::cos(54 * deg2rad), -std::sin(54 * deg2rad)},
  {-std::cos(54 * deg2rad) - std::sin(72 * deg2rad), - std::sin(54 * deg2rad) + std::cos(72 * deg2rad)},
};

// 排斥场
bool RepulsiveField(const Eigen::Matrix2Xd &points, Eigen::Vector2d &force, const int32_t id,
                    const double threshold = 1.0);
// 计算形成编队时无人机的位置坐标
bool GetFormationPos(const Eigen::Vector2d &xy_org, Eigen::Vector2d &xy_cur,
                     const int32_t id, const double distance);
// 计算形成编队时无人机的合力向量
bool GetFormationVector(const Eigen::Matrix2Xd &points, const int32_t id,
                        const double distance, Eigen::Vector2d &form_vec);

} // namespace geo

#endif /* GEO_UTILS_H */