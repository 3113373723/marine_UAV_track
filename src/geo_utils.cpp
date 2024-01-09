#include "geo_utils.h"

namespace geo {

bool RepulsiveField(const Eigen::Matrix2Xd &points, Eigen::Vector2d &force,
                    const int32_t id, const double threshold) {
  //
  force.fill(0.0);
  for (int32_t i = 0, _n = points.cols(); i < _n; i++) {
    if (i == id - 1)
      continue;
    Eigen::Vector2d _v_t = points.col(id - 1) - points.col(i);
    double _dist_t = _v_t.norm();
    if (_dist_t < 10e-4) { // 特判
      continue;            // 忽略重点
    }
    if (_dist_t < threshold) {
      force += (threshold - _dist_t) / threshold * _v_t.normalized();
    }
  }
  return true;
}

bool GetFormationPos(const Eigen::Vector2d &xy_org, Eigen::Vector2d &xy_cur,
                     const int32_t id, const double distance) {
  //
  if (id < 1 || id > 5)
    return false;
  xy_cur.x() = KRegularPentagonPos[id - 1][0] * distance + xy_org.x();
  xy_cur.y() = KRegularPentagonPos[id - 1][1] * distance + xy_org.y();
  return true;
}

bool GetFormationVector(const Eigen::Matrix2Xd &points, const int32_t id,
                        const double distance, Eigen::Vector2d &form_vec) {
  //
  if (id < 1 || id > 5)
    return false;
  // 计算目标矢量
  Eigen::Vector2d _target_xy;
  GetFormationPos(points.col(0), _target_xy, id, distance);
  Eigen::Vector2d _target_vec = _target_xy - points.col(id - 1);
  // 计算斥力
  Eigen::Vector2d _force_vec;
  RepulsiveField(points, _force_vec, id, 0.8 * distance);
  // 计算合力
  form_vec = _target_vec + 5 * _force_vec; 
  return true;
}

} // namespace geo