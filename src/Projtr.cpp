#include "Projtr.h"

namespace projtr {

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

ProjTr::ProjTr() : initialized_(false) {};
// 初始化
bool ProjTr::Init(const Eigen::Vector2d &ll_org) {
  //
  ll_org_.x() = ll_org.x();
  ll_org_.y() = ll_org.y();
  initialized_ = true;
  return true;
}

bool ProjTr::Init(const Eigen::Vector3d &ll_org) {
  //
  ll_org_ = ll_org;
  initialized_ = true;
  return true;
}
// 经纬度转XY坐标
bool ProjTr::LLToXY(const Eigen::Vector2d &ll_cur, Eigen::Vector2d &xy_cur) {
  //
  if (initialized_ == false) return false;
  double _delta_lon = ll_cur.x() - ll_org_.x();
  double _delta_lat = ll_cur.y() - ll_org_.y();
  xy_cur.x() = _delta_lon * deg2rad * C_EARTH;
  xy_cur.y() = _delta_lat * deg2rad * C_EARTH * std::cos(deg2rad * ll_org_.y());
  return true;
}

bool ProjTr::LLToXY(const Eigen::Vector3d &ll_cur, Eigen::Vector3d &xy_cur) {
  //
  if (initialized_ == false) return false;
  Eigen::Vector2d _ll_cur_2d, _xy_cur_2d;
  _ll_cur_2d.x() = ll_cur.x();
  _ll_cur_2d.y() = ll_cur.y();
  LLToXY(_ll_cur_2d, _xy_cur_2d);
  xy_cur.x() = _xy_cur_2d.x();
  xy_cur.y() = _xy_cur_2d.y();
  xy_cur.z() = xy_cur.z();
  return true;
}
// XY坐标转经纬度
bool ProjTr::XYToLL(const Eigen::Vector2d &xy_cur, Eigen::Vector2d &ll_cur) {

  //
  if (initialized_ == false) return false;
  ll_cur.x() = rad2deg * xy_cur.x() / C_EARTH + ll_org_.x();
  ll_cur.y() = rad2deg * xy_cur.y() / C_EARTH / std::cos(deg2rad * ll_org_.y()) + ll_org_.y();
  return true;
}

bool ProjTr::XYToLL(const Eigen::Vector3d &xy_cur, Eigen::Vector3d &ll_cur) {
  if (initialized_ == false) return false;
  Eigen::Vector2d _ll_cur_2d, _xy_cur_2d;
  _xy_cur_2d.x() = xy_cur.x();
  _xy_cur_2d.y() = xy_cur.y();
  XYToLL(_xy_cur_2d, _ll_cur_2d);
  ll_cur.x() = _ll_cur_2d.x();
  ll_cur.y() = _ll_cur_2d.y();
  ll_cur.z() = xy_cur.z();
  return true;
}

} // namespace projtr

