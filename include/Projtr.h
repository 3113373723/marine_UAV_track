/**
 * @file Projtr.h
 * @author Vinson Sheep (775014077@qq.com)
 * @brief 使用圆球方法实现经纬度到XY坐标的转换
 * @version 1.0
 * @date 2023-03-09
 * 
 * @copyright Copyright (c) 2023 xxxxxxx公司
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2023-03-09 <td>v1.0     <td>chen     <td>内容
 * </table>
 */
#pragma once

#ifndef PROJTR_H
#define PROJTR_H

#include <Eigen/Core>

namespace projtr {

class ProjTr {
public:
  //
  ProjTr();
  ~ProjTr(){};
  /**
   * @brief 初始化
   * @param [in] ll_org 经纬度原点
   *
   * @details
   */
  bool Init(const Eigen::Vector2d &ll_org);
  /**
   * @brief 初始化
   * @param [in] ll_org 经纬度原点
   *
   * @details
   */
  bool Init(const Eigen::Vector3d &ll_org);
  /**
   * @brief 经纬度转XY坐标
   * @param [in] ll_cur 目标经纬度
   * @param [in] xy_cur 目标XY坐标
   *
   * @details
   */
  bool LLToXY(const Eigen::Vector2d &ll_cur, Eigen::Vector2d &xy_cur);
  /**
   * @brief 经纬度转XY坐标
   * @param [in] ll_cur 目标经纬度
   * @param [in] xy_cur 目标XY坐标
   *
   * @details
   */
  bool LLToXY(const Eigen::Vector3d &ll_cur, Eigen::Vector3d &xy_cur);
  /**
   * @brief XY坐标转经纬度
   * @param [in] xy_cur 目标XY坐标
   * @param [in] ll_cur 目标经纬度
   *
   * @details
   */
  bool XYToLL(const Eigen::Vector2d &xy_cur, Eigen::Vector2d &ll_cur);
  /**
   * @brief XY坐标转经纬度
   * @param [in] xy_cur 目标XY坐标
   * @param [in] ll_cur 目标经纬度
   *
   * @details
   */
  bool XYToLL(const Eigen::Vector3d &xy_cur, Eigen::Vector3d &ll_cur);

private:
  //
  Eigen::Vector3d ll_org_;  // 经纬度原点
  bool initialized_; 
};

} // namespace projtr

#endif /* PROJTR_H */