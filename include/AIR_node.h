/**
 * @file AIR_node.h
 * @author cl (147507xxx@qq.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-08
 * 
 * @copyright Copyright (c) 2023 xxxxxxx公司
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2023-03-08 <td>v1.0     <td>chen     <td>内容
 * </table>
 */
#pragma once

#ifndef AIR_NODE_H
#define AIR_NODE_H

#include "geo_utils.h"
#include "Projtr.h"

// sdk
#include <dji_osdk_ros/Activation.h>
#include <dji_osdk_ros/DroneTaskControl.h>
#include <dji_osdk_ros/SDKControlAuthority.h>
// ros
#include "ros/ros.h"
#include "radio_proxy/Command.h"
#include "radio_proxy/FlightData.h"
#include "radio_proxy/Status.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/QuaternionStamped.h"
// std
#include <stdint.h>
#include <Eigen/Core>
#include <ctime>


class AIR_Controller {
public:
  //
  enum State { IDLE, TAKEOFF, MOVE, LAND, HOLD, FORMATION, TRACK};

public:
  //
  AIR_Controller();
  ~AIR_Controller();

private:

  const double threshold_factor_ = 2.0;

  //
  int32_t id_;
  int32_t swarm_num_;
  State state_;
  Eigen::Vector2d ll_org_;
  projtr::ProjTr projtr_;
  double height_cur_;
  // target
  double target_x_;
  double target_y_;
  double target_z_;
  double target_yaw_;
  double formation_distance_;
  double track_distance_;
  double dt; //时间步长
  double ki; //积分增益
  double kd; //微分增益
  Eigen::Vector2d integral;
  Eigen::Vector2d prev_err;

  // gps
  Eigen::Matrix3Xd uav_pos_;
  Eigen::Vector2d ugv_pos_;
  std::vector<bool> rtk_connected_;
  std::vector<clock_t> last_update_time_;  // 上次数据更新的时间
  // client
  ros::ServiceClient drone_activation_client_;
  ros::ServiceClient sdk_ctrl_authority_client_;
  ros::ServiceClient drone_task_client_;
  // subscriber
  ros::Subscriber flight_data_sub_;
  ros::Subscriber flight_status_sub_;
  ros::Subscriber gps_pos_sub_;
  ros::Subscriber rtk_pos_sub_;
  ros::Subscriber height_takeoff_sub_;
  ros::Subscriber rtk_status_sub_;
  ros::Subscriber cmd_sub_;
  // publisher
  ros::Publisher ctrlPosYawPub_;
  ros::Publisher ctrlVelYawPub_;
  // timer
  ros::Timer mainloopTimer_;

private:
  //
  void FlightDataSubCB(const radio_proxy::FlightData::ConstPtr &msg);
  void FlightStatusSubCB(const radio_proxy::Status::ConstPtr &msg);
  void CommandSubCB(const radio_proxy::Command::ConstPtr& msg);
  void GpsPosSubCB(const sensor_msgs::NavSatFix::ConstPtr &msg);
  void RtkPosSubCB(const sensor_msgs::NavSatFix::ConstPtr &msg);
  void RtkStatusSubCB(const std_msgs::UInt8::ConstPtr &msg);
  void HeightTakeoffSubCB(const std_msgs::Float32::ConstPtr &msg);
  void MainloopCB(const ros::TimerEvent& event);
  Eigen::Vector2d calculatePID(const Eigen::Vector2d& err);
};

#endif /* AIR_NODE_H */