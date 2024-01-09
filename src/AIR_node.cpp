#include "AIR_node.h"

AIR_Controller::AIR_Controller() : state_(IDLE) {
  //
  ros::NodeHandle _nh;
  ros::NodeHandle _private_nh("~");
  // params
  _private_nh.param<int32_t>("vehicle_id", id_, 1);
  _private_nh.param<int>("swarm_num", swarm_num_, 1);
  swarm_num_ = std::min(swarm_num_, 5);
  // variables
  ll_org_.x() = 113.38815245778486;
  ll_org_.y() = 23.067927699645193;
  projtr_.Init(ll_org_);
  target_x_ = target_y_ = target_z_ = target_yaw_ = formation_distance_ = 0.0;
  height_cur_ = 0.0;
  uav_pos_.resize(3, swarm_num_);
  uav_pos_.fill(0.0);
  ugv_pos_.fill(0.0);
  rtk_connected_.resize(swarm_num_, false);
  last_update_time_.resize(swarm_num_, clock());
  track_distance_ = 0.0;
  dt = 0.05;
  ki = 1;
  kd = 1;
  integral.x() = 0;
  integral.y() = 0;
  prev_err.x() = 0;
  prev_err.y() = 0;
  // client
  drone_activation_client_ =
      _nh.serviceClient<dji_osdk_ros::Activation>("dji_osdk_ros/activation");
  sdk_ctrl_authority_client_ =
      _nh.serviceClient<dji_osdk_ros::SDKControlAuthority>(
          "dji_osdk_ros/sdk_control_authority");
  drone_task_client_ = _nh.serviceClient<dji_osdk_ros::DroneTaskControl>(
      "dji_osdk_ros/drone_task_control");
  // subsciber
  flight_data_sub_ = _nh.subscribe("/AIR_proxy/flight_data", 10,
                                   &AIR_Controller::FlightDataSubCB, this);
  flight_status_sub_ = _nh.subscribe("/AIR_proxy/flight_status", 10,
                                     &AIR_Controller::FlightStatusSubCB, this);
  gps_pos_sub_ = _nh.subscribe("/dji_osdk_ros/gps_position", 10,
                               &AIR_Controller::GpsPosSubCB, this);
  rtk_pos_sub_ = _nh.subscribe("/dji_osdk_ros/rtk_position", 10,
                               &AIR_Controller::RtkPosSubCB, this);
  rtk_status_sub_ = _nh.subscribe("/dji_osdk_ros/rtk_info_position", 10,
                                  &AIR_Controller::RtkStatusSubCB, this);
  height_takeoff_sub_ = _nh.subscribe("/dji_osdk_ros/height_above_takeoff", 10,
                                  &AIR_Controller::HeightTakeoffSubCB, this);
  cmd_sub_ = _nh.subscribe("/AIR_proxy/command", 10,
                           &AIR_Controller::CommandSubCB, this);
  // publisher
  ctrlPosYawPub_ = _nh.advertise<sensor_msgs::Joy>(
      "/dji_osdk_ros/flight_control_setpoint_ENUposition_yaw", 10);
  ctrlVelYawPub_ = _nh.advertise<sensor_msgs::Joy>(
      "/dji_osdk_ros/flight_control_setpoint_generic", 10);
  // timer
  mainloopTimer_ =
      _nh.createTimer(ros::Duration(dt), &AIR_Controller::MainloopCB, this);
}

AIR_Controller::~AIR_Controller() {
  //

}
// 处理接收到的飞行器状态信息，包括位置和高度等，并将其转换为适当的表示形式进行存储和处理。
void AIR_Controller::FlightDataSubCB(
    const radio_proxy::FlightData::ConstPtr &msg) {
  //

  if (msg->id == 6) {
    Eigen::Vector2d _ll;
    _ll << msg->longitude, msg->latitude;
    projtr_.LLToXY(_ll, ugv_pos_);
  }

  if (msg->id < 1 || msg->id > swarm_num_) return;

  Eigen::Vector3d _llh;
  _llh << msg->longitude_rtk, msg->latitude_rtk, msg->altitude_rtk;
  bool _rtk_connected = _llh.norm() > 0.1;

  int32_t _idx_t = msg->id - 1;
  if (_rtk_connected == true) {
    Eigen::Vector2d _ll_cur, _xy_cur;
    _ll_cur << msg->longitude_rtk, msg->latitude_rtk;
    projtr_.LLToXY(_ll_cur, _xy_cur);
    uav_pos_.col(_idx_t).x() = _xy_cur.x();
    uav_pos_.col(_idx_t).y() = _xy_cur.y();
    uav_pos_.col(_idx_t).z() = msg->height_above_takeoff;
  }
  else {
    Eigen::Vector2d _ll_cur, _xy_cur;
    _ll_cur << msg->longitude, msg->latitude;
    projtr_.LLToXY(_ll_cur, _xy_cur);
    uav_pos_.col(_idx_t).x() = _xy_cur.x();
    uav_pos_.col(_idx_t).y() = _xy_cur.y();
    uav_pos_.col(_idx_t).z() = msg->height_above_takeoff;
  }
  last_update_time_[_idx_t] = clock();
}

void AIR_Controller::FlightStatusSubCB(const radio_proxy::Status::ConstPtr &msg) {
  //
  if (msg->id < 1 || msg->id > swarm_num_) return;

  int32_t _idx_t = msg->id - 1;
  rtk_connected_[_idx_t] = msg->rtk_connected;
}
// 节点可以根据接收到的指令消息，执行相应的无人机控制操作
void AIR_Controller::CommandSubCB(const radio_proxy::Command::ConstPtr &msg) {
  //
  dji_osdk_ros::Activation activation;
  dji_osdk_ros::SDKControlAuthority sdkAuthority;
  dji_osdk_ros::DroneTaskControl droneTaskControl;

  uint8_t cmd = msg->mission;

  switch (cmd) {

  case radio_proxy::Command::TAKEOFF: {
    state_ = TAKEOFF;

    // activate
    drone_activation_client_.call(activation);
    if (!activation.response.result) {
      ROS_ERROR("activating failed.");
    }
    // get authority
    sdkAuthority.request.control_enable = 1;
    sdk_ctrl_authority_client_.call(sdkAuthority);
    if (!sdkAuthority.response.result) {
      ROS_ERROR("authorizing failed.");
    }
    // take off
    droneTaskControl.request.task = 4;
    drone_task_client_.call(droneTaskControl);
    if (!droneTaskControl.response.result) {
      ROS_ERROR("take off failed.");
    }
    state_ = HOLD;

    break;
  }

  case radio_proxy::Command::SETPOINT_LOCAL: {
    //
    if (id_ == 1) {
      state_ = MOVE;
      target_x_ = uav_pos_.col(id_ - 1).x() + msg->x;
      target_y_ = uav_pos_.col(id_ - 1).y() + msg->y;
      target_yaw_ = msg->yaw;
    }
    target_z_ = msg->z;
    break;
  }

  case radio_proxy::Command::SETPOINT_GPS: {
    //
    if (id_ == 1) {
      state_ = TRACK;
      target_z_ = msg->z;
      track_distance_ = msg->altitude;
    }
    break;
  }

  case radio_proxy::Command::TRACKING : { // 借用
    if (id_ != 1) {
      state_ = FORMATION;
      formation_distance_ = msg->altitude;
    }
    break;
  }

  case radio_proxy::Command::LAND: {
    //
    state_ = LAND;
    droneTaskControl.request.task = 6;
    drone_task_client_.call(droneTaskControl);
    if (!droneTaskControl.response.result) {
      ROS_ERROR("land failed.");
    }
    state_ = IDLE;
    break;
  }

  case radio_proxy::Command::HOLD: {
    //
    state_ = HOLD;
    break;
  }
  }
}
// 将接收到的GPS位置信息转换为xy坐标，并将其存储在uav_pos_矩阵的相应列中
void AIR_Controller::GpsPosSubCB(const sensor_msgs::NavSatFix::ConstPtr &msg) {
  //
  int32_t _idx_t = id_ - 1;
  if (rtk_connected_[_idx_t] == true) return;
  Eigen::Vector2d _ll_cur, _xy_cur;
  _ll_cur << msg->longitude, msg->latitude;
  projtr_.LLToXY(_ll_cur, _xy_cur);
  uav_pos_.col(_idx_t).x() = _xy_cur.x();
  uav_pos_.col(_idx_t).y() = _xy_cur.y();
  last_update_time_[_idx_t] = clock();
}
// 处理接收到的 RTK GPS 位置信息，并将其转换为xy坐标
void AIR_Controller::RtkPosSubCB(const sensor_msgs::NavSatFix::ConstPtr &msg) {
  //
  int32_t _idx_t = id_ - 1;

  Eigen::Vector3d _llh;
  rtk_connected_[_idx_t] = _llh.norm() > 0.1;

  if (rtk_connected_[_idx_t] == false) return;
  Eigen::Vector2d _ll_cur, _xy_cur;
  _ll_cur << msg->longitude, msg->latitude;
  projtr_.LLToXY(_ll_cur, _xy_cur);
  uav_pos_.col(_idx_t).x() = _xy_cur.x();
  uav_pos_.col(_idx_t).y() = _xy_cur.y();
  last_update_time_[_idx_t] = clock();
}
// 更新rtk_connected_数组中指定索引的元素值
void AIR_Controller::RtkStatusSubCB(const std_msgs::UInt8::ConstPtr &msg) {
  //
  int32_t _idx_t = id_ - 1;
  rtk_connected_[_idx_t] = msg->data;
}
// 更新height_cur_变量和uav_pos_矩阵
void AIR_Controller::HeightTakeoffSubCB(
    const std_msgs::Float32::ConstPtr &msg) {
  //
  height_cur_ = msg->data;
  int32_t _idx_t = id_ - 1;
  uav_pos_.col(_idx_t).z() = msg->data;

}

Eigen::Vector2d calculatePID(const Eigen::Vector2d& err) {
    Eigen::Vector2d pid_output;

    // Calculate P control term
    pid_output = std::min(err.norm(), threshold_factor_) * err.normalized();

    // Calculate I control term using Runge-Kutta method
    double k1x = ki * err.x();
    double k1y = ki * err.y();

    double k2x = ki * (err.x() + k1x * dt / 2.0);
    double k2y = ki * (err.y() + k1y * dt / 2.0);

    double k3x = ki * (err.x() + k2x * dt / 2.0);
    double k3y = ki * (err.y() + k2y * dt / 2.0);

    double k4x = ki * (err.x() + k3x * dt);
    double k4y = ki * (err.y() + k3y * dt);

    integral.x() += (k1x + 2.0 * k2x + 2.0 * k3x + k4x) * dt / 6.0;
    integral.y() += (k1y + 2.0 * k2y + 2.0 * k3y + k4y) * dt / 6.0;

    // Apply clamping to integral term to prevent windup
    integral.x() = std::max(std::min(integral.x(), threshold_factor_), -threshold_factor_);
    integral.y() = std::max(std::min(integral.y(), threshold_factor_), -threshold_factor_);

    pid_output += integral;

    // Calculate D control term using simple difference
    pid_output.x() += kd * (err.x() - prev_err.x()) / dt;
    pid_output.y() += kd * (err.y() - prev_err.y()) / dt;

    prev_err = err; // Update previous error for the next iteration

    return pid_output;
}

void AIR_Controller::MainloopCB(const ros::TimerEvent &event) {
  //

  if (state_ == MOVE) {
    //
    std::cout << "MOVE" << std::endl;
    sensor_msgs::Joy controlPosYaw;
    Eigen::Vector2d _ctr_vec;
    _ctr_vec << target_x_ - uav_pos_.col(id_ - 1).x(), target_y_ - uav_pos_.col(id_ - 1).y();
    // _ctr_vec = std::min(_ctr_vec.norm(), threshold_factor_) * _ctr_vec.normalized();
    _ctr_vec = calculatePID(_ctr_vec);


    controlPosYaw.axes.push_back(_ctr_vec.x());
    controlPosYaw.axes.push_back(_ctr_vec.y());
    controlPosYaw.axes.push_back(target_z_);
    controlPosYaw.axes.push_back(target_yaw_);

    ctrlPosYawPub_.publish(controlPosYaw);
  }
  else if (state_ == FORMATION) {
    //
    // std::cout << formation_distance_ << std::endl;

    sensor_msgs::Joy controlVelYaw;

    Eigen::Vector2d _ctr_vec;
    geo::GetFormationVector(uav_pos_.topRows(2), id_, formation_distance_, _ctr_vec);
    _ctr_vec = std::min(_ctr_vec.norm(), threshold_factor_) * _ctr_vec.normalized();

    controlVelYaw.axes.push_back(_ctr_vec.x());
    controlVelYaw.axes.push_back(_ctr_vec.y());
    controlVelYaw.axes.push_back(uav_pos_.col(0).z());
    controlVelYaw.axes.push_back(0);

    // flag
    uint8_t flag = (16 | 64 | 0 | 0 | 1);

    controlVelYaw.axes.push_back(flag);

    ctrlVelYawPub_.publish(controlVelYaw);
  }
  else if (state_ == TRACK) {
    //
    std::cout << "TRACK" << std::endl;
    sensor_msgs::Joy controlPosYaw;
    Eigen::Vector2d _ctr_vec;
    _ctr_vec << ugv_pos_.x() - uav_pos_.col(id_ - 1).x(), ugv_pos_.y() - uav_pos_.col(id_ - 1).y();

    double _distance_offset = _ctr_vec.norm();
    _distance_offset = std::max(0.0, _distance_offset - track_distance_);
    _ctr_vec = std::min(_distance_offset, threshold_factor_) * _ctr_vec.normalized();

    controlPosYaw.axes.push_back(_ctr_vec.x());
    controlPosYaw.axes.push_back(_ctr_vec.y());
    controlPosYaw.axes.push_back(target_z_);
    controlPosYaw.axes.push_back(0);

    ctrlPosYawPub_.publish(controlPosYaw);
  }
}

int main(int argc, char *argv[]) {
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "dji_test_node");

  AIR_Controller _ins;

  ros::spin();
  return 0;
}
