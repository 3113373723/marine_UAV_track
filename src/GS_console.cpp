#include "Projtr.h"

// ros
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseArray.h"
#include "tf2/utils.h"

// radio_proxy
#include "radio_proxy/FlightData.h"
#include "radio_proxy/Status.h"
#include "radio_proxy/String.h"
#include "radio_proxy/Command.h"

// std
#include <vector>
#include <Eigen/Core>

ros::Publisher kMarkerPub;
ros::Publisher kPoseArrPub;

typedef struct State {
  double x;
  double y;
  double z;
  double vx;
  double vy;
} State;

typedef struct Control {
  double vx;
  double vy;
} Control;

std::vector<radio_proxy::FlightData> kFDs_old;
std::vector<radio_proxy::FlightData> kFDs;
std::vector<radio_proxy::Status> kSTs;
std::vector<bool> kDataReady;

bool _rtk_connected = false;

ros::Subscriber fdSub;
ros::Subscriber stSub;
ros::Subscriber msgSub;

ros::Publisher cmdPub;
radio_proxy::Command cmd;

ros::Timer mainloopTimer;

projtr::ProjTr kProjtr;

// 接收到radio_proxy::FlightData消息后，更新存储的数据，并标记数据准备完毕。
void fdCB(const radio_proxy::FlightData::ConstPtr &msg_p) {
  kFDs_old[msg_p->id - 1] = kFDs[msg_p->id - 1];
  kFDs[msg_p->id - 1] = *msg_p;
  kDataReady[msg_p->id - 1] = true;
}
// 更新存储的数据，并标记数据准备完毕。
void stCB(const radio_proxy::Status::ConstPtr &msg_p) {
  kSTs[msg_p->id - 1] = *msg_p;
  kDataReady[msg_p->id - 1] = true;
}
// 接收到radio_proxy::String消息后，将消息的ID和数据拼接成一个新的字符串，并通过ROS的日志系统将其打印出来。
void msgCB(const radio_proxy::String::ConstPtr &msg_p) {
  std::stringstream ss;
  ss << "vechicle_" << msg_p->id << ": " << msg_p->data;
  ROS_INFO("%s", ss.str().c_str());
}
// 根据输入的一组状态信息，通过ROS的可视化消息类型，发布位姿数组和可视化标记，用于在RViz或其他可视化工具中显示状态信息。
bool VisualizeStates(const std::vector<State> &states) {
  //
  geometry_msgs::PoseArray _pose_array_msg;
  geometry_msgs::Pose _pose_msg;
  _pose_array_msg.header.frame_id = "map";
  _pose_array_msg.header.stamp = ros::Time::now();

  visualization_msgs::Marker _marker_msg;
  geometry_msgs::Point _point_msg;
  _marker_msg.header.frame_id = "map";
  _marker_msg.header.stamp = ros::Time::now();
  _marker_msg.action = _marker_msg.ADD;
  // _marker_msg.color.r = 0.0;
  // _marker_msg.color.g = 0.5;
  // _marker_msg.color.b = 0.5;
  // _marker_msg.color.a = 0.8;
  _marker_msg.id = 0;
  _marker_msg.type = _marker_msg.SPHERE_LIST;
  _marker_msg.scale.x = 1.0;
  _marker_msg.scale.y = 1.0;
  _marker_msg.scale.z = 1.0;
  
  std_msgs::ColorRGBA _color_rgba_msg;

  for (int32_t i = 0, _n = states.size(); i < _n; i++) {
    // 特殊可视化
    if (i == 0) {// 黄色
      _color_rgba_msg.r = 1.0;
      _color_rgba_msg.g = 1.0;
      _color_rgba_msg.b = 0.0;
    }
    if (i == 1) {// 绿色
      _color_rgba_msg.r = 0.0;
      _color_rgba_msg.g = 0.7890625;
      _color_rgba_msg.b = 0.34375;
    }
    if (i == 2) {// 红色
      _color_rgba_msg.r = 1.0;
      _color_rgba_msg.g = 0.0;
      _color_rgba_msg.b = 0.0;
    }
    if (i == 3) {// 蓝色
      _color_rgba_msg.r = 0.0;
      _color_rgba_msg.g = 0.0;
      _color_rgba_msg.b = 1.0;
    }
    if (i == 4) {// 白色
      _color_rgba_msg.r = 1.0;
      _color_rgba_msg.g = 1.0;
      _color_rgba_msg.b = 1.0;
    }
    if (i == 5) {// 紫色
      _color_rgba_msg.r = 0.62890625;
      _color_rgba_msg.g = 0.12890625;
      _color_rgba_msg.b = 0.94140625;
    }
    _color_rgba_msg.a = 0.8;
    _marker_msg.colors.push_back(_color_rgba_msg);

    //
    _pose_msg.position.x = states[i].x;
    _pose_msg.position.y = states[i].y;
    _pose_msg.position.z = 0.5;

    tf2::Quaternion _qtn;
    double _yaw = std::atan2(states[i].vy, states[i].vx);
    _qtn.setRPY(0, 0, _yaw);
    _pose_msg.orientation.x = _qtn.x();
    _pose_msg.orientation.y = _qtn.y();
    _pose_msg.orientation.z = _qtn.z();
    _pose_msg.orientation.w = _qtn.w();
    
    _pose_array_msg.poses.push_back(_pose_msg);

    //
    _point_msg.x = states[i].x;
    _point_msg.y = states[i].y;
    _point_msg.z = states[i].z;

    _marker_msg.points.push_back(_point_msg);
  }

  kMarkerPub.publish(_marker_msg);
  kPoseArrPub.publish(_pose_array_msg);

  return true;
}

// void EmergencyHedging() {
//     //
//     double _distance_12, _distance_13, _distance_23;

//     //
//     geometry_msgs::Vector3 _position_ll_1;
//     _position_ll_1.x = fd[1].longitude;
//     _position_ll_1.y = fd[1].latitude;
//     _position_ll_1.z = fd[1].height_above_takeoff;

//     geometry_msgs::Vector3 _position_ll_2;
//     _position_ll_2.x = fd[2].longitude;
//     _position_ll_2.y = fd[2].latitude;
//     _position_ll_2.z = fd[2].height_above_takeoff;

//     geometry_msgs::Vector3 _position_ll_3;
//     _position_ll_3.x = fd[3].longitude;
//     _position_ll_3.y = fd[3].latitude;
//     _position_ll_3.z = fd[3].height_above_takeoff;

//     //
//     geometry_msgs::Vector3 _delta12;
//     localOffsetFromGpsOffset(_delta12, _position_ll_1, _position_ll_2);
//     _distance_12 = _delta12.x * _delta12.x + _delta12.y * _delta12.y + _delta12.z * _delta12.z;

//     geometry_msgs::Vector3 _delta13;
//     localOffsetFromGpsOffset(_delta13, _position_ll_1, _position_ll_3);
//     _distance_13 = _delta13.x * _delta13.x + _delta13.y * _delta13.y + _delta13.z * _delta13.z;

//     geometry_msgs::Vector3 _delta23;
//     localOffsetFromGpsOffset(_delta23, _position_ll_2, _position_ll_3);
//     _distance_23 = _delta23.x * _delta23.x + _delta23.y * _delta23.y + _delta23.z * _delta23.z;

//     double _threshold = 5.0;

//     if (fabs(_distance_12) < _threshold || fabs(_distance_13) < _threshold || fabs(_distance_23) < _threshold) {
//         cmd.mission = radio_proxy::Command::HOLD;
//         cmd.header.stamp = ros::Time::now();
//         cmdPub.publish(cmd);
//     }
// }
// 在主循环中进行状态信息的打印和可视化处理。它遍历飞行数据和飞行状态，构建打印信息并使用 ROS 的日志系统进行打印，然后根据状态信息构建状态对象，将其添加到状态向量中，并通过 VisualizeStates 函数进行可视化。
void mainLoopCB(const ros::TimerEvent &event){

    // print
    std::stringstream ss;
    for (int i=0; i<10; i++){
        if (kDataReady[i] == false) continue;

        Eigen::Vector3d _llh;
        _llh << kFDs[i].longitude_rtk, kFDs[i].latitude_rtk, kFDs[i].altitude_rtk;
        bool _rtk_connected = _llh.norm() > 0.1;

        ss << std::endl << "vehicle_" << i + 1 << ": ( ";
        if (_rtk_connected == true) {
          ss << "latitude: " << kFDs[i].latitude_rtk << ", longitude: " << kFDs[i].longitude_rtk << ", altitude: " << kFDs[i].altitude_rtk;
        }
        else {
          ss << "latitude: " << kFDs[i].latitude << ", longitude: " << kFDs[i].longitude << ", altitude: " << kFDs[i].altitude;
        }
        ss << ", height: " << kFDs[i].height_above_takeoff;
        ss << ", battery_v: " << kSTs[i].battery_v <<  ", gps_health: " << int(kSTs[i].gps_health);
        ss << ")";
    }
    ROS_INFO("%s", ss.str().c_str());

    // visualize
    std::vector<State> _states;
    for (int32_t i = 0; i < 10; i++) {
      if (kDataReady[i] == false) continue;

      Eigen::Vector3d _llh;
      _llh << kFDs[i].longitude_rtk, kFDs[i].latitude_rtk, kFDs[i].altitude_rtk;
      bool _rtk_connected = _llh.norm() > 0.1;

      Eigen::Vector2d _ll_cur, _ll_old;

      if (_rtk_connected == true) {
        _ll_cur.x() = kFDs[i].longitude_rtk;
        _ll_cur.y() = kFDs[i].latitude_rtk;
        _ll_old.x() = kFDs_old[i].longitude_rtk;
        _ll_old.y() = kFDs_old[i].latitude_rtk;
      } else {
        _ll_cur.x() = kFDs[i].longitude;
        _ll_cur.y() = kFDs[i].latitude;
        _ll_old.x() = kFDs_old[i].longitude;
        _ll_old.y() = kFDs_old[i].latitude;
      }

      Eigen::Vector2d _xy_cur, _xy_old;
      kProjtr.LLToXY(_ll_cur, _xy_cur);
      kProjtr.LLToXY(_ll_old, _xy_old);
      
      State _st;
      _st.x = _xy_cur.x();      
      _st.y = _xy_cur.y();
      _st.z = kFDs[i].height_above_takeoff;
      _st.vx = _xy_cur.x() - _xy_old.x();
      _st.vy = _xy_cur.y() - _xy_old.y();

      _states.push_back(_st);
    }

    VisualizeStates(_states);

//     EmergencyHedging();
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"GS_console");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    kFDs.resize(10);
    kFDs_old.resize(10);
    kSTs.resize(10);
    kDataReady.resize(10, false);

    Eigen::Vector2d _ll_org;
    _ll_org.x() = 113.38815245778486;
    _ll_org.y() = 23.067927699645193;
    kProjtr.Init(_ll_org);

    fdSub = nh.subscribe<radio_proxy::FlightData>("/GS_proxy/flight_data", 1, &fdCB);
    stSub = nh.subscribe<radio_proxy::Status>("/GS_proxy/flight_status", 1, &stCB);
    msgSub = nh.subscribe<radio_proxy::String>("/GS_proxy/message", 1, &msgCB);

    cmdPub = nh.advertise<radio_proxy::Command>("/GS_proxy/command",10);
    cmd.id = 255;
    cmd.header.frame_id = "station";

    // rviz visualization
    kMarkerPub = nh.advertise<visualization_msgs::Marker>("marker",10);
    kPoseArrPub = nh.advertise<geometry_msgs::PoseArray>("pose_arr", 10);

    // initialze timer
    mainloopTimer = nh.createTimer(ros::Duration(1), &mainLoopCB);

    ros::spin();

    return 0;
}