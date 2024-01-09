#include "geo_utils.h"

// ros
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseArray.h"
#include "tf2/utils.h"

// std
#include <random>

// constant
const double KIntergrateDt = 0.001;
const double KStateUpdateFactor = 0.1;

const int32_t KSwarmNum = 5;
const double KFormDist = 5.0;

const double KEnvWidth = 10.0;

ros::Publisher kMarkerPub;
ros::Publisher kPoseArrPub;

typedef struct State {
  double x;
  double y;
  double vx;
  double vy;
} State;

typedef struct Control {
  double vx;
  double vy;
} Control;

void UpdateModel(State &state, const Control &control, const double duration) {
  //
  // update velicity
  double _alpha = duration / KIntergrateDt * KStateUpdateFactor;
  state.vx = (1 - _alpha) * state.vx + _alpha * control.vx;
  state.vy = (1 - _alpha) * state.vy + _alpha * control.vy;
  // modify position
  state.x += state.vx * duration;
  state.y += state.vy * duration;
}

void Intergrator(State &state, const Control &control, const double duration) {
  //
  double _intergrate_time = KIntergrateDt;
  while (_intergrate_time < duration + std::numeric_limits<double>::epsilon()) {
    UpdateModel(state, control, KIntergrateDt);
    _intergrate_time += KIntergrateDt;
    ros::Duration(KIntergrateDt).sleep();
    ros::spinOnce();
  }
  if (_intergrate_time + std::numeric_limits<double>::epsilon() > duration) {
    UpdateModel(state, control, _intergrate_time - duration);
    ros::Duration(_intergrate_time - duration).sleep();
    ros::spinOnce();
  }
}

bool RandomStates(std::vector<State> &states, const int32_t num) {
  //
  states.resize(num);
  std::uniform_real_distribution<double> _u(-KEnvWidth, KEnvWidth);
  std::default_random_engine _e(time(NULL));
  for (int32_t i = 0; i < num; i++) {
    states[i].x = _u(_e);
    states[i].y = _u(_e);
    states[i].vx = 0;
    states[i].vy = 0;
  }
  return true;
}

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
  _marker_msg.color.r = 0.0;
  _marker_msg.color.g = 0.5;
  _marker_msg.color.b = 0.5;
  _marker_msg.color.a = 0.8;
  _marker_msg.id = 0;
  _marker_msg.type = _marker_msg.SPHERE_LIST;
  _marker_msg.scale.x = 0.5;
  _marker_msg.scale.y = 0.5;
  _marker_msg.scale.z = 0.5;
  
  for (int32_t i = 0, _n = states.size(); i < _n; i++) {
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
    _point_msg.z = 0.5;

    _marker_msg.points.push_back(_point_msg);
  }

  kMarkerPub.publish(_marker_msg);
  kPoseArrPub.publish(_pose_array_msg);

  return true;
}

bool Transform(const std::vector<State> &states, Eigen::Matrix2Xd &points) {
  //
  int32_t _n = states.size();
  points.resize(2, _n);
  for (int32_t i = 0; i < _n; i++) {
    points.col(i).x() = states[i].x;
    points.col(i).y() = states[i].y;
  }
  return true;
}

int main(int argc, char *argv[]) {
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "geo_test");
  ros::NodeHandle nh;

  kMarkerPub = nh.advertise<visualization_msgs::Marker>("marker",10);
  kPoseArrPub = nh.advertise<geometry_msgs::PoseArray>("pose_arr", 10);

  std::vector<State> _states;
  RandomStates(_states, KSwarmNum);

  while (ros::ok()) {
    
    Eigen::Matrix2Xd _points;
    Transform(_states, _points);
    Control _ctr;
    for (int32_t i = 0; i < KSwarmNum; i++) {
      Eigen::Vector2d _ctr_vec;
      geo::GetFormationVector(_points, i + 1, KFormDist, _ctr_vec);
      _ctr_vec *= 5.0;
      _ctr.vx = _ctr_vec.x();
      _ctr.vy = _ctr_vec.y();
      Intergrator(_states[i], _ctr, 0.01);
    }
    VisualizeStates(_states);
    ros::spinOnce();
  }

  return 0;
}
