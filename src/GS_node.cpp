///@file station_test_node.cpp
///@author Vinson Sheep (775014077@qq.com)
///@brief 用于发布指令
///@version 1.0
///@date 2021-10-15
///
///@copyright Copyright (c) 2021
///
#include "radio_proxy/Command.h"
#include "radio_proxy/FlightData.h"
#include "ros/ros.h"
#include "string"

ros::Publisher cmdPub;
radio_proxy::Command cmd;
ros::Subscriber fdSub;
ros::Timer mainloopTimer;
// 通过控制台与用户交互，根据用户输入的命令选择不同的操作，并将相应的命令消息发布到 "/GS_proxy/command" 话题
int main(int argc, char *argv[]) {
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "GS_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // initialize command publisher
  cmdPub = nh.advertise<radio_proxy::Command>("/GS_proxy/command", 10);

  cmd.id = 255;
  cmd.header.frame_id = "station";

  // main loop
  while (ros::ok()) {

    // menu
    int input;
    std::cout << "[1]Takeoff, [2]Move, [3]Land, [4]Formation, [5]Track [6]Hold: ";
    std::cin >> input;

    switch (input) {
    // take off
    case 1: {
      cmd.mission = radio_proxy::Command::TAKEOFF;
      break;
    }

    // move
    case 2: {
      std::cout << "Pless move to: (x, y, z, yaw) ";
      std::cin >> cmd.x >> cmd.y >> cmd.z >> cmd.yaw;
      cmd.mission = radio_proxy::Command::SETPOINT_LOCAL;
      break;
    }

    // land
    case 3: {
      cmd.mission = radio_proxy::Command::LAND;
      break;
    }

    // formation
    case 4: {
      cmd.mission = radio_proxy::Command::TRACKING;
      std::cout << "Formation distance (m): ";
      std::cin >> cmd.altitude;
      break;
    }

    // follow
    case 5: {
      std::cout << "Pless move to: (z, distance) ";
      std::cin >> cmd.z >> cmd.altitude;
      cmd.mission = radio_proxy::Command::SETPOINT_GPS;
      break;
    }

    // hold
    case 6: {
      cmd.mission = radio_proxy::Command::HOLD;
      break;
    }

    default:
      printf("\033[1;31;40m WRONG INPUT! \033[0m\n");
      continue;
    }
    cmd.header.stamp = ros::Time::now();
    cmdPub.publish(cmd);
    ros::spinOnce();
  }

  return 0;
}
