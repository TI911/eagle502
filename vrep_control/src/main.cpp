
#include <stdint.h>
#include <ros/ros.h>

#include "control_robot_for_vrep.h"
#include "vrep.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "snake_robot_master_vrep_node");
  ros::NodeHandle node;
  RobotControlForVrep::Initialize();
  ROS_INFO("Initialized : snake_robot_master_vrep_node");
  while(ros::ok()) {
    ros::spin();
  }
}

