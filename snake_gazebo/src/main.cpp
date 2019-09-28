/*
 *
 *
 *
 * */

#include "snake_gazebo.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "snake_gazebo_node");
  SnakeGazebo::Initialize();
  ROS_INFO("Initialized : snake_gazebo_node");

  ros::Rate loop_rate(50);
  while(ros::ok()) {
    loop_rate.sleep();
    ros::spin();
  }
}
