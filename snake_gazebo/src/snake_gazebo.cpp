#include "snake_gazebo.h"

#include <cmath>
#include <vector>

//const float MIN_JOYSTICK_ON = 0.1;  // ジョイスティックの値がこれより大きければ反応する

//--- Subscriber ---//
/******************************/
ros::Subscriber SnakeGazebo::sub_joint_command_for_gazebo_;
/******************************/

//--- Publisher ---//
/******************************/
ros::Publisher SnakeGazebo::pub_joint_command_to_gazebo_[NUM_JOINT];
/******************************/

/************************************************/
void SnakeGazebo::CallBackOfJointCommandForGazebo(const snake_msgs::snake_joint_command joint_command)
{
  if (joint_command.joint_index >= NUM_JOINT) {  // 対象が不適
    ROS_WARN("joint_index %d is not exist.", joint_command.joint_index);
    //return;
  }
	std_msgs::Float64 target_position;
	target_position.data = joint_command.target_position*(M_PI/180.0);
	pub_joint_command_to_gazebo_[joint_command.joint_index].publish(target_position);

}
/************************************************/
