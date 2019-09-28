/**
 * @file snake_gazebo.h
 * @brief class for gazebo
 * @author TI
 * @date 2016/09/6
 * @detail
 */

#ifndef SNAKE_GAZEBO_H_
#define SNAKE_GAZEBO_H_

#include <vector>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include <ros/ros.h>
#include <snake_msgs/snake_joint_command.h>


#define NUM_JOINT 39
class SnakeGazebo {
 public:
  
  static void Initialize() {
    ros::NodeHandle node_handle;
    char tilt_controller_path[64];
    
    for(int i=0; i<NUM_JOINT; i++){
        sprintf(tilt_controller_path, "/snake/joint%d_position_controller/command", i);
        pub_joint_command_to_gazebo_[i] = node_handle.advertise<std_msgs::Float64>(tilt_controller_path, 100);
        
        std_msgs::Float64 target_position;
        target_position.data = 0.0;
        pub_joint_command_to_gazebo_[i].publish(target_position);
        
    }
    sub_joint_command_for_gazebo_ = node_handle.subscribe("joint_command", 100, SnakeGazebo::CallBackOfJointCommandForGazebo);
  }

 private:
  static void CallBackOfJointCommandForGazebo(const snake_msgs::snake_joint_command joint_command);

  //--- Subscriber ---//
  static ros::Subscriber sub_joint_command_for_gazebo_;
  //--- Publisher ---//
  static ros::Publisher pub_joint_command_to_gazebo_[NUM_JOINT];

};

#endif /* SNAKE_GAZEBO_H_ */
