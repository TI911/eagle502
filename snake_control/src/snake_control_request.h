/*
 * snake_control_request.h
 *
 *  Created on: Oct 4, 2016
 *      Author: ubuntu
 */

#ifndef snake_snake_CONTROL_SRC_snake_CONTROL_REQUEST_H_
#define snake_snake_CONTROL_SRC_snake_CONTROL_REQUEST_H_

#include <vector>
#include <ros/ros.h>
#include <snake_msgs/snake_joint_command.h>
#include <snake_msgs/snake_joint_data.h>

#include <snake_msgs/snake_joint_command4V2.h>

class SnakeControlRequest {
 public:
  static void Initialize() {
    ros::NodeHandle node_handle;
    pub_joint_command_         = node_handle.advertise<snake_msgs::snake_joint_command>("joint_command", 100);
    pub_joint_target_position_ = node_handle.advertise<snake_msgs::snake_joint_data>("joint_target_position", 100);

    pub_joint_command4V2_ = node_handle.advertise<snake_msgs::snake_joint_command4V2>("joint_command4V2", 100);

  }

  //--- Joint -----------------------//
  static void RequestJointPing(uint8_t joint_index);
  // 関節への動作要求
  static void RequestJointClearErrorAll();
  static void RequestJointActivate(uint8_t joint_index);
  static void RequestJointActivateAll();
  static void RequestJointFree(uint8_t joint_index);
  static void RequestJointFreeAll();
  static void RequestJointHold(uint8_t joint_index);
  static void RequestJointHoldAll();
  static void RequestJointReset(uint8_t joint_index);
  static void RequestJointResetAll();
  // 関節の情報の読み込み
  static void RequestJointReadPositionAll();
  static void RequestJointReadVelosityAll();
  static void RequestJointReadCurrentAll();
  static void RequestJointReadVoltageAll();
  static void RequestJointReadMotorTemperatureAll();
  static void RequestJointReadPositionVelosityAll();
  static void RequestJointReadPositionCurrentAll();
  static void RequestJointReadPositionVelosityCurrentAll();
  // 関節へのパラメータ書き込み
  static void RequestJointSetPosition(std::vector<double> joint_angle);
  static void RequestJointSetPositionRange(std::vector<double> joint_angle, int32_t start, int32_t last);
  static void RequestJointSetPIDGainAll(uint16_t p_gain, uint16_t i_gain, uint16_t d_gain);
  static void RequestJointSetLockParameterAll(uint8_t time_ms, uint8_t power, uint8_t output);

  //--- IMU -------------------------//
  static void PublishJointTargetPositionAllZero(uint8_t num_joint);

 private:
  //--- Publisher ---//
  static ros::Publisher pub_joint_target_position_;  // 汎用(表示などに使う)
  static ros::Publisher pub_joint_command_;          // ロボット用

  static ros::Publisher pub_joint_command4V2_;

};

#endif /* snake_snake_CONTROL_SRC_snake_CONTROL_REQUEST_H_ */
