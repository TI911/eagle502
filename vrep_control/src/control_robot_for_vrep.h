/**
 * @file contro_robot_for_vrep.h
 * @brief V-REP上のヘビ型ロボットを管理するためのクラス
 * @author	TI
 * @date 	2018/05/13
 * @detail
 *  物理シミュレータV-REP上のロボットを操作する．
 *  V-REPのremoteAPIを使用し，snake_robot_master.hとの共通部分を
 *  できるだけ再利用してシステムを構築する．
 *  V-REPのROS連携機能を使うよりも，remoteAPIを使う方が単純な気がする．
 */


#ifndef CONTROL_ROBOT_FOR_VREP_SRC_CONTROL_ROBOT_FOR_VREP_H_
#define CONTROL_ROBOT_FOR_VREP_SRC_CONTROL_ROBOT_FOR_VREP_H_

#include <ros/ros.h>
#include <vector>
#include "vrep.h"
#include "configuration.h"
#include <snake_msgs/snake_joint_command.h>


class RobotControlForVrep {
 public:
  static bool Initialize() {
    ros::NodeHandle node;
    ros::NodeHandle node_handle_private("~");
    node_handle_private.param("is_static", is_static_, false);

    //--- V-REP用 ---//
    if (not Vrep::Initialize()) { return false; }
    if (not GetHandle()) { return false; }
    for (uint8_t i=0; i<10; i++) {
      SetJointMaxTorque(MAX_JOINT_TORQUE);
      RequestJointSetPIDGain(0, true, P_GAIN, I_GAIN, D_GAIN);
      SetJointPresentAngleTarget();
      Vrep::WaitForReachingLastCommand();
    }
    if(not is_static_) {
      Vrep::StartSimulation();
    }

    sub_joint_command_for_vrep_ = node.subscribe("joint_command", 100, RobotControlForVrep::CallBackOfJointCommandForVrep);
  }
  
  static void SetJointMaxTorque(double torque);

 private:

  static std::vector<Vrep::JointObject>     joint_;
  static std::vector<Vrep::Object>          link_;
  static std::vector<Vrep::ForceSensor>     force_sensor_;
  static std::vector<Vrep::ProximitySensor> proximity_sensor_;

  //--- Subscriber ---//
  static ros::Subscriber sub_joint_command_for_vrep_;

  static void CallBackOfJointCommandForVrep(const snake_msgs::snake_joint_command joint_command);


  //--- V-REP用 ---//
  static bool GetHandle();
  static void SetJointPresentAngleTarget();

  static void RequestJointSetPosition(uint8_t joint_index, bool target_all, double target_position_deg);
  static void RequestJointSetPIDGain(uint8_t joint_index, bool target_all, uint16_t p_gain, uint16_t i_gain, uint16_t d_gain);

  static bool is_static_;  // シミュレーションを静的に行うとき，true

};

#endif /* CONTROL_ROBOT_FOR_VREP_SRC_CONTROL_ROBOT_FOR_VREP_H_ */

