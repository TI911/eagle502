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

#include "control_robot_for_vrep.h"

#include <string>


//=== staticメンバの実体 ============================================================//
//--- V-REP用 ---//
std::vector<Vrep::JointObject> RobotControlForVrep::joint_;
std::vector<Vrep::Object> RobotControlForVrep::link_;
std::vector<Vrep::ForceSensor> RobotControlForVrep::force_sensor_;
std::vector<Vrep::ProximitySensor> RobotControlForVrep::proximity_sensor_;


//--- Subscriber ---//
ros::Subscriber RobotControlForVrep::sub_joint_command_for_vrep_;

bool RobotControlForVrep::is_static_;


void RobotControlForVrep::CallBackOfJointCommandForVrep(const snake_msgs::snake_joint_command joint_command)
{
	uint8_t joint_index = joint_command.joint_index;
	double target_position_deg = joint_command.target_position;

	if (joint_command.set_position){ RequestJointSetPosition(joint_index,false,target_position_deg);}
}



//=== メンバ関数の定義 ==============================================================//
//--- V-REP用 ---//
/** @fn
 * @brief ヘビ型ロボットに必要なhandleを取得する
 * @param なし
 * @return bool false:どこかで失敗
 * @detail
 */
bool RobotControlForVrep::GetHandle() {
  link_.resize(NUM_JOINT+1);
  joint_.resize(NUM_JOINT);
  //force_sensor_.resize(NUM_JOINT+1);
  //proximity_sensor_.resize(NUM_JOINT+1);

  bool is_some_error_in_getting_handles = false;
  // linkのhandleを取得，保存
  for (uint8_t i_link=0; i_link<NUM_JOINT+1; i_link++) {
    std::string obj_name = LINK_OBJ_NAME + std::to_string(i_link+1);
    if ( not link_[i_link].GetHandle( obj_name ) ) {
      is_some_error_in_getting_handles = true;
    }
  }

  // jointのhandleを取得
  for (uint8_t i_joint=0; i_joint<NUM_JOINT; i_joint++) {
    std::string obj_name = JOINT_OBJ_NAME + std::to_string(i_joint+1);
    if( not joint_[i_joint].GetHandle( obj_name ) ) {
      is_some_error_in_getting_handles = true;
    }
  }

#ifdef USE_COP_SENSOR
  // force_sensorのhandleを取得
  for (uint8_t i_link=0; i_link<NUM_JOINT+1; i_link++) {
    std::string obj_name = FORCE_SENSOR_OBJ_NAME + std::to_string(i_link+1);
    if( not force_sensor_[i_link].GetHandle( obj_name ) ) {
      is_some_error_in_getting_handles = true;
    }
  }

  // proximity_sensorのhandleを取得
  for (uint8_t i_link=0; i_link<NUM_JOINT+1; i_link++) {
    std::string obj_name = PROXIMITY_SENSOR_OBJ_NAME + std::to_string(i_link+1);
    if( not proximity_sensor_[i_link].GetHandle( obj_name ) ) {
      is_some_error_in_getting_handles = true;
    }
  }
#endif

  // エラーの有無を確認
  if (is_some_error_in_getting_handles){ return false; }
  return true;
}

//=== Joint Request ============================================================//
/** @fn
 * @brief 全関節に最大トルクを設定
 * @param double torque [Nm] 関節の目標角度
 * @return なし
 * @detail
 */
void RobotControlForVrep::SetJointMaxTorque(double torque) {
  for(Vrep::JointObject::ItrType itr_joint = joint_.begin(); itr_joint != joint_.end(); ++itr_joint) {
    itr_joint->SetMaxTorque(torque);
  }
}

/** @fn
 * @brief 指定した関節のPIDゲインを変更
 * @param uint8_t joint_index 対象の関節番号
 * @param bool target_all 全関節を対象とするときtrue.その場合はjoint_indexは無意味
 * @param uint16_t p_gain []
 * @param uint16_t i_gain []
 * @param uint16_t d_gain []
 * @return なし
 * @detail
 */
void RobotControlForVrep::RequestJointSetPIDGain(uint8_t joint_index, bool target_all, uint16_t p_gain, uint16_t i_gain, uint16_t d_gain) {
  float p_gain_vrep = double(p_gain);
  float i_gain_vrep = double(i_gain);
  float d_gain_vrep = double(d_gain);
  if (target_all) {
    for(Vrep::JointObject::ItrType itr_joint = joint_.begin(); itr_joint != joint_.end(); ++itr_joint) {
      itr_joint->SetPIDGain(p_gain_vrep, i_gain_vrep, d_gain_vrep);
    }
  }
  else joint_[joint_index].SetPIDGain(p_gain_vrep, i_gain_vrep, d_gain_vrep);
}

/** @fn
 * @brief 現在の関節角を目標関節角にする
 * @param なし
 * @return なし
 * @detail
 */
void RobotControlForVrep::SetJointPresentAngleTarget() {
  for(uint8_t i_joint=0; i_joint < joint_.size(); i_joint++) {
    joint_[i_joint].SetTargetAngle(joint_[i_joint].GetAngle(false), false);
  }
  Vrep::WaitForReachingLastCommand();
}

/** @fn
 * @brief 指定した関節に目標位置を設定
 * @param uint8_t joint_index 対象の関節番号
 * @param bool target_all 全関節を対象とするときtrue.その場合はjoint_indexは無意味
 * @param double target_position_deg [deg] 関節の目標角度
 * @return なし
 * @detail
 */
void RobotControlForVrep::RequestJointSetPosition(uint8_t joint_index, bool target_all, double target_position_deg) {
  if (target_all) {
    for(Vrep::JointObject::ItrType itr_joint = joint_.begin(); itr_joint != joint_.end(); ++itr_joint) {
      if(is_static_) { itr_joint->SetAngle(target_position_deg, /* unit_is_deg = */ true); }
      else { itr_joint->SetTargetAngle(target_position_deg, /* unit_is_deg = */ true); }
    }
  } else {
    if(is_static_) { joint_[joint_index].SetAngle(target_position_deg, /* unit_is_deg = */ true); }
    else { joint_[joint_index].SetTargetAngle(target_position_deg, /* unit_is_deg = */ true); }
  }
}
