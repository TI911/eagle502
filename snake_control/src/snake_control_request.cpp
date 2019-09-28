/*
 * snake_control_request.cpp
 *
 *  Created on: Oct 4, 2016
 *      Author: ubuntu
 */

#include <ros/ros.h>
#include "snake_control_request.h"

//--- Publisher ---//
ros::Publisher SnakeControlRequest::pub_joint_target_position_;
ros::Publisher SnakeControlRequest::pub_joint_command_;

ros::Publisher SnakeControlRequest::pub_joint_command4V2_;  //for V2 snake (Dynamixel XH430)

/** @fn
 * @brief
 */
void SnakeControlRequest::RequestJointPing(uint8_t joint_index)
{
	snake_msgs::snake_joint_command joint_command;
	joint_command.joint_index = joint_index;
	joint_command.ping = true;
	pub_joint_command_.publish(joint_command);
}

/** @fn
 * @brief 全ての関節のエラーをクリアする
 */
void SnakeControlRequest::RequestJointClearErrorAll() {
  snake_msgs::snake_joint_command joint_command;
  joint_command.target_all = true;
  joint_command.clear_error = true;
  pub_joint_command_.publish(joint_command);
}

/** @fn
 * @brief 関節の制御を有効化する
 * @param uint8_t joint_index 関節番号
 */
void SnakeControlRequest::RequestJointActivate(uint8_t joint_index) {
  snake_msgs::snake_joint_command joint_command;
  joint_command.joint_index = joint_index;
  joint_command.change_mode_to_active = true;
  pub_joint_command_.publish(joint_command);

}

/** @fn
 * @brief 全ての関節の制御を有効化する
 */
void SnakeControlRequest::RequestJointActivateAll() {
  snake_msgs::snake_joint_command joint_command;
  joint_command.target_all = true;
  joint_command.change_mode_to_active = true;
  pub_joint_command_.publish(joint_command);

  snake_msgs::snake_joint_command4V2 joint_command4V2;
  joint_command4V2.target_all = true;
  joint_command4V2.change_mode_to_active = true;
  pub_joint_command4V2_.publish(joint_command4V2);

}

/** @fn
 * @brief 関節の制御を無効化してモーターをフリーにする
 * @param uint8_t joint_index 関節番号
 */
void SnakeControlRequest::RequestJointFree(uint8_t joint_index) {
  snake_msgs::snake_joint_command joint_command;
  joint_command.joint_index = joint_index;
  joint_command.change_mode_to_free = true;
  pub_joint_command_.publish(joint_command);

}

/** @fn
 * @brief 全ての関節の制御を無効化してモーターをフリーにする
 */
void SnakeControlRequest::RequestJointFreeAll() {
  snake_msgs::snake_joint_command joint_command;
  joint_command.target_all = true;
  joint_command.change_mode_to_free = true;
  pub_joint_command_.publish(joint_command);

  snake_msgs::snake_joint_command4V2 joint_command4V2;
  joint_command4V2.target_all = true;
  joint_command4V2.change_mode_to_free = true;
  pub_joint_command4V2_.publish(joint_command4V2);
}

/** @fn
 * @brief [KONDO用]関節のサーボをHold(モーターブレーキ)モードにする
 * @param uint8_t joint_index 関節番号
 */
void SnakeControlRequest::RequestJointHold(uint8_t joint_index) {
  snake_msgs::snake_joint_command joint_command;
  joint_command.joint_index = joint_index;
  joint_command.change_mode_to_hold = true;
  pub_joint_command_.publish(joint_command);
}

/** @fn
 * @brief [KONDO用]全ての関節のサーボをHold(モーターブレーキ)モードにする
 */
void SnakeControlRequest::RequestJointHoldAll() {
  snake_msgs::snake_joint_command joint_command;
  joint_command.target_all = true;
  joint_command.change_mode_to_hold = true;
  pub_joint_command_.publish(joint_command);
}

/** @fn
 * @brief [KONDO用]関節のサーボをリセットする
 * @param uint8_t joint_index 関節番号
 */
void SnakeControlRequest::RequestJointReset(uint8_t joint_index) {
  snake_msgs::snake_joint_command joint_command;
  joint_command.joint_index = joint_index;
  joint_command.reset = true;
  pub_joint_command_.publish(joint_command);
}

/** @fn
 * @brief [KONDO用]全ての関節のサーボをリセットする
 */
void SnakeControlRequest::RequestJointResetAll() {
  snake_msgs::snake_joint_command joint_command;
  joint_command.target_all = true;
  joint_command.reset = true;
  pub_joint_command_.publish(joint_command);
}

/** @fn
 * @brief 全ての関節の角度を読み込む
 */
void SnakeControlRequest::RequestJointReadPositionAll() {
  snake_msgs::snake_joint_command joint_command;
  joint_command.target_all = true;
  joint_command.read_position = true;
  pub_joint_command_.publish(joint_command);
}

/** @fn
 * @brief 全ての関節の角速度を読み込む
 */
void SnakeControlRequest::RequestJointReadVelosityAll() {
  snake_msgs::snake_joint_command joint_command;
  joint_command.target_all = true;
  joint_command.read_velosity = true;
  pub_joint_command_.publish(joint_command);
}

/** @fn
 * @brief 全ての関節の電流を読み込む
 */
void SnakeControlRequest::RequestJointReadCurrentAll() {
  snake_msgs::snake_joint_command joint_command;
  joint_command.target_all = true;
  joint_command.read_current = true;
  pub_joint_command_.publish(joint_command);
}

/** @fn
 * @brief 全ての関節のサーボへの入力電圧を読み込む
 */
void SnakeControlRequest::RequestJointReadVoltageAll() {
  snake_msgs::snake_joint_command joint_command;
  joint_command.target_all = true;
  joint_command.read_voltage = true;
  pub_joint_command_.publish(joint_command);
}

/** @fn
 * @brief 全ての関節のモーター温度を読み込む
 */
void SnakeControlRequest::RequestJointReadMotorTemperatureAll() {
  snake_msgs::snake_joint_command joint_command;
  joint_command.target_all = true;
  joint_command.read_motor_temperature = true;
  pub_joint_command_.publish(joint_command);
}

/** @fn
 * @brief 全ての関節の角度と角速度を読み込む
 */
void SnakeControlRequest::RequestJointReadPositionVelosityAll() {
  snake_msgs::snake_joint_command joint_command;
  joint_command.target_all = true;
  joint_command.read_position_velosity = true;
  pub_joint_command_.publish(joint_command);
}

/** @fn
 * @brief 全ての関節の角度と電流を読み込む
 */
void SnakeControlRequest::RequestJointReadPositionCurrentAll() {
  snake_msgs::snake_joint_command joint_command;
  joint_command.target_all = true;
  joint_command.read_position_current = true;
  pub_joint_command_.publish(joint_command);
}

/** @fn
 * @brief 全ての関節の角度と速度と電流を読み込む
 */
void SnakeControlRequest::RequestJointReadPositionVelosityCurrentAll() {
  snake_msgs::snake_joint_command joint_command;
  joint_command.target_all = true;
  joint_command.read_position_velosity_current = true;
  pub_joint_command_.publish(joint_command);
}

/** @fn
 * @brief 関節の目標角を設定
 * @param std::vector<double> joint_angle 目標関節角の配列 全関節の情報を含む
 * @return なし
 * @detail
 */
void SnakeControlRequest::RequestJointSetPosition(std::vector<double> joint_angle) {
  RequestJointSetPositionRange(joint_angle, 0, joint_angle.size()-1);
}

/** @fn
 * @brief 関節の目標角を設定
 * @param std::vector<double> joint_angle 目標関節角の配列 全関節の情報を含む
 * @param int32_t start 対象の関節の始めの番号
 * @param int32_t last 対象の関節の最後の番号
 * @return なし
 * @detail
 */
void SnakeControlRequest::RequestJointSetPositionRange(std::vector<double> joint_angle, int32_t start, int32_t last) {
  snake_msgs::snake_joint_command joint_command;
  snake_msgs::snake_joint_data joint_taeget_position;

  snake_msgs::snake_joint_command4V2 joint_command4V2;

  for (uint8_t i_joint = start; i_joint<=last; i_joint++) {
    joint_taeget_position.joint_index = i_joint;
    joint_taeget_position.value = joint_angle[i_joint]*180.0/M_PI;
    pub_joint_target_position_.publish(joint_taeget_position);

    joint_command.set_position = true;
    joint_command.joint_index = i_joint;
    joint_command.target_position = joint_angle[i_joint]*180.0/M_PI;
    pub_joint_command_.publish(joint_command);

    joint_command4V2.set_position=true;
    joint_command4V2.target_position.push_back(joint_angle[i_joint]*180.0/M_PI) ;
  }
  pub_joint_command4V2_.publish(joint_command4V2);
  //joint_command4V2.target_position.clear();
}


/** @fn
 * @brief サーボのPIDゲインを設定する．KONDOサーボとDYNAMIXELで値の意味が異なるので注意
 * @param uint16_t p_gain KONDO:[%], Dynamixel:[-]
 * @param uint16_t i_gain KONDO:[%], Dynamixel:[-]
 * @param uint16_t d_gain KONDO:[%], Dynamixel:[-]
 * @return なし
 * @detail
 *  KONDOの場合はデフォルト値に対する割合，
 *  Dynamixelの場合はそのまま書き込まれる値で設定する
 */
void SnakeControlRequest::RequestJointSetPIDGainAll(uint16_t p_gain, uint16_t i_gain, uint16_t d_gain) {
  snake_msgs::snake_joint_command joint_command;
  joint_command.set_pid_gain = true;
  joint_command.target_all = true;
  joint_command.p_gain = p_gain;
  joint_command.i_gain = i_gain;
  joint_command.d_gain = d_gain;
  pub_joint_command_.publish(joint_command);
}

/** @fn
 * @brief [KONDO用]サーボのロック検出関連のパラメータを設定する
 * @param uint8_t time_ms この時間，power_ratioを越えたらロックと判断
 * @param uint8_t power [%] time_msの時間だけこの値を越えたらロックと判断
 * @param uint8_t output [%] ロックと検出されたときに出力をこの割合にする
 * @return なし
 * @detail
 */
void SnakeControlRequest::RequestJointSetLockParameterAll(uint8_t time_ms, uint8_t power, uint8_t output) {
  snake_msgs::snake_joint_command joint_command;
  joint_command.target_all = true;
  joint_command.set_lock_parameter = true;
  joint_command.lock_time_value = time_ms;
  joint_command.lock_output_value = output;
  joint_command.lock_power_value = power;
  pub_joint_command_.publish(joint_command);
}

/** @fn
 * @brief 全ての関節の目標角を0として送信(ロボットにはいかない)
 * @param なし
 * @return なし
 * @detail
 */
void SnakeControlRequest::PublishJointTargetPositionAllZero(uint8_t num_joint) {
  snake_msgs::snake_joint_data joint_taeget_position;
  for (uint8_t i_joint = 0; i_joint<=num_joint; i_joint++) {
    joint_taeget_position.joint_index = i_joint;
    joint_taeget_position.value = 0.0;
    pub_joint_target_position_.publish(joint_taeget_position);
  }
}
