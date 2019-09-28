/*
 * configuration.h
 *
 *  Created on: 2017/01/24
 *      Author: isnake
 */

#ifndef SNAKE_ROBOT_MASTER_VREP_SRC_CONFIGURATION_H_
#define SNAKE_ROBOT_MASTER_VREP_SRC_CONFIGURATION_H_

// ロボットの構成の情報(joint_manager.h, imu_manager.hの代わり)
#define NUM_JOINT 23
#define NUM_IMU   1  // IMUは，Index番号がそのままリンクの番号
#define MAX_JOINT_TORQUE 4.0 // [Nm] 最大トルク
#define P_GAIN 1.0
#define I_GAIN 0.0
#define D_GAIN 0.0

//#define USE_COP_SENSOR  // CoPセンサを使う場合は定義しておく

// V-REP用の設定 V-REP上のモデルに合わせてオブジェクトの名前を設定
// V-REP上のオブジェクトの名前が「なんとか_OBJ_NAME*」であればよい．(*は数値，先頭が1)
#define LINK_OBJ_NAME "Link"  // V-REP上のリンクの名前の付け方．
#define JOINT_OBJ_NAME  "Joint"  // V-REP上の関節の名前の付け方．
#define FORCE_SENSOR_OBJ_NAME  "force_sensor"  // V-REP上の力センサの名前の付け方．
#define PROXIMITY_SENSOR_OBJ_NAME  "proximity_sensor"  // V-REP上の関節の名前の付け方．

#endif /* SNAKE_ROBOT_MASTER_VREP_SRC_CONFIGURATION_H_ */
