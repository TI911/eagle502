/*
 * snake_control.h
 *
 *  Created on: Feb 21, 2017
 *      Author: ubuntu-ti
 */

#ifndef SNAKE_CONTROL_SRC_SNAKE_CONTROL_H_
#define SNAKE_CONTROL_SRC_SNAKE_CONTROL_H_

#include <ros/ros.h>
#include <joy_handler_ps3/JoySelectedData.h>

#include "snake_control_request.h"

#include "robot_spec.h"
#include "winding_gait.h"
#include "helical_wave_propagate_motion.h"
#include "inchworm_gait.h"
#include "sinus_lifting.h"
#include "pedal_wave_motion.h"
#include "sidewinding.h"

class SnakeControl {   //: public SnakebotControlRequest{
 public:
  enum ControlMode {
    CONTROL_MODE_INIT,
    CONTROL_MODE_ACTIVATE,
    CONTROL_MODE_MOVE,
  };
  enum GaitMode {
	GAIT_MODE_WINDING,
	GATI_MODE_SINUS_LIFTING,
	GAIT_MODE_PEDAL_WAVE,
	GAIT_MODE_SIDEWINDING,
	GAIT_MODE_LATERAL_UNDULATION,
	GAIT_MODE_HELICAL_CURVE,
	GAIT_MODE_LATERAL_ROLLING,
	GAIT_MODE_TEST,

  };

  static void Initialize() {
    SnakeControlRequest::Initialize();
  }

  static double loop_rate(){ return loop_rate_; }

  static double loop_rate_;  	 // [Hz]
  static double sampling_time_;  // [s] サンプリングタイム loop_rateの逆

  static void CallBackOfJoySelectedData(joy_handler_ps3::JoySelectedData joy_data);

  static void OperateMoveWindingShift(joy_handler_ps3::JoySelectedData joy_data);
  static void OperateMoveHelicalWavePropagateMotion(joy_handler_ps3::JoySelectedData joy_data);
  static void OperateMoveInchwormGait(joy_handler_ps3::JoySelectedData joy_data);
  static void OperateMovePedalWaveMotion(joy_handler_ps3::JoySelectedData joy_data);
  static void OperateMoveSinusLifting(joy_handler_ps3::JoySelectedData joy_data);
  static void OperateMoveSideWinding(joy_handler_ps3::JoySelectedData joy_data);

  static void OperateMoveTest(joy_handler_ps3::JoySelectedData joy_data);

  static ControlMode control_mode_;
  static GaitMode    gait_mode_;

  static WindingGait     winding_gait_;
  static HelicalWavePropagateMotion helical_wave_propagate_motion_;
  static InchwormGait    inchworm_gait_;
  static PedalWaveMotion pedal_wave_motion_;
  static SinusLifting    sinus_lifting_;
  static SideWinding     sidewinding_;

};

#endif /* SNAKE_CONTROL_SRC_SNAKE_CONTROL_H_ */
