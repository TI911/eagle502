/*
 * snake_control.cpp
 *
 *  Created on: Feb 21, 2017
 *      Author: ubuntu-ti
 */

#include <ros/ros.h>
#include <cmath>
#include <vector>
#include "snake_control.h"

const float MIN_JOYSTICK_ON = 0.1;  // ジョイスティックの値がこれより大きければ反応する

//=== static メンバ変数の定義 ===============//
SnakeControl::GaitMode SnakeControl::gait_mode_       = SnakeControl::GAIT_MODE_TEST;
SnakeControl::ControlMode SnakeControl::control_mode_ = SnakeControl::CONTROL_MODE_INIT;

double SnakeControl::loop_rate_;
double SnakeControl::sampling_time_;

RobotSpec spec(
    /* num_joint             = */ 30,
    /* link_length_head [m]  = */ 0.20,
    /* link_length_body [m]  = */ 0.061,
    /* link_length_tail [m]  = */ 0.20,
    /* link_diameter [m]     = */ 0.07,
    /* max_joint_angle [rad] = */ 90.0*M_PI/180.0,
    /* max_joint_angle_velocity [rad/s] = */ 80.0*M_PI/180.0,
    /* odd_joint_is_yaw      = */ true

   );

WindingGait SnakeControl::winding_gait_(
    /* spec             = */ spec,
    /* num_ds_link_body = */ //32,   // 関節角を計算する積分の分割数
    /* min_alpha        = */ 0.10,
    /* max_alpha        = */ 0.4,
    /* min_l            = */ 1.5*spec.link_diameter(),
    /* max_l            = */ 0.7,
    spec.link_length_body()// 0.061
  );

HelicalWavePropagateMotion SnakeControl::helical_wave_propagate_motion_(
	    /* spec             = */ spec,
	    spec.link_length_body()// 0.061
);

InchwormGait SnakeControl::inchworm_gait_(
		/* spec             = */ spec,
	    spec.link_length_body()// 0.0905);

);

PedalWaveMotion SnakeControl::pedal_wave_motion_(
		/* spec             = */ spec,
	    spec.link_length_body()// 0.0905);

);

SinusLifting SnakeControl::sinus_lifting_(
		/* spec             = */ spec,
	    spec.link_length_body(),// 0.0905);
		SnakeControl::sampling_time_

);

SideWinding SnakeControl::sidewinding_(
		/* spec             = */ spec,
	    spec.link_length_body()// 0.0905);
);

extern joy_handler_ps3::JoySelectedData joystick;

//=== static メンバ変数の定義 終わり =========//

/** @fn
 * @brief コントローラーからのデータを受け取って動作を行う
 * @param joy_handler::JoySelectedData joy_data
 * @detail
 *  joy_selected_dataからボタン，ジョイスティックの状態を読み取りそれに応じた動作を行う
 */
void SnakeControl::CallBackOfJoySelectedData(joy_handler_ps3::JoySelectedData joy_data)
{
  memcpy(&joystick, &joy_data, sizeof(joy_data));
}

/** @fn
 * @brief 横うねり推進を行う
 * @param joy_handler::JoySelectedData joy_data
 * @detail
 *  joy_selected_dataからボタン，ジョイスティックの状態を読み取りそれに応じた動作を行う
 */
void SnakeControl::OperateMoveWindingShift(joy_handler_ps3::JoySelectedData joy_data)
{
	std::vector<double> target_joint_angle(spec.num_joint(), 0.0);  // 値0.0で初期化;

	const double ALPHA_CHANGING_SPEED = 0.1;	/*	alpha	*/
	if(joy_data.button_r1){
		winding_gait_.add_alpha(ALPHA_CHANGING_SPEED * sampling_time_);
	}else if(joy_data.button_r2){
		winding_gait_.add_alpha(-ALPHA_CHANGING_SPEED * sampling_time_);
	}

	const double L_CHANGING_SPEED = 0.1;	/*	曲線の1/4周期の長さ l 変化速度	*/
	if(joy_data.button_l1){
		winding_gait_.add_l(L_CHANGING_SPEED * sampling_time_);
	}else if(joy_data.button_l2){
		winding_gait_.add_l(-L_CHANGING_SPEED * sampling_time_);
	}

	const double V_CHANGING_SPEED = 0.5;		//体軸に沿った速度
	if(joy_data.joy_stick_l_y_upwards){
		double js_data = joy_data.joy_stick_l_y_upwards;
		winding_gait_.add_s(js_data*V_CHANGING_SPEED*sampling_time_);  //v*t=s
	}

	//const double BIAS_CHANGING_SPEED = 0.1;	//bias
	if(joy_data.joy_stick_r_x_rightwards){
		double js_data = joy_data.joy_stick_r_x_rightwards;
		winding_gait_.add_bias(js_data);
	}else{
		winding_gait_.add_bias(0);
	}

	winding_gait_.WindingShift(spec);
	target_joint_angle = winding_gait_.snake_model_param.angle;
	SnakeControlRequest::RequestJointSetPosition(target_joint_angle);
}

/** @fn
 * @brief Sinus-liftingを行う
 * @param joy_handler::JoySelectedData joy_data
 * @detail
 *  joy_selected_dataからボタン，ジョイスティックの状態を読み取りそれに応じた動作を行う
 */
void SnakeControl::OperateMoveSinusLifting(joy_handler_ps3::JoySelectedData joy_data)
{
	std::vector<double> target_joint_angle(spec.num_joint(), 0.0);  // 値0.0で初期化;

	const double ALPHA_CHANGING_SPEED = 1;	/*	alpha	*/
	if(joy_data.button_r1){
		sinus_lifting_.add_alpha_yaw(ALPHA_CHANGING_SPEED * sampling_time_);
	}else if(joy_data.button_r2){
		sinus_lifting_.add_alpha_yaw(-ALPHA_CHANGING_SPEED * sampling_time_);
	}

	if(joy_data.button_triangle){
		sinus_lifting_.add_alpha_pitch(ALPHA_CHANGING_SPEED * sampling_time_);
	}else if(joy_data.button_cross){
		sinus_lifting_.add_alpha_pitch(-ALPHA_CHANGING_SPEED * sampling_time_);
	}

	const double L_CHANGING_SPEED = 1;	/*	曲線の1/4周期の長さ l 変化速度	*/
	if(joy_data.button_l1){
		sinus_lifting_.add_l(L_CHANGING_SPEED * sampling_time_);
	}else if(joy_data.button_l2){
		sinus_lifting_.add_l(-L_CHANGING_SPEED * sampling_time_);
	}
/*
	if(joy_data.cross_key_up){
		sinus_lifting_.add_l_pitch(L_CHANGING_SPEED * sampling_time_);
	}else if(joy_data.cross_key_down){
		sinus_lifting_.add_l_pitch(-L_CHANGING_SPEED * sampling_time_);
	}
*/

	//const double BIAS_CHANGING_SPEED = 0.1;	//bias
	if(joy_data.joy_stick_r_x_rightwards){
		double js_data = joy_data.joy_stick_r_x_rightwards;
		sinus_lifting_.add_bias(js_data);
	}else{
		sinus_lifting_.add_bias(0);
	}

	const double V_CHANGING_SPEED = 0.5;		//体軸に沿った速度
	if(joy_data.joy_stick_l_y_upwards){
		double js_data = joy_data.joy_stick_l_y_upwards;
		sinus_lifting_.add_s(V_CHANGING_SPEED * js_data * sampling_time_);
	}

	sinus_lifting_.SinusLiftingByShift(spec);
	target_joint_angle = sinus_lifting_.snake_model_param.angle;
	SnakeControlRequest::RequestJointSetPosition(target_joint_angle);
}

/** @fn
 * @brief Pedal Wave推進を行う
 * @param joy_handler::JoySelectedData joy_data
 * @detail
 *  joy_selected_dataからボタン，ジョイスティックの状態を読み取りそれに応じた動作を行う
 */
void SnakeControl::OperateMovePedalWaveMotion(joy_handler_ps3::JoySelectedData joy_data)
{
	std::vector<double> target_joint_angle(spec.num_joint(), 0.0);  // 値0.0で初期化;

	const double kappa_0_CHANGING_SPEED = 1;	/*	曲線の1/4周期の長さ l 変化速度	*/
	if(joy_data.button_r1){
		pedal_wave_motion_.add_kappa_0_pitch(kappa_0_CHANGING_SPEED * sampling_time_);
	}else if(joy_data.button_r2){
		pedal_wave_motion_.add_kappa_0_pitch(-kappa_0_CHANGING_SPEED * sampling_time_);
	}
	if(joy_data.button_l1){
		pedal_wave_motion_.add_kappa_0_yaw(kappa_0_CHANGING_SPEED * sampling_time_);
	}else if(joy_data.button_l2){
		pedal_wave_motion_.add_kappa_0_yaw(-kappa_0_CHANGING_SPEED * sampling_time_);
	}

	const double L_CHANGING_SPEED = 1;	/*	曲線の1/4周期の長さ l 変化速度	*/
	if(joy_data.cross_key_up){
		pedal_wave_motion_.add_l(L_CHANGING_SPEED * sampling_time_);
	}else if(joy_data.cross_key_down){
		pedal_wave_motion_.add_l(-L_CHANGING_SPEED * sampling_time_);
	}

	const double V_CHANGING_SPEED = 0.01;		//体軸に沿った速度
	if(joy_data.joy_stick_l_y_upwards){
		double js_data = joy_data.joy_stick_l_y_upwards;
		pedal_wave_motion_.add_v(V_CHANGING_SPEED * js_data);
	}

	pedal_wave_motion_.PedalWaveMotionByShift(spec);
	target_joint_angle = pedal_wave_motion_.snake_model_param.angle;
	SnakeControlRequest::RequestJointSetPosition(target_joint_angle);
}

/** @fn
 * @brief Sidewinding 推進を行う
 * @param joy_handler::JoySelectedData joy_data
 * @detail
 *  joy_selected_dataからボタン，ジョイスティックの状態を読み取りそれに応じた動作を行う
 */
void SnakeControl::OperateMoveSideWinding(joy_handler_ps3::JoySelectedData joy_data)
{
	std::vector<double> target_joint_angle(spec.num_joint(), 0.0);  // 値0.0で初期化;

	const double kappa_0_CHANGING_SPEED = 0.1;	/*	曲線の1/4周期の長さ l 変化速度	*/
	if(joy_data.button_r1){
		sidewinding_.add_kappa_0_pitch(kappa_0_CHANGING_SPEED * sampling_time_);
	}else if(joy_data.button_r2){
		sidewinding_.add_kappa_0_pitch(-kappa_0_CHANGING_SPEED * sampling_time_);
	}
	if(joy_data.button_l1){
		sidewinding_.add_kappa_0_yaw(kappa_0_CHANGING_SPEED * sampling_time_);
	}else if(joy_data.button_l2){
		sidewinding_.add_kappa_0_yaw(-kappa_0_CHANGING_SPEED * sampling_time_);
	}

	const double L_CHANGING_SPEED = 0.1;	/*	曲線の1/4周期の長さ l 変化速度	*/
	if(joy_data.cross_key_up){
		sidewinding_.add_l(L_CHANGING_SPEED * sampling_time_);
	}else if(joy_data.cross_key_down){
		sidewinding_.add_l(-L_CHANGING_SPEED * sampling_time_);
	}

	const double V_CHANGING_SPEED = 0.01;		//体軸に沿った速度
	if(joy_data.joy_stick_l_y_upwards){
		double js_data = joy_data.joy_stick_l_y_upwards;
		sidewinding_.add_v(js_data*V_CHANGING_SPEED);
	}

	sidewinding_.SideWindingShift(spec);
	target_joint_angle = sidewinding_.snake_model_param.angle;
	SnakeControlRequest::RequestJointSetPosition(target_joint_angle);
}

/** @fn
 * @brief Inchworm gaitを行う
 * @param joy_handler::JoySelectedData joy_data
 * @detail
 *  joy_selected_dataからボタン，ジョイスティックの状態を読み取りそれに応じた動作を行う
 */
void SnakeControl::OperateMoveInchwormGait(joy_handler_ps3::JoySelectedData joy_data)
{
  std::vector<double> target_joint_angle(spec.num_joint(), 0.0);  // 値0.0で初期化;
  /*** h を増加  ***/
  if(joy_data.cross_key_right){
    inchworm_gait_.add_h(0.001);

      /*** h を減少  ***/
  }else if(joy_data.cross_key_left){
    inchworm_gait_.add_h(-0.001);
  }
  /*** c を増加  ***/
  if(joy_data.cross_key_up){
    inchworm_gait_.add_c(0.001);

  /*** c を減少  ***/
  }else if(joy_data.cross_key_down){
    inchworm_gait_.add_c(-0.001);
  }

  const double E_CHANGING_SPEED = 0.01;	/*	曲線の1/4周期の長さ l 変化速度	*/
  if(joy_data.button_r1){
  //	inchworm_gait_.add_pitch(E_CHANGING_SPEED * sampling_time_);
  }else if(joy_data.button_r2){
  //	inchworm_gait_.add_pitch(-E_CHANGING_SPEED * sampling_time_);
  }

  const double V_CHANGING_SPEED = 0.01;		//体軸に沿った速度
  if(joy_data.joy_stick_l_y_upwards!=0){
    inchworm_gait_.add_s(joy_data.joy_stick_l_y_upwards*V_CHANGING_SPEED);
        inchworm_gait_.InchwormGaitByShift(spec);
  }

  const double PSI_CHANGING_SPEED = 0.01;		//psiに沿った速度
  //if(joy_data.joy_stick_r_y_upwards!=0){
  //	inchworm_gait_.add_psi(joy_data.joy_stick_r_y_upwards*PSI_CHANGING_SPEED);
      //	inchworm_gait_.InchwormGaitToRolling(spec);
  //}
  //inchworm_gait_.InchwormGaitByShift(spec);
  target_joint_angle = inchworm_gait_.snake_model_param.angle;
  SnakeControlRequest::RequestJointSetPosition(target_joint_angle);
}


/** @fn
 * @brief Helical wave propagation motionを行う
 * @param joy_handler::JoySelectedData joy_data
 * @detail
 *  joy_selected_dataからボタン，ジョイスティックの状態を読み取りそれに応じた動作を行う
 */
void SnakeControl::OperateMoveHelicalWavePropagateMotion(joy_handler_ps3::JoySelectedData joy_data)
{
	std::vector<double> target_joint_angle(spec.num_joint(), 0.0);  // 値0.0で初期化;

	/*** 螺旋半径を変化させる      radius_   ***/
	const double RADIUS_CHANGING_SPEED = 0.01;	/*	半径を変化させる速度	*/
	if(joy_data.button_r1){
		helical_wave_propagate_motion_.add_radius(RADIUS_CHANGING_SPEED * sampling_time_);
	}else if(joy_data.button_r2){
		helical_wave_propagate_motion_.add_radius(-RADIUS_CHANGING_SPEED * sampling_time_);
	}

	/*** 螺旋ピッチを変化させる    b(t)= n*t/2*π  ***/
	const double DELTA_CHANGING_SPEED = 0.01;	/*	螺旋ピッチを変化させる速度	*/
	if(joy_data.button_l1){
		helical_wave_propagate_motion_.add_delta(DELTA_CHANGING_SPEED * sampling_time_);
	}else if(joy_data.button_l2){
		helical_wave_propagate_motion_.add_delta(-DELTA_CHANGING_SPEED * sampling_time_);
	}

	/***  phiを入れるflag_ ON にする,   A*sech(ω*t-Φ)  ***/
	if(joy_data.button_cross){
		helical_wave_propagate_motion_.set_flag_on();
	}

	/***  phiを入れるflag_ ON にする,   A*sech(ω*t-Φ)  ***/
	if(joy_data.button_circle){
		helical_wave_propagate_motion_.set_flag_off();
	}
    /*** A を増加  ***/
	if(joy_data.cross_key_right){
		helical_wave_propagate_motion_.add_a(0.001);

	/*** A を減少  ***/
	}else if(joy_data.cross_key_left){
		helical_wave_propagate_motion_.add_a(-0.001);
	}
	/*** omega を増加  ***/
	if(joy_data.cross_key_up){
		helical_wave_propagate_motion_.add_omega(0.1);

	/*** omega を減少  ***/
	}else if(joy_data.cross_key_down){
		helical_wave_propagate_motion_.add_omega(-0.1);
	}

	/***  螺旋曲線に沿った s を増加する   ***/
	if(joy_data.joy_stick_l_y_upwards!=0){
		helical_wave_propagate_motion_.add_s(joy_data.joy_stick_l_y_upwards * sampling_time_);
	}

	/***  ***/
	if(joy_data.joy_stick_r_y_upwards!=0){
		helical_wave_propagate_motion_.add_psi_g_ratio(joy_data.joy_stick_r_y_upwards* sampling_time_);
	}

	helical_wave_propagate_motion_.HelicalWavePropagateMotionByShift(spec);
	target_joint_angle = helical_wave_propagate_motion_.snake_model_param.angle;
	SnakeControlRequest::RequestJointSetPosition(target_joint_angle);
}


void SnakeControl::OperateMoveTest(joy_handler_ps3::JoySelectedData joy_data)
{
  std::vector<double> target_joint_angle(spec.num_joint(), 0.0);  // 値0.0で初期化;

  for (uint32_t i_joint=0; i_joint<spec.num_joint(); i_joint++) {  // 関節ごとにループ
    double A = 1.0;
    double omega = 0.05;
    double theta = 1.0;
    static double t = 0.0;

    if (joystick.joy_stick_l_y_upwards > 0) {
      t = t + sampling_time_;
    }
    if (joystick.joy_stick_l_y_upwards < 0) {
      t = t - sampling_time_;
    }
    if(i_joint%2){
    	target_joint_angle[i_joint] = A * sin(theta * i_joint + omega * t);
    }
  }

  static int init_flag = 0;
  if (gait_mode_ != GAIT_MODE_TEST){
    gait_mode_ = GAIT_MODE_TEST;
    init_flag = 1;
  }
  if (init_flag == 1){
    //SnakeControlRequest::RequestJointActivateAll();
    //SnakebotControlRequest::RequestJointSetPosition(target_joint_angle);
    for(int i = 0; i<spec.num_joint(); i++){
      //SnakebotControlRequest::RequestJointSetPositionRange(target_joint_angle, 0, i);
      ros::Duration(0.25).sleep();
    }
   // SnakebotControlRequest::RequestJointReadPositionAll();
    init_flag = 0;
    return;
  }else{
    //SnakeControlRequest::RequestJointActivateAll();
    ///SnakebotControlRequest::RequestJointSetPosition(target_joint_angle);
   // SnakebotControlRequest::RequestJointReadPositionAll();
  }
}
