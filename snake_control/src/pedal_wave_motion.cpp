


#include "pedal_wave_motion.h"

void PedalWaveMotion::set_kappa_0_pitch(double k_0_p)
{
	kappa_zero_pitch_ = k_0_p;
}

void PedalWaveMotion::set_kappa_0_yaw(double k_0_y)
{
	kappa_zero_yaw_   = k_0_y;
}

void PedalWaveMotion::set_l(double l) 
{
	l_ = l;
}

void PedalWaveMotion::set_v(double v)
{
	if (v > 0) direction_ = FORWARD;
	if (v < 0) direction_ = BACK;

	s_ = serpenoid_curve.v += v;
}

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void PedalWaveMotion::print_parameters()
{
	ROS_INFO("* -->  kappa_zero_pitch_ = [%4.3f ] *", kappa_zero_pitch_);
	ROS_INFO("* -->  kappa_zero_yaw_   = [%4.3f ] *", kappa_zero_yaw_);
	ROS_INFO("* -->                L   = [%4.3f ] *", l_);
	ROS_INFO("* -->                s   = [%4.3f ] *", s_);
	ROS_INFO("------   Pedal Wave Motion  ---------");
}

void PedalWaveMotion::PedalWaveMotionByShift(RobotSpec spec)
{
	int NUM_JOINT = spec.num_joint();

	if (direction_==FORWARD) {
		while(s_ > (pre_s_ + step_s_)){  //

			PedalWaveMotion::CalculateCurvature();
			ShiftControlMethod::Shift_Param_Forward(spec);
			PedalWaveMotion::CalculateTargetAngle(spec);
			pre_s_ = pre_s_ + step_s_;
		}

	}else if (direction_ == BACK){
		while (s_ < pre_s_) {

			PedalWaveMotion::CalculateCurvature();
			ShiftControlMethod::Shift_Param_Back(spec);
			PedalWaveMotion::CalculateTargetAngle(spec);
			pre_s_ = pre_s_ - step_s_;
		}
	}
	print_parameters();
}

void PedalWaveMotion::ChangeDirection(RobotSpec spec)
{

	int NUM_JOINT = spec.num_joint();
	if (direction_==FORWARD) {
		double temp_s = ( 2*l_*hold_data.shift_param[0].kappa_hold[0]) / M_PI;
		s_ = temp_s;
		pre_s_ = temp_s;

		kappa_zero_pitch_ = hold_data.shift_param[0].kappa_zero_pitch_hold[0];
		kappa_zero_yaw_   = hold_data.shift_param[0].kappa_zero_yaw_hold[0];

	} else if (direction_ == BACK) {

		int hold_num = (int)hold_data.shift_param[0].kappa_hold.size();
		//最後尾の曲率から，仮のsを求める
		double temp_s = ( 2*l_*hold_data.shift_param[NUM_JOINT-1].kappa_hold[hold_num-1]) / M_PI;
		s_ = temp_s;
		pre_s_ = temp_s;

		//最後尾のバイアスを参照する
		kappa_zero_pitch_ = hold_data.shift_param[NUM_JOINT-1].kappa_zero_pitch_hold[hold_num-1];
		kappa_zero_yaw_   = hold_data.shift_param[NUM_JOINT-1].kappa_zero_yaw_hold[hold_num-1];

		bias_ = hold_data.shift_param[NUM_JOINT-1].bias_hold[hold_num-1];
	}
	pre_direction_ = direction_;
}

void PedalWaveMotion::CalculateCurvature()
{
	double k   = (M_PI/2) * (pre_s_/l_);
	kappa_      = k;
}

void PedalWaveMotion::CalculateTargetAngle(RobotSpec spec)
{
	snake_model_param.angle.clear();

	double kappa = 0,
			kappa_0_pitch = 1,
			kappa_0_yaw   = 1;

	snake_model_param.angle.clear();

	for(int i=0; i<num_link_; i++){
		kappa   	  = snake_model_param.kappa[i];
		kappa_0_pitch = snake_model_param.kappa_zero_pitch[i];
		kappa_0_yaw   = snake_model_param.kappa_zero_yaw[i];

		if(i%2){   /* 奇数番目 pitch joints ? ロボットの構造による*/
			target_angle_ =
					-2*link_length_*kappa_0_pitch*sin(2*M_PI*kappa);

		}else{    /* 偶数番目 yaw joints ? ロボットの構造による*/
			target_angle_ =
					 2*link_length_*kappa_0_yaw*sin(4*M_PI*kappa);
		}
		snake_model_param.angle.push_back(target_angle_);
	}
}
