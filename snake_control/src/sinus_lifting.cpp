/*
 * sinus_lifting.cpp
 *
 *  Created on:  2018
 *      Author: TI
 *
  *               サーペノイド曲線(serpenoid curve) により横うねり推進を実現する.
 *      参考文献:
 *       「森淳，山田浩也，広瀬茂男：“ 三次元索状能動体ACM-R3 の設計開発とその基礎操舵制御”，
 *      日本ロボット学会誌，Vol.23，No.7，pp.886-897，2005.」
  *               の式（18）により曲率を計算する．
 *      α は曲線のくねり角で，
 *      l は1=4 周期長さ，
 *      s は曲線上の位置である．
 *      bias は操舵量であリ
 */


#include "sinus_lifting.h"

void SinusLifting::set_alpha_pitch(double alpha_p)
{
	alpha_pitch_      = alpha_p;
	kappa_zero_pitch_ = (M_PI*alpha_pitch_) / (2*l_);
}

void SinusLifting::set_alpha_yaw(double alpha_y)
{
	alpha_yaw_      = alpha_y;
	kappa_zero_yaw_ = (M_PI*alpha_yaw_) / (2*l_);
}

void SinusLifting::set_l(double l)
{
	l_   = l;
	kappa_zero_yaw_   = (M_PI*alpha_yaw_) / (2*l_);
	kappa_zero_pitch_ = (M_PI*alpha_pitch_) / (2*l_);
}

/*
 * @fn
 * @brief  操舵量を調整する
 * @paran
 * @return なし
 * @detail
*/
void SinusLifting::set_bias(double bias)
{
	double kp_bias = 0.2;       // 操舵バイアス比例ゲイン
	double biasmax = M_PI/4;   // 最大バイアス 単位は[rad]
	double b = 0;

	b = bias* kp_bias;// * serpenoid_curve.v; // 操舵バイアス（速度にも比例してインクリメント）
	if (b >  biasmax) b =  biasmax;
	if (b < -biasmax) b = -biasmax;
	bias_ = b;
}

void SinusLifting::set_s(double s)
{
	if (s > 0) direction_ = FORWARD;
	if (s < 0) direction_ = BACK;

	s_ += s;
}

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void SinusLifting::print_parameters()
{
	ROS_INFO("* -->         alpha_yaw_ = [%4.3f ] *", alpha_yaw_);
	ROS_INFO("* -->       alpha_pitch_ = [%4.3f ] *", alpha_pitch_);
	ROS_INFO("* -->                  s = [%4.3f ] *", s_);
	ROS_INFO("* -->                 l_ = [%4.3f ] *", l_);
	ROS_INFO("* -->              bias_ = [%4.3f ] *", bias_);
	ROS_INFO("* -->  kappa_zero_pitch_ = [%4.3f ] *", kappa_zero_pitch_);
	ROS_INFO("* -->    kappa_zero_yaw_ = [%4.3f ] *", kappa_zero_yaw_);
	ROS_INFO("---------   Sinus Lifting   ---------");
}

/*
 * @fn
 * @brief
 *
 * @paran
 * @return なし
 * @detail
*/
void SinusLifting::ChangeDirection(RobotSpec spec)
{
	int NUM_JOINT = spec.num_joint();
	if (direction_==FORWARD) {
		double temp_s = (2 * l_ / M_PI)
				* asin( (2 * l_ * hold_data.shift_param[0].kappa_hold[0]) / ( M_PI * alpha_yaw_ ) );
		s_ = temp_s;
		pre_s_ = temp_s;

	} else if (direction_ == BACK) {
		int hold_num = (int)hold_data.shift_param[0].kappa_hold.size();
		//最後尾の曲率から，仮のsを求める
		double temp_s = (2 * l_ / M_PI)
				* asin( (2*l_ * hold_data.shift_param[NUM_JOINT-1].kappa_hold[hold_num-1]) / ( M_PI*alpha_yaw_ ) );
		s_ = temp_s;
		pre_s_ = temp_s;

		//最後尾のバイアスを参照する
		bias_ = hold_data.shift_param[NUM_JOINT-1].bias_hold[hold_num-1];
	}
	pre_direction_ = direction_;
}

void SinusLifting::SinusLiftingByShift(RobotSpec spec)
{
	if (direction_ != pre_direction_) SinusLifting::ChangeDirection(spec);
	if (direction_ == FORWARD) {
		while(s_ > (pre_s_ + step_s_)){  //

			SinusLifting::CalculateCurvature();
			ShiftControlMethod::Shift_Param_Forward(spec);
			SinusLifting::CalculateTargetAngle(spec);
			pre_s_ = pre_s_ + step_s_;
		}
	}else if (direction_ == BACK){
		while (s_ < (pre_s_ - step_s_)) {
			SinusLifting::CalculateCurvature();
			ShiftControlMethod::Shift_Param_Back(spec);
			SinusLifting::CalculateTargetAngle(spec);
			pre_s_ = pre_s_ - step_s_;
		}
	}
	print_parameters();
}

void SinusLifting::CalculateCurvature()
{
	double k   = (M_PI/2) * (pre_s_/l_);
	kappa_     = k;
}

void SinusLifting::CalculateTargetAngle(RobotSpec spec)
{
	double kappa = 0, bias = 0,
			kappa_0_pitch = 1,
			kappa_0_yaw   = 1;

	snake_model_param.angle.clear();

	for(int i=0; i<num_link_; i++){
		kappa   	  = snake_model_param.kappa[i];
		kappa_0_pitch = snake_model_param.kappa_zero_pitch[i];
		kappa_0_yaw   = snake_model_param.kappa_zero_yaw[i];
		bias          = snake_model_param.bias[i];

		if(i%2){   /* 奇数番目 pitch joints ? ロボットの構造による*/
			target_angle_ =
					-2*link_length_*kappa_0_pitch*sin(4*M_PI*kappa) + bias;

		}else{    /* 偶数番目 yaw joints ? ロボットの構造による*/
			target_angle_ =
					 2*link_length_*kappa_0_yaw*sin(2*M_PI*kappa) + bias;
		}
		snake_model_param.angle.push_back(target_angle_);
	}
}
