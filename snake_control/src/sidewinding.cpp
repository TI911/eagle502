/*
 * sidewinding.cpp
 *
 *  Created on: Sep 22, 2016
 *      Author: TI
 */

#include <ros/ros.h>
#include <cmath>
#include "sidewinding.h"

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void SideWinding::set_kappa_0_pitch(double k_0_p)
{
	kappa_zero_pitch_ = k_0_p;
}

void SideWinding::set_kappa_0_yaw(double k_0_y)
{
	kappa_zero_yaw_   = k_0_y;
}


/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void SideWinding::set_l(double l)
{
	l_ = l;
}

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void SideWinding::set_bias(double bias)
{
	double kp_bias = 0.22;      // 操舵バイアス比例ゲイン
	double biasmax = M_PI/8;	// 最大バイアス 単位は[rad]
	double b = 0;

	b = bias* kp_bias * serpenoid_curve.v; // 操舵バイアス（速度にも比例してインクリメント）
	if(b > biasmax) b = biasmax;
	if(b < -biasmax) b = -biasmax;

	bias_ = b;
}


/*
 * @fn
 * @brief
 * @param
 * @paran
 * @return なし
 * @detail
*/
void SideWinding::set_v(double v)
{
	serpenoid_curve.v = v;
	s_ += v;
}

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void SideWinding::print_parameters()
{
	ROS_INFO("* -->  kappa_zero_pitch_ = [%4.3f ] *", kappa_zero_pitch_);
	ROS_INFO("* -->  kappa_zero_yaw_   = [%4.3f ] *", kappa_zero_yaw_);
	ROS_INFO("* -->                L   = [%4.3f ] *", l_);
	ROS_INFO("* -->                s   = [%4.3f ] *", s_);
	ROS_INFO("------------     SideWinding      ----------");
}

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void SideWinding::SideWindingShift(RobotSpec spec)
{
	while(s_ > (pre_s_ + step_s_)){  //

		SideWinding::CalculateCurvature();
		ShiftControlMethod::Shift_Param_Forward(spec);
		pre_s_ = pre_s_ + step_s_;
		SideWinding::CalculateTargetAngleToSideWinding(spec);
		
	}
	print_parameters();
}

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void SideWinding::CalculateCurvature(){

	double k   = (M_PI/2) * (pre_s_/l_);
	kappa_     = k;
}

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void SideWinding::CalculateTargetAngleToSideWinding(RobotSpec spec)
{
	double kappa = 0,
			kappa_0_pitch = 0,
			kappa_0_yaw   = 0;

	snake_model_param.angle.clear();

	for(int i=0; i<num_link_; i++){
		kappa   	  = snake_model_param.kappa[i];
		kappa_0_pitch = snake_model_param.kappa_zero_pitch[i];
		kappa_0_yaw   = snake_model_param.kappa_zero_yaw[i];

		if(i%2){   /* 奇数番目 pitch joints ? ロボットの構造による*/
			target_angle_ =
					//-2*link_length_*kappa_zero_yaw_*sin(kappa);
					-2*link_length_*kappa_0_yaw*sin(kappa);

		}else{ /* 偶数番目 yaw joints ? ロボットの構造による*/
			target_angle_ =
					// 2*link_length_*kappa_zero_pitch_*cos(kappa + M_PI/4);
					2*link_length_*kappa_0_pitch*cos(kappa + M_PI/4);
		}
		snake_model_param.angle.push_back(target_angle_);
	}
}
