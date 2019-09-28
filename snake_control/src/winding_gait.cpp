/*
 * winding_gati.cpp
 *
 *  Created on: Sep 22, 2016
 *      Author: TI
 *
  *               サーペノイド曲線(serpenoid curve) により横うねり推進を実現する.
 *      参考文献 「広瀬茂男：“ 生物機械工学”，工業調査会，1987」 の式（4.2）により曲率を計算する．
 *      α は曲線のくねり角で，
 *      l は1=4 周期長さ，
 *      s は曲線上の位置である．
 *      bias は操舵量であリ，横うねり推進の移動方向を調整することができる．
 */

#include <ros/ros.h>
#include <cmath>
#include "winding_gait.h"

/*
 * @fn
 * @brief  曲線のくねり角 α を調整する
 * @param  alpha --> 新たに入力するα
 * @return なし
 * @detail
*/
void WindingGait::set_alpha(double alpha)
{
	serpenoid_curve.alpha = alpha;
}

/*
 * @fn
 * @brief  周期長さ l を調整する
 * @param 
 * @return なし
 * @detail
*/
void WindingGait::set_l(double l) 
{
	serpenoid_curve.l = l;
}

/*
 * @fn
 * @brief  操舵量を調整する
 * @paran 
 * @return なし
 * @detail
*/
void WindingGait::set_bias(double bias)
{
	double kp_bias = 0.2;       // 操舵バイアス比例ゲイン
	double biasmax = M_PI/4;   // 最大バイアス 単位は[rad]
	double b = 0;

	b = bias* kp_bias;// * serpenoid_curve.v; // 操舵バイアス（速度にも比例してインクリメント）
	if (b >  biasmax) b =  biasmax;
	if (b < -biasmax) b = -biasmax;
	bias_ = b;
}

/*
 * @fn
 * @brief 計画した曲線に沿って長さｓ,
 *        入力は移動速度ｖより，
 *        計画した曲線に沿って移動した距離ｓを計算する．
 *
 * @paran
 * @return なし
 * @detail
*/
void WindingGait::set_s(double s)
{
	if (s > 0) direction_ = FORWARD;
	if (s < 0) direction_ = BACK;

	s_ += s;
}

/*
 * @fn
 * @brief
 *
 * @paran
 * @return なし
 * @detail
*/
void WindingGait::ChangeDirection(RobotSpec spec)
{
	int NUM_JOINT = spec.num_joint();
	if (direction_==FORWARD) {
		double temp_s = (2 * serpenoid_curve.l / M_PI)
				* asin( (2 * serpenoid_curve.l * hold_data.shift_param[0].kappa_hold[0]) / ( M_PI * serpenoid_curve.alpha) );
		s_ = temp_s;
		pre_s_ = temp_s;

	} else if (direction_ == BACK) {
		int hold_num = (int)hold_data.shift_param[0].kappa_hold.size();
		//最後尾の曲率から，仮のsを求める
		double temp_s = (2 * serpenoid_curve.l / M_PI)
				* asin( (2*serpenoid_curve.l * hold_data.shift_param[NUM_JOINT-1].kappa_hold[hold_num-1]) / ( M_PI*serpenoid_curve.alpha) );
		s_ = temp_s;
		pre_s_ = temp_s;

		//最後尾のバイアスを参照する
		bias_ = hold_data.shift_param[NUM_JOINT-1].bias_hold[hold_num-1];
	}
	pre_direction_ = direction_;
}

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void WindingGait::print_parameters()
{
	ROS_INFO("* -->  serpenoid_curve.alpha = [%4.3f rad] *", serpenoid_curve.alpha);
	ROS_INFO("* -->  serpenoid_curve.l     = [%4.3f m  ] *", serpenoid_curve.l);
	ROS_INFO("* -->                      s = [%4.3f m  ] *", s_);
	ROS_INFO("* -->                   bias = [%4.3f m  ] *", bias_);
	ROS_INFO("---------     Winding Gait     -------------");
}

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void WindingGait::WindingShift(RobotSpec spec)
{
	if (direction_ != pre_direction_) WindingGait::ChangeDirection(spec);

	if (direction_ == FORWARD) {
		while(s_ > (pre_s_ + step_s_)){

			WindingGait::CalculateCurvature();
			ShiftControlMethod::Shift_Param_Forward(spec);
			WindingGait::CalculateTargetAngleToWinding(spec);
			pre_s_ = pre_s_ + step_s_;
		}
	}else if (direction_ == BACK){
		while (s_ < (pre_s_ - step_s_)) {

			WindingGait::CalculateCurvature();
			ShiftControlMethod::Shift_Param_Back(spec);
			WindingGait::CalculateTargetAngleToWinding(spec);
			pre_s_ = pre_s_ - step_s_;
		}
	}
	print_parameters();
}

/*
 * @fn
 * @brief 参考文献 「広瀬茂男：“ 生物機械工学”，工業調査会，1987」 の式（4.2）により曲率を計算する．
 * @paran 
 * @return なし
 * @detail
*/
void WindingGait::CalculateCurvature()
{
	double a = (M_PI*serpenoid_curve.alpha) / (2*serpenoid_curve.l);
	double ss = (M_PI/2) * (pre_s_/serpenoid_curve.l);
	kappa_ = a*sin(ss);
}

/*
 * @fn
 * @brief 博士論文の式（3.1）により目標角度を計算する．
 * @paran 
 * @return なし
 * @detail
*/
void WindingGait::CalculateTargetAngleToWinding(RobotSpec spec)
{
	snake_model_param.angle.clear();
	for(int i=0; i<num_link_; i++){

		if(i%2){  /* (奇数番目) pitch joints?   */
			target_angle_ = 0;          //-2*link_length_*snake_model_param.kappa[i]*sin(snake_model_param.psi[i]);
										//横うねり推進ではroll 軸の回転はしないため，ψ(s)＝０になる．//そのため，計算上はpis_を消しでもいい．
										// また，平面運動ので   PITCH軸は動かないのでPICHT軸の目標角度を０にした．
		}else{    /* (偶数番目)   */
			target_angle_ =
					2*link_length_*snake_model_param.kappa[i] + snake_model_param.bias[i]; //*cos(snake_model_param.psi[i]);
		}
		snake_model_param.angle.push_back(target_angle_);
	}
}

/*------ END ------*/
