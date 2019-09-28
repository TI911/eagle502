/*
 * shift_control_method.h
 *
 *  Created on: Feb 22, 2017
 *      Author: ubuntu-ti
 *
 *      2017/11/14 删除多余内容，如不必要的头文件，变量等
 *     			   规范变量命名，头文件内的变量 first_tau  --> first_tau_
 *     			   增加注释
 *     			   增加 Shift_Param_Back(RobotSpec spec)
 *
 *     			  => HOW TO MAKE max_hold_num ???
 *
 *      2018/05/01
 *
 *      2019/01/25
 */

#ifndef SNAKE_CONTROL_SRC_SHIFT_CONTROL_METHOD_H_
#define SNAKE_CONTROL_SRC_SHIFT_CONTROL_METHOD_H_

#include <ros/ros.h>
#include <vector>
#include <stdint.h>
#include <cmath>

#include "robot_spec.h"  // for link number

class ShiftControlMethod {

private:
	/***   サーペノイド曲線用パラメータ    ***/
   typedef struct {
	   double alpha_s;       // 生物機械工学の(3.10)式
	   double alpha;         // くねり角[rad]
	   double l;             // 曲線の1/4周期の長さ[m]
	   double v;

   }SERPENOID_CURVE;

      /***   ヘビの各関節の vector のなかのデータ,  シフトするパラメータ    ***/
   typedef struct{
	   std::vector<double> kappa_zero_pitch_hold;
	   std::vector<double> kappa_zero_yaw_hold;
	   std::vector<double> kappa_hold;
	   std::vector<double> tau_hold;
	   std::vector<double> bias_hold;
	   std::vector<double> psi_hold;
	   std::vector<double> psi_hyper_hold;

   }SHIFT_PARAM;

   /*** 角度保持 vector */
   typedef struct{
	   std::vector<SHIFT_PARAM> shift_param;
   }HOLD_DATA;

   /***   最終, ヘビ各関節へ送るパラメータ    ***/
   typedef struct{
	   std::vector<double> angle;  /*  関節角度 */
	   std::vector<double> bias;   /*  操舵バイアス   */
	   std::vector<double> kappa_zero_pitch; /* 式(3.9)の πs/2l をシフトするため*/
	   std::vector<double> kappa_zero_yaw;
	   std::vector<double> kappa;  /*  曲率  */
	   std::vector<double> tau;    /*  捩率  */
	   std::vector<double> psi;	   /*  常螺旋曲線の捻転量 */
	   std::vector<double> psi_hyper;  /* ハイパボリック曲線の捻転量 */

    }SNAKE_MODEL_PARAM;


public:
	virtual ~ShiftControlMethod(){}

	HOLD_DATA 			    hold_data;
	SERPENOID_CURVE     serpenoid_curve;
	SNAKE_MODEL_PARAM   snake_model_param;

	/* 各种移动模式在继承shift_control_method时，将计算后的曲率，扭率等保存到这里 */
	 /*  パラメータｔと螺旋曲線に沿った長さｓの関係により計算した曲率，捩率などをこの変数に保存する  */

	double kappa_;
	double kappa_zero_pitch_;
	double kappa_zero_yaw_;

	double tau_;
	double first_tau_;
	double tau_helical_;
	double tau_hyperbolic_;

	double psi_;
	double psi_hyper_;

	double bias_;

	double angle_;
	double ds_;

	void Init(RobotSpec spec);

	void Shift_Param_Forward(RobotSpec spec);  /* 計算した結果を行列の先頭からシフトする  */
	void Shift_Param_Back(RobotSpec spec);     /* 計算した結果を行列のしっぽからシフトする  */

	void ShiftParamTorsion(RobotSpec spec);
	void ShiftParamPsi(RobotSpec spec);
	void ShiftTargetAngle(RobotSpec spec);

	void ShiftParamCurvatureForHelicalWave(RobotSpec spec);
	void ShiftParamPsiHyperForHelicalWave(RobotSpec spec);
	void ShiftParamPsiForHelicalWave(RobotSpec spec);
  void ShiftParamPsiHyperForHelicalWaveForward(RobotSpec spec);
	void ShiftParamBias(RobotSpec spec);

};

#endif /* SNAKE_CONTROL_SRC_SHIFT_CONTROL_METHOD_H_ */
