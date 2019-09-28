/*
 * dxl_xh_test.cpp
 *
 *  Created on: Mar 29, 2017
 *      Author: ubuntu-ti
 */

#include "dxl_xh_test.h"


void DXL_XH_Test::yaw_move(double goalpos){

	target_angle_ = target_angle_ + goalpos;
	snake_model_param.angle.insert(snake_model_param.angle.begin(), target_angle_);
	snake_model_param.angle.pop_back();

}

void DXL_XH_Test::pitch_move(double goalpos){

	snake_model_param.angle.insert(snake_model_param.angle.begin()+1, target_angle_);
	snake_model_param.angle.pop_back();

}

