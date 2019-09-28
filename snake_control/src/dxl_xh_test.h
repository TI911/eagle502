/*
 * dxl_xh_test.h
 *
 *  Created on: Mar 29, 2017
 *      Author: ubuntu-ti
 */

#ifndef SNAKE_CONTROL_SRC_DXL_XH_TEST_H_
#define SNAKE_CONTROL_SRC_DXL_XH_TEST_H_

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "robot_spec.h"

#include "shift_control_method.h"

class DXL_XH_Test : public ShiftControlMethod {

public:
	virtual ~DXL_XH_Test(){}
	DXL_XH_Test(RobotSpec spec
			){

		target_angle_ = 0;

	}

	void yaw_move(double goalpos);

	void pitch_move(double goalpos);

	double target_angle_;

};



#endif /* SNAKE_CONTROL_SRC_DXL_XH_TEST_H_ */
