/*
 * inchworm_gait.h
 *
 *  Created on: Apr 19, 2017
 *      Author: ubuntu-ti
 */

#ifndef SNAKE_CONTROL_SRC_INCHWORM_GAIT_H_
#define SNAKE_CONTROL_SRC_INCHWORM_GAIT_H_

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "robot_spec.h"
#include "shift_control_method.h"

class InchwormGait: public ShiftControlMethod{

public:
	virtual ~InchwormGait(){}

	InchwormGait(RobotSpec spec, double ds){

		ds_ = ds;
		pre_s_ = 0;
		step_s_= ds/28;
		s_ = 0;
		S_T = 0;
		t_ = 0;
		psi_ = 0;

		rr_ = 0.0575;
		rp_ = 0.035;
		h_  = 0.000;
		a_ = rr_ + rp_ + h_;
		b_ = h_/2.0;
		c_ = 0.05,
		d_ = 2.0,  //d
		omega_ = 0.5;

		num_link_ = spec.num_joint();
		target_angle_ = 0;
		link_length_ = 0.07;

		ShiftControlMethod::Init(spec);
		Init(spec);
	}

	void set_s(double s);
	void add_s(double add_s){set_s(add_s + s_); }
	void set_psi(double psi);
	void add_psi(double add_psi){set_psi(add_psi+psi_); }
	//void set_pitch(double pitch);
	//void add_pitch(double pitch_add){ set_pitch(c_ + pitch_add); }
	void InchwormGaitByShift(RobotSpec spec);
	void InchwormGaitToRolling(RobotSpec spec);
    	double RungeKutta(double x0, double t0, double tn, int n);
	double dxdt(double t, double x);

	void CalculateCurvatureTorsion();
	void CalculateTargetAngle(RobotSpec spec);

	void set_h(double h);
	void add_h(double h_add){ set_h(h_ + h_add);}
	void set_c(double c);
	void add_c(double c_add){ set_c(c_ + c_add);}

	void print_parameters();

	double s_, S_T, t_, psi_;
	double rr_, rp_, h_;
	double pre_s_;
	double step_s_;
	double target_angle_;
	int num_link_;
	double link_length_;
	/*20180525 at Kyoto Univ. */
	double a_, b_, c_, d_, omega_;

};

#endif /* SNAKE_CONTROL_SRC_INCHWORM_GAIT_H_ */
