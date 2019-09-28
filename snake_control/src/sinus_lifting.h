/*
 *
 *
 *
 * */

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "robot_spec.h"
#include "shift_control_method.h"

#define FORWARD 1
#define BACK    0

class SinusLifting: public ShiftControlMethod {
 public:
	virtual ~SinusLifting(){}
	SinusLifting(RobotSpec spec,
			double ds,
			double delta_t
			){

		ds_ = ds;
		delta_t_ = delta_t;

		num_link_ 	  = spec.num_joint();
		link_length_  = spec.link_length_body();
		target_angle_ = 0;

		alpha_pitch_ = M_PI/4;
		alpha_yaw_   = M_PI;
		l_           = (num_link_*link_length_);
		//l_yaw_     = (num_link_*link_length_)/4;

		kappa_zero_pitch_ = (M_PI*alpha_pitch_) / (2*l_);
		kappa_zero_yaw_   = (M_PI*alpha_yaw_)   / (2*l_);

		kappa_ = 0;
		bias_  = 0;

		s_ 		= 0;
		S_T 	= 0;

		pre_s_  = 0;
		step_s_ = ds/28;

		direction_ = pre_direction_= FORWARD;

		Init(spec);
	}

	void SinusLiftingByShift(RobotSpec spec);
    void CalculateCurvature();
    void CalculateTargetAngle(RobotSpec spec);

	//--- 形状パラメータ変更
	void set_alpha_pitch(double alpha_pitch);
	void add_alpha_pitch(double alpha_pitch_add){ set_alpha_pitch(alpha_pitch_ + alpha_pitch_add); }
	void set_alpha_yaw(double alpha_yaw);
	void add_alpha_yaw(double alpha_yaw_add){ set_alpha_yaw(alpha_yaw_ + alpha_yaw_add); }

	void set_l(double l_);
	void add_l(double l_add){ set_l(l_ + l_add); }

/*  void set_l_yaw(double l_yaw);
	void add_l_yaw(double l_yaw_add){ set_l_yaw(l_yaw_ + l_yaw_add); }
*/
	void set_s(double s);
	void add_s(double s_add){ set_s(s_add); }

	void set_bias(double bias);
	void add_bias(double bias){ set_bias(bias); }

	void ChangeDirection(RobotSpec spec);
	void print_parameters();

	double target_angle_;
	int num_link_;
	double link_length_;

	double s_, l_, alpha_yaw_, alpha_pitch_;
	double delta_t_;
	double pre_s_;
	double step_s_;
	double S_T;
	int direction_, pre_direction_;

};

