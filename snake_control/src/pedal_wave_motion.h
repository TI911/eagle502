/*
 *
 *
 * */

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "robot_spec.h"
#include "shift_control_method.h"

#define FORWARD 1
#define BACK    0

class PedalWaveMotion: public ShiftControlMethod {
 public:
	virtual ~PedalWaveMotion(){}
	PedalWaveMotion(RobotSpec spec,	double ds){

		ds_ = ds;

		num_link_ 	  = spec.num_joint();
		link_length_  = spec.link_length_body();
		target_angle_ = 0;

		kappa_zero_pitch_ = 3.0;
		kappa_zero_yaw_   = 3.0;

		l_      = (num_link_*link_length_);

		s_ 		= 0;
		S_T 	= 0;

		pre_s_  = 0;
		step_s_ = ds/28;

		kappa_ = 0;
		bias_  = 0;
		direction_ = pre_direction_= FORWARD;

		Init(spec);
	}

    void init();
    void ChangeDirection(RobotSpec spec);
    void PedalWaveMotionByShift(RobotSpec spec);
    void CalculateCurvature();
    void CalculateTargetAngle(RobotSpec spec);

	//--- 形状パラメータ変更
	void set_kappa_0_pitch(double kappa_0_pitch);
	void add_kappa_0_pitch(double kappa_0_pitch_add){ set_kappa_0_pitch(kappa_zero_pitch_ + kappa_0_pitch_add); }
	void set_kappa_0_yaw(double kappa_0_yaw);
	void add_kappa_0_yaw(double kappa_0_yaw_add){ set_kappa_0_yaw(kappa_zero_yaw_ + kappa_0_yaw_add); }
	void set_l(double l);
	void add_l(double l_add){ set_l(l_ + l_add); }
	void set_v(double v);
	void add_v(double v_add){ set_v(v_add); }

	void print_parameters();

	double target_angle_;
	int num_link_;
	double link_length_;

	double s_, l_;
	double pre_s_;
	double step_s_;
	double S_T;
	   int direction_, pre_direction_;

};

