/*
 * inchworm_gait.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: ubuntu-ti
 *	Modefied Dec 7, 2018
 *			TI
 *			at IMUN
 */


#include "inchworm_gait.h"

//void InchwormGait::set_pitch(double c)
//{
//		c_ = c;
//		print_parameters();
//}

void InchwormGait::set_h(double h)
{
		h_ = h;
		a_ = rr_ + rp_ + h_;
		b_ = h_/2;
		print_parameters();
}

void InchwormGait::set_c(double c)
{
		c_ = c;
		print_parameters();
}

void InchwormGait::set_s(double s)
{
		s_ = s;
		print_parameters();
}

void InchwormGait::set_psi(double psi)
{
		psi_ = psi;
}

/*
 * @fn
 * @brief
 * @param
 * @paran
 * @return なし
 * @detail
*/
void InchwormGait::print_parameters()
{
		ROS_INFO("* -->    a = %4.3f *", a_    );
		ROS_INFO("* -->    b = %4.3f *", b_    );
		ROS_INFO("* -->    h = %4.3f *", h_    );
		ROS_INFO("* -->    c = %4.3f *", c_    );
		ROS_INFO("* -->    d = %4.3f *", d_    );
		ROS_INFO("* -->omega = %4.3f *", omega_);
		ROS_INFO("* -->    s = %4.3f *", s_    );
		ROS_INFO("* -->  psi = %4.3f *", psi_  );
		ROS_INFO("------------    Inchworm Gait    ----------");
}

void InchwormGait::InchwormGaitByShift(RobotSpec spec)
{

    	while(s_>(pre_s_+step_s_)){
       // ROS_INFO("************** 1 ***************");
			 	while(pre_s_+step_s_ > S_T){

					t_ = t_ + 0.1;
					S_T = RungeKutta(S_T, t_, t_+0.1, 100); // ルンゲクッタ法(初期条件x0, 区間[t_, t_+0.1], 分割数100-> 0.1/100 ->0.001

				}
				//D論
				CalculateCurvatureTorsion();

				ShiftControlMethod::Shift_Param_Forward(spec);
				CalculateTargetAngle(spec);
				pre_s_ = pre_s_ + step_s_;
   	 }
}


double InchwormGait::dxdt(double t, double x)
{
	double x1=0, y1=0, z1=0, st=0;
	double tt = t;

	//x1 = (pow(a_,2)*pow(omega_,2)*pow(sinh(phi_hyperbolic_-omega_*tt),2))/pow(cosh(phi_hyperbolic_-omega_*tt),4);
	//y1 = pow(radius_+a_/cosh(phi_hyperbolic_-omega_*tt),2);
	//z1 = pow(delta_,2)/(4*pow(M_PI,2));

	x1 = pow(sin(t_)*(a_-b_*sin(omega_*t_))+b_*omega_*cos(omega_*t_)*cos(t_),2);
	y1 = pow(cos(t_)*(a_-b_*sin(omega_*t_))-b_*omega_*cos(omega_*t_)*sin(t_),2);
	//z1 = pow(c_,2)*pow(d_*omega_*sin(omega_*t_)-1,2);
	z1 = pow(c_,2);

	double j=x1 + y1 + z1;
	return sqrt(j);
}

double InchwormGait::RungeKutta(double x0, double t0, double tn, int n)
{
    int i;
    double x, t, h, d1, d2, d3, d4;
    x = x0;
    t = t0;
    h = (tn - t0) /n;

    // 漸化式を計算
    for ( i=1; i <= n ; i++){
        t = t0 + i*h;
        d1 = dxdt(t,x);
        d2 = dxdt(t,x + d1*h*0.5);
        d3 = dxdt(t,x + d2*h*0.5);
        d4 = dxdt(t,x + d3*h);
        x += (d1 + 2 * d2 + 2 * d3 + d4)*(h/6.0);
    }
	return x;
}

/**
 *   CalculateCurvatureTorsion()  For InchwormGait
 *
 *   常螺旋曲線の曲率と捩率を計算するが，Helical　Wave　Curveの式を使用する．
 *   a_0_ = 0 の場合，ハイパボリック関数が入ってこないので
 *   常螺旋曲線と等しい
 *
 * */
void InchwormGait::CalculateCurvatureTorsion()
{
	  double num   = 0,
			 denom = 0;

	  double x[3], y[3], z[3];

	  // 1 回微分
	  x[0] = (-sin(t_))*(a_-b_*sin(omega_*t_))-b_*omega_*cos(omega_*t_)*cos(t_);
	  y[0] = cos(t_)*(a_-b_*sin(omega_*t_))-b_*omega_*cos(omega_*t_)*sin(t_);
	  //z[0] = (-c_)*(d_*omega_*sin(omega_*t_)-1);
		z[0] = -c_;


	  // 2 回微分
	  x[1] = b_*pow(omega_,2)*sin(omega_*t_)*cos(t_)-cos(t_)*(a_-b_*sin(omega_*t_))+2*b_*omega_*cos(omega_*t_)*sin(t_);
	  y[1] = b_*pow(omega_,2)*sin(omega_*t_)*sin(t_)-sin(t_)*(a_-b_*sin(omega_*t_))+(-2)*b_*omega_*cos(omega_*t_)*cos(t_);
	  //z[1] = (-c_)*d_*pow(omega_,2)*cos(omega_*t_);
		z[1] = 0;


	  // 3 回微分
	  x[2] = sin(t_)*(a_-b_*sin(omega_*t_))+b_*pow(omega_,3)*cos(omega_*t_)*cos(t_)+(-3)*b_*pow(omega_,2)*sin(omega_*t_)*sin(t_)+3*b_*omega_*cos(omega_*t_)*cos(t_);
	  y[2] = 3*b_*pow(omega_,2)*sin(omega_*t_)*cos(t_)-cos(t_)*(a_-b_*sin(omega_*t_))+b_*pow(omega_,3)*cos(omega_*t_)*sin(t_)+3*b_*omega_*cos(omega_*t_)*sin(t_);
	  //z[2] = c_*d_*pow(omega_,3)*sin(omega_*t_);
		z[2] = 0;

	  num =
			  sqrt(pow(y[0]*z[1] - z[0]*y[1], 2)
					  + pow(z[0]*x[1] - x[0]*z[1], 2)
					  + pow(x[0]*y[1] - y[0]*x[1], 2));

	  denom = pow(x[0]*x[0] + y[0]*y[0] + z[0]*z[0], 1.5);
	  kappa_ = num/denom;	  //曲率

	  num =
			  x[2]*(y[0]*z[1] - z[0]*y[1])
			+ y[2]*(z[0]*x[1] - x[0]*z[1])
			+ z[2]*(x[0]*y[1] - y[0]*x[1]);

	  denom =
			  pow(y[0]*z[1] - z[0]*y[1], 2)
			+ pow(z[0]*x[1] - x[0]*z[1], 2)
			+ pow(x[0]*y[1] - y[0]*x[1], 2);

	  double first_tau_helical_ = num/denom;
	  tau_ = tau_ + first_tau_helical_*step_s_;	  //捩率
}

void InchwormGait::CalculateTargetAngle(RobotSpec spec)
{

    snake_model_param.angle.clear();
	for(int i=0; i<num_link_; i++){
		if(i%2){ //(奇数番目)
			target_angle_ =
					-2*link_length_*snake_model_param.kappa[i]*sin(snake_model_param.tau[i]+ snake_model_param.psi[i]);
			snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle_);

		}else{  //(偶数番目)
			target_angle_ =
					2*link_length_*snake_model_param.kappa[i]*cos(snake_model_param.tau[i]+ snake_model_param.psi[i]);
			snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle_);
		}

	}
	usleep(1000*10);        // 制御に時間がかかるので1秒寝て待つ
}
