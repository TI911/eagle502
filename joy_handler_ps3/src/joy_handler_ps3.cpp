/*
*  2018.12.18
*  for SONY PS3
*  Modified by TI
*  qi.w@imun.edu.cn
*
*
**/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <joy_handler_ps3/JoySelectedData.h>

// note on plain values:
// buttons are either 0 or 1
// button axes go from 0 to -1
// stick axes go from 0 to +/-1
// @ref http://wiki.ros.org/ps3joy

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19

class JoyHandlerPS3 {
 public:
  static void Initialize() {
    ros::NodeHandle node;
    sub_joy_ = node.subscribe("joy", 1, CallBackOfJoy);
    pub_joy_selected_data_ = node.advertise<joy_handler_ps3::JoySelectedData>("joy_selected_data", 1);
  }
  static void CallBackOfJoy(sensor_msgs::Joy joy) {
    joy_handler_ps3::JoySelectedData joy_selected_data;

    joy_selected_data.joy_stick_r_x_rightwards = -joy.axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS];
    joy_selected_data.joy_stick_r_y_upwards    =  joy.axes[PS3_AXIS_STICK_RIGHT_UPWARDS];
    joy_selected_data.joy_stick_l_x_rightwards = -joy.axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
    joy_selected_data.joy_stick_l_y_upwards    =  joy.axes[PS3_AXIS_STICK_LEFT_UPWARDS];

    if (joy.buttons[PS3_BUTTON_ACTION_CIRCLE]) joy_selected_data.button_circle     = true;
    if (joy.buttons[PS3_BUTTON_ACTION_TRIANGLE]) joy_selected_data.button_triangle = true;
    if (joy.buttons[PS3_BUTTON_ACTION_SQUARE]) joy_selected_data.button_square     = true;
    if (joy.buttons[PS3_BUTTON_ACTION_CROSS]) joy_selected_data.button_cross       = true;

    if (joy.buttons[PS3_BUTTON_REAR_RIGHT_1]) joy_selected_data.button_r1 = true;
    if (joy.buttons[PS3_BUTTON_REAR_RIGHT_2]) joy_selected_data.button_r2 = true;
    if (joy.buttons[PS3_BUTTON_REAR_LEFT_1]) joy_selected_data.button_l1  = true;
    if (joy.buttons[PS3_BUTTON_REAR_LEFT_2]) joy_selected_data.button_l2  = true;

    if (joy.buttons[PS3_BUTTON_STICK_RIGHT]) joy_selected_data.button_r3 = true;
    if (joy.buttons[PS3_BUTTON_STICK_LEFT]) joy_selected_data.button_l3 = true;

    if (joy.buttons[PS3_BUTTON_CROSS_UP]) joy_selected_data.cross_key_up = true;
    if (joy.buttons[PS3_BUTTON_CROSS_DOWN]) joy_selected_data.cross_key_down = true;
    if (joy.buttons[PS3_BUTTON_CROSS_LEFT]) joy_selected_data.cross_key_left = true;
    if (joy.buttons[PS3_BUTTON_CROSS_RIGHT]) joy_selected_data.cross_key_right = true;

    if (joy.buttons[PS3_BUTTON_SELECT]) joy_selected_data.button_select = true;
    if (joy.buttons[PS3_BUTTON_START]) joy_selected_data.button_start   = true;
    if (joy.buttons[PS3_BUTTON_PAIRING]) joy_selected_data.button_ps    = true;

    pub_joy_selected_data_.publish(joy_selected_data);
  }

 private:
  static ros::Subscriber sub_joy_;
  static ros::Publisher pub_joy_selected_data_;

};

ros::Subscriber JoyHandlerPS3::sub_joy_;
ros::Publisher JoyHandlerPS3::pub_joy_selected_data_;

int main(int argc, char **argv) {
  ros::init(argc, argv, "joy_handler_PS3");
  JoyHandlerPS3::Initialize();
  ROS_INFO("Joy handler SONY PS3 was initialized!");
  ros::spin();
}
