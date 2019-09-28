###########################################################
# JoyStickのデータの簡略化版
# maker : SONY PS3
#
# - buttonは全てbool型であり，押されていたらture
###########################################################

float32 joy_stick_r_x_rightwards  # 右ジョイスティック左右方向 右が正
float32 joy_stick_r_y_upwards     # 右ジョイスティック上下方向 上が正

float32 joy_stick_l_x_rightwards  # 左ジョイスティック左右方向 右が正
float32 joy_stick_l_y_upwards     # 左ジョイスティック上下方向 上が正

bool cross_key_right
bool cross_key_left
bool cross_key_up
bool cross_key_down

bool button_circle
bool button_triangle
bool button_cross
bool button_square

bool button_r1
bool button_r2
bool button_r3

bool button_l1
bool button_l2
bool button_l3

bool button_select
bool button_start
bool button_ps
