###########################################################
# 関節のエラーに関する情報を投げるためのmsg型
# エラーが起きているとtrue
###########################################################

time timestamp
uint8 joint_index         # index number of joint
bool cannot_receive_data  # MCU could not receive data from sarvo.
bool input_voltage        # Input voltage of sarvo is too low or too high.  Motor is forced to stop.
bool mcu_temperature      # Temperature of MCU of sarvo is too high. Motor is forced to stop.
bool motor_temperature    # Temperature of motor is too high. Motor is forced to stop.
bool overload             # Current of motor is too high.

