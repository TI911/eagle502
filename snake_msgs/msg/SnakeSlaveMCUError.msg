###########################################################
# MCUのエラーの内容に関するmsg型
# エラーが起きているところがtrue
###########################################################

time timestamp
uint8 mcu_id
bool reseted_by_power_on
bool reseted_by_software
bool reseted_by_low_power
bool reseted_by_watch_dog_timer
bool voltage
