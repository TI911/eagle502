###########################################################
# 関節に関するデータのためのmsg型
# 位置や速度，電流など，関節に関するどのデータにもこの型を使い，
# topic名で内容を区別する
###########################################################

time timestamp
uint8 joint_index
float64 value
