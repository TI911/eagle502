###########################################################
# 読み込んだIMUのデータを成形するためのmsg型
# どのタイプのデータについてもこのmsg型を用いる
# 中身を更新したデータに関する *_is_updatedをtrueに変更する
###########################################################

time timestamp 
uint8 imu_index

# roll-pitch-yaw形式 [deg]
float64 roll
float64 pitch
float64 yaw
bool rpy_is_updated

# gyroセンサのデータ [deg/sec]
float64 gyro_x
float64 gyro_y
float64 gyro_z
bool gyro_is_updated

# 加速度センサデータ [m/s2]
float64 accel_x
float64 accel_y
float64 accel_z
bool accel_is_updated
