###############################################################################
# snake_robot_master nodeに送るコマンド
# 実行したいコマンドをtrueにしてpublishすると実行される
# 送信するデータがあるものは該当するデータを埋めてからpublishすること
# 対象の関節をjoint_indexで指定するか，全ての関節を対象とする場合はtarget_allをtrueにする
#
# KONDO B3MシリーズとDynamixelシリーズで違いがある．
# 対応しているかどうかは各コマンドのコメントに記載する．
# KONDOが<K>, Dynamixelが<D>であり，書かれているものには対応している
###############################################################################

#--- 対称を指定
uint8 joint_index  # index number of joint
bool target_all  # 全ての関節を対象とする場合はこれをtrueにする．その場合joint_indexは無意味

#--- IDリスト
bool set_id_list  # Slave MCU　にサーボのIDを記録する
bool read_id_list  # Slave MCU　からサーボのIDリストを読み出す

#--- サーボの状態
bool ping  # <KD> サーボの存在確認
bool reset  # <K> サーボをリセットする
bool clear_error  # <KD>　エラーによる停止状態を解除する

bool change_mode_to_hold  # <K>　モーターブレーキモードにする
bool change_mode_to_free  # <KD>　モーターをフリーにする
bool change_mode_to_active  # <KD>　モーターの制御を有効化する．トルクが入る

#--- 書き込み
bool set_position  # <KD>　目標位置を指示する target_positionが必要
bool set_position_time  # <K>　目標位置と到達時間を指示する target_position, target_timeが必要
bool set_position_velosity  # <D>　目標位置とそこに向かう速さを指示する target_position, target_velosityが必要
float64 target_position  # [deg]
float64 target_time  # [sec]
float64 target_velosity  # [deg/sec]

# <KD>　PIDゲインを設定する．
# KONDOの場合はデフォルト値に対する割合，
# Dynamixelの場合はそのまま書き込まれる値
bool set_pid_gain  
uint32 p_gain  # KONDO:[%], Dynamixel:[-]
uint32 i_gain  # KONDO:[%], Dynamixel:[-]
uint32 d_gain  # KONDO:[%], Dynamixel:[-]

bool set_lock_parameter  # <K>　LOCK検出に関するパラメータを設定する
uint8 lock_time_value  # [msec] この時間だけlock_output_valueを越えるとLOCK判定
uint8 lock_output_value  # [%] lock_time_valueだけこの値を越えるとLOCK判定
uint8 lock_power_value  # [%] LOCK判定後，出力をこの割合に絞る

#--- 読み込み
bool read_position  # [deg] 位置の読み込み
bool read_velosity  # [deg/sec] 角速度の読み込み
bool read_current  # [A] モーター電流の読み込み
bool read_voltage  # [V] サーボへの入力電圧の読み込み
bool read_motor_temperature  # [degC] モーター温度の読み込み
bool read_position_velosity  # [deg][deg/sec] 位置と角速度の読み込み
bool read_position_current  # [deg][A] 位置と電流の読み込み
bool read_position_velosity_current  # [deg][deg/sec][A] 位置と角速度と電流の読み込み

#--- アドレスを指定してパラメータを操作
bool set_parameter_by_address
uint8 address_to_set
uint8 length_set  # 1~7 書き込むデータのバイト数
uint8[] data_to_set

bool read_parameter_by_address
uint8 address_to_read
uint8 length_read  # 1~7 読み込むデータのバイト数
