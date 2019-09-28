###############################################################################
# dxl_control nodeに送るコマンド
# 実行したいコマンドをtrueにしてpublishすると実行される
# 送信するデータがあるものは該当するデータを埋めてからpublishすること
# 対象の関節をjoint_indexで指定するか，全ての関節を対象とする場合はtarget_allをtrueにする
#
# Dynamixelが<D>であり，書かれているものには対応している
###############################################################################

#--- 対称を指定
uint8 joint_index  # index number of joint
bool target_all    # 全ての関節を対象とする場合はこれをtrueにする．その場合joint_indexは無意味

#--- 書き込み
bool set_position           # <D>　目標位置を指示する target_positionが必要

bool change_mode_to_free    # <D>　モーターをフリーにする
bool change_mode_to_active  # <D>　モーターの制御を有効化する．トルクが入る
bool clear_error            # <D>　エラーによる停止状態を解除する

float64[] target_position     # [deg]

# <D>　PIDゲインを設定する．
# Dynamixelの場合はそのまま書き込まれる値
bool set_pid_gain  
uint32 p_gain  #  Dynamixel:[-]
uint32 i_gain  #  Dynamixel:[-]
uint32 d_gain  #  Dynamixel:[-]

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
