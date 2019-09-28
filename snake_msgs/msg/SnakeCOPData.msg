###############################################################################
# 鈴木先生のCenter of Presserセンサのやりとりにつかうメッセージ
# joint_indexは、センサーのついているジョイント番号
# １つのセンサで半周のものが２つあるので、cop_indexは0,1の２つ
# 半周の中に8ch分の12bitデータがあるらしいので、uint16のデータを８個ならべる
###############################################################################

time timestamp 
uint8 joint_index
uint8 cop_index
bool data_is_updated
uint16[] value

#uint16 value1
#uint16 value2
#uint16 value3
#uint16 value4
#uint16 value5
#uint16 value6
#uint16 value7
#uint16 value8

