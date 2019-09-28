/***
*  SCS115 舵机测试程序
*
*   ros控制舵机
*
*  作者：TI
*  日期：2019-09-16
*  地点：IMUN 502
*  舵机提供商：深圳飞特模型有限公司
***/

#ifndef SCSCL_CONTROL_H_
#define SCSCL_CONTROL_H_

#include "serial_com.h"
/*波特率定义
*  0-7分别代表波特率:
*      1000000，500000，250000，128000，
*      115200，76800，57600，38400
*      (出厂默认是0, 即 1000000）
*/
#define		BAUD_RATE_1M		0
#define		BAUD_RATE_0_5M		1
#define		BAUD_RATE_250K		2
#define		BAUD_RATE_128K		3
#define		BAUD_RATE_115200	4
#define		BAUD_RATE_76800		5
#define		BAUD_RATE_57600		6
#define		BAUD_RATE_38400		7

//内存表定义
// Control table address
//-------EPROM(只读)--------
#define ADDR_SCSCL_VERSION_L    0
#define ADDR_SCSCL_VERSION_H    1
#define ADDR_SCSCL_MODEL_L      3
#define ADDR_SCSCL_MODEL_H      4

//-------EPROM(读写)--------
#define ADDR_SCSCL_ID                5
#define ADDR_SCSCL_BAUD_RATE         6
#define ADDR_SCSCL_RETURN_DELAY_TIME 7
#define ADDR_SCSCL_RETURN_LEVEL      8
#define ADDR_SCSCL_MIN_ANGLE_LIMIT_L 9
#define ADDR_SCSCL_MIN_ANGLE_LIMIT_H 10
#define ADDR_SCSCL_MAX_ANGLE_LIMIT_L 11
#define ADDR_SCSCL_MAX_ANGLE_LIMIT_H 12
#define ADDR_SCSCL_LIMIT_TEMPERATURE 13
#define ADDR_SCSCL_MAX_LIMIT_VOLTAGE 14
#define ADDR_SCSCL_MIN_LIMIT_VOLTAGE 15
#define ADDR_SCSCL_MAX_TORQUE_L      16
#define ADDR_SCSCL_MAX_TORQUE_H      17
#define ADDR_SCSCL_ALARM_LED         19
#define ADDR_SCSCL_ALARM_SHUTDOWN    20
#define ADDR_SCSCL_COMPLIANCE_P      21
#define ADDR_SCSCL_COMPLIANCE_D      22
#define ADDR_SCSCL_COMPLIANCE_I      23
#define ADDR_SCSCL_PUNCH_L           24
#define ADDR_SCSCL_PUNCH_H           25
#define ADDR_SCSCL_CW_DEAD           26
#define ADDR_SCSCL_CCW_DEAD          27
#define ADDR_SCSCL_PROTECT_TORQUE    37
#define ADDR_SCSCL_PROTECT_TIME      38
#define ADDR_SCSCL_OVLOAD_TORQUE     39

//-------SRAM(读写)--------
#define ADDR_SCSCL_TORQUE_ENABLE   40
#define ADDR_SCSCL_GOAL_POSITION_L 42
#define ADDR_SCSCL_GOAL_POSITION_H 43
#define ADDR_SCSCL_GOAL_TIME_L     44
#define ADDR_SCSCL_GOAL_TIME_H     45
#define ADDR_SCSCL_GOAL_SPEED_L    46
#define ADDR_SCSCL_GOAL_SPEED_H    47
#define ADDR_SCSCL_LOCK            48

//-------SRAM(只读)--------
#define ADDR_SCSCL_PRESENT_POSITION_L  56
#define ADDR_SCSCL_PRESENT_POSITION_H  57
#define ADDR_SCSCL_PRESENT_SPEED_L     58
#define ADDR_SCSCL_PRESENT_SPEED_H     59
#define ADDR_SCSCL_PRESENT_LOAD_L      60
#define ADDR_SCSCL_PRESENT_LOAD_H      61
#define ADDR_SCSCL_PRESENT_VOLTAGE     62
#define ADDR_SCSCL_PRESENT_TEMPERATURE 63
#define ADDR_SCSCL_MOVING              66

class SCSCLControl{

public:
  static void Initialize() {
    SerialCom::Initialize();
  }
    static int SCSCLWritePosition(uint8_t ID, uint16_t Position, uint16_t Time, uint16_t Speed);//普通写位置指令
    static int SCSCLRegisterWritePosition(uint8_t ID, uint16_t Position, uint16_t Time, uint16_t Speed);//异步写位置指令
    static void SCSCLSyncWritePosition(uint8_t ID[], uint8_t IDN, uint16_t Position, uint16_t Time, uint16_t Speed);//同步写位置指令
    static int SCSCLWheelMode(uint8_t ID);//多圈轮子模式
    static int SCSCLJointMode(uint8_t ID, uint16_t minAngle, uint16_t maxAngle);//普通伺服模式
    static int SCSCLReadPosition(uint8_t ID);//读位置
    static int SCSCLWriteSpeed(uint8_t ID, int16_t Speed);//多圈控制指令
    static int SCSCLEnableTorque(uint8_t ID, uint8_t Enable);//扭力控制指令
    static int SCSCLReadTorqueEnable(uint8_t ID);

    static void SCSCLRegWriteAction(void);//执行异步写指令
    static int SCSCLReadLoad(uint8_t ID);//读当输出负载
    static int SCSCLReadVoltage(uint8_t ID);//读电压
    static int SCSCLReadTemperature(uint8_t ID);//读温度
    static int SCSCLPing(uint8_t ID);//Ping指令


};


#endif
