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


#include <stdint.h>
#include "scscl_control.h"
#include "scscl_protocol.h"

static uint8_t	End = 1;   //处理器大小端结构

//1个16位数拆分为2个8位数
//DataL为低位，DataH为高位
void Host2SCS(uint8_t *DataL, uint8_t* DataH, int Data)
{
	if(End){
		*DataL = (Data>>8);
		*DataH = (Data&0xff);
	}else{
		*DataH = (Data>>8);
		*DataL = (Data&0xff);
	}
}

//2个8位数组合为1个16位数
//DataL为低位，DataH为高位
int SCS2Host(uint8_t DataL, uint8_t DataH)
{
	int Data;
	if(End){
		Data = DataL;
		Data<<=8;
		Data |= DataH;
	}else{
		Data = DataH;
		Data<<=8;
		Data |= DataL;
	}
	return Data;
}


int SCSCLControl::SCSCLEnableTorque(uint8_t ID, uint8_t Enable)
{
	return SCSCLProtocol::writeByte(ID, ADDR_SCSCL_TORQUE_ENABLE, Enable);
}


int SCSCLControl::SCSCLReadTorqueEnable(uint8_t ID)
{
	return SCSCLProtocol::readByte(ID, ADDR_SCSCL_TORQUE_ENABLE);
}

//写位置指令
//舵机ID，Position位置，运行时间Time，速度Speed
int SCSCLControl::SCSCLWritePosition(uint8_t ID, uint16_t Position, uint16_t Time, uint16_t Speed)
{
	uint8_t buf[6];
	//flushSCS();

	Host2SCS(buf+0, buf+1, Position);
	Host2SCS(buf+2, buf+3, Time);
	Host2SCS(buf+4, buf+5, Speed);
	SCSCLProtocol::writeBuf(ID, ADDR_SCSCL_GOAL_POSITION_L, buf, 6, INST_WRITE);
	return SCSCLProtocol::Ack(ID);

}



//读位置，超时返回-1
int SCSCLControl::SCSCLReadPosition(uint8_t ID)
{
	return SCSCLProtocol::readWord(ID, ADDR_SCSCL_PRESENT_POSITION_L);
}



/*
//异步写位置指令
//舵机ID，Position位置，运行时间Time，速度Speed
int SCSCLControl::SCSCLRegisterWritePosition(uint8_t ID, uint16_t Position, uint16_t Time, uint16_t Speed)
{
	return writePos(ID, Position, Time, Speed, INST_REG_WRITE);
}

void SCSCLControl::SCSCLRegWriteAction(void)
{
	writeBuf(0xfe, 0, NULL, 0, INST_ACTION);
}

//写位置指令
//舵机ID[]数组，IDN数组长度，Position位置，运行时间Time，速度Speed
void SCSCLControl::SCSCLSyncWritePosition(uint8_t ID[], uint8_t IDN, uint16_t Position, uint16_t Time, uint16_t Speed)
{
	uint8_t buf[6];

	Host2SCS(buf+0, buf+1, Position);
	Host2SCS(buf+2, buf+3, 0);
	Host2SCS(buf+4, buf+5, Speed);
	snycWrite(ID, IDN, SCSCL_GOAL_POSITION_L, buf, 6);
}

//读位置，超时返回-1
int SCSCLControl::SCSCLReadPosition(uint8_t ID)
{
	return readWord(ID, SCSCL_PRESENT_POSITION_L);
}

//速度控制模式
int SCSCLControl::SCSCLWriteSpeed(uint8_t ID, int16_t Speed)
{
	if(Speed<0){
		Speed = -Speed;
		Speed |= (1<<10);
	}
	return writeWord(ID, SCSCL_GOAL_TIME_L, Speed);
}

//读负载，超时返回-1
int SCSCLControl::SCSCLReadLoad(uint8_t ID)
{
	return readWord(ID, SCSCL_PRESENT_LOAD_L);
}

//读电压，超时返回-1
 int SCSCLControl::SCSCLReadVoltage(uint8_t ID)
{
	return readByte(ID, SCSCL_PRESENT_VOLTAGE);
}

//读温度，超时返回-1
int SCSCLControl::SCSCLReadTemperature(uint8_t ID)
{
	return readByte(ID, SCSCL_PRESENT_TEMPERATURE);
}

//Ping指令，返回舵机ID，超时返回-1
int SCSCLControl::Ping(uint8_t ID)
{
	int Size;
	uint8_t bBuf[6];
	flushSCS();
	writeBuf(ID, 0, NULL, 0, INST_PING);
	Size = readSCS(bBuf, 6);
	if(Size==6){
		return bBuf[2];
	}else{
		return -1;
	}
}

int SCSCLControl::SCSCLWheelMode(uint8_t ID)
{
	uint8_t bBuf[4];
	bBuf[0] = 0;
	bBuf[1] = 0;
	bBuf[2] = 0;
	bBuf[3] = 0;
	return genWrite(ID, SCSCL_MIN_ANGLE_LIMIT_L, bBuf, 4);
}

int SCSCLControl::SCSCLJoinMode(uint8_t ID, uint16_t minAngle, uint16_t maxAngle)
{
	uint8_t bBuf[4];
	Host2SCS(bBuf, bBuf+1, minAngle);
	Host2SCS(bBuf+2, bBuf+3, maxAngle);
	return genWrite(ID, SCSCL_MIN_ANGLE_LIMIT_L, bBuf, 4);
}
*/
