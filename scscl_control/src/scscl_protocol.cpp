/***
*   communication protocol
*
*
*
*  作者：TI
*  日期：2019-09-16
*  地点：IMUN 502
*  舵机提供商：深圳飞特模型有限公司
*
***/

#include <stdint.h>
#include "scscl_protocol.h"
#include "serial_com.h"


static uint8_t	Level =1;  //舵机返回等级

int SCSCLProtocol::writeByte(uint8_t ID, uint8_t MemAddr, uint8_t bDat)
{
	SerialCom::FlushAllBuf();	//flushSCS();
	writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
	return Ack(ID);
}

int SCSCLProtocol::readByte(uint8_t ID, uint8_t MemAddr)
{
	uint8_t bDat;
	int Size = Read(ID, MemAddr, &bDat, 1);
	if(Size!=1){
		return -1;
	}else{
		return bDat;
	}
}

int SCSCLProtocol::readWord(uint8_t ID, uint8_t MemAddr)
{
	uint8_t nDat[2];
	int Size;
	uint16_t wDat;
	Size = Read(ID, MemAddr, nDat, 2);
	if(Size!=2)
		return -1;
	//wDat = SCS2Host(nDat[0], nDat[1]);
	return wDat;
}


//读指令
//舵机ID，MemAddr内存表地址，返回数据nData，数据长度nLen
int SCSCLProtocol::Read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen)
{
	int Size;
	uint8_t bBuf[5];
	SerialCom::FlushAllBuf(); //flushSCS();
  writeBuf(ID, MemAddr, &nLen, 1, INST_READ);
	if(SerialCom::ReadData(bBuf, 5)!=5){
		return 0;
	}
	Size = SerialCom::ReadData(nData, nLen);
	if(SerialCom::ReadData(bBuf, 1)){
		return Size;
	}
	return 0;
}

//舵机ID，MemAddr内存表地址，写入数据，写入长度,zhiling
void SCSCLProtocol::writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun)
{
	uint8_t i;
	uint8_t msgLen = 2;
	uint8_t bBuf[6];
	uint8_t CheckSum = 0;

	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = ID;
	bBuf[4] = Fun;
	if(nDat){
		msgLen += nLen + 1;
		bBuf[3] = msgLen;
		bBuf[5] = MemAddr;
		//writeSCS(bBuf, 6);
		//std::string data = (char*)bBuf;
		SerialCom::WriteData(bBuf, 6);
	}else{
		bBuf[3] = msgLen;
		//writeSCS(bBuf, 5);
		//std::string data = (char*)bBuf;
		SerialCom::WriteData(bBuf, 5);
	}
	CheckSum = ID + msgLen + Fun + MemAddr;
	if(nDat){
		for(i=0; i<nLen; i++){
			CheckSum += nDat[i];
			//SerialCom::WriteData(nDat[i], 1);

		}
		//writeSCS(nDat, nLen);
		//std::string data = (char*)nDat;
		SerialCom::WriteData(nDat, nLen);

	}
	CheckSum = ~CheckSum;
	//writeSCS(&CheckSum, 1);
	//std::string data;// = (char*)CheckSum;
	SerialCom::WriteData(&CheckSum, 1);

}

//同步写指令
//舵机ID[]数组，IDN数组长度，MemAddr内存表地址，写入数据，写入长度
void SCSCLProtocol::snycWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
	uint8_t i, j;
	uint8_t mesLen = ((nLen+1)*IDN+4);
	uint8_t checkSum = 0;
	uint8_t bBuf[7];
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = 0xfe;
	bBuf[3] = mesLen;
	bBuf[4] = INST_SYNC_WRITE;
	bBuf[5] = MemAddr;
	bBuf[6] = nLen;
	//writeSCS(bBuf, 7);
	SerialCom::WriteData(bBuf, 7);

	checkSum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;
	for(i=0; i<IDN; i++){
		//writeSCS(ID+i, 1);
		//writeSCS(nDat, nLen);
		SerialCom::WriteData(ID+i, 1);
		SerialCom::WriteData(nDat, nLen);

		checkSum += ID[i];
		for(j=0; j<nLen; j++){
			checkSum += nDat[j];
		}
	}
	checkSum = ~checkSum;
	//writeSCS(&checkSum, 1);
	SerialCom::WriteData(&checkSum, 1);
}



int SCSCLProtocol::Ack(uint8_t ID)
{
	if(ID != 0xfe && Level){
		uint8_t buf[6];
		uint8_t Size = SerialCom::GetBufSize(); //readSCS(buf, 6);
		if(Size!=6){
			return 0;
		}
	}
	return 1;
}
