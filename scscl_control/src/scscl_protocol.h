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
#ifndef SCSCL_PROTOCOL_H_
#define SCSCL_PROTOCOL_H_


#include <stdint.h>

#ifndef NULL
#define NULL ((void *)0)
#endif

#define INST_PING        0x01
#define INST_READ        0x02
#define INST_WRITE       0x03
#define INST_REG_WRITE   0x04
#define INST_ACTION      0x05
#define INST_RESET       0x06
#define INST_SYNC_WRITE  0x83
#define INST_ANGLE_RESET 0x0a

class SCSCLProtocol{

public:
  static int writeByte(uint8_t ID, uint8_t MemAddr, uint8_t bDat);  //Write one byte
  static int readByte(uint8_t ID, uint8_t MemAddr);
  static int readWord(uint8_t ID, uint8_t MemAddr);
  static int Read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen);
  static void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun);
  static int Ack(uint8_t ID);


};

#endif
