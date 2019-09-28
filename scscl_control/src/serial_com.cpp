/***
*
*
*  作者：TI
*  日期：2019-09-16
*  地点：IMUN 502
*
*  2019-09-16 sudo apt-get-install ros-indigo-serial
*
*
***/

#include <stdio.h>
#include <sys/time.h>
#include <string>

#include <ros/ros.h>        //
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include "serial_com.h"


serial::Serial SerialCom::ser;
//--- Subscriber ---//
ros::Subscriber write_sub;

//--- Publisher ---//
ros::Publisher read_pub;

//回调函数
void SerialCom::CallBackOfWriteSub(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO_STREAM("Writing to serial port" <<msg->data);
	//ser.write(msg->data); //发送串口数据
}

void SerialCom::FlushAllBuf()
{
	ser.flush();
}

size_t SerialCom::GetBufSize()
{
	size_t n = ser.available();
	return n;
}

void SerialCom::WriteData(std::string buf)
{
	std::string buffer = (std::string)buf;
	ser.write(buffer);
}

size_t SerialCom::WriteData(const uint8_t *Data, size_t ln)
{
	//std::string buffer = (std::string)buf;
	return ser.write(Data, ln);
}

size_t SerialCom::ReadData(uint8_t *buf, size_t n)
{
	return ser.read(buf, n);
}

size_t SerialCom::TestFunction(uint8_t *buf, uint8_t *read_buf, size_t n)
{
	ser.write(buf, n);
	//return ser.read(ser.available(););
  //ser.flush();
	return ser.read(read_buf, GetBufSize());
	//return 0;
}
