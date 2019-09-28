/**
 * @file serial_com.h
 * @brief class for serial communication
 * @author TI
 * @date 2019/09/16
 * @detail
 */

#ifndef SERIAL_GOM_H_
#define SERIAL_GOM_H_

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include <ros/ros.h>
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string>

class SerialCom {
 public:

  static int Initialize() {
    	ros::NodeHandle nh_com;
	   // write_sub = nh_com.subscribe("write", 1000, CallBackOfWriteSub); //订阅主题，并配置回调函数
	   // read_pub  = nh_com.advertise<std_msgs::String>("read", 1000); //发布主题

      //设置串口属性，并打开串口
	   try{
		  ser.setPort("/dev/ttyUSB0");
      ser.setBaudrate(115200);  //1000000, 500000, 250000, 128000,115200,
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
	   }catch(serial::IOException& e) {
      ROS_ERROR_STREAM("Unable to open port ");
      return -1;
	   }

	    //检测串口是否已经打开，并给出提示信息
	   if(ser.isOpen()) {
        	ROS_INFO_STREAM("Serial Port initialized");
	   }else {
        	return -1;
	   }
   }

  static size_t TestFunction(uint8_t *buf, uint8_t *read_buf,size_t n);

  static void FlushAllBuf();
  static size_t GetBufSize();
  static void WriteData(std::string buf);
  static size_t WriteData(const uint8_t *Data, size_t ln);
  static size_t ReadData(uint8_t *buf, size_t n);
  static void CallBackOfWriteSub(const std_msgs::String::ConstPtr& msg);

 private:
  //--- Subscriber ---//
  static ros::Subscriber write_sub;
  //--- Publisher ---//
  static ros::Publisher read_pub;

  static serial::Serial ser; //声明串口对象

};

#endif /* SNAKE_GAZEBO_H_ */
