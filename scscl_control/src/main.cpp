/***
*  SCS115 舵机测试程序
*
*   ros控制舵机
*
*  作者：TI
*  日期：2019-09-16
*  地点：IMUN 502
*  舵机提供商：深圳飞特模型有限公司
*
*  2019-09-16 必须安装串口通信的ROS包，
*             命令：sudo apt-get-install ros-indigo-serial
*             modified by TI911
*
*  2019-09-23 串口通信成功 serial communication is ok. modified by TI911
*  2019-09-26 读/写舵机指令 通信成功 read write ok. modified by TI911
*
***/

#include <stdio.h>
#include <sys/time.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include "scscl_control.h"
#include "scscl_protocol.h"
#include "serial_com.h"

int main(int argc, char **argv)    //
{
	ros::init(argc, argv, "scscl_control_node");
	ROS_INFO("scscl_control node initialized !");

	ros::NodeHandle nh;


  SCSCLControl::Initialize();

	// 2019-09-27  test-1 测试单一舵机力矩使能指令 成功
  //SCSCLControl::SCSCLEnableTorque(10, 1);

	// 2019-09-27  test-2, 测试读舵机是否使能指令，未知。maybe you must to test READ function
  //int d = SCSCLControl::SCSCLReadTorqueEnable(10);
  //ROS_INFO("Read: %d" ,d);

///<<<<<<< HEAD
	// 2019-09-27  test-3, maybe you must to test READ function
	//SCSCLControl::SCSCLWritePosition(10, 700, 0, 1000);
//=======
	// 2019-09-27  test-3, 测试单一舵机写入目标角度指令，成功。maybe you must to test READ function
	//SCSCLControl::SCSCLWritePosition(10, 700, 0, 1000);
//>>>>>>> 81e1c718dd3f414fe2f5e30bf0b062fb9f81e22a

	//2019-09-279  test-4, 测试同步写入目标角度指令，成功。需要进一步测试
	uint8_t ID[] = {10, 11};
  SCSCLControl::SCSCLSyncWritePosition(ID, 2, 300, 0, 500);
  //指定循环的频率
  ros::Rate loop_rate(50);

	//ros::Publisher dis_pub = nh.advertise<hc-sr04::msgDis>(¨hc-sr04_msg¨, 100);
	//uint8_t data[] = {0xff, 0xff, 0x0a, 0x09, 0x04, 0x2a, 0x00, 0x03, 0x0e, 0x00, 0xe8, 0x03, 0xcb};
  //uint8_t data[] = {0xff, 0xff, 0x0a, 0x09, 0x03, 0x28, 0x00, 0x08, 0x00, 0x00, 0xe8, 0x03, 0xd5};
	//SerialCom::WriteData(data, 13);
	// uint8_t data[] = {0xff, 0xff, 0x0a, 0x09, 0x03, 0x2a, 0x00, 0x08, 0x00, 0x00, 0xe8, 0x03, 0xd5};

	//uint8_t data[] = {0xff, 0xff, 0x0a, 0x09, 0x03, 0x2a, 0x02, 0xbc, 0x00, 0x00, 0x01, 0xf4, 0x0c};
  //uint8_t data[] = {0xff, 0xff, 0x0a, 0x09, 0x03, 0x2a, 0x02, 0xbc, 0x00, 0x00, 0x01, 0xf4, 0x0c};
	//uint8_t data[] = {0xff, 0xff, 0x0a, 0x02, 0x01, 0xf2};
	//uint8_t read_buff[6] = {0};
  //SerialCom::WriteData(data, 13);
	uint8_t nDat[] = {0x02, 0xbc, 0x00, 0x00, 0x01, 0xf4};
	while(ros::ok()){
		//SerialCom::TestFunction(data, read_buff, 13);
		//SCSCLProtocol::writeBuf(0x0a, 0x2a, nDat, 0x06, 0x03);
		//ROS_INFO("Read: %x,%x,%x,%x,%x,%x" ,data[0],data[1],data[2],data[3],data[4],data[5]);
    //SerialCom::FlushAllBuf();
		//SerialCom::WriteData(data, 13);

        //处理ROS的信息，比如订阅消息,并调用回调函数
        ros::spinOnce();
        loop_rate.sleep();
	}

	return 0;

}
