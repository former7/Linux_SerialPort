#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <cstdio>
#include <cstring>
#include "string"
#include "SerialPort.h"

using namespace std;

#define O_PI  		  3.1415926  //PI
#define WheelDiameter     0.071      //轮子直径
#define WheelWidth        0.286      //轮子间距
#define EncoderTicks      36864.0    //编码器线数
#define UpdateTime        0.02       //下位机的发送频率

#define FRAME_LENGTH      13         //数据帧的长度
#define DATA_LENGTH	  10	     //数据长度

#define Linear_Factor     1.08578    //线性矫正参数
#define Angular_Factor    0.985      //旋转校正参数

double x = 0,y = 0,theta = 0;        //小车的位置和角度
double vx = 0,vy = 0,vth = 0;        //小车的线速度和角速度
SerialPort serialport;     	     //串口
string encoderBuffer;		     //缓冲区
int  bufferSize = 0;		     //缓冲区大小
int  tmp_count = 0;
int  tmp_sum = 0;

/***************************************************
	    Twist转换为小车轮子速度信息
在此函数中：
把从cmd_vel主题中的Twist类型信息
转换为小车左右两轮的速度信息 单位:count/s
***************************************************/
void twistToWspd(const geometry_msgs::Twist& msg,float& lspd,float& rspd)
{
  geometry_msgs::Twist twist = msg;  
  float vel_x = twist.linear.x;
  float vel_theta = twist.angular.z;
  
  vel_theta = vel_theta ;//* (1 / Angular_Factor);

  float vel_left = 0,vel_right = 0;

  vel_left = vel_x - vel_theta * WheelWidth  / 2.0;
  vel_right = vel_x + vel_theta * WheelWidth / 2.0;

  //   m/s
  lspd = vel_left;
  rspd = vel_right;

  //   rad/s
  float factor = O_PI*WheelDiameter;
  lspd /= factor;
  rspd /= factor;

  //   count/s
  lspd *= EncoderTicks;
  rspd *= EncoderTicks;

  //   count/5ms
  lspd /= 200;
  rspd /= 200;
}

//下位发过来的码盘数据转换为里程计数据
/***************************************************
	       编码器转换为里程计
在此函数中：
把编码器信息转换为里程计数据
包括坐标、角度、速度、角速度
***************************************************/
void encoderToOdometry(int leftEncoderCounts, int rightEncoderCounts)
{
   int deltaLeft = leftEncoderCounts;  
   int deltaRight = rightEncoderCounts; 

   double deltaLeftdis = (double)deltaLeft / EncoderTicks * WheelDiameter * O_PI;
   double deltaRightdis = (double)deltaRight / EncoderTicks * WheelDiameter * O_PI;

   double deltaDis = 0.5f * (deltaLeftdis + deltaRightdis);
   double deltaTheta = (double)(deltaRightdis - deltaLeftdis) / WheelWidth;  
   ROS_INFO("deltaDis: %f",deltaDis);

   //修正参数
   deltaDis = deltaDis * Linear_Factor;
   deltaTheta = deltaTheta * Angular_Factor;

   double deltaX = deltaDis * (double)cos(theta);  
   double deltaY = deltaDis * (double)sin(theta);  
   
   x += deltaX;
   y += deltaY;
   theta += deltaTheta;

   vx = deltaX / UpdateTime;
   vy = deltaY / UpdateTime;
   vth = deltaTheta / UpdateTime;
   
   if(theta > O_PI)
     theta -= 2*O_PI;
   else if(theta < -O_PI)
     theta += 2*O_PI;
}

/***************************************************
	        cmd_vel回调函数
在此回调函数：
1.把Twist信息转换为小车两轮速度
2.通过串口把速度信息发给下位机
***************************************************/
void cmdVelCb(const geometry_msgs::Twist& msg)
{
  float lspd = 0,rspd = 0;
  tmp_sum ++;
  twistToWspd(msg,lspd,rspd);
  int len = serialport.sendWheelSpd(lspd,rspd);
  if(len != 11)
  {
     ROS_INFO("Received but Data lost");
  }
  else
  {
     ROS_INFO("received good L:%.1f R:%.1f",lspd,rspd);
     if(lspd > 10 || rspd > 10)
     tmp_count ++;  
  }
}

/***************************************************
    int dealWith(string& data,int& left,int& right)
在此函数中：

***************************************************/
int dealWith(string& data,int& left,int& right)
{
  if(data[FRAME_LENGTH  - 1] != 'E')
  {
    printf("\r\n dealwith:");
    for(int i = 0; i< FRAME_LENGTH ;i++)
    {
      printf("%d ",data[i]);
    }
    printf("\r\n");

    data.erase(0,1);
    int pos = data.find('S');
    if(pos != data.npos)
    {
      data.erase(0,pos);
    }
    else
    {
      data.clear();
    }
    return 0;
  }
 // printf("\r\n pos1 ");

 //校验数据 
 unsigned char temp = XorCode(data.substr(1,10),10);
 if(temp != (data[11] & 0x7f))
 {
   ROS_INFO("Xor: %d  %d",temp,data[11]&0x7f);
   data.erase(0,FRAME_LENGTH);
   return 0;
 }
// printf("\r\n pos2 ");
 unsigned char temp_data[DATA_LENGTH + 1];
 for(int i = 1;i< DATA_LENGTH + 1 ;i++)
    temp_data[i] = data[i];

 left = 0;
 for(int i = 0; i< 4;i++)
 {
   left = left*10 + temp_data[2+i] - '0';
 }
 //符号位
 if(temp_data[1] == 1)left = -left; 

 right = 0;
 for(int i = 0; i<4;i++)
 {
   right = right*10 + temp_data[i+7] - '0';
 }
 //符号位
 if(temp_data[6] == 1)right = -right;

 data.erase(0,FRAME_LENGTH);
 return 1;
}

/***************************************************
	        getEncoder回调函数
在此回调函数：
1.把Twist信息转换为小车两轮速度
2.通过串口把速度信息发给下位机
***************************************************/
void getEncoder(void )
{
   /*读取串口数据*/
   unsigned char temp_buff[15];
   memset(temp_buff,0,15);

   int len = serialport.readBuffer((char *)temp_buff,FRAME_LENGTH);
   if(len <= 0)
   {
      ROS_INFO("No information");
      return ;
   }
/*
   printf("rbuffer: %d  ",len);
   for(int i = 0; i< len;i++)
   {
      printf("%d ",temp_buff[i]);
   }
   printf("\r\n");
*/
   string temp_str((const char *)temp_buff);
   /* printf("temp_str: %d  ",temp_str.size());
   for(int i = 0;i<temp_str.size();i++)
   {
       printf("%d ",temp_str[i]);
   }
   printf("\r\n");
   */
   encoderBuffer = encoderBuffer + temp_str;
   bufferSize += len;

   /*进行处理*/
   if(bufferSize >= FRAME_LENGTH)
   {
      int pos = encoderBuffer.find('S');
      if(pos == encoderBuffer.npos)
      {
         encoderBuffer.clear();
	 bufferSize = 0;
      }
      encoderBuffer.erase(0,pos);   //把'S'前面的字符都去掉
      if(encoderBuffer.size() < FRAME_LENGTH)return ;
      /*
      printf("\r\n recevied data:");
      for(int i = 0; i< encoderBuffer.size();i++)
      {
          printf("%x ",encoderBuffer[i]);
      }
      printf("\r\n");
      */
      int left = 0,right = 0;
      int tag = dealWith(encoderBuffer,left,right);   //提取字符
      if(tag == 1)
      {
          ROS_INFO("decode good");
          encoderToOdometry(left,right);        //把编码器数值转换为里程计数据
          ROS_INFO("Enocder Data:left :%d right:%d",left,right);
      }
      else ROS_INFO("decode bad");
      printf(" x:%.1f y:%.1f theta:%.1f\r\n",x,y,theta*57.3);
   }
}

int main(int argc,char ** argv)
{
  unsigned char data[]={"serialport test"};
  ros::init(argc,argv,"OdometryNode");
  if(serialport.Open(0,115200) == UART_FALSE)
  {
     cout <<"ERROR:can not open serialpoint!!"<<endl;
     return 0;
  }
  ros::NodeHandle nodehandle;
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  geometry_msgs::Quaternion odom_quat;;
  nav_msgs::Odometry odom;
  ros::Publisher  odom_pub = nodehandle.advertise<nav_msgs::Odometry>("odom", 10);
  ros::Subscriber sub = nodehandle.subscribe("cmd_vel",1,cmdVelCb);
  ros::Rate rate(50);

  //tf坐标
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  //里程计数据
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  while(ros::ok())
  {
     ros::spinOnce();
     
     /*
     int len = serialport.writeBuffer(data,strlen((const char *)data));
     if(len == UART_FALSE)
     {
      ROS_INFO("%d ",strlen((const char*)data));
      ROS_INFO("can not send serialport !!!");
     }
     else
     {
       ROS_INFO("SEND GOOD");
     }
     */
     //读取编码器信息并转换为里程计数据     
     getEncoder();
     
     //发布TF消息
     ros::Time current_time = ros::Time::now();
     odom_quat = tf::createQuaternionMsgFromYaw(theta);
     odom_trans.header.stamp = current_time;
     odom_trans.transform.translation.x = x;
     odom_trans.transform.translation.y = y;
     odom_trans.transform.translation.z = 0.0;
     odom_trans.transform.rotation = odom_quat;
     odom_broadcaster.sendTransform(odom_trans);
    
     //发布里程消息
     odom.header.stamp = current_time;
     odom.pose.pose.position.x = x;
     odom.pose.pose.position.y = y;
     odom.pose.pose.position.z = 0.0;
     odom.pose.pose.orientation = odom_quat;
     
     odom.twist.twist.linear.x = vx;
     odom.twist.twist.linear.y = vy;
     odom.twist.twist.angular.z = vth;
     odom_pub.publish(odom);
     ROS_INFO("TF send");
     
     rate.sleep();
  }
  return 0;
}

