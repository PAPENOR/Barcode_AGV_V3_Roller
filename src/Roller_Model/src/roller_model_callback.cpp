#include "roller_model_motec.h"
void Roller_Model_Motec::Right_Limit_Callback(const std_msgs::Bool& Msg)
{
  Right_Limit_Msg=Msg;
}
void Roller_Model_Motec::Left_Limit_Callback(const std_msgs::Bool& Msg)
{
  Left_Limit_Msg=Msg;
}

void Roller_Model_Motec::Left_Stick_State_Postion_1_Callback(const std_msgs::Bool& Msg)
{
  Left_stick_state_postion_1_Msg=Msg;
}

void Roller_Model_Motec::Right_Stick_State_Postion_1_Callback(const std_msgs::Bool& Msg)
{
  Right_stick_state_postion_1_Msg=Msg;
}

void Roller_Model_Motec::Front_Stick_State_Postion_1_Callback(const std_msgs::Bool& Msg)
{
  Front_stick_state_postion_1_Msg=Msg;
}

void Roller_Model_Motec::Back_Stick_State_Postion_1_Callback(const std_msgs::Bool& Msg)
{
  Back_stick_state_postion_1_Msg=Msg;
}

void Roller_Model_Motec::Left_Stick_State_Postion_2_Callback(const std_msgs::Bool& Msg)
{
  Left_stick_state_postion_2_Msg=Msg;
}

void Roller_Model_Motec::Right_Stick_State_Postion_2_Callback(const std_msgs::Bool& Msg)
{
  Right_stick_state_postion_2_Msg=Msg;
}

void Roller_Model_Motec::Front_Stick_State_Postion_2_Callback(const std_msgs::Bool& Msg)
{
  Front_stick_state_postion_2_Msg=Msg;
}

void Roller_Model_Motec::Back_Stick_State_Postion_2_Callback(const std_msgs::Bool& Msg)
{
  Back_stick_state_postion_2_Msg=Msg;
}


void Roller_Model_Motec::Z_Up_Limit_Callback(const std_msgs::Bool& Msg)
{
  Z_Up_Limit_Msg=Msg;
}
void Roller_Model_Motec::Z_Down_Limit_Callback(const std_msgs::Bool& Msg)
{
  Z_Down_Limit_Msg=Msg;
}
void Roller_Model_Motec::Z_Zero_Callback(const std_msgs::Bool& Msg)
{
  Z_Zero_Msg=Msg;
}
void Roller_Model_Motec::Hand_Stop_Callback(const std_msgs::Bool& Msg)
{
  Hand_stop_Msg=Msg;
}

void Roller_Model_Motec::Left_Stick_Inp_Callback(const std_msgs::Bool& Msg)
{
  Left_Stick_Inp_Msg=Msg;
}
void Roller_Model_Motec::Right_Stick_Inp_Callback(const std_msgs::Bool& Msg)
{
  Right_Stick_Inp_Msg=Msg;
}
void Roller_Model_Motec::Front_Stick_Inp_Callback(const std_msgs::Bool& Msg)
{
  Front_Stick_Inp_Msg=Msg;
}
void Roller_Model_Motec::Back_Stick_Inp_Callback(const std_msgs::Bool& Msg)
{
  Back_Stick_Inp_Msg=Msg;
}

void Roller_Model_Motec::Left_Stick_Home_Callback(const std_msgs::Bool& Msg)
{
  Left_Stick_Home_Msg=Msg;
}
void Roller_Model_Motec::Right_Stick_Home_Callback(const std_msgs::Bool& Msg)
{
  Right_Stick_Home_Msg=Msg;
}
void Roller_Model_Motec::Front_Stick_Home_Callback(const std_msgs::Bool& Msg)
{
  Front_Stick_Home_Msg=Msg;
}
void Roller_Model_Motec::Back_Stick_Home_Callback(const std_msgs::Bool& Msg)
{
  Back_Stick_Home_Msg=Msg;
}

void Roller_Model_Motec::Z_action_Callback(const std_msgs::Int64MultiArray& Msg)
{
 if(Msg.data.size()>0)
  {
    Task_Order.Hand_Mode=Msg.data.at(0);
    if(Task_Order.Hand_Mode==0)
    {
      Task_Order.Hand_Speed_Z=Msg.data.at(1);//(unit mm)
      ROS_INFO("Speed mode:%d Z:%d"
               ,Task_Order.Hand_Mode
               ,Task_Order.Hand_Speed_Z);
    }
    if(Task_Order.Hand_Mode==1)
    {
      Task_Order.Hand_Position_Z=Msg.data.at(1);//(unit mm)
      ROS_INFO("Position mode:%d Z:%d"
               ,Task_Order.Hand_Mode
               ,Task_Order.Hand_Position_Z);
    }
    if(Task_Order.Hand_Mode==2)
    {
      Task_Order.Hand_Position_Z=Msg.data.at(1);//(unit mm)
      ROS_INFO("Position mode:%d Z:%d"
               ,Task_Order.Hand_Mode
               ,Task_Order.Hand_Position_Z);
    }
  }
}
void Roller_Model_Motec::Odom_Callback(const nav_msgs::Odometry& Msg)
{
State_Of_Hand.Z=Msg.pose.pose.position.z;
}
void Roller_Model_Motec::sendRoller_AxiscommandMsg(int data1)
{
    std_msgs::Int16 Stick_msg;
    Stick_msg.data  =data1;
    Roller_Axis_pub.publish(Stick_msg);
}
void Roller_Model_Motec::Stick_motion(bool Turn_on,ros::Publisher &Stick_pub)
{

  std_msgs::Int16 Stick_msg;

  if(Turn_on==true)
  {
   Stick_msg.data  =3;
  }
  else
  {
   Stick_msg.data  =4;
  }

  Stick_pub.publish(Stick_msg);
}
void Roller_Model_Motec::Stick_Off(ros::Publisher &Stick_pub)
{
    std_msgs::Int16 Stick_msg;
    Stick_msg.data  =0;
    Stick_pub.publish(Stick_msg);
}
