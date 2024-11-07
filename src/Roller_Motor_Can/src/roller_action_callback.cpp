#include "Roller_Action_Coder.h"
using namespace ros;
void Roller_Action_Coder::Can_state_Callback(const Roller_Motor_Can::can& msg)
{
Can_state=msg;
}
void Roller_Action_Coder::Hand_Stop_Callback(const std_msgs::Bool& Msg)
{
  Hand_stop_Msg=Msg;
}
void Roller_Action_Coder::Z_Up_Limit_Callback(const std_msgs::Bool& Msg)
{
  Z_Up_Limit_Msg=Msg;
}
void Roller_Action_Coder::Z_Down_Limit_Callback(const std_msgs::Bool& Msg)
{
  Z_Down_Limit_Msg=Msg;
}
void Roller_Action_Coder::Z_state_Callback(const std_msgs::Int64MultiArray& msg)
{
  /*Receive the topic msgs*/
  Z_state=msg;
}
