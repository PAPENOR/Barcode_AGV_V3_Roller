#include "tcpip_server.h"
void TcpIP_Server::Mission_stateCallback(const std_msgs::Int16::ConstPtr& msg)
{
  Mission_state=msg->data;
}
void TcpIP_Server::Mission_type_DirectionCallback(const std_msgs::Int16::ConstPtr& msg)
{
  State_Of_Hand.Direction=msg->data;
}
void TcpIP_Server::Mission_type_MotionCallback(const std_msgs::Int16::ConstPtr& msg)
{
  State_Of_Hand.Put_Or_Get=msg->data;
}
void TcpIP_Server::Mission_type_HighCallback(const std_msgs::Int16::ConstPtr& msg)
{
  State_Of_Hand.High=msg->data;
}
void TcpIP_Server::Goods_StateCallback(const std_msgs::Int16::ConstPtr& msg)
{
  State_Of_Hand.Good_state=msg->data;
}
void TcpIP_Server::publishGoodsStateMessage(ros::Publisher& pub, int value)
{
    std_msgs::Int16 goods_state_msg;
    goods_state_msg.data = value;
    pub.publish(goods_state_msg);
}
void TcpIP_Server::publishHandStopMessage(ros::Publisher& pub)
{
    std_msgs::Bool hand_stop_msg;
    hand_stop_msg.data = true;
    pub.publish(hand_stop_msg);
}
void TcpIP_Server::sendHandcommandMsg(const ros::Publisher& missionPub,int data1,int data2,int data3)
{
    std_msgs::Int64MultiArray commandMsg;
    commandMsg.data.push_back(data1);
    commandMsg.data.push_back(data2);
    commandMsg.data.push_back(data3);
    missionPub.publish(commandMsg);
}

