#ifndef TCPIP_SERVER_H
#define TCPIP_SERVER_H
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Bool.h"
#include "param_init.h"
#include "ros/console.h"


using namespace std;
using namespace ros;
class Hand_state
{
public:
  int Direction=0;    //0 for None X;1 for L;2 for R
  int Put_Or_Get=0;   //0 for None;1 for P;2 for G
  int High=0;         //ABS
  int Good_state=0;   //0 for None Setting ;1=empty 2=box with nothing 3=full
};

class TcpIP_Server
{
protected:
    NodeHandle  Node_Handle;
    Publisher   Z_Command_Pub;
    Publisher   X_Stop_Pub;
    Publisher   Lora_power_Pub;
    Publisher   Clear_Command_Pub;
    Publisher   Mission_pub;
    Publisher   Goods_state_pub;
    Publisher Error_pub;
    Subscriber  Mission_state_Sub;
    Subscriber  Mission_type_Motion_Sub;
    Subscriber  Mission_type_Direction_Sub;
    Subscriber  Mission_type_High_Sub;
    Subscriber  Good_state_Sub;
    Hand_state  State_Of_Hand;
    int         Mission_state;
    int         Lora_ID=4;
    int         Lora_channel=23;
    string Hand_State_Command   = "(Hand_state)";
    string Hand_Initial_Command = "(Hand_initial)";
    string Hand_stop_Command    = "(Hand,M:Stop)";
    string Hand_open_Command    = "(Hand,M:Open)";
    string Hand_close_Command   = "(Hand,M:Close)";
    string Clear_Command        = "(Clear_Command)";
    string Exit_Command         = "(Exit)";
    string Good_Empty_Command   = "(Good:Empty)";
    string Good_Case_Command    = "(Good:Case)";
    string Good_Full_Command    = "(Good:Full)";
    string Recv_String;

public:
    TcpIP_Server(string Name);
    ~TcpIP_Server(void);
    void Mission_stateCallback(const std_msgs::Int16::ConstPtr& msg);
    void Mission_type_DirectionCallback(const std_msgs::Int16::ConstPtr& msg);
    void Mission_type_MotionCallback(const std_msgs::Int16::ConstPtr& msg);
    void Mission_type_HighCallback(const std_msgs::Int16::ConstPtr& msg);
    void Goods_StateCallback(const std_msgs::Int16::ConstPtr& msg);
    void Char_Left_Right(const char *Recvbuf, int Beginpoint, int Checkpoint, int &CommandX);
    void Char_Put_Get(const char *Recvbuf, int Beginpoint, int Checkpoint, int &CommandX);
    void Char_int(const char *Recvbuf, int Beginpoint, int Checkpoint, int &CommandX);
    void ProcessRecvbuf(char *Recvbuf, int &CommandD, int &CommandM, int &CommandHigh, ros::Publisher &XZ_Command_Pub, ros::Publisher &X_Stop_Pub);
    void setupSocket(int& LOG,int& listenfd, const int port, const string& handName);
    void publishGoodsStateMessage(ros::Publisher& pub, int value);
    void publishHandStopMessage(ros::Publisher& pub);
    void sendHandStateRequest(int direction, int putOrGet, int high, int missionState, int goodState, const string& handName, int &connectfd);
    void sendHandcommandMsg(const ros::Publisher& missionPub,int data1,int data2,int data3);
};

#endif // TCPIP_SERVER_H
