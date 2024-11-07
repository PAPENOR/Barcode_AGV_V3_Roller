#ifndef ROLLER_ACTION_CODER_H
#define ROLLER_ACTION_CODER_H
#include <ros/ros.h>
#include <ros/console.h>
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "Roller_Motor_Can/can.h"
#include <actionlib/server/simple_action_server.h>
#include <Roller_Motor_Can/Single_Action_CanAction.h>
#include "param_init.h"
using namespace ros;
using namespace std;
class Roller_Action_Coder
{
protected:
/*Define the topic type*/
NodeHandle                                                                  nh_;
Publisher                                                                   Roller_action_can_pub;
Subscriber                                                                  Roller_state_can_sub;
Subscriber                                                                  Roller_state_sub;
Subscriber                                                                  Z_Up_Limit_Sub;
Subscriber                                                                  Z_Down_Limit_Sub;
Subscriber                                                                  Hand_stop_sub;
actionlib::SimpleActionServer<Roller_Motor_Can::Single_Action_CanAction>    action_server_;
std::string                                                                 action_name_;
Roller_Motor_Can::Single_Action_CanFeedback                                 feedback_;
Roller_Motor_Can::Single_Action_CanResult                                   result_;
Roller_Motor_Can::can Can_state;
std_msgs::Bool Z_Up_Limit_Msg;
std_msgs::Bool Z_Down_Limit_Msg;
std_msgs::Bool Hand_stop_Msg;
std_msgs::Int64MultiArray Z_state;
int Z_RT_ID=517;
int Z_SDO_ID=1541;
int CAN_PORT=0;
double start_time;
public:
    Roller_Action_Coder(string Name);
    ~Roller_Action_Coder();
    void dec2HEX(int input_dec,int &output_HEX1,int &output_HEX2,int &output_HEX3,int &output_HEX4);
    void ORI_data2HEX(std_msgs::Int64MultiArray& motor_motion, int ID, int Port,
                      int data1, int data2, int data3, int data4,
                      int data5, int data6, int data7, int data8);
    void Speed_data2HEX(int ID, int PORT, std_msgs::Int64MultiArray& motor_motion, int speed);
    void SET_Position_data2HEX(int ID,int PORT,std_msgs::Int64MultiArray &motor_motion,int position);
    void Roller_Order(const Roller_Motor_Can::Single_Action_CanGoalConstPtr &goal);
    void Can_state_Callback(const Roller_Motor_Can::can& Msg);
    void Hand_Stop_Callback(const std_msgs::Bool& Msg);
    void Z_Up_Limit_Callback(const std_msgs::Bool& Msg);
    void Z_Down_Limit_Callback(const std_msgs::Bool& Msg);
    void Z_state_Callback(const std_msgs::Int64MultiArray& msg);
    double Get_the_msec_time_of_computer(struct timeval &time_now);
    void Sending_the_Speed_msgs_single(std_msgs::Int64MultiArray &motor_motion,int wheel_ID,int motor_speed);
    void Sending_the_ORI_single(std_msgs::Int64MultiArray &motor_motion,int wheel_ID,
                                int data1,int data2,int data3,int data4,
                                int data5,int data6,int data7,int data8);
    void publishMotionData(std_msgs::Int64MultiArray& motor_motion, int motor_ID,
                           int data1, int data2, int data3, int data4,
                           int data5, int data6, int data7, int data8);
};

#endif // ROLLER_ACTION_CODER_H
