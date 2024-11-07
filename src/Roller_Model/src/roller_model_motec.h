#ifndef ROLLER_MODEL_MOTEC_H
#define ROLLER_MODEL_MOTEC_H
#define Go_left 1
#define Go_right 2
#include <ros/ros.h>
#include <string>
#include <cstring>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <Roller_Motor_Can/Single_Action_CanAction.h>
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Int16.h"
#include "nav_msgs/Odometry.h"
#include "param_init.h"
#include "std_msgs/Bool.h"
using namespace ros;
using namespace std;
class Hand_command
{
public:
  int Hand_Mode;
  int Hand_Speed_Z;
  int Hand_Position_Z;
};
class Hand_state
{
public:
  double  Z;
  double  X_Left_Light;
  double  X_Right_Light;
  double  Z_Up_Light;
  double  Z_Down_Light;
  double  Z_Zero_Light;

};
class Roller_Model_Motec
{
protected:
    NodeHandle nh;
    Publisher   Z_Arrived_pub;
    Publisher   Mission_state_pub;
    Publisher   Left_Stick_pub;
    Publisher   Right_Stick_pub;//OFF Position 1 ON Position 2
    Publisher   Front_Stick_pub;
    Publisher   Back_Stick_pub;
    Publisher   Roller_Axis_pub;
    Publisher   Error_pub;


    Subscriber action_sub;
    Subscriber odom_sub;
    Subscriber Hand_stop_sub;
    Subscriber Right_Limit_Sub;
    Subscriber Left_Limit_Sub;
    Subscriber Z_Up_Limit_Sub;
    Subscriber Z_Down_Limit_Sub;
    Subscriber Z_Zero_Sub;

    Subscriber   Left_Stick_Inp_sub;
    Subscriber   Right_Stick_Inp_sub;
    Subscriber   Front_Stick_Inp_sub;
    Subscriber   Back_Stick_Inp_sub;

    Subscriber   Left_Stick_Home_sub;
    Subscriber   Right_Stick_Home_sub;
    Subscriber   Front_Stick_Home_sub;
    Subscriber   Back_Stick_Home_sub;

    Subscriber   Left_stick_state_postion_1_sub;
    Subscriber   Right_stick_state_postion_1_sub;
    Subscriber   Front_stick_state_postion_1_sub;
    Subscriber   Back_stick_state_postion_1_sub;

    Subscriber   Left_stick_state_postion_2_sub;
    Subscriber   Right_stick_state_postion_2_sub;
    Subscriber   Front_stick_state_postion_2_sub;
    Subscriber   Back_stick_state_postion_2_sub;

    Hand_command Task_Order;
    Hand_state   State_Of_Hand;
    Roller_Motor_Can::Single_Action_CanGoal Goal;
    std_msgs::Bool Right_Limit_Msg;
    std_msgs::Bool Left_Limit_Msg;
    std_msgs::Bool Z_Up_Limit_Msg;
    std_msgs::Bool Z_Down_Limit_Msg;
    std_msgs::Bool Z_Zero_Msg;
    std_msgs::Bool Hand_stop_Msg;

    std_msgs::Bool Left_Stick_Inp_Msg;
    std_msgs::Bool Right_Stick_Inp_Msg;
    std_msgs::Bool Front_Stick_Inp_Msg;
    std_msgs::Bool Back_Stick_Inp_Msg;

    std_msgs::Bool Left_Stick_Home_Msg;
    std_msgs::Bool Right_Stick_Home_Msg;
    std_msgs::Bool Front_Stick_Home_Msg;
    std_msgs::Bool Back_Stick_Home_Msg;

    std_msgs::Bool Left_stick_state_postion_1_Msg;
    std_msgs::Bool Right_stick_state_postion_1_Msg;
    std_msgs::Bool Front_stick_state_postion_1_Msg;
    std_msgs::Bool Back_stick_state_postion_1_Msg;

    std_msgs::Bool Left_stick_state_postion_2_Msg;
    std_msgs::Bool Right_stick_state_postion_2_Msg;
    std_msgs::Bool Front_stick_state_postion_2_Msg;
    std_msgs::Bool Back_stick_state_postion_2_Msg;

    bool Left_Stick_Initial_Flag=false;
    bool Right_Stick_Initial_Flag=false;
    bool Front_Stick_Initial_Flag=false;
    bool Back_Stick_Initial_Flag=false;

    int Z_RT_ID       =518;
    int Z_SDO_ID      =1542;
    int Z_RT_ID_READ  =390;
    int Z_Motor_Reverse =-1;

    bool limit_error=false;
    bool None_Good=false;
    bool Time_out_error=false;

    double Z_mm_circle = 0.0;
    int Stage = 0;
    int Stop_Count = 0;
    double SpeedHandZ = 0.0;
    double PositionHandZ = 0.0;
    double Timer_Record ;
    bool Initial = true;
    double Timer1;
    double Now_Time;
    struct timeval time_now{};

    double Stick_Runing_Time=4.0;
    double Roller_Delay_Time=1.0;

public:
    Roller_Model_Motec(string Name);
    ~Roller_Model_Motec();
    void Right_Limit_Callback(const std_msgs::Bool& Msg);
    void Left_Limit_Callback(const std_msgs::Bool& Msg);
    void Z_Up_Limit_Callback(const std_msgs::Bool& Msg);
    void Z_Down_Limit_Callback(const std_msgs::Bool& Msg);
    void Z_Zero_Callback(const std_msgs::Bool& Msg);

    void Left_Stick_Inp_Callback(const std_msgs::Bool& Msg);
    void Right_Stick_Inp_Callback(const std_msgs::Bool& Msg);
    void Front_Stick_Inp_Callback(const std_msgs::Bool& Msg);
    void Back_Stick_Inp_Callback(const std_msgs::Bool& Msg);

    void Left_Stick_Home_Callback(const std_msgs::Bool& Msg);
    void Right_Stick_Home_Callback(const std_msgs::Bool& Msg);
    void Front_Stick_Home_Callback(const std_msgs::Bool& Msg);
    void Back_Stick_Home_Callback(const std_msgs::Bool& Msg);

    void Left_Stick_State_Postion_1_Callback(const std_msgs::Bool& Msg);
    void Right_Stick_State_Postion_1_Callback(const std_msgs::Bool& Msg);
    void Front_Stick_State_Postion_1_Callback(const std_msgs::Bool& Msg);
    void Back_Stick_State_Postion_1_Callback(const std_msgs::Bool& Msg);

    void Left_Stick_State_Postion_2_Callback(const std_msgs::Bool& Msg);
    void Right_Stick_State_Postion_2_Callback(const std_msgs::Bool& Msg);
    void Front_Stick_State_Postion_2_Callback(const std_msgs::Bool& Msg);
    void Back_Stick_State_Postion_2_Callback(const std_msgs::Bool& Msg);


    void Hand_Stop_Callback(const std_msgs::Bool& Msg);
    void Z_action_Callback(const std_msgs::Int64MultiArray& Msg);
    void Odom_Callback(const nav_msgs::Odometry& Msg);
    void Motor_Moving_Mode(int Motor_ID,int Moving_type,int Condition,int Moving_speed,bool Brake_mode,actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void Stick_motion(bool Turn_on,ros::Publisher &Stick_pub);
    void StopAllMotion(int Z_RT_ID,actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void sendRoller_AxiscommandMsg(int data1);
    int  calculate_PositionMotorZ();
    void Stick_Off(ros::Publisher &Stick_pub);
    void Set_Open_Close_Stick(bool Open);

    void handle_limit_up_down_error(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void handle_speed_control(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void Hand_stop_Command(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void Positon_mode_Setting_signal();
    void Positon_mode_Launching(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void initialize_Z_AXIS() ;
    void initialize_Z_AXIS_up(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client) ;
    void initialize_Z_AXIS_down(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void initialize_Z_AXIS_up_again(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void initialize_Z_AXIS_up_slow(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void reset_Z_AXIS_zero(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void Goods_Put_in_waiting_Turn_On_Stick(bool left_stick);
    void Goods_Put_in_waiting_Launch_the_Roller_waiting_Goods(bool left_stick, actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void Goods_Put_in_waiting_Get_Goods_Stop_Motion(bool left_stick, actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void Goods_Roll_Out_waiting_Turn_On_Stick(bool is_left);
    void Goods_Roll_Out_waiting_Launch_the_Roller(bool is_left, actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void Left_Goods_Put_in_Turn_On_Stick_lifting(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void Left_Goods_Put_in_waiting_Goods(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void Right_Goods_Put_in_Turn_On_Stick_lifting(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void Right_Goods_Put_in_waiting_Goods(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void Left_Goods_Roll_Out_waiting_Turn_On_Stick_lifting(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
    void Right_Goods_Roll_Out_waiting_Turn_On_Stick_lifting(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client);
};

#endif // ROLLER_MODEL_MOTEC_H
