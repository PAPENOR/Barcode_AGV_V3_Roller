#ifndef ROLLER_STATE_CAN_H
#define ROLLER_STATE_CAN_H
#include <ros/ros.h>
#include <string>
#include <cstring>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "Roller_Motor_Can/can.h"
#include <Roller_Motor_Can/Single_Action_CanAction.h>
#include "std_msgs/Int64MultiArray.h"
#include "param_init.h"
#include <cmath>
using namespace ros;
using namespace std;
class Roller_State_Can
{
protected:
/*Define the topic type*/
NodeHandle                                                                  nh_;

int Z_RT_ID         =517;
int Z_SDO_ID        =1541;
int Z_RT_ID_READ    =389;
int Z_IDLE_ID_READ  =1797;
int R_speed=0;
int R_position=0;
int state=0;
Roller_Motor_Can::can Can_state;
Roller_Motor_Can::Single_Action_CanGoal goal;
public:
    Roller_State_Can(string Name);
    ~Roller_State_Can();
    void Can_state_Callback(const Roller_Motor_Can::can& msg);
    int speed_data(int speeddata1, int speeddata2);
    int position_data(int positiondata1, int positiondata2, int positiondata3, int positiondata4);
    void motor_moving_mode(int motor_ID,int moving_type,int condition,int moving_speed,bool brake_mode,actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_state_client);
    void Set_state(int state_code,int Rspeed,int Rposition,std_msgs::Int64MultiArray &state_code_R);
};

#endif // ROLLER_STATE_CAN_H
