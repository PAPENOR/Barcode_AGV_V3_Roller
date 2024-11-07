#include "ros/ros.h"
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <Roller_Motor_Can/Single_Action_CanAction.h>
Roller_Motor_Can::Single_Action_CanGoal Goal;
void Motor_Moving_Mode(int Motor_ID,int Moving_type,int Condition,int Moving_speed,bool Brake_mode,actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
{
  Goal.motor_ID         = Motor_ID;
  Goal.moving_type      = Moving_type;
  Goal.moving_condition = Condition;
  Goal.moving_speed     = Moving_speed;
  Goal.Brake_wheel      = Brake_mode;
  Can_State_Client.sendGoal(Goal);
  bool Finished_Before_Timeout = Can_State_Client.waitForResult(ros::Duration(30.0));
  if (Finished_Before_Timeout)
  {
    actionlib::SimpleClientGoalState state = Can_State_Client.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_INFO("Action did not finish before the time out.");
  }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Roller_Can_test");
    ros::NodeHandle nh;
    int Z_SDO_ID      =1542;

    actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> Can_State_Client("Roller_Action_Can", true);
    Can_State_Client.waitForServer();
    Motor_Moving_Mode(Z_SDO_ID,7,    0,   0,false,Can_State_Client);


    return 0;
}
