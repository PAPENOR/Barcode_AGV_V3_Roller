#include "roller_state_can.h"
#include <std_msgs/Int16.h>
Roller_State_Can::Roller_State_Can(string Name)
{
    actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> Can_state_client("Roller_Action_Can", true);
    ROS_INFO("Waiting for action server to start.");
    Can_state_client.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    /*Set the topic name*/
    ros::NodeHandle nh_;
    ros::Subscriber state_can_sub    =nh_.subscribe("/Z_state_can",10,&Roller_State_Can::Can_state_Callback,this);
    ros::Publisher  state_pub        =nh_.advertise<std_msgs::Int64MultiArray>("Z_state",10);
    ros::Publisher Error_pub; //***~
    stringstream Can_error_name;
    Can_error_name<<ros::this_node::getName()<<"/Error_Code_Can_state";
    Error_pub = nh_.advertise<std_msgs::Int16>(Can_error_name.str(), 10);
    bool LR_wheel_error =false;
    bool canalyst_out_of_time_error = false;  //~***
    bool can_state_ID_error = false; //***
    Init_param(nh_,"Can_State_Hand/Z_RT_ID"         ,Z_RT_ID);
    Init_param(nh_,"Can_State_Hand/Z_SDO_ID"        ,Z_SDO_ID);
    Init_param(nh_,"Can_State_Hand/Z_RT_ID_READ"    ,Z_RT_ID_READ);
    Init_param(nh_,"Can_State_Hand/Z_IDLE_ID_READ"  ,Z_IDLE_ID_READ);
    std_msgs::Int64MultiArray state_code;
    ros::Rate r(100);
    while (ros::ok())
    {
      if(Can_state.data.size()>0)
      {
        if(Can_state.id==Z_RT_ID_READ )
        {
          /*Reading the error code .This code is define by the MOTEC handbook*/
             can_state_ID_error = false; //***
          int state_code=Can_state.data.at(0);
          int error_code=0;
          error_code=state_code/8;
          error_code=error_code%2;

          if(error_code==1)
          {
            /*If error code =1 ,There is an emgercy accitant*/
            ROS_ERROR("Z ERROR");
            state=999;
            R_speed=0;
          }
        }
        else if(Can_state.id==Z_IDLE_ID_READ )
        {
            can_state_ID_error = false; //***
          /*If there is no error and system had receive the data from motor*/
          int state_code=Can_state.data.at(0);
          if(state_code==127)
          {
            /*Code 127 is the standby info from motor*/
            ROS_WARN("Inital the hand");
            motor_moving_mode    (Z_RT_ID ,5,0    ,0    ,false,Can_state_client);
          }
        }
        if(Can_state.id==998 )
        {
          /*This system define the code(998) as the state of "Canalyst-II state is out of time"*/
          ROS_ERROR("Canalyst-II state is out of time");
          canalyst_out_of_time_error =true; //***
          state=998;
          R_speed=0;
        }
        else
        {
          /*There is no error in the system*/
            canalyst_out_of_time_error =false;
          state=0;
        }
        /*Update the LR_state data*/
        state_code.data.clear();
        Set_state(state,R_speed,R_position,state_code);
        state_pub.publish(state_code);
      }
      std_msgs::Int16 Error_pub_msg;
      if (LR_wheel_error == true)  //***~
      {
          Error_pub_msg.data = 1;  // 將Error_pub_msg的數值設為1，表示系統未連接的錯誤碼
          Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
      }
      else
      {
         if(canalyst_out_of_time_error == true)
         {
             Error_pub_msg.data = 2;  // 將Error_pub_msg的數值設為2，表示系統未連接的的錯誤碼
             Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
         }
         else
         {
             if(can_state_ID_error == true)
             {
                 Error_pub_msg.data = 3;  // 將Error_pub_msg的數值設為3，表示系統未連接的的錯誤碼
                 Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
             }
             else
             {
                 Error_pub_msg.data = 0;  // 將Error_pub_msg的數值設為0，表示系統未錯誤碼
                 Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
             }
         }
      } //~***
      ros::spinOnce();
      r.sleep();
    }
}
Roller_State_Can::~Roller_State_Can()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Roller_State_Can");
    ros::NodeHandle nh;

    ROS_INFO("Roller_Action_Can");
    Roller_State_Can Roller_State_Can("Roller_Action_Can");
    ros::spin();
    return 0;
}
