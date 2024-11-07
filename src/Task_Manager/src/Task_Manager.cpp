/*
 * 任務狀態，機器人啟動時的決策節點
 * State:
 * 0 Init
 * 1 Ack
 * 2 Zready
 * 3 Done
 * 4 Go_init
 * 5 IDLE
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include <yaml-cpp/yaml.h>
std_msgs::Int64MultiArray XZ_Command;
std_msgs::Int16 Mission_state;
std_msgs::Bool X_Arrived_state;
std_msgs::Bool Z_Arrived_state;
std_msgs::Bool Roller_state;
std_msgs::Bool Clear_Command;
using namespace std;
void Woring_state_pub(int working_state, ros::Publisher &Mission_state_pub)
{
    // 定義工作狀態的字符串數組
    std::string states[] = {"Done", "Ready", "Busy", "ERROR!"};
    // 確保傳入的工作狀態在合理的範圍內
    if (working_state < 1 || working_state > 4)
    {
        ROS_ERROR("Invalid working state!");
        return;
    }
    // 打印工作狀態信息
    ROS_INFO("Working State: %s", states[working_state - 1].c_str());
    // 創建消息並發佈到 ROS 主題上
    std_msgs::Int16 Mission_msg;
    Mission_msg.data = working_state;
    Mission_state_pub.publish(Mission_msg);
}
class Command
{
  public:
  //L GET >4,0,0 // D:2 P
  int Direction=0;    //0 for None X;
                      //1 for L;
                      //2 for R ;
                      //3 for Done
  int Put_Or_Get=0;   //0 for None;
                      //1 for Put;
                      //2 for Get
  int High=0;         //ABS mm

  int Direction_Pass;
  int Put_Or_Get_Pass;
  int High_Pass;
  bool New_command=false;
};
Command Task_Command;

double Get_the_msec_time_of_computer(struct timeval &time_now)
{
  /*Get the msec_time of computer*/
  double now_time;
  gettimeofday(&time_now,nullptr);
  now_time = time_now.tv_sec*1000+time_now.tv_usec/1000;
  return now_time;
}

void XZ_CommandCallback(const std_msgs::Int64MultiArray& msg)
{
  XZ_Command=msg;
  ROS_INFO_STREAM(Task_Command.Direction<<":"<<Task_Command.Put_Or_Get<<":"<<Task_Command.High);
  if(msg.data.size()>=3)
  {
  Task_Command.Direction  =msg.data.at(0);
  Task_Command.Put_Or_Get =msg.data.at(1);
  Task_Command.High       =msg.data.at(2);
  }

}


void State_Num_Pub(int State_Num,ros::Publisher  &XZ_Mission_State_pub)
{
  ros::spinOnce();
  ros::Duration(0.02).sleep();
  std_msgs::Int64 State_Num_Msg;
  State_Num_Msg.data=State_Num;
  XZ_Mission_State_pub.publish(State_Num_Msg);
  ros::spinOnce();
  ros::Duration(0.02).sleep();
}

void XZ_Mission_Msg_Pub(int Command1,int Command2,ros::Publisher  &XZ_Mission_Pub)
{
  ros::spinOnce();
  ros::Duration(0.02).sleep();
  std_msgs::Int64MultiArray Command_msg;
  Command_msg.data.push_back(Command1);
  Command_msg.data.push_back(Command2);
  XZ_Mission_Pub.publish(Command_msg);
  ros::spinOnce();
  ros::Duration(0.02).sleep();
}
void X_ArrivedCallback(const std_msgs::Bool& msg)
{
  X_Arrived_state=msg;
}
void Z_ArrivedCallback(const std_msgs::Bool& msg)
{
  Z_Arrived_state=msg;
}
void Roller_state_Callback(const std_msgs::Bool& msg)
{
  Roller_state=msg;
}
void Mission_StateCallback(const std_msgs::Int16& msg)
{
  Mission_state=msg;
}
void Clear_Command_Callback(const std_msgs::Bool& msg)
{
  Clear_Command=msg;
}
void Clear_Pub(bool Clear_Data,ros::Publisher   &Clear_Command_Pub)
{

    std_msgs::Bool Clear_msg;
    Clear_msg.data=Clear_Data;
    Clear_Command_Pub.publish(Clear_msg);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Task_Manager");
  ros::NodeHandle nh;

  ros::Subscriber XZ_Command_Sub            =nh.subscribe("/XZ_Command", 1000, XZ_CommandCallback);
  ros::Publisher  XZ_Mission_State_Pub      =nh.advertise<std_msgs::Int64>("/XZ_Mission_Stage",10);//定義任務狀態
  ros::Publisher  XZ_Mission_Pub            =nh.advertise<std_msgs::Int64MultiArray>("/XZ_action",10);//
  ros::Subscriber X_Arrived_Sub             =nh.subscribe("/X_Arrived",1000,X_ArrivedCallback);
  ros::Subscriber Z_Arrived_Sub             =nh.subscribe("/Z_Arrived",1000,Z_ArrivedCallback);
  ros::Subscriber Mission_State_Sub         =nh.subscribe("/Mission_state",1000,Mission_StateCallback);
  ros::Subscriber Roller_state_Sub          =nh.subscribe("/Roller_state",1000,Roller_state_Callback);

  ros::Subscriber Clear_Command_Sub         =nh.subscribe("caraction/ClearCommand"   ,10,Clear_Command_Callback);

  ros::Publisher  Mission_type_Direction_pub=nh.advertise<std_msgs::Int16>("/Mission_type_Direction",10);
  ros::Publisher  Mission_type_Motion_pub   =nh.advertise<std_msgs::Int16>("/Mission_type_Motion",10);
  ros::Publisher  Mission_type_High_pub     =nh.advertise<std_msgs::Int16>("/Mission_type_High",10);
  ros::Publisher  Clear_Command_Pub         =nh.advertise<std_msgs::Bool>("caraction/ClearCommand",10);
  ros::Publisher  Mission_state_pub         =nh.advertise<std_msgs::Int16>("/Mission_state",10);

  ros::Publisher Error_pub;  //***~
  stringstream Can_error_name;
  ros::NodeHandle Node;
  Can_error_name<<ros::this_node::getName()<<"/Error_Code_Task";
  Error_pub = Node.advertise<std_msgs::Int16>(Can_error_name.str(), 10);
  bool Non_define_state     =true;
  bool Non_finfish_initial  =true;
  ros::Rate Loop_Rate(100.0);
  int State_Num=0;
  struct timeval time_now{};
  gettimeofday(&time_now,nullptr);
  time_t  begin=time_now.tv_sec*1000+time_now.tv_usec/1000,new_time;
  double  Timer_Record=begin;

  while(nh.ok())
  {
    double Now_Time = Get_the_msec_time_of_computer(time_now);
    double Timer1=(Now_Time-Timer_Record)/1000;
    State_Num_Pub(State_Num,XZ_Mission_State_Pub);
    std_msgs::Int16 Direction_msgs;
    Direction_msgs.data=Task_Command.Direction;
    Mission_type_Direction_pub.publish(Direction_msgs);
    std_msgs::Int16 Motion_msgs;
    Motion_msgs.data=Task_Command.Put_Or_Get;
    Mission_type_Motion_pub.publish(Motion_msgs);
    std_msgs::Int16 High_msgs;
    High_msgs.data=Task_Command.High;
    Mission_type_High_pub.publish(High_msgs);
    Non_define_state    =false;
    Non_finfish_initial =false;
    if(State_Num==0)
    {
      //Initial 系統啟動時會先將座標歸零
      State_Num=4;
      State_Num_Pub(State_Num,XZ_Mission_State_Pub);
      ros::spinOnce();
      ros::Duration(0.5).sleep();
      XZ_Mission_Msg_Pub(0,0,XZ_Mission_Pub);
      ros::spinOnce();
      ros::Duration(0.5).sleep();
      XZ_Mission_Msg_Pub(3,0,XZ_Mission_Pub);
      Timer_Record=Get_the_msec_time_of_computer(time_now);
    }
    if(State_Num==1)//start mission
    {
       if(Roller_state.data==true)
       {
       //開始升降
       State_Num_Pub(State_Num,XZ_Mission_State_Pub);
       ros::spinOnce();
       ros::Duration(0.1).sleep();
       XZ_Mission_Msg_Pub(0,0,XZ_Mission_Pub);
       ros::spinOnce();
       ros::Duration(0.1).sleep();       
       if(Task_Command.Direction==1 && Task_Command.Put_Or_Get==2)
       {
           XZ_Mission_Msg_Pub(8,Task_Command.High,XZ_Mission_Pub);
       }
       if(Task_Command.Direction==2 && Task_Command.Put_Or_Get==2)
       {
           XZ_Mission_Msg_Pub(9,Task_Command.High,XZ_Mission_Pub);
       }
       if(Task_Command.Direction==1 && Task_Command.Put_Or_Get==1)
       {
           XZ_Mission_Msg_Pub(10,Task_Command.High,XZ_Mission_Pub);
       }
       if(Task_Command.Direction==2 && Task_Command.Put_Or_Get==1)
       {
           XZ_Mission_Msg_Pub(11,Task_Command.High,XZ_Mission_Pub);
       }
       Timer_Record=Get_the_msec_time_of_computer(time_now);
       State_Num=6;
       if(Task_Command.Put_Or_Get==3)
       {
        XZ_Mission_Msg_Pub(1,Task_Command.High,XZ_Mission_Pub);
       }
       State_Num=6;

       Timer_Record=Get_the_msec_time_of_computer(time_now);
       ros::Duration(0.1).sleep();
       ros::spinOnce();
       }
        else
       {
        ROS_ERROR_STREAM("NonSafty_mission");
        Woring_state_pub(4, Mission_state_pub);
        State_Num==5;
       }
    }
    else if(State_Num==2)
    {
    }
    else if(State_Num==3)
    {
      ROS_WARN_STREAM("Mission Done");
    }
    else if(State_Num==4)
    {
      //系統正在執行座標歸零
      if(Timer1>60)
      {
       XZ_Mission_Msg_Pub(0,0,XZ_Mission_Pub);
       ROS_ERROR("Too long to waiting for the Initial");
       ros::Duration(0.5).sleep();
       Timer_Record=Get_the_msec_time_of_computer(time_now);
       State_Num=5;
      }
      else if(Timer1>5 && Mission_state.data==1)
      {
        XZ_Mission_Msg_Pub(0,0,XZ_Mission_Pub);
        ROS_WARN("Initial Done");
        ros::Duration(0.5).sleep();
        Timer_Record=Get_the_msec_time_of_computer(time_now);
        State_Num=5;
      }
      State_Num_Pub(State_Num,XZ_Mission_State_Pub);
    }
    else if(State_Num==5)//比对新任物
    {
      //系統正在等待新指令
      if(Task_Command.Direction_Pass  ==Task_Command.Direction    &&
         Task_Command.Put_Or_Get_Pass ==Task_Command.Put_Or_Get  &&
         Task_Command.High_Pass       ==Task_Command.High)
        {
         Task_Command.New_command=false;
        }
      else
        {
         Task_Command.Direction_Pass  =Task_Command.Direction    ;
         Task_Command.Put_Or_Get_Pass =Task_Command.Put_Or_Get   ;
         Task_Command.High_Pass       =Task_Command.High         ;
         Task_Command.New_command=true;
        }
      if(Clear_Command.data==true)
      {
        Task_Command.Direction_Pass  =Task_Command.Direction    ;
        Task_Command.Put_Or_Get_Pass =Task_Command.Put_Or_Get   ;
        Task_Command.High_Pass       =Task_Command.High         +5;
        Task_Command.New_command=true;
        Clear_Pub(false,Clear_Command_Pub);
      }
      if(Task_Command.New_command==true)
      {
        ROS_WARN("NEW MISSION");
        State_Num=1;
        Timer_Record=Get_the_msec_time_of_computer(time_now);
      }
      if(Timer1>2)
      {
        Timer_Record=Get_the_msec_time_of_computer(time_now);
      }
    }
    else if(State_Num==6)
    {
      State_Num_Pub(State_Num,XZ_Mission_State_Pub);
      if(Timer1>15)
      {
      Timer_Record=Get_the_msec_time_of_computer(time_now);
      State_Num=5;
      State_Num_Pub(State_Num,XZ_Mission_State_Pub);
      }

    }    
    else if (State_Num==7)
    {
      ROS_ERROR("UNdefine State");
      Non_define_state=true;
    }

    std_msgs::Int16 Error_pub_msg;
    if (Non_define_state == true)
    {
        Error_pub_msg.data = 1;  // 將Error_pub_msg的數值設為1，
        Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
    }
    else
    {
        // 如果Socket已登入，繼續檢查系統是否連接
        if (Non_finfish_initial == false) {
            Error_pub_msg.data = 2;  // 將Error_pub_msg的數值設為2，
            Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
        }
        else
        {

                Error_pub_msg.data = 0;  // 將Error_pub_msg的數值設為0，表示系統未連接的錯誤碼
                Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題


        }
    }
    Loop_Rate.sleep();
    ros::spinOnce();
  }

  ros::spin();

  return 0;
}
