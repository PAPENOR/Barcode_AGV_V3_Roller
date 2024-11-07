#include "roller_model_motec.h"
#include "roller_model_function.h"
#define PI 3.1415926
void Roller_Model_Motec::Motor_Moving_Mode(int Motor_ID,int Moving_type,int Condition,int Moving_speed,bool Brake_mode,actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
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
void Roller_Model_Motec::StopAllMotion(int Z_RT_ID,actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
{
  Motor_Moving_Mode(Z_RT_ID,1,5,0,false,Can_State_Client);
  sendRoller_AxiscommandMsg(0);
}
Roller_Model_Motec::Roller_Model_Motec(string Name)
{   
    ROS_INFO("Welcome to Hand_model_Motec.");
    ROS_INFO("This is Two Axis Version.");
    actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> Can_State_Client("Roller_Action_Can", true);
    Z_Arrived_pub     =nh.advertise<std_msgs::Bool>("/Z_Arrived",10);
    Mission_state_pub =nh.advertise<std_msgs::Int16>("/Mission_state",10);
    Left_Stick_pub    =nh.advertise<std_msgs::Int16>("/hand/Left_Stick_action",10);
    Right_Stick_pub   =nh.advertise<std_msgs::Int16>("/hand/Right_Stick_action",10);
    Front_Stick_pub   =nh.advertise<std_msgs::Int16>("/hand/Front_Stick_action",10);
    Back_Stick_pub    =nh.advertise<std_msgs::Int16>("/hand/Back_Stick_action",10);
    Roller_Axis_pub   =nh.advertise<std_msgs::Int16>("hand/Roller_Axis_action",10);    
    Error_pub         =nh.advertise<std_msgs::Int16>(ros::this_node::getName() + "/Error_Code_Rollor_model", 10);

    action_sub        =nh.subscribe("/XZ_action",10,&Roller_Model_Motec::Z_action_Callback,this);
    odom_sub          =nh.subscribe("/odom_hand",10,&Roller_Model_Motec::Odom_Callback,this);
    Hand_stop_sub     =nh.subscribe("/Hand_Stop_Command",10,&Roller_Model_Motec::Hand_Stop_Callback,this);
    Right_Limit_Sub   =nh.subscribe("hand/Right_Limit",10,&Roller_Model_Motec::Right_Limit_Callback,this);
    Left_Limit_Sub    =nh.subscribe("hand/Left_Limit",10,&Roller_Model_Motec::Left_Limit_Callback,this);
    Z_Up_Limit_Sub    =nh.subscribe("hand/Z_Up_Limit",10,&Roller_Model_Motec::Z_Up_Limit_Callback,this);
    Z_Down_Limit_Sub  =nh.subscribe("hand/Z_Down_Limit",10,&Roller_Model_Motec::Z_Down_Limit_Callback,this);
    Z_Zero_Sub        =nh.subscribe("hand/Z_Zero",10,&Roller_Model_Motec::Z_Zero_Callback,this);

    Left_Stick_Inp_sub  =nh.subscribe("hand/left_stick_inp_state",10,&Roller_Model_Motec::Left_Stick_Inp_Callback,this);
    Right_Stick_Inp_sub =nh.subscribe("hand/right_stick_inp_state",10,&Roller_Model_Motec::Right_Stick_Inp_Callback,this);
    Front_Stick_Inp_sub =nh.subscribe("hand/front_stick_inp_state",10,&Roller_Model_Motec::Front_Stick_Inp_Callback,this);
    Back_Stick_Inp_sub  =nh.subscribe("hand/back_stick_inp_state",10,&Roller_Model_Motec::Back_Stick_Inp_Callback,this);

    Left_Stick_Home_sub  =nh.subscribe("hand/left_stick_home_state",10,&Roller_Model_Motec::Left_Stick_Home_Callback,this);
    Right_Stick_Home_sub =nh.subscribe("hand/right_stick_home_state",10,&Roller_Model_Motec::Right_Stick_Home_Callback,this);
    Front_Stick_Home_sub =nh.subscribe("hand/front_stick_home_state",10,&Roller_Model_Motec::Front_Stick_Home_Callback,this);
    Back_Stick_Home_sub  =nh.subscribe("hand/back_stick_home_state",10,&Roller_Model_Motec::Back_Stick_Home_Callback,this);

    Left_stick_state_postion_1_sub      =nh.subscribe("hand/Left_stick_state_postion_1",10,&Roller_Model_Motec::Left_Stick_State_Postion_1_Callback,this);
    Right_stick_state_postion_1_sub     =nh.subscribe("hand/Right_stick_state_postion_1",10,&Roller_Model_Motec::Right_Stick_State_Postion_1_Callback,this);
    Front_stick_state_postion_1_sub     =nh.subscribe("hand/Front_stick_state_postion_1",10,&Roller_Model_Motec::Front_Stick_State_Postion_1_Callback,this);
    Back_stick_state_postion_1_sub      =nh.subscribe("hand/Back_stick_state_postion_1",10,&Roller_Model_Motec::Back_Stick_State_Postion_1_Callback,this);

    Left_stick_state_postion_2_sub      =nh.subscribe("hand/Left_stick_state_postion_2",10,&Roller_Model_Motec::Left_Stick_State_Postion_2_Callback,this);
    Right_stick_state_postion_2_sub     =nh.subscribe("hand/Right_stick_state_postion_2",10,&Roller_Model_Motec::Right_Stick_State_Postion_2_Callback,this);
    Front_stick_state_postion_2_sub     =nh.subscribe("hand/Front_stick_state_postion_2",10,&Roller_Model_Motec::Front_Stick_State_Postion_2_Callback,this);
    Back_stick_state_postion_2_sub      =nh.subscribe("hand/Back_stick_state_postion_2",10,&Roller_Model_Motec::Back_Stick_State_Postion_2_Callback,this);

    ros::Rate r(100);

    Init_param(nh,"/Hand_Model_Motec/Z_RT_ID"     ,Z_RT_ID);
    Init_param(nh,"/Hand_Model_Motec/Z_SDO_ID"    ,Z_SDO_ID);
    Init_param(nh,"/Hand_Model_Motec/Z_RT_ID_READ",Z_RT_ID_READ);
    Init_param(nh,"/Hand_Model_Motec/Z_MOTOR_REVERSE",Z_Motor_Reverse);

    // 定义全局变量
    Timer_Record = Get_the_msec_time_of_computer(time_now);
    Can_State_Client.waitForServer();
    while (ros::ok())
    {
      Now_Time = Get_the_msec_time_of_computer(time_now);
      Timer1=(Now_Time-Timer_Record)/1000;
      ros::spinOnce();
      if (Stage == -1) {
          std::string mode_description;
          switch (Task_Order.Hand_Mode) {
              case 4:
                  mode_description = "Left Goods input";
                  break;
              case 5:
                  mode_description = "Right Goods input";
                  break;
              case 6:
                  mode_description = "Left Goods output";
                  break;
              case 7:
                  mode_description = "Right Goods output";
                  break;
              default:
                  // Handle unexpected mode
                  break;
          }
          if (!mode_description.empty()) {
              ROS_WARN_STREAM("Starting New mission:");
              ROS_WARN_STREAM(mode_description);
              Stage = 0;
          }
      }
      // 重置停止計數
      if (Task_Order.Hand_Mode != 0) {
          Stop_Count = 0;
      }

      // 執行不同的操作模式和階段
      switch (Task_Order.Hand_Mode) {
          case 0: // 速度模式
              handle_speed_control(Can_State_Client);
              break;
          case 1: // 位置模式1
              switch (Stage) {
                  case 0:
                      Positon_mode_Setting_signal();
                      break;
                  case 1:
                      Positon_mode_Launching(Can_State_Client);
                      break;
                  default:
                      break;
              }
              break;
          case 3: // 位置模式2
              switch (Stage) {
                  case 0:
                      initialize_Z_AXIS();
                      break;
                  case 1:
                      initialize_Z_AXIS_up(Can_State_Client);
                      break;
                  case 2:
                      initialize_Z_AXIS_down(Can_State_Client);
                      break;
                  case 3:
                      initialize_Z_AXIS_up_again(Can_State_Client);
                      break;
                  case 4:
                      reset_Z_AXIS_zero(Can_State_Client);
                      break;
                  case 5:
                      initialize_Z_AXIS_up_slow(Can_State_Client);
                      break;
                  default:
                      break;
              }
              break;
          case 4: // 左側商品放置
          case 5: // 右側商品放置
              if (Stage == 0)
              {
                  Goods_Put_in_waiting_Turn_On_Stick(Task_Order.Hand_Mode == 4);
              }
              else if (Stage == 1)
              {
                  Goods_Put_in_waiting_Launch_the_Roller_waiting_Goods(Task_Order.Hand_Mode == 4, Can_State_Client);
              }
              else if (Stage == 2)
              {
                  Goods_Put_in_waiting_Get_Goods_Stop_Motion(Task_Order.Hand_Mode == 4, Can_State_Client);
              }
              break;
          case 6: // 左側商品取出
          case 7: // 右側商品取出
              if (Stage == 0)
              {
                  Goods_Roll_Out_waiting_Turn_On_Stick(Task_Order.Hand_Mode == 6);
              }
              else if (Stage == 1)
              {
                  Goods_Roll_Out_waiting_Launch_the_Roller(Task_Order.Hand_Mode == 6, Can_State_Client);
              }
              break;
          case 8: // 位置模式 + 4 模式
          case 9: // 位置模式 + 5 模式
              if (Stage == 0)
              {
                  Positon_mode_Setting_signal();
              }
              else if (Stage == 1)
              {
                  if (Task_Order.Hand_Mode == 8)
                  {
                      Left_Goods_Put_in_Turn_On_Stick_lifting(Can_State_Client);
                  }
                  else
                  {
                      Right_Goods_Put_in_Turn_On_Stick_lifting(Can_State_Client);
                  }
              }
              else if (Stage == 2)
              {
                  if (Task_Order.Hand_Mode == 8)
                  {
                      Left_Goods_Put_in_waiting_Goods(Can_State_Client);
                  }
                  else
                  {
                      Right_Goods_Put_in_waiting_Goods(Can_State_Client);
                  }
              }
              else if (Stage == 3)
              {
                  Goods_Put_in_waiting_Get_Goods_Stop_Motion(Task_Order.Hand_Mode == 8, Can_State_Client);
              }
              break;
          case 10: // 位置模式 + 6 模式
          case 11: // 位置模式 + 7 模式
              if (Stage == 0)
              {
                  Positon_mode_Setting_signal();
              }
              else if (Stage == 1)
              {
                  if (Task_Order.Hand_Mode == 10)
                  {
                      Left_Goods_Roll_Out_waiting_Turn_On_Stick_lifting(Can_State_Client);
                  }
                  else
                  {
                      Right_Goods_Roll_Out_waiting_Turn_On_Stick_lifting(Can_State_Client);
                  }
              }
              else if (Stage == 2)
              {
                  Goods_Roll_Out_waiting_Launch_the_Roller(Task_Order.Hand_Mode == 10, Can_State_Client);
              }
              break;
           case 12:
           case 13:
             if (Stage == 0)
             {
              Set_Open_Close_Stick(Task_Order.Hand_Mode == 12);
             }
              break;
      }
      if(Stage==999)//init the Z AXIS
      {

      }
      std_msgs::Int16 Error_pub_msg; //***~
      if (limit_error == true) {
          Error_pub_msg.data = 1;  // 將Error_pub_msg的數值設為1，表示系統未連接的錯誤碼
          Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
      }
      else
      {
          if(None_Good == true)
          {
              Error_pub_msg.data = 2;  // 將Error_pub_msg的數值設為2，表示系統未init的錯誤碼
              Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
          }
          else
          {
           if(Time_out_error == true)
           {
               Error_pub_msg.data = 3;  // 將Error_pub_msg的數值設為3，表示系統pass time error的錯誤碼
               Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
           }
           else
           {
               Error_pub_msg.data = 0;  // 將Error_pub_msg的數值設為0，表示系統no錯誤碼
               Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
           }
          }
      }  //~***
      ros::spinOnce();
      r.sleep();
    }
    ros::spin();

}
Roller_Model_Motec::~Roller_Model_Motec()
{

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Roller_Model_Motec");
    ros::NodeHandle nh;

    ROS_INFO("Roller_Model_Motec");
    Roller_Model_Motec Roller_Model_Motec("Roller_Model_Motec");
    ros::spin();
    return 0;
}
