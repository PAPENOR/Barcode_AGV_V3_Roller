/*
這段程式是一個ROS節點，使用C++語言編寫，名為"Roller_Action_Can"。它實現了一個動作伺服器(Action Server)，接收指令並控制一個滾筒機器人的運動。程式碼的主要功能如下：

    初始化ROS節點和相關的訂閱者和發布者。
    實現了一個Roller_Action_Coder類，並在構造函數中設置了動作伺服器(action server)。
    動作伺服器提供一個名為"Roller_Order"的回調函數，該函數處理滾筒機器人的運動指令。
    根據傳入的運動指令類型(type)，執行不同的運動模式：
        如果是速度模式(speed mode)，則根據設定的運動速度(motor_speed)進行直線運動，並可以選擇是否將輪子制動。
        如果是位置模式(position mode)，則根據設定的目標位置(moving_condition)進行運動，並在到達目標位置前檢查限位開關和停止指令。
        如果是模式更換模式(change mode)，則根據設定的模式類型(moving_condition)切換到速度模式或位置模式。
        如果是初始模式(initial mode)，則依次執行一系列初始化指令，包括開啟設備、上電、啟動伺服等。
        如果是其他模式，則執行特定的指令，如重置里程計、開啟伺服、關閉伺服等。
    根據運動指令的執行情況，發送反饋(feedback)和結果(result)到動作伺服器。
    通過ROS的spin函數使程式進入事件迴圈，等待並處理訂閱的訊息和動作指令。

總的來說，這段程式提供了一個ROS節點，實現了滾筒機器人的運動控制功能，並且可以根據不同的運動指令類型執行相應的動作。
*/
#include "Roller_Action_Coder.h"
Roller_Action_Coder::Roller_Action_Coder(string Name):
    action_server_(nh_,Name,boost::bind(&Roller_Action_Coder::Roller_Order,this,_1),false),
    action_name_(Name)
{
  console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,console::levels::Info);
  Roller_action_can_pub       =nh_.advertise<std_msgs::Int64MultiArray>("/Hand_action_can", 10);
  Roller_state_can_sub        =nh_.subscribe("Z_state_can",100,&Roller_Action_Coder::Can_state_Callback,this);
  Roller_state_sub            =nh_.subscribe("Z_state",100,&Roller_Action_Coder::Z_state_Callback,this);
  Z_Up_Limit_Sub              =nh_.subscribe("hand/Z_Up_Limit",10,&Roller_Action_Coder::Z_Up_Limit_Callback,this);
  Z_Down_Limit_Sub            =nh_.subscribe("hand/Z_Down_Limit",10,&Roller_Action_Coder::Z_Down_Limit_Callback,this);
  Hand_stop_sub               =nh_.subscribe("/Hand_Stop_Command",10,&Roller_Action_Coder::Hand_Stop_Callback,this);
  Init_param(nh_,"Can_Action_Hand/Z_RT_ID",Z_RT_ID);
  Init_param(nh_,"Can_Action_Hand/Z_SDO_ID",Z_SDO_ID);
  Init_param(nh_,"Can_Action_Hand/CAN_PORT",CAN_PORT);
  action_server_.start();

}
void Roller_Action_Coder::Roller_Order(const Roller_Motor_Can::Single_Action_CanGoalConstPtr &goal)
{
    /*Main function*/
    /*Get the system time at the begin*/
    struct timeval time_now{};
    gettimeofday(&time_now,nullptr);
    time_t  begin=time_now.tv_sec*1000+time_now.tv_usec/1000,new_time;
    start_time=begin;

    /*Initial the node state*/
    Rate    r(200);
    bool    success =true;
    feedback_.now_state.clear();
    std_msgs::Int64MultiArray motor_motion;

    /*Choose the type of moving*/
    int  type=goal->moving_type;
    if (type==1)
    {
        /*Moving type is speed mode*/
        int motor_speed = goal->moving_speed;
        double pass_sec = 0.0;

        ROS_INFO("Speed mode, Motor ID: %d", goal->motor_ID);
        ROS_INFO("Go Straight for %d msec", goal->moving_condition);
        ROS_INFO("Speed: %d", motor_speed);
        ROS_INFO("begin_time %lf", start_time);

        /*Single motor mode*/
        while (pass_sec < goal->moving_condition)
        {
            /*Counting the passing time*/
            new_time = Get_the_msec_time_of_computer(time_now);
            pass_sec = ((double)(new_time - start_time));
            //ROS_INFO("pass time %lf", pass_sec);

            /*Sending the speed command*/
            Sending_the_Speed_msgs_single(motor_motion, goal->motor_ID, motor_speed);
        }

        if (goal->Brake_wheel == true)
        {
            ROS_INFO("Brake one wheel");
            /*Sending the speed command*/
            Sending_the_Speed_msgs_single(motor_motion, goal->motor_ID, 0);
        }
    }
    if (type==2)
    {
        ROS_INFO("Position Mode, Motor ID: %d", goal->motor_ID);
        ROS_INFO("begin_time %lf", start_time);
        double pass_sec = 0.0;
        ROS_INFO_STREAM("Single motor mode,state:"<<Can_state.id);
        /*Single motor mode*/
        double delaytime = 0.0;

        for (int i = 0; i < 5; i++)
        {
          /*Counting the passing time*/
          new_time = Get_the_msec_time_of_computer(time_now);
          pass_sec = static_cast<double>(new_time - start_time);

          motor_motion.data.clear();

          switch (i)
          {
            case 0:
              // Power on
              ORI_data2HEX(motor_motion, goal->motor_ID, CAN_PORT, 43, 0, 32, 0, 1, 0, 0, 0);
              if (Can_state.id != 998)
              {
                Roller_action_can_pub.publish(motor_motion);
                ros::Duration(0.02).sleep();
              }
              break;

            case 1:
              // Set the position index
              ROS_INFO_STREAM("Moving condition: " << goal->moving_condition);
              ROS_INFO_STREAM("Moving Now positionZ: " << Z_state.data.at(2));
              delaytime = 5;
              if (motor_motion, goal->motor_ID == Z_SDO_ID)
              {
                delaytime = static_cast<double>(abs(goal->moving_condition - Z_state.data.at(2)));

                if(delaytime>1000000)
                    {
                    delaytime = delaytime / 5000000.0 + 2.0;
                    }
                else
                    {
                    delaytime=1.0;
                    }
              }
              ROS_INFO_STREAM("Delaytime: " << delaytime);
              SET_Position_data2HEX(goal->motor_ID, CAN_PORT, motor_motion, goal->moving_condition);
              Roller_action_can_pub.publish(motor_motion);
              ros::Duration(0.02).sleep();
              break;

            case 2:
              // Launching the motor
              ORI_data2HEX(motor_motion, goal->motor_ID, CAN_PORT, 43, 1, 32, 0, 1, 0, 0, 0); // start moving
              if (Can_state.id != 998)
              {
                Roller_action_can_pub.publish(motor_motion);
                ros::Duration(0.02).sleep();
              }
              new_time = Get_the_msec_time_of_computer(time_now);
              break;

            case 3:
              if (pass_sec > delaytime * 1000)
              {

              }
              else
              {
                if (Z_Up_Limit_Msg.data == true)
                {
                  ROS_ERROR_STREAM("Z_Up_Limit_Msg ERROR");
                  result_.end_state.push_back(999);
                  Sending_the_ORI_single(motor_motion, goal->motor_ID, 43, 2, 32, 0, 1, 0, 0, 0);
                  Roller_action_can_pub.publish(motor_motion);
                  ros::Duration(0.02).sleep();
                }
                else if (Z_Down_Limit_Msg.data == true)
                {
                  ROS_ERROR_STREAM("Z_Down_Limit_Msg ERROR");
                  result_.end_state.push_back(999);
                  Sending_the_ORI_single(motor_motion, goal->motor_ID, 43, 2, 32, 0, 1, 0, 0, 0);
                  Roller_action_can_pub.publish(motor_motion);
                  ros::Duration(0.02).sleep();
                }
                else if (Hand_stop_Msg.data == true)
                {
                  ROS_ERROR_STREAM("Clear Mission");
                  result_.end_state.push_back(999);
                  ORI_data2HEX(motor_motion, goal->motor_ID, CAN_PORT, 43, 96, 96, 0, 3, 0, 0, 0);
                  Roller_action_can_pub.publish(motor_motion);
                  ros::Duration(0.02).sleep();
                  motor_motion.data.clear();
                  ORI_data2HEX(motor_motion, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0); // pub null
                  Roller_action_can_pub.publish(motor_motion);
                  ros::Duration(0.02).sleep();
                  i = 3;
                }
                else
                {
                  i = 2;
                }
              }
              break;
                default:
              break;
              }
          }
    }
    if (type==3)
    {
        ROS_INFO("Change mode, Motor ID: %d", goal->motor_ID);
        ROS_INFO("begin_time %lf", start_time);
        int mode_type = goal->moving_condition;
        ROS_INFO("Mode type %d", mode_type);
        double pass_sec = 0.0;
        for (int i = 0; i < 6; i++)
        {
          new_time = Get_the_msec_time_of_computer(time_now);
          pass_sec = static_cast<double>(new_time - start_time);
          ROS_INFO("pass time %lf", pass_sec);
          motor_motion.data.clear();
          switch (i)
          {
            case 1:
              // Set Z speed 0
              Speed_data2HEX(Z_RT_ID, CAN_PORT, motor_motion, 0);
              Roller_action_can_pub.publish(motor_motion);
              ros::Duration(0.02).sleep();
              break;

            case 3:
              // Set Z position 0
              SET_Position_data2HEX(Z_SDO_ID, CAN_PORT, motor_motion, 0);
              Roller_action_can_pub.publish(motor_motion);
              ros::Duration(0.02).sleep();
              break;

            case 4:
              if (mode_type == 1)
              {
                ROS_INFO("Speed mode");
                // Set motor to speed mode
                ORI_data2HEX(motor_motion, goal->motor_ID, CAN_PORT, 43, 96, 96, 0, 3, 0, 0, 0);
                Roller_action_can_pub.publish(motor_motion);
                ros::Duration(0.02).sleep();
              }
              else if (mode_type == 2)
              {
                ROS_INFO("Position mode");
                // Set motor to position mode
                ORI_data2HEX(motor_motion, goal->motor_ID, CAN_PORT, 43, 96, 96, 0, 1, 0, 0, 0);
                Roller_action_can_pub.publish(motor_motion);
                ros::Duration(0.02).sleep();
                motor_motion.data.clear();
                ORI_data2HEX(motor_motion, goal->motor_ID, CAN_PORT, 43, 6, 32, 0, 1, 0, 0, 0);
                Roller_action_can_pub.publish(motor_motion);
                ros::Duration(0.02).sleep();
              }
              break;
            case 5:
              ORI_data2HEX(motor_motion,0,2,0,0,0,0,0,0,0,0);//pub null
              break;
            default:
              break;
          }
          if(Can_state.id!=998)
          {
            Roller_action_can_pub.publish(motor_motion);
            ros::Duration(0.02).sleep();
          }
          else
          {
            ROS_INFO("Publish command");
          }
        }
    }
    if (type==5)
    {
      ROS_INFO("Inital,Motor ID:%d" ,goal->motor_ID);
      ROS_INFO("begin_time %lf",start_time);
      double pass_sec=0.0;
      for(int i=0;i<5;i++)
      {
        new_time = Get_the_msec_time_of_computer(time_now);
        pass_sec=((double)(new_time-start_time));
        ROS_INFO("pass time %lf",pass_sec);
        motor_motion.data.clear();
        switch (i)
          {
            case 0:
              // open all device
              ORI_data2HEX(motor_motion, 0, CAN_PORT, 1, 0, 0, 0, 0, 0, 0, 0);
              break;

            case 1:
              // power on, allow brake
              ORI_data2HEX(motor_motion, goal->motor_ID, CAN_PORT, 6, 0, 0, 0, 0, 0, 0, 0);
              break;

            case 2:
              // power on, allow brake, server on
              ORI_data2HEX(motor_motion, goal->motor_ID, CAN_PORT, 7, 0, 0, 0, 0, 0, 0, 0);
              break;

            case 4:
              // Stop the Can command Publisher
              ORI_data2HEX(motor_motion, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0);
              break;

            default:
              break;
          }

        Roller_action_can_pub.publish(motor_motion);
        ros::Duration(0.02).sleep();
      }
    }

    switch (type)
    {
      case 6:
        ROS_INFO("Reset odom, Motor ID: %d", goal->motor_ID);
        publishMotionData(motor_motion, goal->motor_ID, 43, 4, 32, 0, 1, 0, 0, 0);
        break;
      case 7:
        ROS_INFO("Server on, Motor ID: %d", goal->motor_ID);
        publishMotionData(motor_motion, goal->motor_ID, 43, 0, 32, 0, 1, 0, 0, 0);
        break;
      case 8:
        ROS_INFO("Server off, Motor ID: %d", goal->motor_ID);
        publishMotionData(motor_motion, goal->motor_ID, 43, 0, 32, 0, 0, 0, 0, 0);
        break;
      default:
        break;
    }

    r.sleep();

    if(success)
    {
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      action_server_.setSucceeded(result_);
    }

}
Roller_Action_Coder::~Roller_Action_Coder(void)
{

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Roller_Action_Can");
    ros::NodeHandle nh;

    ROS_INFO("Roller_Action_Can");
    Roller_Action_Coder Roller_Action_Coder("Roller_Action_Can");
    ros::spin();
    return 0;
}
