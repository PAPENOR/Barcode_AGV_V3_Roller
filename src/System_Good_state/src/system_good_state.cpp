#include "system_good_state.h"

bool redButton;         // 红色按钮状态
bool yellowButton;      // 黄色按钮状态
bool greenButton;       // 绿色按钮状态
int missionState;       // 任务状态

void redButtonCallback(const std_msgs::Bool::ConstPtr& msg)
{
  redButton = msg->data;   // 更新红色按钮状态
}

void yellowButtonCallback(const std_msgs::Bool::ConstPtr& msg)
{
  yellowButton = msg->data;   // 更新黄色按钮状态
}

void greenButtonCallback(const std_msgs::Bool::ConstPtr& msg)
{
  greenButton = msg->data;   // 更新绿色按钮状态
}

void missionStateCallback(const std_msgs::Int16::ConstPtr& msg)
{
  missionState = msg->data;   // 更新任务状态
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "System_state_Manager");
  ros::NodeHandle nh;
   // 订阅绿色按钮状态
  ros::Subscriber missionStateSub = nh.subscribe("Mission_state", 1000, missionStateCallback);             // 订阅任务状态

  ros::Publisher redLightActionPub        = nh.advertise<std_msgs::Bool>("hand/Red_light_action", 10);        // 发布红色灯光控制信号
  ros::Publisher yellowLightActionPub     = nh.advertise<std_msgs::Bool>("hand/Yellow_light_action", 10);     // 发布黄色灯光控制信号
  ros::Publisher greenLightActionPub      = nh.advertise<std_msgs::Bool>("hand/Green_light_action", 10);      // 发布绿色灯光控制信号
  ros::Publisher goodsStatePub             = nh.advertise<std_msgs::Int16>("Goods_state", 10);              // 发布货物状态
  ros::Publisher buzzerPub                  = nh.advertise<std_msgs::Bool>("hand/Buzzer", 10);               // 发布蜂鸣器控制信号

  ros::Rate loopRate(100.0);

  while (nh.ok())
  {
    switch (missionState)
    {
      case 1:
        {
          std_msgs::Bool turnOn;
          turnOn.data = true;
          greenLightActionPub.publish(turnOn);         // 打开绿色灯光
          yellowLightActionPub.publish(turnOn);        // 打开黄色灯光
          turnOn.data = false;
          redLightActionPub.publish(turnOn);           // 关闭红色灯光
          buzzerPub.publish(turnOn);                   // 关闭蜂鸣器
          break;
        }
      case 2:
        {
          std_msgs::Bool turnOn;
          turnOn.data = true;
          yellowLightActionPub.publish(turnOn);        // 打开黄色灯光
          turnOn.data = false;
          redLightActionPub.publish(turnOn);           // 关闭红色灯光
          greenLightActionPub.publish(turnOn);         // 关闭绿色灯光
          buzzerPub.publish(turnOn);                   // 关闭蜂鸣器
          break;
        }
      case 3:
        {
          std_msgs::Bool turnOn;
          turnOn.data = true;
          greenLightActionPub.publish(turnOn);         // 打开绿色灯光
          turnOn.data = false;
          redLightActionPub.publish(turnOn);           // 关闭红色灯光
          yellowLightActionPub.publish(turnOn);        // 关闭黄色灯光
          buzzerPub.publish(turnOn);                   // 关闭蜂鸣器
          break;
        }
      case 4:
        {
          std_msgs::Bool turnOn;
          turnOn.data = true;
          redLightActionPub.publish(turnOn);           // 打开红色灯光
          buzzerPub.publish(turnOn);                   // 打开蜂鸣器
          turnOn.data = false;
          greenLightActionPub.publish(turnOn);         // 关闭绿色灯光
          yellowLightActionPub.publish(turnOn);        // 关闭黄色灯光
          std_msgs::Int16 goodStateMsgs;
          goodStateMsgs.data = 0;
          goodsStatePub.publish(goodStateMsgs);         // 发布货物状态为0
          break;
        }
    }
    loopRate.sleep();
    ros::spinOnce();
  }

  ros::spin();

  return 0;
}
