#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt16MultiArray.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
using namespace std;
using namespace ros;
class IO_decoder
{
public:
    IO_decoder();
private:
    ros::NodeHandle node;

    ros::Subscriber  Regs_Read;

    ros::Subscriber  Lora_power;
    ros::Subscriber  Roller_Axis;    
    ros::Subscriber  Buzzer;
    ros::Subscriber  Red_light;
    ros::Subscriber  Yellow_light;
    ros::Subscriber  Green_light;

    ros::Publisher  Regs_Write;

    ros::Publisher  Right_Limit;
    ros::Publisher  Left_Limit;
    ros::Publisher  Right_Out;
    ros::Publisher  Left_Out;
    ros::Publisher  Z_Up_Limit;
    ros::Publisher  Z_Down_Limit;
    ros::Publisher  Z_Zero;
    ros::Publisher  Left_stick_state_postion_1;
    ros::Publisher  Left_stick_state_postion_2;
    ros::Publisher  Right_stick_state_postion_1;
    ros::Publisher  Right_stick_state_postion_2;
    ros::Publisher  Front_stick_state_postion_1;
    ros::Publisher  Front_stick_state_postion_2;
    ros::Publisher  Back_stick_state_postion_1;
    ros::Publisher  Back_stick_state_postion_2;

    int             Input_ID;
    int             Output_ID;

    int             Right_Limit_ID;
    bool            Right_Limit_Link;
    int             Left_Limit_ID;
    bool            Left_Limit_Link;
    int             Z_Up_Limit_ID;
    bool            Z_Up_Limit_Link;
    int             Z_Down_Limit_ID;
    bool            Z_Down_Limit_Link;
    int             Z_Zero_ID;
    bool            Z_Zero_Link;
    int             Right_Out_ID;
    bool            Right_Out_Link;
    int             Left_Out_ID;
    bool            Left_Out_Link;

    int             Left_stick_state_postion_1_ID;
    bool            Left_stick_state_postion_1_Link;
    int             Left_stick_state_postion_2_ID;
    bool            Left_stick_state_postion_2_Link;
    int             Right_stick_state_postion_1_ID;
    bool            Right_stick_state_postion_1_Link;
    int             Right_stick_state_postion_2_ID;
    bool            Right_stick_state_postion_2_Link;
    int             Front_stick_state_postion_1_ID;
    bool            Front_stick_state_postion_1_Link;
    int             Front_stick_state_postion_2_ID;
    bool            Front_stick_state_postion_2_Link;
    int             Back_stick_state_postion_1_ID;
    bool            Back_stick_state_postion_1_Link;
    int             Back_stick_state_postion_2_ID;
    bool            Back_stick_state_postion_2_Link;

    int             Buzzer_ID;
    bool            Buzzer_Link;
    bool            Buzzer_command=false;
    int             LoraPower_ID;
    bool            LoraPower_Link;
    bool            LoraPower_command=false;
    int             Roller_Axis_ID_Down;
    int             Roller_Axis_ID_Launch;
    bool            Roller_Axis_Link;
    int             Roller_Axis_command=0;
    int             Red_light_ID;
    bool            Red_light_Link;
    bool            Red_light_command=false;
    int             Yellow_light_ID;
    bool            Yellow_light_Link;
    bool            Yellow_light_command=false;
    int             Green_light_ID;
    bool            Green_light_Link;
    bool            Green_light_command=false;

    std_msgs::UInt16MultiArray Input_Value;
    void regs_callBack(const std_msgs::UInt16MultiArray::ConstPtr &regs_data);
    void buzzer_callBack(const std_msgs::Bool::ConstPtr &buzzer_data);
    void Lora_Power_callBack(const std_msgs::Bool::ConstPtr &Lora_Power_data);
    void Red_light_callBack(const std_msgs::Bool::ConstPtr &Red_light_data);
    void Yellow_light_callBack(const std_msgs::Bool::ConstPtr &Yellow_light_data);
    void Green_light_callBack(const std_msgs::Bool::ConstPtr &Green_light_data);
    void Roller_Axis_callBack(const std_msgs::Int16::ConstPtr &Roller_Axis_data);

    void Right_Limit_Setting(string CodeInputB)
    {
      std_msgs::Bool RLS;
      if(CodeInputB.at(Right_Limit_ID)=='1')
      {
        RLS.data=true;
      }
      else
      {
        RLS.data=false;
      }
      if(Right_Limit_Link==true)
      {
        Right_Limit.publish(RLS);
      }
    }
    void Right_Out_Setting(string CodeInputB)
    {
      std_msgs::Bool ROS_;
      if(CodeInputB.at(Right_Out_ID)=='1')
      {
        ROS_.data=true;
      }
      else
      {
        ROS_.data=false;
      }
      if(Right_Out_Link==true)
      {
        Right_Out.publish(ROS_);
      }
    }
    void Left_Limit_Setting(string CodeInputB)
    {
      std_msgs::Bool LLS;
      if(CodeInputB.at(Left_Limit_ID)=='1')
      {
        LLS.data=true;
      }
      else
      {
        LLS.data=false;
      }
      if(Left_Limit_Link==true)
      {
        Left_Limit.publish(LLS);
      }
    }
    void Left_Out_Setting(string CodeInputB)
    {
      std_msgs::Bool LOS;
      if(CodeInputB.at(Left_Out_ID)=='1')
      {
        LOS.data=true;
      }
      else
      {
        LOS.data=false;
      }
      if(Left_Out_Link==true)
      {
        Left_Out.publish(LOS);
      }
    }
    void Z_Up_Limit_Setting(string CodeInputB)
    {
      std_msgs::Bool ZULS;
      if(CodeInputB.at(Z_Up_Limit_ID)=='1')
      {
        ZULS.data=false;
      }
      else
      {
        ZULS.data=true;
      }
      if(Z_Up_Limit_Link==true)
      {
        Z_Up_Limit.publish(ZULS);
      }
    }
    void Z_Down_Limit_Setting(string CodeInputB)
    {
      std_msgs::Bool ZDLS;
      if(CodeInputB.at(Z_Down_Limit_ID)=='1')
      {
        ZDLS.data=false;
      }
      else
      {
        ZDLS.data=true;
      }
      if(Z_Down_Limit_Link==true)
      {
        Z_Down_Limit.publish(ZDLS);
      }
    }
    void Z_Zero_Setting(string CodeInputB)
    {
      std_msgs::Bool ZZS;
      if(CodeInputB.at(Z_Zero_ID)=='1')
      {
        ZZS.data=true;
      }
      else
      {
        ZZS.data=false;
      }
      if(Z_Zero_Link==true)
      {
        Z_Zero.publish(ZZS);
      }
    }
    void Left_stick_state_postion_1_Setting(string CodeInputB)
    {
      std_msgs::Bool LSSP1S;
      if(CodeInputB.at(Left_stick_state_postion_1_ID)!='1')
      {
        LSSP1S.data=true;
      }
      else
      {
        LSSP1S.data=false;
      }
      if(Left_stick_state_postion_1_Link==true)
      {
        Left_stick_state_postion_1.publish(LSSP1S);
      }
    }
    void Left_stick_state_postion_2_Setting(string CodeInputB)
    {
      std_msgs::Bool LSSP2S;
      if(CodeInputB.at(Left_stick_state_postion_2_ID)!='1')
      {
        LSSP2S.data=true;
      }
      else
      {
        LSSP2S.data=false;
      }
      if(Left_stick_state_postion_2_Link==true)
      {
        Left_stick_state_postion_2.publish(LSSP2S);
      }
    }
    void Right_stick_state_postion_1_Setting(string CodeInputB)
    {
      std_msgs::Bool RSSP1S;
      if(CodeInputB.at(Right_stick_state_postion_1_ID)!='1')
      {
        RSSP1S.data=true;
      }
      else
      {
        RSSP1S.data=false;
      }
      if(Right_stick_state_postion_1_Link==true)
      {
        Right_stick_state_postion_1.publish(RSSP1S);
      }
    }
    void Right_stick_state_postion_2_Setting(string CodeInputB)
    {
      std_msgs::Bool RSSP2S;
      if(CodeInputB.at(Right_stick_state_postion_2_ID)!='1')
      {
        RSSP2S.data=true;
      }
      else
      {
        RSSP2S.data=false;
      }
      if(Right_stick_state_postion_2_Link==true)
      {
        Right_stick_state_postion_2.publish(RSSP2S);
      }
    }
    void Front_stick_state_postion_1_Setting(string CodeInputB)
    {
      std_msgs::Bool FSSP1S;
      if(CodeInputB.at(Front_stick_state_postion_1_ID)!='1')
      {
        FSSP1S.data=true;
      }
      else
      {
        FSSP1S.data=false;
      }
      if(Front_stick_state_postion_1_Link==true)
      {
        Front_stick_state_postion_1.publish(FSSP1S);
      }
    }
    void Front_stick_state_postion_2_Setting(string CodeInputB)
    {
      std_msgs::Bool FSSP2S;
      if(CodeInputB.at(Front_stick_state_postion_2_ID)!='1')
      {
        FSSP2S.data=true;
      }
      else
      {
        FSSP2S.data=false;
      }
      if(Front_stick_state_postion_2_Link==true)
      {
        Front_stick_state_postion_2.publish(FSSP2S);
      }
    }
    void Back_stick_state_postion_1_Setting(string CodeInputB)
    {
      std_msgs::Bool BSSP1S;
      if(CodeInputB.at(Back_stick_state_postion_1_ID)!='1')
      {
        BSSP1S.data=true;
      }
      else
      {
        BSSP1S.data=false;
      }
      if(Back_stick_state_postion_1_Link==true)
      {
        Back_stick_state_postion_1.publish(BSSP1S);
      }
    }
    void Back_stick_state_postion_2_Setting(string CodeInputB)
    {
      std_msgs::Bool BSSP2S;
      if(CodeInputB.at(Back_stick_state_postion_2_ID)!='1')
      {
        BSSP2S.data=true;
      }
      else
      {
        BSSP2S.data=false;
      }
      if(Back_stick_state_postion_2_Link==true)
      {
        Back_stick_state_postion_2.publish(BSSP2S);
      }
    }
};
string toBinary(uint16_t number)
{
  string B_number;
  while(number !=0)
  {
    B_number +=(number%2==0 ? "0":"1");
    number /=2;
  }
  //ROS_WARN_STREAM("Size:"<<B_number.size());
  while(B_number.size()<16)
  {
    B_number +="0";
  }
  return B_number;
}
IO_decoder::IO_decoder()
{
  Regs_Read         = node.subscribe<std_msgs::UInt16MultiArray>("modbus/hand_regs_read", 100,&IO_decoder::regs_callBack, this);
  Buzzer            = node.subscribe<std_msgs::Bool>  ("hand/Buzzer", 100,&IO_decoder::buzzer_callBack, this);
  Lora_power        = node.subscribe<std_msgs::Bool>  ("hand/Lorapower", 100,&IO_decoder::Lora_Power_callBack, this);
  Roller_Axis       = node.subscribe<std_msgs::Int16>  ("hand/Roller_Axis_action", 100,&IO_decoder::Roller_Axis_callBack, this);
  Red_light         = node.subscribe<std_msgs::Bool>  ("hand/Red_light_action", 100,&IO_decoder::Red_light_callBack, this);
  Yellow_light      = node.subscribe<std_msgs::Bool>  ("hand/Yellow_light_action", 100,&IO_decoder::Yellow_light_callBack, this);
  Green_light       = node.subscribe<std_msgs::Bool>  ("hand/Green_light_action", 100,&IO_decoder::Green_light_callBack, this);

  Regs_Write        = node.advertise<std_msgs::UInt16MultiArray>("modbus/hand_regs_write", 100);

  Right_Limit                   = node.advertise<std_msgs::Bool>  ("hand/Right_Limit", 10);
  Left_Limit                    = node.advertise<std_msgs::Bool>  ("hand/Left_Limit", 10);
  Right_Out                     = node.advertise<std_msgs::Bool>  ("hand/Right_Out", 10);
  Left_Out                      = node.advertise<std_msgs::Bool>  ("hand/Left_Out", 10);
  Z_Up_Limit                    = node.advertise<std_msgs::Bool>  ("hand/Z_Up_Limit", 10);
  Z_Down_Limit                  = node.advertise<std_msgs::Bool>  ("hand/Z_Down_Limit", 10);
  Z_Zero                        = node.advertise<std_msgs::Bool>  ("hand/Z_Zero", 10);
  Left_stick_state_postion_1    = node.advertise<std_msgs::Bool>  ("hand/Left_stick_state_postion_1", 10);
  Left_stick_state_postion_2    = node.advertise<std_msgs::Bool>  ("hand/Left_stick_state_postion_2", 10);
  Right_stick_state_postion_1   = node.advertise<std_msgs::Bool>  ("hand/Right_stick_state_postion_1", 10);
  Right_stick_state_postion_2   = node.advertise<std_msgs::Bool>  ("hand/Right_stick_state_postion_2", 10);
  Front_stick_state_postion_1   = node.advertise<std_msgs::Bool>  ("hand/Front_stick_state_postion_1", 10);
  Front_stick_state_postion_2   = node.advertise<std_msgs::Bool>  ("hand/Front_stick_state_postion_2", 10);
  Back_stick_state_postion_1    = node.advertise<std_msgs::Bool>  ("hand/Back_stick_state_postion_1", 10);
  Back_stick_state_postion_2    = node.advertise<std_msgs::Bool>  ("hand/Back_stick_state_postion_2", 10);

  node.param("Modbus_Decode_CH1/Input_ID"               ,Input_ID           ,0);
  node.param("Modbus_Decode_CH1/Output_ID"              ,Output_ID          ,1);

  node.param("Modbus_Decode_CH1/Right_stick_state_postion_1_ID"     ,Right_stick_state_postion_1_ID     ,1);
  node.param("Modbus_Decode_CH1/Right_stick_state_postion_1_Link"   ,Right_stick_state_postion_1_Link   ,true);
  node.param("Modbus_Decode_CH1/Right_stick_state_postion_2_ID"     ,Right_stick_state_postion_2_ID     ,2);
  node.param("Modbus_Decode_CH1/Right_stick_state_postion_2_Link"   ,Right_stick_state_postion_2_Link   ,true);
  node.param("Modbus_Decode_CH1/Left_stick_state_postion_1_ID"      ,Left_stick_state_postion_1_ID      ,3);
  node.param("Modbus_Decode_CH1/Left_stick_state_postion_1_Link"    ,Left_stick_state_postion_1_Link    ,true);
  node.param("Modbus_Decode_CH1/Left_stick_state_postion_2_ID"      ,Left_stick_state_postion_2_ID      ,4);
  node.param("Modbus_Decode_CH1/Left_stick_state_postion_2_Link"    ,Left_stick_state_postion_2_Link    ,true);
  node.param("Modbus_Decode_CH1/Z_Up_Limit_ID"                      ,Z_Up_Limit_ID                      ,5);
  node.param("Modbus_Decode_CH1/Z_Up_Limit_Link"                    ,Z_Up_Limit_Link                    ,true);
  node.param("Modbus_Decode_CH1/Z_Zero_ID"                          ,Z_Zero_ID                          ,6);
  node.param("Modbus_Decode_CH1/Zero_Link"                          ,Z_Zero_Link                        ,true);
  node.param("Modbus_Decode_CH1/Z_Down_Limit_ID"                    ,Z_Down_Limit_ID                    ,7);
  node.param("Modbus_Decode_CH1/Z_Down_Limit_Link"                  ,Z_Down_Limit_Link                  ,true);
  node.param("Modbus_Decode_CH1/Right_Limit_ID"                     ,Right_Limit_ID                     ,8);
  node.param("Modbus_Decode_CH1/Right_Limit_Link"                   ,Right_Limit_Link                   ,true);
  node.param("Modbus_Decode_CH1/Left_Limit_ID"                      ,Left_Limit_ID                      ,9);
  node.param("Modbus_Decode_CH1/Left_Limit_Link"                    ,Left_Limit_Link                    ,true);
  node.param("Modbus_Decode_CH1/Right_Out_ID"                       ,Right_Out_ID                       ,10);
  node.param("Modbus_Decode_CH1/Right_Out_Link"                     ,Right_Out_Link                     ,true);
  node.param("Modbus_Decode_CH1/Left_Out_ID"                        ,Left_Out_ID                        ,11);
  node.param("Modbus_Decode_CH1/Left_Out_Link"                      ,Left_Out_Link                      ,true);
  node.param("Modbus_Decode_CH1/Front_stick_state_postion_1_ID"     ,Front_stick_state_postion_1_ID     ,12);
  node.param("Modbus_Decode_CH1/Front_stick_state_postion_1_Link"   ,Front_stick_state_postion_1_Link   ,true);
  node.param("Modbus_Decode_CH1/Front_stick_state_postion_2_ID"     ,Front_stick_state_postion_2_ID     ,13);
  node.param("Modbus_Decode_CH1/Front_stick_state_postion_2_Link"   ,Front_stick_state_postion_2_Link   ,true);
  node.param("Modbus_Decode_CH1/Back_stick_state_postion_1_ID"      ,Back_stick_state_postion_1_ID      ,14);
  node.param("Modbus_Decode_CH1/Back_stick_state_postion_1_Link"    ,Back_stick_state_postion_1_Link    ,true);
  node.param("Modbus_Decode_CH1/Back_stick_state_postion_2_ID"      ,Back_stick_state_postion_2_ID      ,15);
  node.param("Modbus_Decode_CH1/Back_stick_state_postion_2_Link"    ,Back_stick_state_postion_2_Link    ,true);

  node.param("Modbus_Decode_CH1/Roller_Axis_ID_Down"                ,Roller_Axis_ID_Down                ,9);
  node.param("Modbus_Decode_CH1/Roller_Axis_ID_Launch"              ,Roller_Axis_ID_Launch              ,8);
  node.param("Modbus_Decode_CH1/Roller_Axis_Link"                   ,Roller_Axis_Link                   ,true);
  node.param("Modbus_Decode_CH1/LoraPower_ID"                       ,LoraPower_ID                       ,10);
  node.param("Modbus_Decode_CH1/LoraPower_Link"                     ,LoraPower_Link                     ,true);
  node.param("Modbus_Decode_CH1/buzzer_ID"                          ,Buzzer_ID                          ,12);
  node.param("Modbus_Decode_CH1/buzzer_Link"                        ,Buzzer_Link                        ,true);
  node.param("Modbus_Decode_CH1/Red_light_ID"                       ,Red_light_ID                       ,13);
  node.param("Modbus_Decode_CH1/Red_light_Link"                     ,Red_light_Link                     ,true);
  node.param("Modbus_Decode_CH1/Yellow_light_ID"                    ,Yellow_light_ID                    ,14);
  node.param("Modbus_Decode_CH1/Yellow_light_Link"                  ,Yellow_light_Link                  ,true);
  node.param("Modbus_Decode_CH1/Green_light_ID"                     ,Green_light_ID                     ,15);
  node.param("Modbus_Decode_CH1/Green_light_Link"                   ,Green_light_Link                   ,true);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    for (int i = 0; i < Input_Value.data.size(); i++)
    {
      if(Input_ID==i)
      {
        string CodeInputB=toBinary(Input_Value.data.at(i));
        Right_Limit_Setting(CodeInputB);
        Left_Limit_Setting(CodeInputB);
        Right_Out_Setting(CodeInputB);
        Left_Out_Setting(CodeInputB);
        Z_Up_Limit_Setting(CodeInputB);
        Z_Down_Limit_Setting(CodeInputB);
        Z_Zero_Setting(CodeInputB);
        Left_stick_state_postion_1_Setting(CodeInputB);
        Left_stick_state_postion_2_Setting(CodeInputB);
        Right_stick_state_postion_1_Setting(CodeInputB);
        Right_stick_state_postion_2_Setting(CodeInputB);
        Front_stick_state_postion_1_Setting(CodeInputB);
        Front_stick_state_postion_2_Setting(CodeInputB);
        Back_stick_state_postion_1_Setting(CodeInputB);
        Back_stick_state_postion_2_Setting(CodeInputB);

        std_msgs::UInt16MultiArray regs_write_msgs;
        if(Output_ID>Input_ID)
        {
          for(int j=0;j<=Output_ID;j++)
          {
            regs_write_msgs.data.push_back(0);
          }
        }
        else
        {
          for(int j=0;j<=Input_ID;j++)
          {
            regs_write_msgs.data.push_back(0);
          }
        }

        uint16_t OutputCode=0;
        uint16_t BuzzerCode=0;
        uint16_t LoraPowerCode=0;
        uint16_t Roller_Axis_ID_DownCode=0;
        uint16_t Roller_Axis_ID_LaunchCode=0;
        uint16_t Red_lightCode=0;
        uint16_t Yellow_lightCode=0;
        uint16_t Green_lightCode=0;
        if(Buzzer_Link==true)
        {
          BuzzerCode=int(Buzzer_command)*pow(2,Buzzer_ID);
        }
        if(Red_light_Link==true)
        {
          Red_lightCode=int(Red_light_command)*pow(2,Red_light_ID);
        }
        if(Yellow_light_Link==true)
        {
          Yellow_lightCode=int(Yellow_light_command)*pow(2,Yellow_light_ID);
        }
        if(Green_light_Link==true)
        {
          Green_lightCode=int(Green_light_command)*pow(2,Green_light_ID);
        }
        if(LoraPower_Link==true)
        {
          LoraPowerCode=int(LoraPower_command)*pow(2,LoraPower_ID);
        }
        if(Roller_Axis_Link==true)
        {
          if(Roller_Axis_command==0)
          {
           Roller_Axis_ID_LaunchCode=0*pow(2,Roller_Axis_ID_Launch);
          }
          else if (Roller_Axis_command==1)
          {
           Roller_Axis_ID_LaunchCode=1*pow(2,Roller_Axis_ID_Launch);
           Roller_Axis_ID_DownCode=0*pow(2,Roller_Axis_ID_Down);

          }
          else if (Roller_Axis_command==2)
          {
            Roller_Axis_ID_LaunchCode=1*pow(2,Roller_Axis_ID_Launch);
            Roller_Axis_ID_DownCode=1*pow(2,Roller_Axis_ID_Down);
          }
        }

        OutputCode=BuzzerCode+LoraPowerCode                   
                   +Roller_Axis_ID_DownCode+Roller_Axis_ID_LaunchCode
                   +Red_lightCode
                   +Yellow_lightCode
                   +Green_lightCode;
        regs_write_msgs.data.at(Output_ID)=OutputCode;
        Regs_Write.publish(regs_write_msgs);

      }

    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
void IO_decoder::regs_callBack(const std_msgs::UInt16MultiArray::ConstPtr &regs_data)
{
 Input_Value.data=regs_data->data;
}
void IO_decoder::buzzer_callBack(const std_msgs::Bool::ConstPtr &buzzer_data)
{
 Buzzer_command=buzzer_data->data;
}
void IO_decoder::Lora_Power_callBack(const std_msgs::Bool::ConstPtr &Lora_Power_data)
{
 LoraPower_command=Lora_Power_data->data;
}
void IO_decoder::Roller_Axis_callBack(const std_msgs::Int16::ConstPtr &Roller_Axis_data)
{
 Roller_Axis_command=Roller_Axis_data->data;
}
void IO_decoder::Red_light_callBack(const std_msgs::Bool::ConstPtr &Red_light_data)
{
 Red_light_command=Red_light_data->data;
}
void IO_decoder::Yellow_light_callBack(const std_msgs::Bool::ConstPtr &Yellow_light_data)
{
 Yellow_light_command=Yellow_light_data->data;
}
void IO_decoder::Green_light_callBack(const std_msgs::Bool::ConstPtr &Green_light_data)
{
 Green_light_command=Green_light_data->data;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Modbus_Decode_CH1");
  IO_decoder IOD;
  return 0;
}
