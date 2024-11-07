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

    ros::Subscriber Regs_Read;
    ros::Publisher  Regs_Write;

    ros::Publisher  Right_Stick_Home_State;
    ros::Publisher  Right_Stick_Err_State;
    ros::Publisher  Right_Stick_Inp_State;
    ros::Subscriber Right_Stick_Action;

    ros::Publisher  Left_Stick_Home_State;
    ros::Publisher  Left_Stick_Err_State;
    ros::Publisher  Left_Stick_Inp_State;
    ros::Subscriber Left_Stick_Action;

    ros::Publisher  Front_Stick_Home_State;
    ros::Publisher  Front_Stick_Err_State;
    ros::Publisher  Front_Stick_Inp_State;
    ros::Subscriber Front_Stick_Action;

    ros::Publisher  Back_Stick_Home_State;
    ros::Publisher  Back_Stick_Err_State;
    ros::Publisher  Back_Stick_Inp_State;
    ros::Subscriber Back_Stick_Action;

    int             Input_ID;
    int             Output_ID;

    int             Right_Stick_Home_State_ID;
    bool            Right_Stick_Home_State_Link;
    int             Right_Stick_Err_State_ID;
    bool            Right_Stick_Err_State_Link;
    int             Right_Stick_Inp_State_ID;
    bool            Right_Stick_Inp_State_Link;

    int             Left_Stick_Home_State_ID;
    bool            Left_Stick_Home_State_Link;
    int             Left_Stick_Err_State_ID;
    bool            Left_Stick_Err_State_Link;
    int             Left_Stick_Inp_State_ID;
    bool            Left_Stick_Inp_State_Link;

    int             Front_Stick_Home_State_ID;
    bool            Front_Stick_Home_State_Link;
    int             Front_Stick_Err_State_ID;
    bool            Front_Stick_Err_State_Link;
    int             Front_Stick_Inp_State_ID;
    bool            Front_Stick_Inp_State_Link;

    int             Back_Stick_Home_State_ID;
    bool            Back_Stick_Home_State_Link;
    int             Back_Stick_Err_State_ID;
    bool            Back_Stick_Err_State_Link;
    int             Back_Stick_Inp_State_ID;
    bool            Back_Stick_Inp_State_Link;

    int             Right_Stick_Position_Action_ID;
    int             Right_Stick_Launch_Action_ID;
    int             Right_Stick_Home_Action_ID;
    int             Right_Stick_Reset_Action_ID;
    bool            Right_Stick_Action_Link;
    int             Right_Stick_Action_command=0;

    int             Left_Stick_Position_Action_ID;
    int             Left_Stick_Launch_Action_ID;
    int             Left_Stick_Home_Action_ID;
    int             Left_Stick_Reset_Action_ID;
    bool            Left_Stick_Action_Link;
    int             Left_Stick_Action_command=0;

    int             Front_Stick_Position_Action_ID;
    int             Front_Stick_Launch_Action_ID;
    int             Front_Stick_Home_Action_ID;
    int             Front_Stick_Reset_Action_ID;
    bool            Front_Stick_Action_Link;
    int             Front_Stick_Action_command=0;

    int             Back_Stick_Position_Action_ID;
    int             Back_Stick_Launch_Action_ID;
    int             Back_Stick_Home_Action_ID;
    int             Back_Stick_Reset_Action_ID;
    bool            Back_Stick_Action_Link;
    int             Back_Stick_Action_command=0;

    std_msgs::UInt16MultiArray Input_Value;
    void regs_callBack(const std_msgs::UInt16MultiArray::ConstPtr &regs_data);
    void Right_Stick_Action_CallBack(const std_msgs::Int16::ConstPtr &Right_Stick_Action_Data);
    void Right_Stick_Home_State_Setting(string CodeInputB)
    {
      std_msgs::Bool RSHS;
      if(CodeInputB.at(Right_Stick_Home_State_ID)=='1')
      {
        RSHS.data=true;
      }
      else
      {
        RSHS.data=false;
      }
      if(Right_Stick_Home_State_Link==true)
      {
        Right_Stick_Home_State.publish(RSHS);
      }
    }
    void Right_Stick_Err_State_Setting(string CodeInputB)
    {
      std_msgs::Bool RSES;
      if(CodeInputB.at(Right_Stick_Err_State_ID)=='1')
      {
        RSES.data=true;
      }
      else
      {
        RSES.data=false;
      }
      if(Right_Stick_Err_State_Link==true)
      {
        Right_Stick_Err_State.publish(RSES);
      }
    }
    void Right_Stick_Inp_State_Setting(string CodeInputB)
    {
      std_msgs::Bool RSIS;
      if(CodeInputB.at(Right_Stick_Inp_State_ID)=='1')
      {
        RSIS.data=true;
      }
      else
      {
        RSIS.data=false;
      }
      if(Right_Stick_Inp_State_Link==true)
      {
        Right_Stick_Inp_State.publish(RSIS);
      }
    }

    void Left_Stick_Action_CallBack(const std_msgs::Int16::ConstPtr &Left_Stick_Action_Data);
    void Left_Stick_Home_State_Setting(string CodeInputB)
    {
      std_msgs::Bool LSHS;
      if(CodeInputB.at(Left_Stick_Home_State_ID)=='1')
      {
        LSHS.data=true;
      }
      else
      {
        LSHS.data=false;
      }
      if(Left_Stick_Home_State_Link==true)
      {
        Left_Stick_Home_State.publish(LSHS);
      }
    }
    void Left_Stick_Err_State_Setting(string CodeInputB)
    {
      std_msgs::Bool LSES;
      if(CodeInputB.at(Left_Stick_Err_State_ID)=='1')
      {
        LSES.data=true;
      }
      else
      {
        LSES.data=false;
      }
      if(Left_Stick_Err_State_Link==true)
      {
        Left_Stick_Err_State.publish(LSES);
      }
    }
    void Left_Stick_Inp_State_Setting(string CodeInputB)
    {
      std_msgs::Bool LSIS;
      if(CodeInputB.at(Left_Stick_Inp_State_ID)=='1')
      {
        LSIS.data=true;
      }
      else
      {
        LSIS.data=false;
      }
      if(Left_Stick_Inp_State_Link==true)
      {
        Left_Stick_Inp_State.publish(LSIS);
      }
    }

    void Front_Stick_Action_CallBack(const std_msgs::Int16::ConstPtr &Front_Stick_Action_Data);
    void Front_Stick_Home_State_Setting(string CodeInputB)
    {
      std_msgs::Bool FSHS;
      if(CodeInputB.at(Front_Stick_Home_State_ID)=='1')
      {
        FSHS.data=true;
      }
      else
      {
        FSHS.data=false;
      }
      if(Front_Stick_Home_State_Link==true)
      {
        Front_Stick_Home_State.publish(FSHS);
      }
    }
    void Front_Stick_Err_State_Setting(string CodeInputB)
    {
      std_msgs::Bool FSES;
      if(CodeInputB.at(Front_Stick_Err_State_ID)=='1')
      {
        FSES.data=true;
      }
      else
      {
        FSES.data=false;
      }
      if(Front_Stick_Err_State_Link==true)
      {
        Front_Stick_Err_State.publish(FSES);
      }
    }
    void Front_Stick_Inp_State_Setting(string CodeInputB)
    {
      std_msgs::Bool FSIS;
      if(CodeInputB.at(Front_Stick_Inp_State_ID)=='1')
      {
        FSIS.data=true;
      }
      else
      {
        FSIS.data=false;
      }
      if(Front_Stick_Inp_State_Link==true)
      {
        Front_Stick_Inp_State.publish(FSIS);
      }
    }

    void Back_Stick_Action_CallBack(const std_msgs::Int16::ConstPtr &Back_Stick_Action_Data);
    void Back_Stick_Home_State_Setting(string CodeInputB)
    {
      std_msgs::Bool BSHS;
      if(CodeInputB.at(Back_Stick_Home_State_ID)=='1')
      {
        BSHS.data=true;
      }
      else
      {
        BSHS.data=false;
      }
      if(Back_Stick_Home_State_Link==true)
      {
        Back_Stick_Home_State.publish(BSHS);
      }
    }
    void Back_Stick_Err_State_Setting(string CodeInputB)
    {
      std_msgs::Bool BSES;
      if(CodeInputB.at(Back_Stick_Err_State_ID)=='1')
      {
        BSES.data=true;
      }
      else
      {
        BSES.data=false;
      }
      if(Back_Stick_Err_State_Link==true)
      {
        Back_Stick_Err_State.publish(BSES);
      }
    }
    void Back_Stick_Inp_State_Setting(string CodeInputB)
    {
      std_msgs::Bool BSIS;
      if(CodeInputB.at(Back_Stick_Inp_State_ID)=='1')
      {
        BSIS.data=true;
      }
      else
      {
        BSIS.data=false;
      }
      if(Back_Stick_Inp_State_Link==true)
      {
        Back_Stick_Inp_State.publish(BSIS);
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
  while(B_number.size()<16)
  {
    B_number +="0";
  }
  return B_number;
}
IO_decoder::IO_decoder()
{
  Regs_Read                     = node.subscribe<std_msgs::UInt16MultiArray>("modbus/hand_regs_read_ch2", 10,&IO_decoder::regs_callBack, this);
  Right_Stick_Action            = node.subscribe<std_msgs::Int16>("hand/Right_Stick_action", 10,&IO_decoder::Right_Stick_Action_CallBack, this);
  Left_Stick_Action             = node.subscribe<std_msgs::Int16>("hand/Left_Stick_action", 10,&IO_decoder::Left_Stick_Action_CallBack, this);
  Front_Stick_Action            = node.subscribe<std_msgs::Int16>("hand/Front_Stick_action", 10,&IO_decoder::Front_Stick_Action_CallBack, this);
  Back_Stick_Action             = node.subscribe<std_msgs::Int16>("hand/Back_Stick_action", 10,&IO_decoder::Back_Stick_Action_CallBack, this);

  Regs_Write                    = node.advertise<std_msgs::UInt16MultiArray>("modbus/hand_regs_write_ch2", 10);
  Right_Stick_Home_State        = node.advertise<std_msgs::Bool>  ("hand/right_stick_home_state", 10);
  Right_Stick_Err_State         = node.advertise<std_msgs::Bool>  ("hand/right_stick_err_state", 10);
  Right_Stick_Inp_State         = node.advertise<std_msgs::Bool>  ("hand/right_stick_inp_state", 10);

  Left_Stick_Home_State         = node.advertise<std_msgs::Bool>  ("hand/left_stick_home_state", 10);
  Left_Stick_Err_State          = node.advertise<std_msgs::Bool>  ("hand/left_stick_err_state", 10);
  Left_Stick_Inp_State          = node.advertise<std_msgs::Bool>  ("hand/left_stick_inp_state", 10);

  Front_Stick_Home_State        = node.advertise<std_msgs::Bool>  ("hand/front_stick_home_state", 10);
  Front_Stick_Err_State         = node.advertise<std_msgs::Bool>  ("hand/front_stick_err_state", 10);
  Front_Stick_Inp_State         = node.advertise<std_msgs::Bool>  ("hand/front_stick_inp_state", 10);

  Back_Stick_Home_State         = node.advertise<std_msgs::Bool>  ("hand/back_stick_home_state", 10);
  Back_Stick_Err_State          = node.advertise<std_msgs::Bool>  ("hand/back_stick_err_state", 10);
  Back_Stick_Inp_State          = node.advertise<std_msgs::Bool>  ("hand/back_stick_inp_state", 10);

  node.param("Modbus_Decode_CH2/Input_ID"             ,Input_ID           ,0);
  node.param("Modbus_Decode_CH2/Output_ID"            ,Output_ID          ,1);
  node.param("Modbus_Decode_CH2/Right_Stick_Home_State_ID"      ,Right_Stick_Home_State_ID      ,1);
  node.param("Modbus_Decode_CH2/Right_Stick_Home_State_Link"    ,Right_Stick_Home_State_Link    ,true);
  node.param("Modbus_Decode_CH2/Right_Stick_Err_State_ID"       ,Right_Stick_Err_State_ID       ,2);
  node.param("Modbus_Decode_CH2/Right_Stick_Err_State_Link"     ,Right_Stick_Err_State_Link     ,true);
  node.param("Modbus_Decode_CH2/Right_Stick_Inp_State_ID"       ,Right_Stick_Inp_State_ID       ,3);
  node.param("Modbus_Decode_CH2/Right_Stick_Inp_State_Link"     ,Right_Stick_Inp_State_Link     ,true);

  node.param("Modbus_Decode_CH2/Right_Stick_Position_Action_ID" ,Right_Stick_Position_Action_ID ,0);
  node.param("Modbus_Decode_CH2/Right_Stick_Launch_Action_ID"   ,Right_Stick_Launch_Action_ID   ,1);
  node.param("Modbus_Decode_CH2/Right_Stick_Home_Action_ID"     ,Right_Stick_Home_Action_ID     ,2);
  node.param("Modbus_Decode_CH2/Right_Stick_Reset_Action_ID"    ,Right_Stick_Reset_Action_ID    ,3);
  node.param("Modbus_Decode_CH2/Right_Stick_Action_Link"        ,Right_Stick_Action_Link        ,true);

  node.param("Modbus_Decode_CH2/Left_Stick_Home_State_ID"      ,Left_Stick_Home_State_ID        ,5);
  node.param("Modbus_Decode_CH2/Left_Stick_Home_State_Link"    ,Left_Stick_Home_State_Link      ,true);
  node.param("Modbus_Decode_CH2/Left_Stick_Err_State_ID"       ,Left_Stick_Err_State_ID         ,6);
  node.param("Modbus_Decode_CH2/Left_Stick_Err_State_Link"     ,Left_Stick_Err_State_Link       ,true);
  node.param("Modbus_Decode_CH2/Left_Stick_Inp_State_ID"       ,Left_Stick_Inp_State_ID         ,7);
  node.param("Modbus_Decode_CH2/Left_Stick_Inp_State_Link"     ,Left_Stick_Inp_State_Link       ,true);

  node.param("Modbus_Decode_CH2/Left_Stick_Position_Action_ID" ,Left_Stick_Position_Action_ID   ,4);
  node.param("Modbus_Decode_CH2/Left_Stick_Launch_Action_ID"   ,Left_Stick_Launch_Action_ID     ,5);
  node.param("Modbus_Decode_CH2/Left_Stick_Home_Action_ID"     ,Left_Stick_Home_Action_ID       ,6);
  node.param("Modbus_Decode_CH2/Left_Stick_Reset_Action_ID"    ,Left_Stick_Reset_Action_ID      ,7);
  node.param("Modbus_Decode_CH2/Left_Stick_Action_Link"        ,Left_Stick_Action_Link          ,true);

  node.param("Modbus_Decode_CH2/Front_Stick_Home_State_ID"     ,Front_Stick_Home_State_ID       ,9);
  node.param("Modbus_Decode_CH2/Front_Stick_Home_State_Link"   ,Front_Stick_Home_State_Link     ,true);
  node.param("Modbus_Decode_CH2/Front_Stick_Err_State_ID"      ,Front_Stick_Err_State_ID        ,10);
  node.param("Modbus_Decode_CH2/Front_Stick_Err_State_Link"    ,Front_Stick_Err_State_Link      ,true);
  node.param("Modbus_Decode_CH2/Front_Stick_Inp_State_ID"      ,Front_Stick_Inp_State_ID        ,11);
  node.param("Modbus_Decode_CH2/Front_Stick_Inp_State_Link"    ,Front_Stick_Inp_State_Link      ,true);

  node.param("Modbus_Decode_CH2/Front_Stick_Position_Action_ID",Front_Stick_Position_Action_ID  ,8);
  node.param("Modbus_Decode_CH2/Front_Stick_Launch_Action_ID"  ,Front_Stick_Launch_Action_ID    ,9);
  node.param("Modbus_Decode_CH2/Front_Stick_Home_Action_ID"    ,Front_Stick_Home_Action_ID      ,10);
  node.param("Modbus_Decode_CH2/Front_Stick_Reset_Action_ID"   ,Front_Stick_Reset_Action_ID     ,11);
  node.param("Modbus_Decode_CH2/Front_Stick_Action_Link"       ,Front_Stick_Action_Link         ,true);

  node.param("Modbus_Decode_CH2/Back_Stick_Home_State_ID"      ,Back_Stick_Home_State_ID        ,13);
  node.param("Modbus_Decode_CH2/Back_Stick_Home_State_Link"    ,Back_Stick_Home_State_Link      ,true);
  node.param("Modbus_Decode_CH2/Back_Stick_Err_State_ID"       ,Back_Stick_Err_State_ID         ,14);
  node.param("Modbus_Decode_CH2/Back_Stick_Err_State_Link"     ,Back_Stick_Err_State_Link       ,true);
  node.param("Modbus_Decode_CH2/Back_Stick_Inp_State_ID"       ,Back_Stick_Inp_State_ID         ,15);
  node.param("Modbus_Decode_CH2/Back_Stick_Inp_State_Link"     ,Back_Stick_Inp_State_Link       ,true);

  node.param("Modbus_Decode_CH2/Back_Stick_Position_Action_ID" ,Back_Stick_Position_Action_ID   ,12);
  node.param("Modbus_Decode_CH2/Back_Stick_Launch_Action_ID"   ,Back_Stick_Launch_Action_ID     ,13);
  node.param("Modbus_Decode_CH2/Back_Stick_Home_Action_ID"     ,Back_Stick_Home_Action_ID       ,14);
  node.param("Modbus_Decode_CH2/Back_Stick_Reset_Action_ID"    ,Back_Stick_Reset_Action_ID      ,15);
  node.param("Modbus_Decode_CH2/Back_Stick_Action_Link"        ,Back_Stick_Action_Link          ,true);

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    for (int i = 0; i < Input_Value.data.size(); i++)
    {
      if(Input_ID==i)
      {
        string CodeInputB=toBinary(Input_Value.data.at(i));

        Right_Stick_Home_State_Setting(CodeInputB);
        Right_Stick_Err_State_Setting(CodeInputB);
        Right_Stick_Inp_State_Setting(CodeInputB);

        Left_Stick_Home_State_Setting(CodeInputB);
        Left_Stick_Err_State_Setting(CodeInputB);
        Left_Stick_Inp_State_Setting(CodeInputB);

        Front_Stick_Home_State_Setting(CodeInputB);
        Front_Stick_Err_State_Setting(CodeInputB);
        Front_Stick_Inp_State_Setting(CodeInputB);

        Back_Stick_Home_State_Setting(CodeInputB);
        Back_Stick_Err_State_Setting(CodeInputB);
        Back_Stick_Inp_State_Setting(CodeInputB);


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
        uint16_t Right_Stick_Action_Code=0;
        uint16_t Left_Stick_Action_Code=0;
        uint16_t Front_Stick_Action_Code=0;
        uint16_t Back_Stick_Action_Code=0;
        if(Right_Stick_Action_Link==true)
        {
            if(Right_Stick_Action_command==0)
            {
                Right_Stick_Action_Code=0;
            }
            if(Right_Stick_Action_command==1)
            {
                Right_Stick_Action_Code=1*pow(2,Right_Stick_Home_Action_ID);
            }
            if(Right_Stick_Action_command==2)
            {
                Right_Stick_Action_Code=1*pow(2,Right_Stick_Reset_Action_ID);
            }
            if(Right_Stick_Action_command==3)
            {
                Right_Stick_Action_Code=1*pow(2,Right_Stick_Launch_Action_ID);
            }
            if(Right_Stick_Action_command==4)
            {
                Right_Stick_Action_Code=1*pow(2,Right_Stick_Launch_Action_ID)+1*pow(2,Right_Stick_Position_Action_ID);
            }
        }
        if(Left_Stick_Action_Link==true)
        {
            if(Left_Stick_Action_command==0)
            {
                Left_Stick_Action_Code=0;
            }
            if(Left_Stick_Action_command==1)
            {
                Left_Stick_Action_Code=1*pow(2,Left_Stick_Home_Action_ID);
            }
            if(Left_Stick_Action_command==2)
            {
                Left_Stick_Action_Code=1*pow(2,Left_Stick_Reset_Action_ID);
            }
            if(Left_Stick_Action_command==3)
            {
                Left_Stick_Action_Code=1*pow(2,Left_Stick_Launch_Action_ID);
            }
            if(Left_Stick_Action_command==4)
            {
                Left_Stick_Action_Code=1*pow(2,Left_Stick_Launch_Action_ID)+1*pow(2,Left_Stick_Position_Action_ID);
            }
        }
        if(Front_Stick_Action_Link==true)
        {
            if(Front_Stick_Action_command==0)
            {
                Front_Stick_Action_Code=0;
            }
            if(Front_Stick_Action_command==1)
            {
                Front_Stick_Action_Code=1*pow(2,Front_Stick_Home_Action_ID);
            }
            if(Front_Stick_Action_command==2)
            {
                Front_Stick_Action_Code=1*pow(2,Front_Stick_Reset_Action_ID);
            }
            if(Front_Stick_Action_command==3)
            {
                Front_Stick_Action_Code=1*pow(2,Front_Stick_Launch_Action_ID);
            }
            if(Front_Stick_Action_command==4)
            {
                Front_Stick_Action_Code=1*pow(2,Front_Stick_Launch_Action_ID)+1*pow(2,Front_Stick_Position_Action_ID);
            }
        }
        if(Back_Stick_Action_Link==true)
        {
            if(Back_Stick_Action_command==0)
            {
                Back_Stick_Action_Code=0;
            }
            if(Back_Stick_Action_command==1)
            {
                Back_Stick_Action_Code=1*pow(2,Back_Stick_Home_Action_ID);
            }
            if(Back_Stick_Action_command==2)
            {
                Back_Stick_Action_Code=1*pow(2,Back_Stick_Reset_Action_ID);
            }
            if(Back_Stick_Action_command==3)
            {
                Back_Stick_Action_Code=1*pow(2,Back_Stick_Launch_Action_ID);
            }
            if(Back_Stick_Action_command==4)
            {
                Back_Stick_Action_Code=1*pow(2,Back_Stick_Launch_Action_ID)+1*pow(2,Back_Stick_Position_Action_ID);
            }
        }
        OutputCode=Right_Stick_Action_Code+ Left_Stick_Action_Code+ Front_Stick_Action_Code+ Back_Stick_Action_Code;

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

void IO_decoder::Right_Stick_Action_CallBack(const std_msgs::Int16::ConstPtr &Right_Stick_Action_Data)
{
 Right_Stick_Action_command=Right_Stick_Action_Data->data;
}
void IO_decoder::Left_Stick_Action_CallBack(const std_msgs::Int16::ConstPtr &Left_Stick_Action_Data)
{
 Left_Stick_Action_command=Left_Stick_Action_Data->data;
}
void IO_decoder::Front_Stick_Action_CallBack(const std_msgs::Int16::ConstPtr &Front_Stick_Action_Data)
{
 Front_Stick_Action_command=Front_Stick_Action_Data->data;
}
void IO_decoder::Back_Stick_Action_CallBack(const std_msgs::Int16::ConstPtr &Back_Stick_Action_Data)
{
 Back_Stick_Action_command=Back_Stick_Action_Data->data;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Modbus_Decode_CH2");
  IO_decoder IOD;
  return 0;
}
