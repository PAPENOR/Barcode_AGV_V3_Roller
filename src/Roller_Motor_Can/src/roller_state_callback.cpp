#include "roller_state_can.h"
using namespace ros;
int Roller_State_Can::speed_data(int speeddata1,int speeddata2)
{
  /*Set the speed_data*/
  int speed_return;
  stringstream A1,D1;
  string E1;

  /*Set the input_dec data to the HEX-type data*/
  A1<<std::hex<<speeddata2;
  if(speeddata1<=15)
  {
  A1<<std::hex<<0;
  }
  A1<<std::hex<<speeddata1;

  /*Change the data format to String*/
  string B(A1.str());

  /*Set data back to dec-type data*/
  D1<<"0X"<<std::dec<<B;

  /*Change the data format to String*/
  E1=D1.str();

  /*Change the data format to int*/
  speed_return=stoi(E1,nullptr,16);



  /*set the value as "+" or "-"*/
  if(speed_return>32767)
  {
    speed_return=speed_return-65536;
  }

  //ROS_WARN("speed_return:%d ",speed_return);

  return  speed_return;
}
int Roller_State_Can::position_data(int positiondata1,int positiondata2,int positiondata3,int positiondata4)
{
  /*Set the position_data*/
  int position_return,nev;
  stringstream A1,A2,A3,D1,D2,D3;
  string E1,E2;

  /*Set the input_dec data to the HEX-type data*/

  if(positiondata4>15)
  {
  A1<<std::hex<<positiondata4;
  }
  else
  {
  A1<<std::hex<<0;
  A1<<std::hex<<positiondata4;
  }
  if(positiondata3>15)
  {
  A1<<std::hex<<positiondata3;
  }
  else
  {
  A1<<std::hex<<0;
  A1<<std::hex<<positiondata3;
  }

  /*Change the data format to String*/
  string B1(A1.str());

  /*Set data back to dec-type data*/
  D1<<"0X"<<std::dec<<B1;

  /*Change the data format to String*/
  E1=D1.str();

  /*Change the data format to int*/
  nev=stoi(E1,nullptr,16);

  if(nev>32768)
  {
    /*set the value as it is "-"*/
    /*first number of digits*/
    int positionunit1=65535-nev,positionunit2;

    /*Set the input_dec data to the HEX-type data*/
    if(positiondata2>15)
    {
    A3<<std::hex<<positiondata2;
    }
    else
    {
    A3<<std::hex<<0;
    A3<<std::hex<<positiondata2;
    }
    if(positiondata1>15)
    {
    A3<<std::hex<<positiondata1;
    }
    else
    {
    A3<<std::hex<<0;
    A3<<std::hex<<positiondata1;
    }

    /*Change the data format to String*/
    string B3(A3.str());

    /*Set data back to dec-type data*/
    D3<<"0X"<<std::dec<<B3;

    /*Change the data format to String*/
    E2=D3.str();

    /*Change the data format to int*/
    positionunit2=stoi(E2,nullptr,16);

    /*second number of digits*/
    positionunit2=65536-positionunit2;

    /*return the position data*/
    position_return=-(positionunit2+positionunit1*65536);

  }
  else
  {
  /*set the value as it is "+"*/
  /*Set the input_dec data to the HEX-type data*/
  if(positiondata4>15)
  {
  A2<<std::hex<<positiondata4;
  }
  else
  {
  A2<<std::hex<<0;
  A2<<std::hex<<positiondata4;
  }
  if(positiondata3>15)
  {
  A2<<std::hex<<positiondata3;
  }
  else
  {
  A2<<std::hex<<0;
  A2<<std::hex<<positiondata3;
  }
  if(positiondata2>15)
  {
  A2<<std::hex<<positiondata2;
  }
  else
  {
  A2<<std::hex<<0;
  A2<<std::hex<<positiondata2;
  }
  if(positiondata1>15)
  {
  A2<<std::hex<<positiondata1;
  }
  else
  {
  A2<<std::hex<<0;
  A2<<std::hex<<positiondata1;
  }

  /*Change the data format to String*/
  string B2(A2.str());

  /*Set data back to dec-type data*/
  D2<<"0X"<<std::dec<<B2;

  /*Change the data format to String*/
  E2=D2.str();



  /*Change the data format to int*/
  position_return=stoi(E2,nullptr,16);

  }
  return  position_return;
}
void Roller_State_Can::motor_moving_mode(int motor_ID,int moving_type,int condition,int moving_speed,bool brake_mode,actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_state_client)
{
  /*Sending a single motor command*/
  /*Setting the index*/
  goal.motor_ID         = motor_ID;
  goal.moving_type      = moving_type;
  goal.moving_condition = condition;
  goal.moving_speed     = moving_speed;
  goal.Brake_wheel      = brake_mode;

  /*Sending the command*/
  Can_state_client.sendGoal(goal);

  /*Waiting for the action result*/
  bool finished_before_timeout = Can_state_client.waitForResult(ros::Duration(30.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = Can_state_client.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_INFO("Action did not finish before the time out.");
  }
}

void Roller_State_Can::Can_state_Callback(const Roller_Motor_Can::can& msg)
{
  /*Receive the topic msgs*/
  Can_state=msg;
  if(Can_state.id==Z_RT_ID_READ)
  {
//    ROS_INFO_STREAM("Size:"<<Can_state.data.size());
    /*According to the Can_state ID to set the speed data and position data*/
    int speeddata1=Can_state.data.at(2),speeddata2=Can_state.data.at(3);
    int positiondata1=Can_state.data.at(4),positiondata2=Can_state.data.at(5)
        ,positiondata3=Can_state.data.at(6),positiondata4=Can_state.data.at(7);
//    ROS_INFO_STREAM("speeddata1:"<<speeddata1);
//    ROS_INFO_STREAM("speeddata2:"<<speeddata2);
//    ROS_INFO_STREAM("positiondata1:"<<positiondata1);
//    ROS_INFO_STREAM("positiondata2:"<<positiondata2);
//    ROS_INFO_STREAM("positiondata3:"<<positiondata3);
//    ROS_INFO_STREAM("positiondata4:"<<positiondata4);
    R_speed=speed_data(speeddata1,speeddata2);
    R_position=position_data(positiondata1,positiondata2,positiondata3,positiondata4);
  }
}

void Roller_State_Can::Set_state(int state_code,int Rspeed,int Rposition,std_msgs::Int64MultiArray &state_code_R)
{
  /*Setting the Car_msgs as decimal type */
  state_code_R.data.push_back(state_code);
  state_code_R.data.push_back(Rspeed);
  state_code_R.data.push_back(Rposition);
}
