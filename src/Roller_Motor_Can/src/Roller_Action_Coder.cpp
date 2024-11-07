#include "Roller_Action_Coder.h"
void Roller_Action_Coder::dec2HEX(int input_dec,int &output_HEX1,int &output_HEX2,int &output_HEX3,int &output_HEX4)
{
  /*Set the input_dec data to 4 pieces of Hex-type data*/
  stringstream A1,A2,D1,D2,D3,D4;
  string C1,C2,C3,C4;
  string E1,E2,E3,E4;

  /*Set the input_dec data to the HEX-type data*/
  A1<<std::hex<<input_dec;

  /*Repair "0" to data.Let the data have the same size(8)*/
  int size=8-A1.str().size();
  for(int i=0;i<size;i++)
  {
    A2<<"0";
  }
  A2<<std::hex<<input_dec;

  /*Change the data format to String*/
  string B(A2.str());

  /*Cut the data to 4 pieces*/
  C1=C1.assign(B,0,2);
  C2=C2.assign(B,2,2);
  C3=C3.assign(B,4,2);
  C4=C4.assign(B,6,2);

  /*Set those pieces of data back to dec-type data*/
  D1<<"0X"<<std::dec<<C1;
  D2<<"0X"<<std::dec<<C2;
  D3<<"0X"<<std::dec<<C3;
  D4<<"0X"<<std::dec<<C4;

  /*Change the data format to String*/
  E1=D1.str();
  E2=D2.str();
  E3=D3.str();
  E4=D4.str();

  /*Change the data format to int*/
  output_HEX1=stoi(E1,nullptr,16);
  output_HEX2=stoi(E2,nullptr,16);
  output_HEX3=stoi(E3,nullptr,16);
  output_HEX4=stoi(E4,nullptr,16);

}
void Roller_Action_Coder::ORI_data2HEX(std_msgs::Int64MultiArray& motor_motion, int ID, int Port,
                  int data1, int data2, int data3, int data4,
                  int data5, int data6, int data7, int data8)
{
    /*Set the vector "motoe_motion" data as the HEX data*/
    motor_motion.data.push_back(ID    );
    motor_motion.data.push_back(Port  );
    motor_motion.data.push_back(data1 );
    motor_motion.data.push_back(data2 );
    motor_motion.data.push_back(data3 );
    motor_motion.data.push_back(data4 );
    motor_motion.data.push_back(data5 );
    motor_motion.data.push_back(data6 );
    motor_motion.data.push_back(data7 );
    motor_motion.data.push_back(data8 );

}
void Roller_Action_Coder::Speed_data2HEX(int ID,int PORT,std_msgs::Int64MultiArray &motor_motion,int speed)
{
  /*Set the speed value to HEX-type*/
  int speedunit1,speedunit2,speedunit3,speedunit4;
  if(speed>=0)
    {
      speedunit1=speed/256;
      speedunit2=speed-speedunit1*256;
    }
  else
    {
      dec2HEX(speed,speedunit3,speedunit4,speedunit1,speedunit2);
    }

  /*Set the vector "motor_motion" data as the HEX data*/
  motor_motion.data.push_back(ID        );
  motor_motion.data.push_back(PORT      );
  motor_motion.data.push_back(15        );//"0F"
  motor_motion.data.push_back(0         );//"00"
  motor_motion.data.push_back(speedunit2);
  motor_motion.data.push_back(speedunit1);
  motor_motion.data.push_back(0         );
  motor_motion.data.push_back(0         );
  motor_motion.data.push_back(0         );
  motor_motion.data.push_back(0         );

}
void Roller_Action_Coder::SET_Position_data2HEX(int ID,int PORT,std_msgs::Int64MultiArray &motor_motion,int position)
{
  /*Set the position value to HEX-type*/
  int positionunit1,positionunit2,positionunit3,positionunit4;
  dec2HEX(position,positionunit3,positionunit4,positionunit1,positionunit2);
  ROS_WARN_STREAM("position:"<<position);
  ROS_WARN_STREAM("P1:"<<positionunit1);
  ROS_WARN_STREAM("P2:"<<positionunit2);
  ROS_WARN_STREAM("P3:"<<positionunit3);
  ROS_WARN_STREAM("P4:"<<positionunit4);

  /*Set the vector "motoe_motion" data as the HEX data*/
  motor_motion.data.push_back(ID            );
  motor_motion.data.push_back(PORT          );
  motor_motion.data.push_back(35            );//"23"
  motor_motion.data.push_back(122           );//"7A"
  motor_motion.data.push_back(96            );//"60"
  motor_motion.data.push_back(0             );//"00"
  motor_motion.data.push_back(positionunit2 );
  motor_motion.data.push_back(positionunit1 );
  motor_motion.data.push_back(positionunit4 );
  motor_motion.data.push_back(positionunit3 );

}
double Roller_Action_Coder::Get_the_msec_time_of_computer(struct timeval &time_now)
{
  /*Get the msec_time of computer*/
  double now_time;
  gettimeofday(&time_now,nullptr);
  now_time = time_now.tv_sec*1000+time_now.tv_usec/1000;
  return now_time;
}

void Roller_Action_Coder::Sending_the_Speed_msgs_single(std_msgs::Int64MultiArray &motor_motion,int wheel_ID,int motor_speed)
{
  /*Sending the Speed msgs single*/
  motor_motion.data.clear();
  Speed_data2HEX(wheel_ID,CAN_PORT,motor_motion,motor_speed);
  if(Can_state.id!=998)
  {
    /*If device is active*/
    Roller_action_can_pub.publish(motor_motion);
  }
}
void Roller_Action_Coder::Sending_the_ORI_single(std_msgs::Int64MultiArray &motor_motion,int wheel_ID,
                            int data1,int data2,int data3,int data4,
                            int data5,int data6,int data7,int data8)
{
  /*Sending the ORI msgs single*/

  motor_motion.data.clear();
  ORI_data2HEX(motor_motion,wheel_ID,CAN_PORT,data1,data2,data3,data4,data5,data6,data7,data8);
  if(Can_state.id!=998)
  {
    /*If device is active*/
    Roller_action_can_pub.publish(motor_motion);
  }

}
void Roller_Action_Coder::publishMotionData(std_msgs::Int64MultiArray& motor_motion, int motor_ID, int data1, int data2, int data3, int data4, int data5, int data6, int data7, int data8)
{
  ROS_INFO("Motor ID: %d", motor_ID);
  ROS_INFO("begin_time %lf", start_time);

  motor_motion.data.clear();
  ORI_data2HEX(motor_motion, motor_ID, CAN_PORT, data1, data2, data3, data4, data5, data6, data7, data8);
  Roller_action_can_pub.publish(motor_motion);
  ros::Duration(0.02).sleep();
}



