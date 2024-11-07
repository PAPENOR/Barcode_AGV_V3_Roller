#include "roller_model_motec.h"
#define PI 3.1415926
#define Go_left 1
#define Go_right 2
double Get_the_msec_time_of_computer(struct timeval &time_now)
{
  /*Get the msec_time of computer*/
  double now_time;
  gettimeofday(&time_now,nullptr);
  now_time = time_now.tv_sec*1000+time_now.tv_usec/1000;
  return now_time;
}
void Woring_state_pub(int working_state,ros::Publisher &Mission_state_pub)
{
  //Mission state 1=Done;2=Ready;3=Busy;4=Error
  ROS_INFO("Working State:");
  switch (working_state)
  {
  case 1  :
    ROS_INFO("Done");
    break;
  case 2  :
    ROS_INFO("Ready");
    break;
  case 3  :
    ROS_INFO("Busy");
    break;
  case 4  :
    ROS_ERROR("ERROR!");
    break;
  }
  std_msgs::Int16 Mission_msg;
  Mission_msg.data=working_state;//4=Error
  Mission_state_pub.publish(Mission_msg);
}
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
    X_Arrived_pub     =nh.advertise<std_msgs::Bool>("/X_Arrived",10);
    Z_Arrived_pub     =nh.advertise<std_msgs::Bool>("/Z_Arrived",10);
    Mission_state_pub =nh.advertise<std_msgs::Int16>("/Mission_state",10);
    Left_Stick_pub    =nh.advertise<std_msgs::Bool>("/hand/Left_Stick_action",10);
    Right_Stick_pub   =nh.advertise<std_msgs::Bool>("/hand/Right_Stick_action",10);
    Roller_Axis_pub   =nh.advertise<std_msgs::Int16>("hand/Roller_Axis_action",10);

    action_sub        =nh.subscribe("/XZ_action",10,&Roller_Model_Motec::Z_action_Callback,this);
    odom_sub          =nh.subscribe("/odom_hand",10,&Roller_Model_Motec::Odom_Callback,this);
    Hand_stop_sub     =nh.subscribe("/Hand_Stop_Command",10,&Roller_Model_Motec::Hand_Stop_Callback,this);

    X_Right_Limit_Sub =nh.subscribe("hand/X_Right_Limit",10,&Roller_Model_Motec::X_Right_Limit_Callback,this);
    X_Left_Limit_Sub  =nh.subscribe("hand/X_Left_Limit",10,&Roller_Model_Motec::X_Left_Limit_Callback,this);
    Z_Up_Limit_Sub    =nh.subscribe("hand/Z_Up_Limit",10,&Roller_Model_Motec::Z_Up_Limit_Callback,this);
    Z_Down_Limit_Sub  =nh.subscribe("hand/Z_Down_Limit",10,&Roller_Model_Motec::Z_Down_Limit_Callback,this);
    Z_Zero_Sub        =nh.subscribe("hand/Z_Zero",10,&Roller_Model_Motec::Z_Zero_Callback,this);
\
    stringstream Tcpip_Server_error_name;
    Tcpip_Server_error_name<<ros::this_node::getName()<<"/Error_Code_Rollor_model";
    Error_pub                  =nh.advertise<std_msgs::Int16>(Tcpip_Server_error_name.str(), 10);

    ros::Rate r(100);

    double Z_mm_circle;
    Init_param(nh,"/Hand_Model_Motec/Z_RT_ID"     ,Z_RT_ID);
    Init_param(nh,"/Hand_Model_Motec/Z_SDO_ID"    ,Z_SDO_ID);
    Init_param(nh,"/Hand_Model_Motec/Z_RT_ID_READ",Z_RT_ID_READ);
    Init_param(nh,"/Hand_Model_Motec/Z_MOTOR_REVERSE",Z_Motor_Reverse);
    int Stage=0;
    int Stop_Count=0;
    double SpeedHandZ;
    double PositionHandZ;
    struct timeval time_now{};
    gettimeofday(&time_now,nullptr);
    time_t  begin=time_now.tv_sec*1000+time_now.tv_usec/1000,new_time;
    double  Timer_Record=begin;
    bool Initial=true;

    Can_State_Client.waitForServer();

    while (ros::ok())
    {
      double Now_Time = Get_the_msec_time_of_computer(time_now);
      double Timer1=(Now_Time-Timer_Record)/1000;
      ros::spinOnce();
      if(Stage==-1)
      {
        if(Task_Order.Hand_Mode==4)
        {
        ROS_WARN("Starting New mission:");
        ROS_WARN("Left Goods input:");
        Stage=0;
        }
        if(Task_Order.Hand_Mode==5)
        {
        ROS_WARN("Starting New mission:");
        ROS_WARN("Right Goods input:");
        Stage=0;
        }
        if(Task_Order.Hand_Mode==6)
        {
        ROS_WARN("Starting New mission:");
        ROS_WARN("Left Goods output:");
        Stage=0;
        }
        if(Task_Order.Hand_Mode==7)
        {
        ROS_WARN("Starting New mission:");
        ROS_WARN("Right Goods output:");
        Stage=0;
        }
      }
      if(Task_Order.Hand_Mode!=0)
      {
        Stop_Count=0;
      }
      if(Task_Order.Hand_Mode==0)//speed mode
      {
        SpeedHandZ=double(Task_Order.Hand_Speed_Z)/1000;
        double SpeedMotorX,SpeedMotorZ;
        Z_mm_circle=0.08276*PI;
        SpeedMotorZ=Z_Motor_Reverse*SpeedHandZ*64*60;
        SpeedMotorZ=SpeedMotorZ/Z_mm_circle;
        limit_error=false;
        limit_error=false;
        None_Good=false;
        Time_out_error=false;
        if(Z_Up_Limit_Msg.data==true && Task_Order.Hand_Speed_Z >0)
        {
            ROS_ERROR_STREAM("Up limit Error");
            Motor_Moving_Mode(Z_RT_ID,1,5,0,false,Can_State_Client);
            Woring_state_pub(4,Mission_state_pub);
            limit_error=true;
            Stage=999;
        }
        else if(Z_Down_Limit_Msg.data==true && Task_Order.Hand_Speed_Z <0)
        {
            Motor_Moving_Mode(Z_RT_ID,1,5,0,false,Can_State_Client);
            ROS_ERROR_STREAM("Down limit Error");
            Woring_state_pub(4,Mission_state_pub);
            limit_error=true;
            Stage=999;
        }
        else if(Stop_Count<4 && Initial==false )
        {

          ROS_INFO_STREAM("SpeedMotorZ:"<<SpeedMotorZ);
          Motor_Moving_Mode(Z_RT_ID,1,5,SpeedMotorZ,false,Can_State_Client);

          Woring_state_pub(3,Mission_state_pub);
        }
        if(Stop_Count>=4)
        {
          Stop_Count=4;
        }
        if(SpeedHandZ ==0)
        {
          Stage=0;//stop
          Woring_state_pub(1,Mission_state_pub);
          Stop_Count++;
        }
        else
        {
          std_msgs::Bool Arrived;
          Arrived.data=false;
          X_Arrived_pub.publish(Arrived);
          Z_Arrived_pub.publish(Arrived);
          Stage=-1;//speed mode
          Stop_Count=0;
        }
        Initial=false;
        ROS_WARN_STREAM("Action Done");
      }
      if(Task_Order.Hand_Mode==1 && Stage==0)//positon mode
      {
        std_msgs::Bool Arrived;
        Arrived.data=false;
        Z_Arrived_pub.publish(Arrived);
        ROS_WARN("Position Canbus mode:");
        PositionHandZ=double(Task_Order.Hand_Position_Z)/1000;
        ROS_WARN_STREAM("This Hand will do this");
        ROS_WARN_STREAM("PositionHandZ:"<<PositionHandZ);
        Stage=1;
        Woring_state_pub(3,Mission_state_pub);
        ros::Duration(0.1).sleep();
      }
      if(Task_Order.Hand_Mode==3 && Stage==0)//init the Z AXIS
      {
        Woring_state_pub(3,Mission_state_pub);
        ROS_WARN_STREAM("Init the Z AXIS");
        ROS_WARN_STREAM("This Hand will do this");
        if(Z_Up_Limit_Msg.data==false)
        {
          ROS_WARN_STREAM("Z AXIS will moving up");
          Stage=2;
        }
        else
        {
          ROS_WARN_STREAM("Z AXIS will moving slow down");
          Stage=2;
        }

      }
      if(Task_Order.Hand_Mode==4 && Stage==0)//init the Z AXIS
      {
        Hand_stop_Msg.data=false;
        Stick_motion(true,Right_Stick_pub);
        Stick_motion(false,Left_Stick_pub);
        Woring_state_pub(3,Mission_state_pub);
        ROS_WARN_STREAM("Left In");
        ROS_WARN_STREAM("This Hand will do this");
        ros::Duration(4.0).sleep();
        if(X_Left_Limit_Msg.data==false)
        {
          ROS_WARN_STREAM("X AXIS will rolling right to XLL");
          Stage=1;
        }
        else
        {
          ROS_WARN_STREAM("X AXIS will rolling right to XRL");
          Stage=2;
        }
      }
      if(Task_Order.Hand_Mode==5 && Stage==0)
      {
        Hand_stop_Msg.data=false;
        Stick_motion(true,Left_Stick_pub);
        Stick_motion(false,Right_Stick_pub);
        Woring_state_pub(3,Mission_state_pub);
        ros::Duration(4.0).sleep();
        ROS_WARN_STREAM("Right In");
        ROS_WARN_STREAM("This Hand will do this");
        if(X_Right_Limit_Msg.data==false)
        {
          ROS_WARN_STREAM("X AXIS will rolling left to XRL");
          Stage=1;
        }
        else
        {
          ROS_WARN_STREAM("X AXIS will rolling left to XLL");
          Stage=2;
        }


      }
      if(Task_Order.Hand_Mode==6 && Stage==0)
      {
        Hand_stop_Msg.data=false;
        Stick_motion(false,Left_Stick_pub);
        Stick_motion(true,Right_Stick_pub);
        ros::Duration(4.0).sleep();
        ROS_WARN_STREAM("Left Out");
        ROS_WARN_STREAM("This Hand will do this");
        if(X_Left_Limit_Msg.data==false && X_Right_Limit_Msg.data==false)
        {
        ROS_ERROR_STREAM("No Good on the table");
        None_Good=true;
        Woring_state_pub(4,Mission_state_pub);
        }
        if(X_Left_Limit_Msg.data==false && X_Right_Limit_Msg.data==false)
        {
        ROS_ERROR_STREAM("No Good on the table");
        None_Good=true;
        Woring_state_pub(4,Mission_state_pub);
        }
        else
        {
          None_Good=false;
          ROS_WARN_STREAM("X AXIS will rolling left to XRL Off");
          Woring_state_pub(3,Mission_state_pub);
          Stage=1;
        }
        Timer_Record=Get_the_msec_time_of_computer(time_now);
        Timer1=(Now_Time-Timer_Record)/1000;
      }
      if(Task_Order.Hand_Mode==7 && Stage==0)
      {
        Hand_stop_Msg.data=false;
        Stick_motion(true,Left_Stick_pub);
        Stick_motion(false,Right_Stick_pub);
        ros::Duration(4.0).sleep();
        ROS_WARN_STREAM("Right Out");
        ROS_WARN_STREAM("This Hand will do this");
        if(X_Left_Limit_Msg.data==false && X_Right_Limit_Msg.data==false)
        {
        None_Good=true;
        ROS_ERROR_STREAM("No Good on the table");
        Woring_state_pub(4,Mission_state_pub);
        }
        else
        {
          ROS_WARN_STREAM("X AXIS will rolling left to XLL Off");
          Woring_state_pub(3,Mission_state_pub);
          Stage=1;
        }
        Timer_Record=Get_the_msec_time_of_computer(time_now);
        Timer1=(Now_Time-Timer_Record)/1000;
      }
      if(Task_Order.Hand_Mode==8 && Stage==0)//positon mode +4 mode
      {
          std_msgs::Bool Arrived;
          Arrived.data=false;
          Z_Arrived_pub.publish(Arrived);
          ROS_WARN("Position Canbus mode:");
          PositionHandZ=double(Task_Order.Hand_Position_Z)/1000;
          ROS_WARN_STREAM("This Hand will do this");
          ROS_WARN_STREAM("PositionHandZ:"<<PositionHandZ);
          Stage=1;
          Woring_state_pub(3,Mission_state_pub);
      }
      if(Task_Order.Hand_Mode==9 && Stage==0)//positon mode +5 mode
      {
          std_msgs::Bool Arrived;
          Arrived.data=false;
          Z_Arrived_pub.publish(Arrived);
          ROS_WARN("Position Canbus mode:");
          PositionHandZ=double(Task_Order.Hand_Position_Z)/1000;
          ROS_WARN_STREAM("This Hand will do this");
          ROS_WARN_STREAM("PositionHandZ:"<<PositionHandZ);
          Stage=1;
          Woring_state_pub(3,Mission_state_pub);
      }
      if(Task_Order.Hand_Mode==10 && Stage==0)//positon mode +6 mode
      {
          std_msgs::Bool Arrived;
          Arrived.data=false;
          Z_Arrived_pub.publish(Arrived);
          ROS_WARN("Position Canbus mode:");
          PositionHandZ=double(Task_Order.Hand_Position_Z)/1000;
          ROS_WARN_STREAM("This Hand will do this");
          ROS_WARN_STREAM("PositionHandZ:"<<PositionHandZ);
          Stage=1;
          Woring_state_pub(3,Mission_state_pub);
      }
      if(Task_Order.Hand_Mode==11 && Stage==0)//positon mode +7 mode
      {
          std_msgs::Bool Arrived;
          Arrived.data=false;
          Z_Arrived_pub.publish(Arrived);
          ROS_WARN("Position Canbus mode:");
          PositionHandZ=double(Task_Order.Hand_Position_Z)/1000;
          ROS_WARN_STREAM("This Hand will do this");
          ROS_WARN_STREAM("PositionHandZ:"<<PositionHandZ);
          Stage=1;
          Woring_state_pub(3,Mission_state_pub);
      }
      if(Task_Order.Hand_Mode==1 && Stage==1)
      {
        Woring_state_pub(3,Mission_state_pub);
        int PositionMotorX,PositionMotorZ;
        //PositionMotorZ(m)
        PositionMotorZ=Z_Motor_Reverse*PositionHandZ*100;
        PositionMotorZ=-1958.4*pow(PositionMotorZ,2)+742989*PositionMotorZ+205488;
        ROS_WARN("Position Canbus mode:");
        ROS_WARN("stage:%d",Stage);
        ROS_WARN_STREAM("Step 1:"<<endl       <<"Z:"<<PositionMotorZ<<endl);

        Motor_Moving_Mode(Z_SDO_ID,3,    2,   0,false,Can_State_Client);
        Motor_Moving_Mode(Z_SDO_ID,2,PositionMotorZ,0,false,Can_State_Client);
        Motor_Moving_Mode(Z_SDO_ID,3,    1,   0,false,Can_State_Client);
        Motor_Moving_Mode(Z_RT_ID,1,200,0,false,Can_State_Client);
        std_msgs::Bool Arrived;
        Arrived.data=true;
        Z_Arrived_pub.publish(Arrived);
        Stage=-1;

      }
      if(Task_Order.Hand_Mode==3 && Stage==1)//init the Z AXIS
      {
        Woring_state_pub(3,Mission_state_pub);
        ROS_WARN_STREAM("Z AXIS is moving Up");
        double SpeedMotorX,SpeedMotorZ;
        SpeedHandZ=0.2;
        SpeedMotorZ=Z_Motor_Reverse*SpeedHandZ*10000/8;
        Motor_Moving_Mode(Z_RT_ID,1,5,SpeedMotorZ,false,Can_State_Client);
        if(Z_Zero_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Stage=2;
        }
        if(Hand_stop_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Hand_stop_Msg.data=false;
          Stage=-1;
        }
      }
      if(Task_Order.Hand_Mode==4 && Stage==1)
      {
        ROS_WARN_STREAM("X AXIS is moving to XLL");
        double SpeedMotorX,SpeedMotorZ;

        if(X_Left_Limit_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Stage=2;
        }
        else if(Z_Up_Limit_Msg.data==true | Z_Down_Limit_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(4,Mission_state_pub);
        }
        else if(Hand_stop_Msg.data==true )
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(1,Mission_state_pub);
        Hand_stop_Msg.data=false;
        Task_Order.Hand_Mode=0;
        Stage=-1;
        }
        else
        {
        sendRoller_AxiscommandMsg(Go_left);
        Woring_state_pub(2,Mission_state_pub);
        }
      }
      if(Task_Order.Hand_Mode==5 && Stage==1)
      {
        ROS_WARN_STREAM("X AXIS is moving left XRL");
        double SpeedMotorX,SpeedMotorZ;
        if(X_Right_Limit_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Stage=2;
        }
        else if(Z_Up_Limit_Msg.data==true | Z_Down_Limit_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(4,Mission_state_pub);
        }
        else if(Hand_stop_Msg.data==true )
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(1,Mission_state_pub);
        Hand_stop_Msg.data=false;
        Task_Order.Hand_Mode=0;
        Stage=-1;
        }
        else
        {
        sendRoller_AxiscommandMsg(Go_right);
        Woring_state_pub(2,Mission_state_pub);
        }
      }
      if(Task_Order.Hand_Mode==6 && Stage==1)
      {
        ROS_WARN_STREAM("X AXIS is moving left XRL out");
        ROS_INFO_STREAM("Time:"<<Timer1);
        double SpeedMotorX,SpeedMotorZ;
        if(Hand_stop_Msg.data==true)
        {
        Stick_motion(true,Left_Stick_pub);
        Stick_motion(true,Right_Stick_pub);
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Task_Order.Hand_Mode=0;
        Hand_stop_Msg.data=false;
        Stage=-1;
        ros::Duration(2.0).sleep();
        Woring_state_pub(1,Mission_state_pub);
        }
        else if(Z_Up_Limit_Msg.data==true | Z_Down_Limit_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(4,Mission_state_pub);
        Stage=-1;
        }
        else if(Timer1>50)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(4,Mission_state_pub);
        Stage=-1;
        ROS_ERROR_STREAM("Moving left XRL out Time Out !!");
        Timer_Record=Get_the_msec_time_of_computer(time_now);
        Time_out_error=true;
        }
        else
        {
        sendRoller_AxiscommandMsg(Go_right);
        Woring_state_pub(2,Mission_state_pub);
        }
      }
      if(Task_Order.Hand_Mode==7 && Stage==1)
      {
        ROS_WARN_STREAM("X AXIS is moving left XLL out");
        ROS_INFO_STREAM("Time:"<<Timer1);
        double SpeedMotorX,SpeedMotorZ;
        if(Hand_stop_Msg.data==true)
        {
        Stick_motion(true,Left_Stick_pub);
        Stick_motion(true,Right_Stick_pub);
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Task_Order.Hand_Mode=0;
        Hand_stop_Msg.data=false;
        Stage=-1;
        ros::Duration(2.0).sleep();
        Woring_state_pub(1,Mission_state_pub);
        }
        else if(Z_Up_Limit_Msg.data==true | Z_Down_Limit_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(4,Mission_state_pub);
        Stage=-1;
        }
        else if(Timer1>50)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(4,Mission_state_pub);
        Stage=-1;
        ROS_ERROR_STREAM("Moving left XLL out Time Out !!");
        Time_out_error=true;
        Timer_Record=Get_the_msec_time_of_computer(time_now);
        }
        else
        {
        sendRoller_AxiscommandMsg(Go_left);
        Woring_state_pub(2,Mission_state_pub);
        }
      }
      if(Task_Order.Hand_Mode==8 && Stage==1)//positon mode +4 mode
      {
          Woring_state_pub(3,Mission_state_pub);
          int PositionMotorX,PositionMotorZ;
          //PositionMotorZ(m)
          PositionMotorZ=Z_Motor_Reverse*PositionHandZ*100;
          PositionMotorZ=-1958.4*pow(PositionMotorZ,2)+742989*PositionMotorZ+205488;
          ROS_WARN("Position Canbus mode:");
          ROS_WARN("stage:%d",Stage);
          ROS_WARN_STREAM("Step 1:"<<endl       <<"Z:"<<PositionMotorZ<<endl);

          Hand_stop_Msg.data=false;
          Stick_motion(true,Right_Stick_pub);
          Stick_motion(false,Left_Stick_pub);
          Woring_state_pub(3,Mission_state_pub);
          ROS_WARN_STREAM("Left In");
          ROS_WARN_STREAM("This Hand will do this");

          Motor_Moving_Mode(Z_SDO_ID,3,    2,   0,false,Can_State_Client);
          Motor_Moving_Mode(Z_SDO_ID,2,PositionMotorZ,0,false,Can_State_Client);
          Motor_Moving_Mode(Z_SDO_ID,3,    1,   0,false,Can_State_Client);
          Motor_Moving_Mode(Z_RT_ID,1,200,0,false,Can_State_Client);

          if(X_Left_Limit_Msg.data==false)
          {
            ROS_WARN_STREAM("X AXIS will rolling right to XLL");
            Stage=2;
          }
          else
          {
            ROS_WARN_STREAM("X AXIS will rolling right to XRL");
            Stage=3;
          }
          std_msgs::Bool Arrived;
          Arrived.data=true;
          Z_Arrived_pub.publish(Arrived);
      }
      if(Task_Order.Hand_Mode==9 && Stage==1)
      {
          Woring_state_pub(3,Mission_state_pub);
          int PositionMotorX,PositionMotorZ;
          //PositionMotorZ(m)
          PositionMotorZ=Z_Motor_Reverse*PositionHandZ*100;
          PositionMotorZ=-1958.4*pow(PositionMotorZ,2)+742989*PositionMotorZ+205488;
          ROS_WARN("Position Canbus mode:");
          ROS_WARN("stage:%d",Stage);
          ROS_WARN_STREAM("Step 1:"<<endl       <<"Z:"<<PositionMotorZ<<endl);
        Hand_stop_Msg.data=false;
        Stick_motion(true,Left_Stick_pub);
        Stick_motion(false,Right_Stick_pub);
        Woring_state_pub(3,Mission_state_pub);

        ROS_WARN_STREAM("Right In");
        ROS_WARN_STREAM("This Hand will do this");
        Motor_Moving_Mode(Z_SDO_ID,3,    2,   0,false,Can_State_Client);
        Motor_Moving_Mode(Z_SDO_ID,2,PositionMotorZ,0,false,Can_State_Client);
        Motor_Moving_Mode(Z_SDO_ID,3,    1,   0,false,Can_State_Client);
        Motor_Moving_Mode(Z_RT_ID,1,200,0,false,Can_State_Client);

        if(X_Right_Limit_Msg.data==false)
        {
          ROS_WARN_STREAM("X AXIS will rolling left to XRL");
          Stage=2;
        }
        else
        {
          ROS_WARN_STREAM("X AXIS will rolling left to XLL");
          Stage=3;
        }


      }
      if(Task_Order.Hand_Mode==10 && Stage==1)
      {
          Woring_state_pub(3,Mission_state_pub);
          int PositionMotorX,PositionMotorZ;
          //PositionMotorZ(m)
          PositionMotorZ=Z_Motor_Reverse*PositionHandZ*100;
          PositionMotorZ=-1958.4*pow(PositionMotorZ,2)+742989*PositionMotorZ+205488;
          ROS_WARN("Position Canbus mode:");
          ROS_WARN("stage:%d",Stage);
          ROS_WARN_STREAM("Step 1:"<<endl       <<"Z:"<<PositionMotorZ<<endl);
        Hand_stop_Msg.data=false;
        Stick_motion(false,Left_Stick_pub);
        Stick_motion(true,Right_Stick_pub);
        ROS_WARN_STREAM("Left Out");
        ROS_WARN_STREAM("This Hand will do this");
        Motor_Moving_Mode(Z_SDO_ID,3,    2,   0,false,Can_State_Client);
        Motor_Moving_Mode(Z_SDO_ID,2,PositionMotorZ,0,false,Can_State_Client);
        Motor_Moving_Mode(Z_SDO_ID,3,    1,   0,false,Can_State_Client);
        Motor_Moving_Mode(Z_RT_ID,1,200,0,false,Can_State_Client);
        if(X_Left_Limit_Msg.data==false && X_Right_Limit_Msg.data==false)
        {
        ROS_ERROR_STREAM("No Good on the table");
        None_Good=true;
        Woring_state_pub(4,Mission_state_pub);
        }
        if(X_Left_Limit_Msg.data==false && X_Right_Limit_Msg.data==false)
        {
        ROS_ERROR_STREAM("No Good on the table");
        None_Good=true;
        Woring_state_pub(4,Mission_state_pub);
        }
        else
        {
          None_Good=false;
          ROS_WARN_STREAM("X AXIS will rolling left to XRL Off");
          Woring_state_pub(3,Mission_state_pub);
          Stage=2;
        }
        Timer_Record=Get_the_msec_time_of_computer(time_now);
        Timer1=(Now_Time-Timer_Record)/1000;
      }
      if(Task_Order.Hand_Mode==11 && Stage==1)
      {
        Woring_state_pub(3,Mission_state_pub);
        int PositionMotorX,PositionMotorZ;
        //PositionMotorZ(m)
        PositionMotorZ=Z_Motor_Reverse*PositionHandZ*100;
        PositionMotorZ=-1958.4*pow(PositionMotorZ,2)+742989*PositionMotorZ+205488;
        ROS_WARN("Position Canbus mode:");
        ROS_WARN("stage:%d",Stage);
        ROS_WARN_STREAM("Step 1:"<<endl       <<"Z:"<<PositionMotorZ<<endl);
        Hand_stop_Msg.data=false;
        Stick_motion(true,Left_Stick_pub);
        Stick_motion(false,Right_Stick_pub);
        ROS_WARN_STREAM("Right Out");
        ROS_WARN_STREAM("This Hand will do this");
        Motor_Moving_Mode(Z_SDO_ID,3,    2,   0,false,Can_State_Client);
        Motor_Moving_Mode(Z_SDO_ID,2,PositionMotorZ,0,false,Can_State_Client);
        Motor_Moving_Mode(Z_SDO_ID,3,    1,   0,false,Can_State_Client);
        Motor_Moving_Mode(Z_RT_ID,1,200,0,false,Can_State_Client);
        if(X_Left_Limit_Msg.data==false && X_Right_Limit_Msg.data==false)
        {
        None_Good=true;
        ROS_ERROR_STREAM("No Good on the table");
        Woring_state_pub(4,Mission_state_pub);
        }
        else
        {
          ROS_WARN_STREAM("X AXIS will rolling left to XLL Off");
          Woring_state_pub(3,Mission_state_pub);
          Stage=2;
        }
        Timer_Record=Get_the_msec_time_of_computer(time_now);
        Timer1=(Now_Time-Timer_Record)/1000;
        std_msgs::Bool Arrived;
        Arrived.data=true;
        Z_Arrived_pub.publish(Arrived);
        Stage=-1;
      }
      if(Task_Order.Hand_Mode==3 && Stage==2)//init the Z AXIS
      {
        Woring_state_pub(3,Mission_state_pub);
        ROS_WARN_STREAM("Z AXIS is moving down");
        double SpeedMotorX,SpeedMotorZ;
        SpeedHandZ=-0.8;
        SpeedMotorZ=Z_Motor_Reverse*SpeedHandZ*10000/8;
        Motor_Moving_Mode(Z_RT_ID,1,5,SpeedMotorZ,false,Can_State_Client);

        if(Z_Zero_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Stage=3;
        }
        if(Hand_stop_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Hand_stop_Msg.data=false;
          Stage=-1;
        }
        if(Z_Down_Limit_Msg.data==true)
        {
          StopAllMotion(Z_RT_ID,Can_State_Client);
          Stage=1;
        }

      }
      if(Task_Order.Hand_Mode==4 && Stage==2)
      {

        ROS_WARN_STREAM("X AXIS is moving right XRL");
        double SpeedMotorX,SpeedMotorZ;
        if(X_Right_Limit_Msg.data==true)
        {
        ros::Duration(1.0).sleep();
        Stick_motion(true,Left_Stick_pub);
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Task_Order.Hand_Mode=0;
        Stage=-1;
        ros::Duration(2.0).sleep();
        Woring_state_pub(1,Mission_state_pub);
        Timer_Record=Get_the_msec_time_of_computer(time_now);
        Timer1=(Now_Time-Timer_Record)/1000;
        }
        else if(Z_Up_Limit_Msg.data==true | Z_Down_Limit_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(4,Mission_state_pub);
        }
        else if(Hand_stop_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(1,Mission_state_pub);
        Timer_Record=Get_the_msec_time_of_computer(time_now);
        Timer1=(Now_Time-Timer_Record)/1000;
        Task_Order.Hand_Mode=0;
        Hand_stop_Msg.data=false;
        Stage=-1;
        }
        else
        {
        sendRoller_AxiscommandMsg(Go_left);
        Woring_state_pub(3,Mission_state_pub);
        }
      }
      if(Task_Order.Hand_Mode==5 && Stage==2)
      {
        ROS_WARN_STREAM("X AXIS is moving left XLL");
        double SpeedMotorX,SpeedMotorZ;
        if(X_Left_Limit_Msg.data==true)
        {
        ros::Duration(1.0).sleep();
        Stick_motion(true,Right_Stick_pub);
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Task_Order.Hand_Mode=0;
        Stage=-1;
        ros::Duration(2.0).sleep();
        Woring_state_pub(1,Mission_state_pub);
        }
        else if(Z_Up_Limit_Msg.data==true | Z_Down_Limit_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(4,Mission_state_pub);
        }
        else if(Hand_stop_Msg.data==true )
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(1,Mission_state_pub);
        Hand_stop_Msg.data=false;
        Task_Order.Hand_Mode=0;
        Stage=-1;
        }
        else
        {
        sendRoller_AxiscommandMsg(Go_right);
        Woring_state_pub(3,Mission_state_pub);
        }
      }
      if(Task_Order.Hand_Mode==8 && Stage==2)
      {
        ROS_WARN_STREAM("X AXIS is moving to XLL");
        double SpeedMotorX,SpeedMotorZ;

        if(X_Left_Limit_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Stage=3;
        }
        else if(Z_Up_Limit_Msg.data==true | Z_Down_Limit_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(4,Mission_state_pub);
        }
        else if(Hand_stop_Msg.data==true )
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(1,Mission_state_pub);
        Hand_stop_Msg.data=false;
        Task_Order.Hand_Mode=0;
        Stage=-1;
        }
        else
        {
        sendRoller_AxiscommandMsg(Go_left);
        Woring_state_pub(2,Mission_state_pub);
        }
      }
      if(Task_Order.Hand_Mode==9 && Stage==2)
      {
        ROS_WARN_STREAM("X AXIS is moving left XRL");
        double SpeedMotorX,SpeedMotorZ;
        if(X_Right_Limit_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Stage=3;
        }
        else if(Z_Up_Limit_Msg.data==true | Z_Down_Limit_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(4,Mission_state_pub);
        }
        else if(Hand_stop_Msg.data==true )
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(1,Mission_state_pub);
        Hand_stop_Msg.data=false;
        Task_Order.Hand_Mode=0;
        Stage=-1;
        }
        else
        {
        sendRoller_AxiscommandMsg(Go_right);
        Woring_state_pub(2,Mission_state_pub);
        }
      }
      if(Task_Order.Hand_Mode==10 && Stage==2)
      {
        ROS_WARN_STREAM("X AXIS is moving left XRL out");
        ROS_INFO_STREAM("Time:"<<Timer1);
        double SpeedMotorX,SpeedMotorZ;
        if(Hand_stop_Msg.data==true)
        {
        Stick_motion(true,Left_Stick_pub);
        Stick_motion(true,Right_Stick_pub);
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Task_Order.Hand_Mode=0;
        Hand_stop_Msg.data=false;
        Stage=-1;
        ros::Duration(2.0).sleep();
        Woring_state_pub(1,Mission_state_pub);
        }
        else if(Z_Up_Limit_Msg.data==true | Z_Down_Limit_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(4,Mission_state_pub);
        Stage=-1;
        }
        else if(Timer1>50)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(4,Mission_state_pub);
        Stage=-1;
        ROS_ERROR_STREAM("Moving left XRL out Time Out !!");
        Timer_Record=Get_the_msec_time_of_computer(time_now);
        Time_out_error=true;
        }
        else
        {
        sendRoller_AxiscommandMsg(Go_right);
        Woring_state_pub(2,Mission_state_pub);
        }
      }
      if(Task_Order.Hand_Mode==11 && Stage==2)
      {
        ROS_WARN_STREAM("X AXIS is moving left XLL out");
        ROS_INFO_STREAM("Time:"<<Timer1);
        double SpeedMotorX,SpeedMotorZ;
        if(Hand_stop_Msg.data==true)
        {
        Stick_motion(true,Left_Stick_pub);
        Stick_motion(true,Right_Stick_pub);
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Task_Order.Hand_Mode=0;
        Hand_stop_Msg.data=false;
        Stage=-1;
        ros::Duration(2.0).sleep();
        Woring_state_pub(1,Mission_state_pub);
        }
        else if(Z_Up_Limit_Msg.data==true | Z_Down_Limit_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(4,Mission_state_pub);
        Stage=-1;
        }
        else if(Timer1>50)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(4,Mission_state_pub);
        Stage=-1;
        ROS_ERROR_STREAM("Moving left XLL out Time Out !!");
        Time_out_error=true;
        Timer_Record=Get_the_msec_time_of_computer(time_now);
        }
        else
        {
        sendRoller_AxiscommandMsg(Go_left);
        Woring_state_pub(2,Mission_state_pub);
        }
      }
      if(Task_Order.Hand_Mode==3 && Stage==3)//init the Z AXIS
      {
        ROS_WARN_STREAM("Z AXIS is moving up");
        double SpeedMotorX,SpeedMotorZ;
        SpeedHandZ=0.02;
        SpeedMotorZ=Z_Motor_Reverse*SpeedHandZ*10000/8;
        Motor_Moving_Mode(Z_RT_ID,1,5,SpeedMotorZ,false,Can_State_Client);
        if(Z_Zero_Msg.data==false)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Stage=4;
        }
        Woring_state_pub(3,Mission_state_pub);
      }
      if(Task_Order.Hand_Mode==8 && Stage==3)
      {

        ROS_WARN_STREAM("X AXIS is moving right XRL");
        double SpeedMotorX,SpeedMotorZ;
        if(X_Right_Limit_Msg.data==true)
        {
        ros::Duration(1.0).sleep();
        Stick_motion(true,Left_Stick_pub);
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Task_Order.Hand_Mode=0;
        Stage=-1;
        ros::Duration(2.0).sleep();
        Woring_state_pub(1,Mission_state_pub);
        Timer_Record=Get_the_msec_time_of_computer(time_now);
        Timer1=(Now_Time-Timer_Record)/1000;
        }
        else if(Z_Up_Limit_Msg.data==true | Z_Down_Limit_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(4,Mission_state_pub);
        }
        else if(Hand_stop_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(1,Mission_state_pub);
        Timer_Record=Get_the_msec_time_of_computer(time_now);
        Timer1=(Now_Time-Timer_Record)/1000;
        Task_Order.Hand_Mode=0;
        Hand_stop_Msg.data=false;
        Stage=-1;
        }
        else
        {
        sendRoller_AxiscommandMsg(Go_left);
        Woring_state_pub(3,Mission_state_pub);
        }
      }
      if(Task_Order.Hand_Mode==9 && Stage==3)
      {

        ROS_WARN_STREAM("X AXIS is moving right XRL");
        double SpeedMotorX,SpeedMotorZ;
        if(X_Right_Limit_Msg.data==true)
        {
        ros::Duration(1.0).sleep();
        Stick_motion(true,Left_Stick_pub);
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Task_Order.Hand_Mode=0;
        Stage=-1;
        ros::Duration(2.0).sleep();
        Woring_state_pub(1,Mission_state_pub);
        Timer_Record=Get_the_msec_time_of_computer(time_now);
        Timer1=(Now_Time-Timer_Record)/1000;
        }
        else if(Z_Up_Limit_Msg.data==true | Z_Down_Limit_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(4,Mission_state_pub);
        }
        else if(Hand_stop_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Woring_state_pub(1,Mission_state_pub);
        Timer_Record=Get_the_msec_time_of_computer(time_now);
        Timer1=(Now_Time-Timer_Record)/1000;
        Task_Order.Hand_Mode=0;
        Hand_stop_Msg.data=false;
        Stage=-1;
        }
        else
        {
        sendRoller_AxiscommandMsg(Go_left);
        Woring_state_pub(3,Mission_state_pub);
        }
      }
      if(Task_Order.Hand_Mode==3 && Stage==4)//init the Z AXIS
      {
        ROS_WARN_STREAM("Z AXIS is reset the zero");
        ros::Duration(0.5).sleep();
        Motor_Moving_Mode(Z_SDO_ID,8,    0,   0,false,Can_State_Client);
        ros::Duration(0.5).sleep();
        Motor_Moving_Mode(Z_SDO_ID,6,    0,   0,false,Can_State_Client);
        ros::Duration(0.5).sleep();
        Motor_Moving_Mode(Z_SDO_ID,7,    0,   0,false,Can_State_Client);
        ros::Duration(0.5).sleep();
        Motor_Moving_Mode(Z_SDO_ID,3,    1,   0,false,Can_State_Client);
        ros::Duration(0.5).sleep();
        Woring_state_pub(1,Mission_state_pub);
        Stage=-1;
      }
      if(Task_Order.Hand_Mode==3 && Stage==5)//init the Z AXIS
      {
        ROS_WARN_STREAM("Z AXIS is moving up");
        double SpeedMotorX,SpeedMotorZ;
        SpeedHandZ=0.005;
        SpeedMotorZ=Z_Motor_Reverse*SpeedHandZ*10000/8;
        Motor_Moving_Mode(Z_RT_ID,1,5,0,false,Can_State_Client);
        if(Z_Zero_Msg.data==true)
        {
        StopAllMotion(Z_RT_ID,Can_State_Client);
        Stage=2;
        }
        Woring_state_pub(3,Mission_state_pub);
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
