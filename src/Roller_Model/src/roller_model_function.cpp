#include "roller_model_function.h"
#include "roller_model_motec.h"

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
double Get_the_msec_time_of_computer(struct timeval &time_now)
{
    // 獲取當前時間
    gettimeofday(&time_now, nullptr);
    // 計算當前時間（以毫秒為單位）
    double now_time = time_now.tv_sec * 1000 + time_now.tv_usec / 1000;
    return now_time;
}
int Roller_Model_Motec::calculate_PositionMotorZ()
{
    int PositionMotorZ;
    PositionMotorZ = Z_Motor_Reverse * PositionHandZ * 100;
    PositionMotorZ = -1958.4 * pow(PositionMotorZ, 2) + 742989 * PositionMotorZ + 205488;
    return PositionMotorZ;
}
void Roller_Model_Motec::handle_limit_up_down_error(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
{
    Motor_Moving_Mode(Z_RT_ID, 1, 5, 0, false, Can_State_Client);
    Woring_state_pub(4, Mission_state_pub);
    limit_error = true;
    Stage = 999;
}
void Roller_Model_Motec::handle_speed_control(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
{
    // 计算速度
    SpeedHandZ = double(Task_Order.Hand_Speed_Z) / 1000;
    double SpeedMotorZ = Z_Motor_Reverse * SpeedHandZ * 64 * 60 / Z_mm_circle;

    // 初始化错误状态
    limit_error = false;
    None_Good = false;
    Time_out_error = false;

    // 处理上下限位错误
    if (Z_Up_Limit_Msg.data && Task_Order.Hand_Speed_Z > 0) {
        ROS_ERROR_STREAM("Up limit Error");
        handle_limit_up_down_error(Can_State_Client);
    } else if (Z_Down_Limit_Msg.data && Task_Order.Hand_Speed_Z < 0) {
        ROS_ERROR_STREAM("Down limit Error");
        handle_limit_up_down_error(Can_State_Client);
    } else if (Stop_Count < 4 && !Initial) {
        ROS_INFO_STREAM("SpeedMotorZ:" << SpeedMotorZ);
        Motor_Moving_Mode(Z_RT_ID, 1, 5, SpeedMotorZ, false, Can_State_Client);
        Woring_state_pub(3, Mission_state_pub);
    }

    // 处理停止计数和到达状态
    if (Stop_Count >= 4) {
        Stop_Count = 4;
    }
    if (SpeedHandZ == 0) {
        Stage = 0; // 停止状态
        Woring_state_pub(1, Mission_state_pub);
        Stop_Count++;
    } else {
        std_msgs::Bool Arrived;
        Arrived.data = false;
        Z_Arrived_pub.publish(Arrived);
        Stage = -1; // 速度模式
        Stop_Count = 0;
    }

    // 更新初始化状态
    Initial = false;
    ROS_WARN_STREAM("Action Done");
}
void Roller_Model_Motec::Positon_mode_Setting_signal()
{
    // 發布到達訊號
    publish_arrived_signal(Z_Arrived_pub);
    // 顯示訊息
    ROS_WARN("Position Canbus mode:");
    // 計算目標位置
    PositionHandZ = double(Task_Order.Hand_Position_Z) / 1000;
    // 顯示訊息
    ROS_WARN_STREAM("This Hand will do this");
    ROS_WARN_STREAM("PositionHandZ:" << PositionHandZ<<"m");
    // 設定階段為1
    Stage = 1;
    // 發布工作狀態
    Woring_state_pub(3, Mission_state_pub);
    // 延遲0.05秒
    ros::Duration(0.05).sleep();
}

void Roller_Model_Motec::Positon_mode_Launching(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
{
    // 發布工作狀態
    Woring_state_pub(3, Mission_state_pub);
    // 計算馬達位置
    int PositionMotorZ=calculate_PositionMotorZ();
    // 顯示訊息
    ROS_WARN("Position Canbus mode:");
    ROS_WARN("stage:%d", Stage);
    ROS_WARN_STREAM("Step 1:" << endl << "Z:" << PositionMotorZ <<" Pulse"<< endl);
    // 控制馬達移動
    Motor_Moving_Mode(Z_SDO_ID, 3, 2, 0, false, Can_State_Client);
    Motor_Moving_Mode(Z_SDO_ID, 2, PositionMotorZ, 0, false, Can_State_Client);
    Motor_Moving_Mode(Z_SDO_ID, 3, 1, 0, false, Can_State_Client);
    Motor_Moving_Mode(Z_RT_ID, 1, 200, 0, false, Can_State_Client);
    // 發布到達訊號
    publish_arrived_signal(Z_Arrived_pub);
    // 設定階段為-1
    Stage = -1;
}
void Roller_Model_Motec::initialize_Z_AXIS()
{
    Woring_state_pub(3, Mission_state_pub);
    ROS_WARN_STREAM("Init the Z AXIS");
    ROS_WARN_STREAM("This Hand will do this");

    Left_Stick_Initial_Flag=false;
    Right_Stick_Initial_Flag=false;
    Front_Stick_Initial_Flag=false;
    Back_Stick_Initial_Flag=false;

    if (Z_Up_Limit_Msg.data == false) {
        ROS_WARN_STREAM("Z AXIS will moving up");
        Stage = 2;//too slow
    } else {
        ROS_WARN_STREAM("Z AXIS will moving slow down");
        Stage = 2;
    }
}
void Roller_Model_Motec::initialize_Z_AXIS_up(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
{
    Woring_state_pub(3, Mission_state_pub);
    ROS_WARN_STREAM("Z AXIS is moving Up");
    SpeedHandZ = 0.2;
    double SpeedMotorZ = Z_Motor_Reverse * SpeedHandZ * 10000 / 8;
    Motor_Moving_Mode(Z_RT_ID, 1, 5, SpeedMotorZ, false, Can_State_Client);

    if (Z_Zero_Msg.data == true) {
        StopAllMotion(Z_RT_ID, Can_State_Client);
        Stage = 2;
    }

    if (Hand_stop_Msg.data == true) {
        StopAllMotion(Z_RT_ID, Can_State_Client);
        Hand_stop_Msg.data = false;
        Stage = -1;
    }
}
void Roller_Model_Motec::initialize_Z_AXIS_up_slow(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
{
    Woring_state_pub(3, Mission_state_pub);
    ROS_WARN_STREAM("Z AXIS is moving Up");
    SpeedHandZ = 0.005;
    double SpeedMotorZ = Z_Motor_Reverse * SpeedHandZ * 10000 / 8;
    Motor_Moving_Mode(Z_RT_ID, 1, 5, SpeedMotorZ, false, Can_State_Client);

    if (Z_Zero_Msg.data == true) {
        StopAllMotion(Z_RT_ID, Can_State_Client);
        Stage = 2;
    }

    if (Hand_stop_Msg.data == true) {
        StopAllMotion(Z_RT_ID, Can_State_Client);
        Hand_stop_Msg.data = false;
        Stage = -1;
    }
}

void Roller_Model_Motec::initialize_Z_AXIS_down(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
{
    Woring_state_pub(3, Mission_state_pub);
    ROS_WARN_STREAM("Z AXIS is moving down");
    SpeedHandZ = -0.8;
    double SpeedMotorZ = Z_Motor_Reverse * SpeedHandZ * 10000 / 8;
    Motor_Moving_Mode(Z_RT_ID, 1, 5, SpeedMotorZ, false, Can_State_Client);
    std_msgs::Int16 Initial_Stick;
    Initial_Stick.data=1;
    Left_Stick_pub.publish(Initial_Stick);
    Right_Stick_pub.publish(Initial_Stick);
    Front_Stick_pub.publish(Initial_Stick);
    Back_Stick_pub.publish(Initial_Stick);
    if(Left_Stick_Inp_Msg.data==false)
    {
      Left_Stick_Initial_Flag=true;
    }
    if(Right_Stick_Inp_Msg.data==false)
    {
      Right_Stick_Initial_Flag=true;
    }
    if(Front_Stick_Inp_Msg.data==false)
    {
      Front_Stick_Initial_Flag=true;
    }
    if(Back_Stick_Inp_Msg.data==false)
    {
      Back_Stick_Initial_Flag=true;
    }


    if (Z_Zero_Msg.data == true) {
        StopAllMotion(Z_RT_ID, Can_State_Client);
        if(     Left_Stick_Initial_Flag==true &&
                Right_Stick_Initial_Flag==true &&
                Front_Stick_Initial_Flag==true &&
                Back_Stick_Initial_Flag==true)
        {
        ros::Duration(0.2).sleep();
        Stage = 3;
        }
    }

    if (Hand_stop_Msg.data == true) {
        StopAllMotion(Z_RT_ID, Can_State_Client);
        Hand_stop_Msg.data = false;
        Stage = -1;
    }

    if (Z_Down_Limit_Msg.data == true) {
        StopAllMotion(Z_RT_ID, Can_State_Client);
        Stage = 1;
    }
}

void Roller_Model_Motec::initialize_Z_AXIS_up_again(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
{
    ROS_WARN_STREAM("Z AXIS is moving up");
    SpeedHandZ = 0.02;
    double SpeedMotorZ = Z_Motor_Reverse * SpeedHandZ * 10000 / 8;
    Motor_Moving_Mode(Z_RT_ID, 1, 5, SpeedMotorZ, false, Can_State_Client);

    if (Z_Zero_Msg.data == false) {
        StopAllMotion(Z_RT_ID, Can_State_Client);
        if(     Left_Stick_Home_Msg.data==true &&
                Right_Stick_Home_Msg.data==true &&
                Front_Stick_Home_Msg.data==true &&
                Back_Stick_Home_Msg.data==true)
        {
            if(     Left_stick_state_postion_1_Msg.data==true &&
                    Right_stick_state_postion_1_Msg.data==true &&
                    Front_stick_state_postion_1_Msg.data==true &&
                    Back_stick_state_postion_1_Msg.data==true)
            {
                Stage = 4;
            }
            else
            {
                ROS_ERROR_STREAM("Home None position");
            }
        }
    }

    Woring_state_pub(3, Mission_state_pub);
}

void Roller_Model_Motec::reset_Z_AXIS_zero(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
{
    ROS_WARN_STREAM("Z AXIS is reset the zero");
    ros::Duration(0.2).sleep();
    Motor_Moving_Mode(Z_SDO_ID, 8, 0, 0, false, Can_State_Client);
    ros::Duration(0.2).sleep();
    Motor_Moving_Mode(Z_SDO_ID, 6, 0, 0, false, Can_State_Client);
    ros::Duration(0.2).sleep();
    Motor_Moving_Mode(Z_SDO_ID, 7, 0, 0, false, Can_State_Client);
    ros::Duration(0.2).sleep();
    Motor_Moving_Mode(Z_SDO_ID, 3, 1, 0, false, Can_State_Client);
    ros::Duration(0.2).sleep();
    Woring_state_pub(1, Mission_state_pub);
    Stage = -1;
}
void Roller_Model_Motec::Set_Open_Close_Stick(bool Open)
{
   if(Open)
   {
       Stick_motion(true, Back_Stick_pub);
       Stick_motion(true, Front_Stick_pub);
   }
   else
   {
       Stick_motion(false, Back_Stick_pub);
       Stick_motion(false, Front_Stick_pub);
   }
   Woring_state_pub(3, Mission_state_pub);
   ros::Duration(Stick_Runing_Time).sleep();
   bool finish_Stick_motion=false;
   if(Open==true)
   {
       if(Left_stick_state_postion_1_Msg.data==true && Right_stick_state_postion_1_Msg.data == true)
       {
           finish_Stick_motion=true;
           Stick_Off(Back_Stick_pub);
           Stick_Off(Front_Stick_pub);
       }
       else
       {
           ROS_WARN_STREAM("stick is moving");
       }
   }
   else
   {
       if(Left_stick_state_postion_2_Msg.data==true && Right_stick_state_postion_2_Msg.data == true)
       {
           finish_Stick_motion=true;
           Stick_Off(Back_Stick_pub);
           Stick_Off(Front_Stick_pub);
       }
       else
       {
           ROS_WARN_STREAM("stick is moving");
       }
   }
   if(finish_Stick_motion ==true)
   {
   Stage = -1;
   }
}

void Roller_Model_Motec::Goods_Put_in_waiting_Turn_On_Stick(bool left_stick)
{
    Hand_stop_Msg.data = false;
    if (left_stick)
    {
        Stick_motion(false, Left_Stick_pub);
        Stick_motion(true, Right_Stick_pub);
    } else {
        Stick_motion(true, Left_Stick_pub);
        Stick_motion(false, Right_Stick_pub);
    }
    Woring_state_pub(3, Mission_state_pub);
    ros::Duration(Stick_Runing_Time).sleep();
    ROS_WARN_STREAM((left_stick ? "Left" : "Right") << " In");
    bool finish_Stick_motion=false;
    if(left_stick==true)
    {
        if(Left_stick_state_postion_1_Msg.data==true && Right_stick_state_postion_2_Msg.data == true)
        {
            finish_Stick_motion=true;
            Stick_Off(Right_Stick_pub);
            Stick_Off(Left_Stick_pub);
        }
        else
        {
            ROS_WARN_STREAM("stick is moving");
        }
    }
    else
    {
        if(Left_stick_state_postion_2_Msg.data==true && Right_stick_state_postion_1_Msg.data == true)
        {
            finish_Stick_motion=true;
            Stick_Off(Right_Stick_pub);
            Stick_Off(Left_Stick_pub);
        }
        else
        {
            ROS_WARN_STREAM("stick is moving");
        }
    }
    if(finish_Stick_motion ==true)
    {
        ROS_WARN_STREAM("This Hand will do this");
        if ((left_stick && Left_Limit_Msg.data == false) || (!left_stick && Right_Limit_Msg.data == false))
        {
            ROS_WARN_STREAM("X AXIS will rolling " << (left_stick ? "right" : "left") << " to " << (left_stick ? "XLL" : "XRL"));
            Stage = 1;
        }
        else
        {
            ROS_WARN_STREAM("X AXIS will rolling " << (left_stick ? "right" : "left") << " to " << (left_stick ? "XRL" : "XLL"));
            Stage = 2;
        }
    }
}
void Roller_Model_Motec::Goods_Put_in_waiting_Launch_the_Roller_waiting_Goods(bool left_stick, actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
{
    ROS_WARN_STREAM("X AXIS is moving " << (left_stick ? "left" : "right") << (left_stick ? "XRL" : "XLL"));
    if ((left_stick && Left_Limit_Msg.data == true) || (!left_stick && Right_Limit_Msg.data == true))
    {
        StopAllMotion(Z_RT_ID, Can_State_Client);
        Stage = 2;
    }
    else if (Z_Up_Limit_Msg.data == true || Z_Down_Limit_Msg.data == true)
    {
        StopAllMotion(Z_RT_ID, Can_State_Client);
        Woring_state_pub(4, Mission_state_pub);
    }
    else if (Hand_stop_Msg.data == true)
    {
         Hand_stop_Command(Can_State_Client);
    }
    else
    {
        sendRoller_AxiscommandMsg(left_stick ? Go_left : Go_right);
        Woring_state_pub(2, Mission_state_pub);
    }
}
void Roller_Model_Motec::Goods_Put_in_waiting_Get_Goods_Stop_Motion(bool left_stick, actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
{
    ROS_WARN_STREAM("X AXIS is moving " << (left_stick ? "right" : "left") << (left_stick ? "XLL" : "XRL"));
    if ((!left_stick && Left_Limit_Msg.data == true) || (left_stick && Right_Limit_Msg.data == true))
    {
        ros::Duration(Roller_Delay_Time).sleep();
        StopAllMotion(Z_RT_ID, Can_State_Client);
        bool finish_Stick_motion=false;
        if(left_stick==true)
        {
            if(Left_stick_state_postion_1_Msg.data==true)
            {
              finish_Stick_motion=true;
            }
        }
        else
        {
            if(Right_stick_state_postion_1_Msg.data==true)
            {
              finish_Stick_motion=true;
            }
        }
        if(finish_Stick_motion==true)
        {
            Stick_Off(Right_Stick_pub);
            Stick_Off(Left_Stick_pub);
            Hand_stop_Command(Can_State_Client);
        }
        else
        {
        Stick_motion(!left_stick, left_stick ? Right_Stick_pub : Left_Stick_pub);
        }
        ros::Duration(Stick_Runing_Time).sleep();

        Timer_Record = Get_the_msec_time_of_computer(time_now);
        Timer1 = (Now_Time - Timer_Record) / 1000;
    }
    else if (Z_Up_Limit_Msg.data == true || Z_Down_Limit_Msg.data == true)
    {
        StopAllMotion(Z_RT_ID, Can_State_Client);
        Woring_state_pub(4, Mission_state_pub);
    }
    else if (Hand_stop_Msg.data == true)
    {
        Hand_stop_Command(Can_State_Client);
        Timer_Record = Get_the_msec_time_of_computer(time_now);
        Timer1 = (Now_Time - Timer_Record) / 1000;
    }
    else
    {
        sendRoller_AxiscommandMsg(left_stick ? Go_left : Go_right);
        Woring_state_pub(3, Mission_state_pub);
    }
}

void Roller_Model_Motec::Goods_Roll_Out_waiting_Turn_On_Stick(bool is_left)
{
    Hand_stop_Msg.data = false;
    if (is_left)
    {
        Stick_motion(false, Left_Stick_pub);
        Stick_motion(true, Right_Stick_pub);
    } else {
        Stick_motion(true, Left_Stick_pub);
        Stick_motion(false, Right_Stick_pub);
    }
    ros::Duration(Stick_Runing_Time).sleep();
    ROS_WARN_STREAM((is_left ? "Left" : "Right") << " Out");

    bool finish_Stick_motion=false;

    if(is_left==true)
    {
        if(Left_stick_state_postion_1_Msg.data==true && Right_stick_state_postion_2_Msg.data == true)
        {
            finish_Stick_motion=true;
            Stick_Off(Right_Stick_pub);
            Stick_Off(Left_Stick_pub);
        }
        else
        {
            ROS_WARN_STREAM("stick is moving");
        }
    }
    else
    {
        if(Left_stick_state_postion_2_Msg.data==true && Right_stick_state_postion_1_Msg.data == true)
        {
            finish_Stick_motion=true;
            Stick_Off(Right_Stick_pub);
            Stick_Off(Left_Stick_pub);
        }
        else
        {
            ROS_WARN_STREAM("stick is moving");
        }
    }
    if(finish_Stick_motion ==true)
    {
        ROS_WARN_STREAM("This Hand will do this");
        if (Left_Limit_Msg.data == false && Right_Limit_Msg.data == false)
        {
            None_Good = true;
            ROS_ERROR_STREAM("No Good on the table");
            Woring_state_pub(4, Mission_state_pub);
        }
        else
        {
            None_Good = false;
            ROS_WARN_STREAM("X AXIS will rolling " << (is_left ? "left" : "right") << " Off");
            Woring_state_pub(3, Mission_state_pub);
            Stage = 1;
        }
    }
    Timer_Record = Get_the_msec_time_of_computer(time_now);
    Timer1 = (Now_Time - Timer_Record) / 1000;
}

void Roller_Model_Motec::Goods_Roll_Out_waiting_Launch_the_Roller(bool is_left, actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
{
    ROS_WARN_STREAM("X AXIS is moving " << (is_left ? "left" : "right") << " out");
    ROS_INFO_STREAM("Time:" << Timer1);
    if (Hand_stop_Msg.data == true)
    {
        Stick_motion(true, Left_Stick_pub);
        Stick_motion(true, Right_Stick_pub);
        bool finish_Stick_motion=false;
        if(Left_stick_state_postion_1_Msg.data==true && Right_stick_state_postion_1_Msg.data == true)
        {
            finish_Stick_motion=true;
            Stick_Off(Right_Stick_pub);
            Stick_Off(Left_Stick_pub);
        }
        else
        {
            ROS_WARN_STREAM("stick is moving");
        }
        if(finish_Stick_motion==false)
        {
        Hand_stop_Command(Can_State_Client);
        }

        ros::Duration(Stick_Runing_Time).sleep();
    }
    else if (Z_Up_Limit_Msg.data == true || Z_Down_Limit_Msg.data == true)
    {
        StopAllMotion(Z_RT_ID, Can_State_Client);
        Woring_state_pub(4, Mission_state_pub);
        Stage = -1;
    }
    else if (Timer1 > 50)
    {
        StopAllMotion(Z_RT_ID, Can_State_Client);
        Woring_state_pub(4, Mission_state_pub);
        Stage = -1;
        ROS_ERROR_STREAM("Moving " << (is_left ? "left" : "right") << " out Time Out !!");
        Time_out_error = true;
        Timer_Record = Get_the_msec_time_of_computer(time_now);
    }
    else
    {
        sendRoller_AxiscommandMsg(is_left ? Go_left : Go_right);
        Woring_state_pub(2, Mission_state_pub);
    }
}

void Roller_Model_Motec::Hand_stop_Command(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
{
    StopAllMotion(Z_RT_ID, Can_State_Client);
    Woring_state_pub(1, Mission_state_pub);
    Hand_stop_Msg.data = false;
    Task_Order.Hand_Mode = 0;
    Stage = -1;

}

void publish_arrived_signal(ros::Publisher& publisher) {
    std_msgs::Bool Arrived;
    Arrived.data = true;
    publisher.publish(Arrived);
}
