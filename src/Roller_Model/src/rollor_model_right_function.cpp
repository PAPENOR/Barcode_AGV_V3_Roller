#include "rollor_model_right_function.h"
#include "roller_model_motec.h"
void Roller_Model_Motec::Right_Goods_Put_in_Turn_On_Stick_lifting(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
{
    Hand_stop_Msg.data = false;
    Stick_motion(true, Left_Stick_pub);
    Stick_motion(false, Right_Stick_pub);
    Woring_state_pub(3, Mission_state_pub);
    bool finish_Stick_motion=false;
    if(Left_stick_state_postion_1_Msg.data==true && Right_stick_state_postion_2_Msg.data == true)
    {
        finish_Stick_motion=true;
        Stick_Off(Right_Stick_pub);
        Stick_Off(Left_Stick_pub);
        ROS_WARN_STREAM("Right In");
        ROS_WARN_STREAM("This Hand will do this");
        Positon_mode_Launching(Can_State_Client);
        if (Right_Limit_Msg.data == false)
        {
            ROS_WARN_STREAM("X AXIS will rolling left to XRL");
            Stage = 2;
        }
        else
        {
            ROS_WARN_STREAM("X AXIS will rolling left to XLL");
            Stage = 3;
        }
    }
}

void Roller_Model_Motec::Right_Goods_Put_in_waiting_Goods(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
{
    ROS_WARN_STREAM("X AXIS is moving left XRL");
    if (Right_Limit_Msg.data == true)
    {
        StopAllMotion(Z_RT_ID, Can_State_Client);
        Stage = 3;
    }
    else if (Z_Up_Limit_Msg.data || Z_Down_Limit_Msg.data)
    {
        StopAllMotion(Z_RT_ID, Can_State_Client);
        Woring_state_pub(4, Mission_state_pub);
    }
    else if (Hand_stop_Msg.data)
    {
        Hand_stop_Command(Can_State_Client);
        Timer_Record=Get_the_msec_time_of_computer(time_now);
        Timer1=(Now_Time-Timer_Record)/1000;
    }
    else
    {
        sendRoller_AxiscommandMsg(Go_right);
        Woring_state_pub(2, Mission_state_pub);
    }
}
void Roller_Model_Motec::Right_Goods_Roll_Out_waiting_Turn_On_Stick_lifting(actionlib::SimpleActionClient<Roller_Motor_Can::Single_Action_CanAction> &Can_State_Client)
{
    Hand_stop_Msg.data=false;
    Stick_motion(true,Left_Stick_pub);
    Stick_motion(false,Right_Stick_pub);
    Woring_state_pub(3, Mission_state_pub);
    bool finish_Stick_motion=false;
    if(Left_stick_state_postion_2_Msg.data==true && Right_stick_state_postion_2_Msg.data == true)
    {
    finish_Stick_motion=true;
    Stick_Off(Right_Stick_pub);
    Stick_Off(Left_Stick_pub);
    ROS_WARN_STREAM("Right Out");
    ROS_WARN_STREAM("This Hand will do this");
    Positon_mode_Launching(Can_State_Client);
    if(Left_Limit_Msg.data==false && Right_Limit_Msg.data==false)
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
    }
    Timer_Record=Get_the_msec_time_of_computer(time_now);
    Timer1=(Now_Time-Timer_Record)/1000;
}
