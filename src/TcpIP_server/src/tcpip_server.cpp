#include "tcpip_server.h"

TcpIP_Server::TcpIP_Server(std::string Name)
{
    console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,console::levels::Debug);
    std_msgs::Int64MultiArray Command_Pose;
    Z_Command_Pub              = Node_Handle.advertise<std_msgs::Int64MultiArray>("/XZ_Command",10);
    X_Stop_Pub                 = Node_Handle.advertise<std_msgs::Bool>("/Hand_Stop_Command",10);
    Lora_power_Pub             = Node_Handle.advertise<std_msgs::Bool>("hand/Lorapower",10);
    Clear_Command_Pub          = Node_Handle.advertise<std_msgs::Bool>("caraction/ClearCommand",10);
    Mission_pub                = Node_Handle.advertise<std_msgs::Int64MultiArray>("/XZ_action", 10);
    Goods_state_pub            = Node_Handle.advertise<std_msgs::Int16>("Goods_state",10);
    Mission_state_Sub          = Node_Handle.subscribe("/Mission_state", 1000, &TcpIP_Server::Mission_stateCallback,this);
    Mission_type_Direction_Sub = Node_Handle.subscribe("/Mission_type_Direction", 1000,&TcpIP_Server::Mission_type_DirectionCallback,this);
    Mission_type_Motion_Sub    = Node_Handle.subscribe("/Mission_type_Motion", 1000,&TcpIP_Server::Mission_type_MotionCallback,this);
    Mission_type_High_Sub      = Node_Handle.subscribe("/Mission_type_High", 1000,&TcpIP_Server::Mission_type_HighCallback,this);
    Good_state_Sub             = Node_Handle.subscribe("/Goods_state", 1000,&TcpIP_Server::Goods_StateCallback,this);
    stringstream Tcpip_Server_error_name;
    Tcpip_Server_error_name<<ros::this_node::getName()<<"/Error_Code_Tcpip_Server";
    Error_pub                  =Node_Handle.advertise<std_msgs::Int16>(Tcpip_Server_error_name.str(), 10);
    bool Listenfd_state=true;
    bool Connect_state=true;
    int PORT = 1357;
    int Listenfd,LOG,Connectfd;
    struct sockaddr_in  Client;
    socklen_t           Addrlen;
    std::string Hand_Name = "HND01";
    Init_param(Node_Handle, "/TcpIP_Lora_Server/PORT", PORT);
    Init_param(Node_Handle, "/TcpIP_Lora_Server/Name", Hand_Name);
    Init_param(Node_Handle, "/TcpIP_Lora_Server/Server_ID", Lora_ID);
    Init_param(Node_Handle, "/TcpIP_Lora_Server/Channel", Lora_channel);
    setupSocket(LOG,Listenfd, PORT,Hand_Name);
    // 等待客戶端連線
    ros::Rate Loop_Rate(50);
    while(ros::ok())
    {
        Command_Pose.data.clear();
        char Recvbuf[1024]={};
        if(listen(Listenfd,LOG)<0)
        {
          ROS_ERROR("Listen() error");
          Listenfd_state=false;
        }
        else
        {
          Listenfd_state=true;
        }
        //取得客戶端位置
        Addrlen = sizeof(Client);
        //接收客戶端的指令
//        ROS_WARN("Ready for order");
        //與客戶端連線
        Connectfd = accept(Listenfd,(struct sockaddr *)&Client,&Addrlen);
        ROS_DEBUG("Connecting");
        if(Connectfd < 0)
        {
          ROS_ERROR("Connect error");
          std_msgs::Bool Lora_Connect;
          Lora_Connect.data=false;
          Lora_power_Pub.publish(Lora_Connect);
          ros::Duration(5.0).sleep();
          Lora_Connect.data=true;
          Lora_power_Pub.publish(Lora_Connect);
          ros::Duration(5.0).sleep();
          Connect_state=false;
        }
        else
        {
            Connect_state=true;
        }
        //設定更新頻率"
        int Recv_state=0;
        Recv_state=recv(Connectfd, Recvbuf, sizeof(Recvbuf), 0);
        if(Recv_state<0)
        {
          ROS_INFO_STREAM("None Command!");
        }
        else
        {
          ROS_WARN_STREAM("recv:"<<Recvbuf);
        }
        int CommandM=0,CommandD=0,CommandHigh=0;

        Recv_String.assign(Recvbuf);
        ros::spinOnce();
        ProcessRecvbuf(Recvbuf,CommandD,CommandM,CommandHigh,Z_Command_Pub,X_Stop_Pub);
        if (Recv_String==Hand_stop_Command)
              {
                ROS_WARN("Hand_stop");
                publishHandStopMessage(X_Stop_Pub);
              }
              else if (Recv_String==Hand_open_Command)
              {
                ROS_WARN("(Hand,M:Open)");
                sendHandcommandMsg(Mission_pub,12,0,0);
              }
              else if (Recv_String==Hand_close_Command)
              {
                ROS_WARN("(Hand,M:Close)");
                sendHandcommandMsg(Mission_pub,13,0,0);
              }
              else if (Recv_String==Good_Empty_Command)
              {
                ROS_WARN("Good_Empty");
                publishGoodsStateMessage(Goods_state_pub,1);
              }
              else if (Recv_String==Good_Case_Command)
              {
                ROS_WARN("Good_Case");
                publishGoodsStateMessage(Goods_state_pub,2);
              }
              else if (Recv_String==Good_Full_Command)
              {
                ROS_WARN("Good_Full");
                publishGoodsStateMessage(Goods_state_pub,3);
              }
              else if (Recv_String==Clear_Command)
              {
                ROS_WARN("Command type:Clear Command");
                sendHandcommandMsg(Mission_pub,0,0,0);
              }
              else if (Recv_String==Hand_Initial_Command)
              {
                ROS_WARN("Command type:Hand_Initial");
                sendHandcommandMsg(Mission_pub,3,0,0);
              }
             else if (Recv_String==Hand_State_Command)//如果收到狀態請求指令
              {
                sendHandStateRequest(State_Of_Hand.Direction,State_Of_Hand.Put_Or_Get,
                                     State_Of_Hand.High,Mission_state, State_Of_Hand.Good_state,
                                     Hand_Name,Connectfd);
              }
        close(Connectfd);
        std_msgs::Int16 Error_pub_msg;
        if (Listenfd_state == false)
        {
            Error_pub_msg.data = 1;  // 將Error_pub_msg的數值設為1，表示Socket未登入的錯誤碼
            Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
        }
        else
        {
            // 如果Socket已登入，繼續檢查系統是否連接
            if (Connect_state == false) {
                Error_pub_msg.data = 2;  // 將Error_pub_msg的數值設為2，表示系統未連接的錯誤碼
                Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
            }
            else
            {

                    Error_pub_msg.data = 0;  // 將Error_pub_msg的數值設為0，表示系統未連接的錯誤碼
                    Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題


            }
        }
        ros::spinOnce();
        Loop_Rate.sleep();
    }
    close(Connectfd);
}
TcpIP_Server::~TcpIP_Server(void)
{
    ROS_INFO_STREAM("TcpIP_Server is Turning Off");
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "TcpIP_Server");
  TcpIP_Server TcpIP_Server("TcpIP_Server");
  return 0;
}
