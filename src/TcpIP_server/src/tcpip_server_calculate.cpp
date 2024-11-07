#include "tcpip_server.h"

void TcpIP_Server::Char_Left_Right(const char *Recvbuf, int Beginpoint, int Checkpoint, int &CommandX)
{
    // 定义命令类型和输入缓冲区
    const string Command_Type_Left = "Left";
    const string Command_Type_Right = "Right";
    // 解析接收到的数据并转换为字符串
    string Command_Input(Recvbuf + Beginpoint, Checkpoint - Beginpoint);
    // 判断命令类型并设置方向
    if (Command_Input == Command_Type_Left)
    {
        ROS_WARN_STREAM("Left");
        CommandX = 1;
    }
    else if (Command_Input == Command_Type_Right)
    {
        ROS_WARN_STREAM("Right");
    CommandX = 2;
    }
}
void TcpIP_Server::Char_Put_Get(const char *Recvbuf, int Beginpoint, int Checkpoint, int &CommandX)
{
    // 定义命令类型和输入缓冲区
    const string Command_Type_Put = "Put";
    const string Command_Type_Get = "Get";
    const string Command_Type_No = "No";
    // 解析接收到的数据并转换为字符串
    string Command_Input(Recvbuf + Beginpoint, Checkpoint - Beginpoint);
    // 判断命令类型并设置Put_Get值
    if (Command_Input == Command_Type_Put)
    {
        ROS_WARN_STREAM("Put");
        CommandX = 1;
    }
    else if (Command_Input == Command_Type_Get)
    {
        ROS_WARN_STREAM("Get");
        CommandX = 2;
    }
    else if (Command_Input == Command_Type_No)
    {
        ROS_WARN_STREAM("NO");
        CommandX = 3;
    }
}
void TcpIP_Server::Char_int(const char *Recvbuf, int Beginpoint, int Checkpoint, int &CommandX)
{
    int sign = 1;
    int startIndex = Beginpoint;
    if (Recvbuf[startIndex] == '-')
    {
        sign = -1;
        startIndex++;
    }
    CommandX = 0;
    for (int i = startIndex; i < Checkpoint; i++)
    {
        if (Recvbuf[i] >= '0' && Recvbuf[i] <= '9')
        {
            CommandX = CommandX * 10 + (Recvbuf[i] - '0');
        }
        else
        {
        // 非数字字符，停止解析
            break;
        }
    }
    CommandX *= sign;
    ROS_WARN("H:%d", CommandX);
}


void TcpIP_Server::ProcessRecvbuf(char *Recvbuf, int &CommandD, int &CommandM, int &CommandHigh, ros::Publisher &XZ_Command_Pub, ros::Publisher &X_Stop_Pub)
{
  string recv_buf_string;
  recv_buf_string=Recvbuf;
  int RecvbufSize=recv_buf_string.size();
  if (RecvbufSize > 5 && Recvbuf[1] == 'H' && Recvbuf[2] == 'a' && Recvbuf[3] == 'n' && Recvbuf[4] == 'd' && Recvbuf[5] == ',' && Recvbuf[6] != 'M')
  {
    int checkpoint = 0;
    int beginpoint = 8;

    // 解析 Left/Right 命令
    for (int i = beginpoint; i < RecvbufSize; i++)
    {
      if (Recvbuf[i] == ',')
      {
        checkpoint = i;
        break;
      }
    }

    Char_Left_Right(Recvbuf, beginpoint, checkpoint, CommandD);

    beginpoint = checkpoint + 3;

    // 解析 Put/Get 命令
    for (int i = beginpoint; i < RecvbufSize; i++)
    {
      if (Recvbuf[i] == ',')
      {
        checkpoint = i;
        break;
      }
    }
    Char_Put_Get(Recvbuf, beginpoint, checkpoint, CommandM);

    beginpoint = checkpoint + 3;

    // 解析高度值
    for (int i = beginpoint; i < RecvbufSize; i++)
    {
      if (Recvbuf[i] == ')')
      {
        checkpoint = i;
        break;
      }
    }
    Char_int(Recvbuf, beginpoint, checkpoint, CommandHigh);

    // 将解析得到的命令参数添加到 Command_Pose 数据中
    sendHandcommandMsg(XZ_Command_Pub,CommandD,CommandM,CommandHigh);

    // 延时一段时间
    ros::Duration(0.02).sleep();

    // 发布手停止信号
    std_msgs::Bool Hand_Stop_Msg;
    Hand_Stop_Msg.data = false;
    X_Stop_Pub.publish(Hand_Stop_Msg);
  }
}


void TcpIP_Server::setupSocket(int& LOG,int& listenfd, const int port, const string& handName)
{
    struct sockaddr_in Server;
    struct timeval Tv;
    Tv.tv_sec = 1;
    Tv.tv_usec = 0;
    // Create socket
    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    if (listenfd == -1)
    {
        ROS_ERROR("Socket Error");
        return;
    }
    // Set server information
    memset((void*)&Server, 0, sizeof(Server));
    Server.sin_family = AF_INET;
    Server.sin_addr.s_addr = htonl(INADDR_ANY);
    Server.sin_port = htons(port);
    setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &Server, sizeof(Server));
    setsockopt(listenfd, SOL_SOCKET, SO_RCVTIMEO, &Tv, sizeof(struct timeval));
    if (bind(listenfd, (struct sockaddr*)&Server, sizeof(Server)) < 0)
    {
        printf("Bind() error\n");
        return;
    }
}

void TcpIP_Server::sendHandStateRequest(int direction, int putOrGet, int high, int missionState, int goodState, const string& handName, int &connectfd)
{
    ROS_WARN("Command type: Ask for the Hand_state~~");

    stringstream send_word;
    send_word << "(Hand,D:";

    switch (direction)
    {
        case 1:
            send_word << "Left,";
            break;
        case 2:
            send_word << "Right,";
            break;
        default:
            send_word << "None,";
            break;
    }

    send_word << "M:";
    switch (putOrGet)
    {
        case 1:
            send_word << "Put,";
            break;
        case 2:
            send_word << "Get,";
            break;
        default:
            send_word << "None,";
            break;
    }

    send_word << "H:" << high << ",";
    switch (missionState)
    {
        case 0:
            send_word << "S:Initial,";
            break;
        case 1:
            send_word << "S:Done,";
            break;
        case 2:
            send_word << "S:Ready,";
            break;
        case 3:
            send_word << "S:Busy,";
            break;
        case 4:
            send_word << "S:Error,";
            break;
    }

    send_word << "G:" << goodState << ",";
    send_word << "NAME:" << handName << ")";
    ROS_WARN_STREAM(send_word.str());
    string send_str = send_word.str();
    int size_of_msg = send_str.size();
    char Lora_ID_msg=Lora_ID;
    char Lora_channel_msg=Lora_channel;
    char char_mix[64] = {0,Lora_ID_msg,Lora_channel_msg};


    for (int i = 0; i < size_of_msg; i++)
    {
        char_mix[i + 3] = send_str[i];
    }

    char_mix[size_of_msg + 3] = 13;
    char_mix[size_of_msg + 4] = 10;
//    for (int i = 0; i < size_of_msg+5; i++)
//    {
//        ROS_WARN_STREAM((int)char_mix[i]<<":"<<char_mix[i]);
//    }
    ros::Duration(0.05).sleep();
    send(connectfd, char_mix, size_of_msg + 5, 0);
    ros::Duration(0.05).sleep();
}
