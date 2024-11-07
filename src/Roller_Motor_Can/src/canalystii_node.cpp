#include "canalystii_node.h"
CANalystii_node::CANalystii_node(int vci_device_type, int vci_device_ind) : 
CANalystii(vci_device_type,vci_device_ind), node_handle_("~")
{
    ROS_INFO("Inititalizing CANalyst-ii node...");
    node_handle_.param<std::string>("can_msg_pub_topic", can_msg_pub_topic_, "/Z_state_can");
    ROS_INFO("can_msg_pub_topic_: %s", can_msg_pub_topic_.c_str());
    can_msg_pub_ = node_handle_.advertise<canalystii_node_msg::can>(can_msg_pub_topic_, 30);
    can_msg_sub  = node_handle_.subscribe("/Hand_action_can",30,canCallback);

}

canalystii_node_msg::can CANalystii_node::can_obj2msg(const VCI_CAN_OBJ& can_obj)
{
    canalystii_node_msg::can can_msg;
    can_msg.header.frame_id = "/canalystii_XZ";
    can_msg.header.stamp = ros::Time::now();

    //set data    
    can_msg.id = can_obj.ID;
    can_msg.timeflag = can_obj.TimeFlag;
    can_msg.sendtype = can_obj.SendType;
    can_msg.remoteflag = can_obj.RemoteFlag;
    can_msg.externflag = can_obj.ExternFlag;
    can_msg.datalen = can_obj.DataLen;
    for(int i =0;i<8;i++){
        can_msg.data[i] = can_obj.Data[i];
    }
    return can_msg;
}

VCI_CAN_OBJ CANalystii_node::can_msg2obj(const canalystii_node_msg::can& can_msg)
{
    VCI_CAN_OBJ can_obj;
    //TODO: (vincent.cheung.mcer@gmail.com) type not consistent
    //can_obj.TimeStamp = can_msg.header.stamp
    can_obj.ID = can_msg.id;
    can_obj.TimeFlag = can_msg.timeflag;
    can_obj.SendType = can_msg.sendtype;
    can_obj.RemoteFlag = can_msg.remoteflag;
    can_obj.ExternFlag = can_msg.externflag;
    can_obj.DataLen = can_msg.datalen;
    for(int i =0;i<8;i++)
    {
        can_obj.Data[i] = can_msg.data[i];
    }
    return can_obj;    
}



void CANalystii_init()
{
  VCI_INIT_CONFIG config;
  config.AccCode=0;
  config.AccMask=0xFFFFFFFF;
  config.Filter=1;//接收所有帧
  config.Timing0=0x00;
  config.Timing1=0x1C;
  config.Mode=0;//正常模式


  if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
  {
    ROS_WARN(">>Init CAN1 error");
    VCI_CloseDevice(VCI_USBCAN2,0);
  }
  if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
  {
    ROS_WARN(">>Start CAN1 error");
    VCI_CloseDevice(VCI_USBCAN2,0);

  }
  if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
  {
    ROS_WARN(">>Init can2 error");
    VCI_CloseDevice(VCI_USBCAN2,0);
  }
  if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
  {
    ROS_WARN(">>Start can2 error\n");
    VCI_CloseDevice(VCI_USBCAN2,0);
  }

}

void CANalystii_send_msg(VCI_CAN_OBJ* send,int ID   ,int can_port ,
                                           int data1,int data2    ,int data3  ,int data4,
                                           int data5,int data6    ,int data7  ,int data8)
{
  send[0].ID=ID;
  send[0].SendType=0;
  send[0].RemoteFlag=0;
  send[0].ExternFlag=0;
  send[0].DataLen=8;
  send[0].Data[0] = data1;
  send[0].Data[1] = data2;
  send[0].Data[2] = data3;
  send[0].Data[3] = data4;
  send[0].Data[4] = data5;
  send[0].Data[5] = data6;
  send[0].Data[6] = data7;
  send[0].Data[7] = data8;
  if(VCI_Transmit(VCI_USBCAN2, 0, can_port, send, 1) == 1)
      {
        ROS_DEBUG("CAN%d TX ID:0x%08X",can_port,send[0].ID);
        if(send[0].ExternFlag==0) printf(" Standard ");
        if(send[0].ExternFlag==1) printf(" Extend   ");
        if(send[0].RemoteFlag==0) printf(" Data   ");
        if(send[0].RemoteFlag==1) printf(" Remote ");
        ROS_DEBUG("DLC:0x%02X",send[0].DataLen);
        ROS_DEBUG(" data:0x");
        for(int i=0;i<send[0].DataLen;i++)
        {
          ROS_DEBUG(" %02X",send[0].Data[i]);
        }
        ROS_DEBUG("\n");
      }
}


