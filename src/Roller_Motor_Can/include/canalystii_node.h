#ifndef CANALYSTII_NODE_H
#define CANALYSTII_NODE_H

#include "ros/ros.h"
#include "canalystii.h"
#include "canalystii_node_msg/can.h"
#include "std_msgs/Int64MultiArray.h"
// Simple usage:
// CANalystii_node can_device;
// can_device.start_device();
// can_device.init_can_interface();
// can_device.receive_can_frame();//listening from CAN bus
// can_device.send_can_frame();//send to CAN bus

void CANalystii_init();
void CANalystii_send_msg(VCI_CAN_OBJ* send,int ID,int can_port,int data1,int data2,int data3
                         ,int data4,int data5,int data6,int data7,int data8);
void canCallback(const std_msgs::Int64MultiArray &msg);



class CANalystii_node:public CANalystii{
protected:

private:
    ros::NodeHandle node_handle_;    
    std::string can_msg_pub_topic_;

    //TODO:(vincent.cheung.mcer@gmail.com) Noy yet implement subscriber.
    //ros::Subscriber can_msg_sub_;
    //std::string can_msg_sub_topic_;
    //void can_msg_cb();
      
public:
    CANalystii_node(int vci_device_type=4, int vci_device_ind=0);
    static canalystii_node_msg::can can_obj2msg(const VCI_CAN_OBJ& can_obj);
    static VCI_CAN_OBJ can_msg2obj(const canalystii_node_msg::can& can_msg);
    ros::Publisher      can_msg_pub_;
    ros::Subscriber     can_msg_sub;
    ~CANalystii_node(){}


};
#endif
