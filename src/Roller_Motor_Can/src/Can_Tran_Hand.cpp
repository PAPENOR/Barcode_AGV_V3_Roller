#include "ros/ros.h"
#include "canalystii_node.h"
#include <vector>
#include <std_msgs/Int16.h>
using namespace std;
/*
  This node describe:
        This is a node can only active by the Canalyst-II device.
        It communicate between ROS and Canopen by the Canalyst-II device.

  This node I/O:

  Input:
        the action command from ROS (/Single_Action_Can)
        the device state from Canalyst-II

  Output:
        the device state to ROS (/LR_state_can)
        the action command to Canalyst-II
*/
std_msgs::Int64MultiArray can_sub_ros_msg;
void canCallback(const std_msgs::Int64MultiArray& msg)
  {
    can_sub_ros_msg=msg;
  }
VCI_INIT_CONFIG init_config()
  {
    VCI_INIT_CONFIG   vci_conf;
    vci_conf.AccCode  = 0x80000008;
    vci_conf.AccMask  = 0xFFFFFFFF;
    vci_conf.Filter   = 1;            //Receive all frames
    vci_conf.Timing0  = 0x00;
    vci_conf.Timing1  = 0x1C;         //Baudrate 500kbps
    vci_conf.Mode     = 0;            //Normal mode
    return vci_conf;
  }

double Get_the_msec_time_of_computer(struct timeval &time_now)
  {
    double now_time;
    gettimeofday(&time_now,nullptr);
    now_time = time_now.tv_sec*1000+time_now.tv_usec/1000;
    return now_time;
  }


int main(int argc, char **argv)
    {
      ros::init(argc, argv, "Can_Tran_Hand");
      CANalystii_node can_node;

      ros::Publisher Error_pub;  //***~
      stringstream Can_error_name;
      ros::NodeHandle Node;
      Can_error_name<<ros::this_node::getName()<<"/Error_Code_Can_tran";
      Error_pub = Node.advertise<std_msgs::Int16>(Can_error_name.str(), 10);
      bool Can_bus_start_error=false;
      bool Can_bus_init_error = false;
      bool Can_bus_pass_time_error = false;  //~***


      /*Launch the Canbus device*/
      if(!can_node.start_device())
        {
          ROS_ERROR("Device starts error");
          Can_bus_start_error=true;  //***
          return -1;
        }
      else
        {
          ROS_INFO("Device starts complete");
          Can_bus_start_error=false;  //***
        }

      /*Initial the Canbus port CAN 1*/
      VCI_INIT_CONFIG vci_conf=init_config();
      unsigned int can_idx = 0;/*CAN 1*/
      if(!can_node.init_can_interface(can_idx,vci_conf))
        {
          ROS_ERROR("Device port init error");
          Can_bus_init_error = true;  //***
          return -1;
        }
      else
        {
          ROS_INFO("Device port init complete");
          Can_bus_init_error = false;  //***
        }

      /*Waiting for the Canbus topic*/
      ROS_INFO("Listening to can bus");
      CANalystii_init();

      /*Reading the computer timers*/
      struct timeval time_now{};
      gettimeofday(&time_now,nullptr);
      time_t begin=time_now.tv_sec*1000+time_now.tv_usec/1000,new_time;
      double start_time=begin;

      /*Initial the values*/
      double pass_sec=0.0;
      int counter=0;
      VCI_CAN_OBJ can_obj;
      VCI_CAN_OBJ send[1];
      bool hadmsg=false;
      bool samemsgs=true;
      vector<int> last_msgs;

      ros::Rate r(2000);

      while(ros::ok())
        {
          unsigned int recv_len = 1;

          if(can_node.receive_can_frame(can_idx,can_obj,recv_len,20))//receive_can_frame(can_idx,recv_obj,recv_len,wait_time)
            {
              /*If Canalyst-II receive the msgs*/
              /*Set msgs to the ROS type,then publish it*/
              ROS_INFO("Device receive the msgs");
              canalystii_node_msg::can msg = CANalystii_node::can_obj2msg(can_obj);
              can_node.can_msg_pub_.publish(msg);

              /*Recording the time of receive the msgs*/
              /*Mark the msg state as "hadmsg"*/
              start_time  = Get_the_msec_time_of_computer(time_now);
              Can_bus_pass_time_error = false;  //***
              hadmsg=true;

            }
            else
            {
              /*If Canalyst-II don't receive the msgs*/
              /*Recording the time that between two msgs*/
              new_time  = Get_the_msec_time_of_computer(time_now);
              pass_sec=((double)(new_time-start_time));

            }
          if(pass_sec>400)
            {
              /*If the time that between two msgs is longer than 400msec*/
              /*It means the system is waiting too long*/
              ROS_ERROR("Canalyst-II state is out of time");

              /*Return the Error code(998) to the topic:/LR_state_can*/
              /*This system define the code(998) as the state of "Canalyst-II state is out of time"*/
              canalystii_node_msg::can msg;
              msg.id=998;
              msg.datalen=99;
              can_node.can_msg_pub_.publish(msg);

              /*Reset the recording time*/
              /*Mark the msg state as "hadmsg"*/
              start_time = Get_the_msec_time_of_computer(time_now);
              hadmsg=false;
              Can_bus_pass_time_error = true;  //***
            }
          /*counter is controlling the frequency*/
          if(counter>5)
            {
              if(can_sub_ros_msg.data.size()>=10)
                {
                  /*If this program receive the data from topic:/Single_Action_Can */
                  if(can_sub_ros_msg.data.at(1)==2)
                  {
                  /*If msgs state is "2".It means this msgs is an empty message.*/
                  /*If msgs state is "2".It also means user wants to stop the publish.*/
                  //std::cout<<"stop publish"<<std::endl;
                  }
                  else
                  {
                    /*If the size of data which we want to send is longer than "10"*/
                    /*If Canalyst-II receice any data.It means the motor is active*/                    
                    samemsgs=true;
                    if(last_msgs.size()>=10)
                    {
                      for(int i=0;i<last_msgs.size();i++)
                      {
                        //cout<<"I:"<<i<<endl;
                        //cout<<"LAST:"<<last_msgs.at(i)<<endl;
                        //cout<<"NOW:"<<can_sub_ros_msg.data.at(i)<<endl;
                        if(last_msgs.at(i)!=(int)can_sub_ros_msg.data.at(i))
                        {
                          samemsgs=false;

                        }

                      }
                    }
                    else
                    {
                      samemsgs=false;
                    }

                    if(hadmsg==true && samemsgs==false)
                    {
                      /*Sent the data from topic:/Single_Action_Can*/
                      CANalystii_send_msg(send, can_sub_ros_msg.data.at(0),can_sub_ros_msg.data.at(1),
                                                can_sub_ros_msg.data.at(2),can_sub_ros_msg.data.at(3),can_sub_ros_msg.data.at(4),can_sub_ros_msg.data.at(5),
                                                can_sub_ros_msg.data.at(6),can_sub_ros_msg.data.at(7),can_sub_ros_msg.data.at(8),can_sub_ros_msg.data.at(9));

                      last_msgs.clear();
                      for(int i=0;i<can_sub_ros_msg.data.size();i++)
                      {
                        last_msgs.push_back(can_sub_ros_msg.data.at(i));

                      }
                    }

                  }

                }
              counter=0;
            }
          counter++;
          std_msgs::Int16 Error_pub_msg; //***~
          if (Can_bus_start_error == true) {
              Error_pub_msg.data = 1;  // 將Error_pub_msg的數值設為1，表示系統未連接的錯誤碼
              Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
          }
          else
          {
              if(Can_bus_init_error == true)
              {
                  Error_pub_msg.data = 2;  // 將Error_pub_msg的數值設為2，表示系統未init的錯誤碼
                  Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
              }
              else
              {
               if(Can_bus_pass_time_error == true)
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

        ros::spinOnce();//Callback and update the ROS msgs
        r.sleep();
      }
    return 0;
   }
