#include "param_init.h"
void Init_param(ros::NodeHandle nh,const std::string msg_name,double& param)
{
  if(nh.getParam(msg_name,param))
  {
    ROS_INFO("[%s]: %lf",msg_name.c_str(),param);
  }
  else
  {
    ROS_WARN("[%s]: Param Fail",msg_name.c_str());
  }
}
void Init_param(ros::NodeHandle nh,const std::string msg_name,int& param)
{
  if(nh.getParam(msg_name,param))
  {
    ROS_INFO("[%s]: %d",msg_name.c_str(),param);
  }
  else
  {
    ROS_WARN("[%s]: Param Fail",msg_name.c_str());
  }
}

void Init_param(ros::NodeHandle nh,const std::string msg_name,bool& param)
{
  if(nh.getParam(msg_name,param))
  {
    if(param==true)
    {
    ROS_INFO("[%s]:TRUE",msg_name.c_str());
    }
    else
    {
    ROS_INFO("[%s]:FALSE",msg_name.c_str());
    }
  }
  else
  {
    ROS_WARN("[%s]: Param Fail",msg_name.c_str());
  }
}

void Init_param(ros::NodeHandle nh,const std::string msg_name,string& param)
{
  if(nh.getParam(msg_name,param))
  {
    ROS_INFO("[%s]: %s",msg_name.c_str(),param.c_str());
  }
  else
  {
    ROS_WARN("[%s]: Param Fail",msg_name.c_str());
  }
}

