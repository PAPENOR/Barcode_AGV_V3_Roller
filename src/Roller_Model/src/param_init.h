#ifndef PARAM_INIT_H
#define PARAM_INIT_H

#include <ros/ros.h>
#include <string>
using namespace std;

void Init_param(const ros::NodeHandle nh,const std::string msg_name,double& param);
void Init_param(const ros::NodeHandle nh,const std::string msg_name,bool& param);
void Init_param(const ros::NodeHandle nh,const std::string msg_name,int& param);
void Init_param(ros::NodeHandle nh,const std::string msg_name,string& param);


#endif // PARAM_INIT_H
