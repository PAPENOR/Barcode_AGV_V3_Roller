#ifndef ROLLER_MODEL_FUNCTION_H
#define ROLLER_MODEL_FUNCTION_H
#include <ros/ros.h>
#include "std_msgs/Int16.h"
using namespace ros;
using namespace std;
void Woring_state_pub(int working_state, ros::Publisher &Mission_state_pub);
double Get_the_msec_time_of_computer(struct timeval &time_now);
void publish_arrived_signal(ros::Publisher& publisher);
#endif // ROLLER_MODEL_FUNCTION_H
