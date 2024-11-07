#ifndef ROLLER_ODOM_H
#define ROLLER_ODOM_H
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "std_msgs/Int64MultiArray.h"
using namespace ros;
using namespace std;
class roller_odom
{
protected:
    double vx = 0.0;
    double vz = 0.0;
    double vth = 0.0;
    double X_Pitch=1;//mm
    double Z_Pitch=1;//mm
    double X_position=0.0;
    double Z_position=0.0;


    ros::NodeHandle nh;
public:

    roller_odom(string Name);
    ~roller_odom();
    void ZCallback(const std_msgs::Int64MultiArray& msg);
};

#endif // ROLLER_ODOM_H
