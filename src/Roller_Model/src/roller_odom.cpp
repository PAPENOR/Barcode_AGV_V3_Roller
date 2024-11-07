#include "roller_odom.h"

roller_odom::roller_odom(string Name)
{

}
void roller_odom::ZCallback(const std_msgs::Int64MultiArray& msg)
{
  float AxisX,AxisZ,VX,VZ;
  if(msg.data.at(0)!=998)
  {
    AxisZ= msg.data.at(2);
    VZ=((AxisZ)*Z_Pitch/(60*64))*0.001*82.76;//m/s
    Z_position=(msg.data.at(4)*0.001*Z_Pitch*82.76*M_PI)/(131072*64)+0.05;
    vz=VZ;

  }
  else
  {
    vz=0;
    vx=0;
  }


}
roller_odom::~roller_odom()
{
    ros::Subscriber sub = nh.subscribe("/Z_state", 10, &roller_odom::ZCallback,this);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom_hand", 100);
    tf::TransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double z = 0.0;
    double th = 0.0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    struct timeval time_now{};
    gettimeofday(&time_now,nullptr);


    time_t begin=time_now.tv_sec*1000+time_now.tv_usec/1000,new_time;
    double start_time=begin;
    double last_timer=0.0;
    ros::Rate r(30.0);
    while(nh.ok())
    {
      ros::spinOnce();               // check for incoming messages
      current_time = ros::Time::now();

      //compute odometry in a typical way given the velocities of the robot
      gettimeofday(&time_now,nullptr);
      new_time = time_now.tv_sec*1000+time_now.tv_usec/1000;
      double dt = ((double)(new_time-last_timer)/1000);
      //cout<<"DT:"<<dt<<endl;
      double delta_x = (0) * dt;
      double delta_z = (vz) * dt;

      x += delta_x;
      z += delta_z;


      //since all odometry is 6DOF we'll need a quaternion created from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom_hand";
      odom_trans.child_frame_id = "base_hand";

      odom_trans.transform.translation.x = 0.0;
      odom_trans.transform.translation.y = 0.0;
      odom_trans.transform.translation.z = Z_position;
      odom_trans.transform.rotation = odom_quat;

      //send the transform
      odom_broadcaster.sendTransform(odom_trans);

      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom_hand";

      //set the position
      odom.pose.pose.position.x = 0.0;
      odom.pose.pose.position.y = 0.0;
      odom.pose.pose.position.z = Z_position;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "base_hand";
      odom.twist.twist.linear.x = 0;
      odom.twist.twist.linear.z = vz;

      //publish the message
      odom_pub.publish(odom);

      last_time = current_time;
      gettimeofday(&time_now,nullptr);
      last_timer = time_now.tv_sec*1000+time_now.tv_usec/1000;
      r.sleep();
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "roller_odom");
    ros::NodeHandle nh;

    ROS_INFO("roller_odom");
    roller_odom roller_odom("roller_odom");
    ros::spin();
    return 0;
}
