#include "ros/ros.h"
#include "mainwindow.h"
#include "std_msgs/Int64MultiArray.h"
#include <QApplication>
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Loader_UI");
  ros::NodeHandle nh;
  ros::Publisher  Mission_pub;

  QApplication a(argc,argv);
  MainWindow w;
  w.showMaximized();
  return a.exec();
}
