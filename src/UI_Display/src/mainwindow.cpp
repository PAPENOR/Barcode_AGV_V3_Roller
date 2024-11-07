#include "ros/ros.h"
#include "std_msgs/Int64MultiArray.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QDebug"
#include "QTimer"
using namespace std;
MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  auto palette=ui->Z_position->palette();
  palette.setColor(palette.WindowText,Qt::blue);
  ui->Z_position->setPalette(palette);

  QTimer *timer1 =new QTimer(this);
  connect(timer1,SIGNAL(timeout()),this,SLOT(MyTimerSlot()));
  timer1->start(100);
  cout<<"Ui launching"<<endl;
}
MainWindow::~MainWindow()
{

  delete ui;
}
void MainWindow::XZ_state_Callback(const std_msgs::Int64MultiArray& Msg)
{
  if(Msg.data.size()==3)
  {
    Z_position_mm=Msg.data.at(2);

  }
}
 void MainWindow::Can_state_Callback(const std_msgs::Int16::ConstPtr& msg)
 {
    can_state_error = msg->data;
    ui->Can_state_ERROR->setText(QString::number(can_state_error));
 }
 void MainWindow::Can_tran_Callback(const std_msgs::Int16::ConstPtr& msg)
 {
    can_tran_error = msg->data;
    ui->Can_tran_ERROR->setText(QString::number(can_tran_error));
 }
 void MainWindow::Hand_Model_Callback(const std_msgs::Int16::ConstPtr& msg)
 {
    hand_model_error = msg->data;
    ui->Hand_Model_ERROR->setText(QString::number(hand_model_error));
 }
 void MainWindow::Tcpip_Lora_Callback(const std_msgs::Int16::ConstPtr& msg)
 {
    tcpip_lora_error = msg->data;
    ui->Tcpip_LORA_ERROR->setText(QString::number(tcpip_lora_error));
 }
 void MainWindow::Tcpip_Mobus_Callback(const std_msgs::Int16::ConstPtr& msg)
 {
    tcpip_mobus_error = msg->data;
    ui->Tcpip_Mobus_ERROR->setText(QString::number(tcpip_lora_error));
 }
void MainWindow::MyTimerSlot()
{
  cout<<Z_position_mm<<endl;
          double a = -1958.4;
          double b = 742989;
          double c = 205488;
          double x = 10*(-b + std::sqrt(b*b-4*a*(c-Z_position_mm)))/(2*a);
  cout<<x<<endl;
  ui->Z_position->display(x);
  ros::spinOnce();
}

void MainWindow::Play_Initial()
{
  std_msgs::Bool stop_msg;
  stop_msg.data=false;
  Stop_x_pub.publish(stop_msg);
  std_msgs::Int64MultiArray Command_msg;
  Command_msg.data.push_back(3);
  Command_msg.data.push_back(0);
  Mission_pub.publish(Command_msg);
  cout<<"Initial"<<endl;
  ros::spinOnce();
}
void MainWindow::Right_in()
{
  std_msgs::Int64MultiArray Command_msg;
  Command_msg.data.push_back(5);
  Command_msg.data.push_back(0);
  Mission_pub.publish(Command_msg);
  cout<<"Right_in"<<endl;
  ros::spinOnce();
}
void MainWindow::Right_out()
{
  std_msgs::Int64MultiArray Command_msg;
  Command_msg.data.push_back(7);
  Command_msg.data.push_back(0);
  Mission_pub.publish(Command_msg);
  cout<<"Right_out"<<endl;
  ros::spinOnce();
}
void MainWindow::Left_in()
{
  std_msgs::Int64MultiArray Command_msg;
  Command_msg.data.push_back(4);
  Command_msg.data.push_back(0);
  Mission_pub.publish(Command_msg);
  cout<<"Left_in"<<endl;
  ros::spinOnce();
}
void MainWindow::Left_out()
{
  std_msgs::Int64MultiArray Command_msg;
  Command_msg.data.push_back(6);
  Command_msg.data.push_back(0);
  Mission_pub.publish(Command_msg);
  cout<<"Left_out"<<endl;
  ros::spinOnce();
}
void MainWindow::Z_GO()
{ std_msgs::Bool stop_msg;
  stop_msg.data=false;
  Stop_x_pub.publish(stop_msg);
  std_msgs::Int64MultiArray Command_msg;
  Command_msg.data.push_back(1);
  Command_msg.data.push_back(Z_command);
  Mission_pub.publish(Command_msg);
  cout<<"Z_move"<<endl;
  ros::spinOnce();
}
void MainWindow::Stop_x()
{
  std_msgs::Bool Command_msg;
  Command_msg.data=true;
  Stop_x_pub.publish(Command_msg);
  cout<<"Stop_x"<<endl;
  ros::spinOnce();
}
void MainWindow::Z_size(int Z_value)
{
Z_command=Z_value;
cout<<"Z_value"<<Z_value<<endl;

}
void MainWindow::Move_stop()
{
  std_msgs::Int64MultiArray Command_msg;
  Command_msg.data.push_back(0);
  Command_msg.data.push_back(0);
  Command_msg.data.push_back(0);
  Mission_pub.publish(Command_msg);
  cout<<"Move_stop"<<endl;
  ros::spinOnce();
}
void MainWindow::Up_Speed()
{
  std_msgs::Int64MultiArray Command_msg;
  Command_msg.data.push_back(0);
  Command_msg.data.push_back(0);
  Command_msg.data.push_back(10);
  Mission_pub.publish(Command_msg);
  cout<<"Move_Up"<<endl;
  ros::spinOnce();

}
void MainWindow::Down_Speed()
{
  std_msgs::Int64MultiArray Command_msg;
  Command_msg.data.push_back(0);
  Command_msg.data.push_back(0);
  Command_msg.data.push_back(-10);
  Mission_pub.publish(Command_msg);
  cout<<"Move_Down"<<endl;
  ros::spinOnce();

}
void MainWindow::Left_Speed()
{
  std_msgs::Int64MultiArray Command_msg;
  Command_msg.data.push_back(0);
  Command_msg.data.push_back(-20);
  Command_msg.data.push_back(0);
  Mission_pub.publish(Command_msg);
  cout<<"Move_Left"<<endl;
  ros::spinOnce();

}
void MainWindow::Right_Speed()
{
  std_msgs::Int64MultiArray Command_msg;
  Command_msg.data.push_back(0);
  Command_msg.data.push_back(20);
  Command_msg.data.push_back(0);
  Mission_pub.publish(Command_msg);
  cout<<"Move_Right"<<endl;
  ros::spinOnce();

}
void MainWindow::Right_Stick_On()
{
  std_msgs::Bool Command_msg;
  Command_msg.data=true;
  Right_Stick_pub.publish(Command_msg);
  cout<<"Right_Stick_On"<<endl;
  ros::spinOnce();
}
void MainWindow::Right_Stick_Off()
{
  std_msgs::Bool Command_msg;
  Command_msg.data=false;
  Right_Stick_pub.publish(Command_msg);
  cout<<"Right_Stick_Off"<<endl;
  ros::spinOnce();
}
void MainWindow::Left_Stick_On()
{
  std_msgs::Bool Command_msg;
  Command_msg.data=true;
  Left_Stick_pub.publish(Command_msg);
  cout<<"Left_Stick_On"<<endl;
  ros::spinOnce();
}
void MainWindow::Left_Stick_Off()
{
  std_msgs::Bool Command_msg;
  Command_msg.data=false;
  Left_Stick_pub.publish(Command_msg);
  cout<<"Left_Stick_Off"<<endl;
  ros::spinOnce();
}
