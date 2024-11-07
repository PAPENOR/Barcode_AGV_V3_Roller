#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "ros/ros.h"
#include <QMainWindow>
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
namespace Ui {
class MainWindow;
}
class MainWindow : public QMainWindow
{
  Q_OBJECT
protected:
/*Define the topic type*/
ros::NodeHandle   nh_;
int               Z_command;
double            Z_position_mm;
double            can_state_error;
double            can_tran_error;
double            hand_model_error;
double            tcpip_lora_error;
double            tcpip_mobus_error;
ros::Publisher    Mission_pub   =nh_.advertise<std_msgs::Int64MultiArray>("/XZ_action", 10);
ros::Publisher  Left_Stick_pub  =nh_.advertise<std_msgs::Bool>("/hand/Left_Stick_action",10);
ros::Publisher  Right_Stick_pub =nh_.advertise<std_msgs::Bool>("/hand/Right_Stick_action",10);
ros::Publisher    Stop_x_pub    =nh_.advertise<std_msgs::Bool>("Hand_Stop_Command",10);
ros::Subscriber XZ_state_sub    =nh_.subscribe("/Z_state",10,&MainWindow::XZ_state_Callback,this);
ros::Subscriber Can_state_sub    =nh_.subscribe("/Can_State_Hand/Error_Code_Can_state",10,&MainWindow::Can_state_Callback,this);
ros::Subscriber Can_tran_sub    =nh_.subscribe("//Can_Tran_Hand/Error_Code_Can_tran",10,&MainWindow::Can_tran_Callback,this);
ros::Subscriber Hamd_Model_sub    =nh_.subscribe("/Hand_Model_Motec/Error_Code_Rollor_model",10,&MainWindow::Hand_Model_Callback,this);
ros::Subscriber Tcpip_Lora_sub    =nh_.subscribe("/TcpIP_Lora_Server/Error_Code_Tcpip_Server",10,&MainWindow::Tcpip_Lora_Callback,this);
ros::Subscriber Tcpip_Mobus_sub    =nh_.subscribe("/TcpIp_Modbus_Reader/Error_Code_ICPIP_modbus",10,&MainWindow::Tcpip_Mobus_Callback,this);
public:
  explicit MainWindow(QWidget *parent = nullptr);  
  void XZ_state_Callback(const std_msgs::Int64MultiArray& Msg);
  void Can_state_Callback(const std_msgs::Int16::ConstPtr& Msg);
  void Can_tran_Callback(const std_msgs::Int16::ConstPtr& Msg);
  void Hand_Model_Callback(const std_msgs::Int16::ConstPtr& Msg);
  void Tcpip_Lora_Callback(const std_msgs::Int16::ConstPtr& Msg);
  void Tcpip_Mobus_Callback(const std_msgs::Int16::ConstPtr& Msg);
  ~MainWindow();
public slots:
  void Play_Initial();
  void Right_in();
  void Right_out();
  void Left_in();
  void Left_out();
  void Stop_x();
  void Z_size(int Z_value);
  void Z_GO();
  void Move_stop();
  void Up_Speed();
  void Down_Speed();
  void Left_Speed();
  void Right_Speed();
  void Right_Stick_On();
  void Right_Stick_Off();
  void Left_Stick_On();
  void Left_Stick_Off();
  void MyTimerSlot();
private:
  Ui::MainWindow *ui;

};

#endif // MAINWINDOW_H
