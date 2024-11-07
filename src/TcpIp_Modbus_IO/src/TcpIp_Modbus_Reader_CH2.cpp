/*
這是一個使用ROS框架和Modbus通訊協議的程式，用於從PLC設備中讀取和寫入多個連續的16位元的整數值。
程式設計師可以在ROS參數伺服器中設置PLC設備的IP地址、埠號、需要讀取和寫入的註冊地址。
當程序運行時，它連接到指定的PLC設備，並以指定的頻率進行循環查詢，從PLC設備讀取指定的註冊地址中的整數值。
程式還具有一個訂閱器，用於從ROS主題接收寫入指定註冊地址的整數值。
程式可以將讀取的整數值發佈到ROS主題，以便其他程式可以使用它們。
*/
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <modbus/modbus.h>
#include <string.h>
#include <std_msgs/Int16.h>
using namespace std;
class plc_modbus_manager {
public:
    plc_modbus_manager();

private:
    ros::NodeHandle Node;
    ros::Publisher Regs_Read;
    ros::Publisher Error_pub;
    ros::Subscriber Regs_Write;
    std::vector<int> Regs_Addr;
    std::vector<int> Regs_Addr_Skip;
    std_msgs::UInt16MultiArray Regs_Val;
    modbus_t *Plc;
    std::string Ip_Address;
    bool Link_to_network=true;
    bool Read_error=false;
    bool Write_error=false;
    int Port;
    int Spin_Rate;

    void regs_callBack(const std_msgs::UInt16MultiArray::ConstPtr &Regs_Data);
};


plc_modbus_manager::plc_modbus_manager() {

    // 設定 ROS Publisher 和 Subscriber
    stringstream ICPIP_modbus_error_name;
    ICPIP_modbus_error_name<<ros::this_node::getName()<<"/Error_Code_ICPIP_modbus";
    Error_pub =Node.advertise<std_msgs::Int16>(ICPIP_modbus_error_name.str(), 10);
    Regs_Read = Node.advertise<std_msgs::UInt16MultiArray>("modbus/hand_regs_read_ch2", 100);
    Regs_Write = Node.subscribe<std_msgs::UInt16MultiArray>("modbus/hand_regs_write_ch2", 100,
                                                            &plc_modbus_manager::regs_callBack, this);

    // 讀取 ROS 參數
    Node.param<std::string>("TcpIp_Modbus_Reader_Roller_CH2/ip", Ip_Address, "192.168.1.120");
    Node.param("TcpIp_Modbus_Reader_Roller_CH2/port", Port, 502);
    Node.param("TcpIp_Modbus_Reader_Roller_CH2/spin_rate", Spin_Rate, 30);

    // 讀取要讀取的 Modbus Register Address 和要跳過的 Address
    if (!Node.getParam("TcpIp_Modbus_Reader/regs_addr_read", Regs_Addr)) {
        ROS_WARN("No reg addrs given!");
    }
    if (!Node.getParam("TcpIp_Modbus_Reader/regs_addr_write_skip", Regs_Addr_Skip)) {
        ROS_WARN("No regs_addr_skip given!");
    }

    // 連線到 Modbus 設備
    ROS_INFO("Connecting to modbus device on %s/%d", Ip_Address.c_str(), Port);
    Plc = modbus_new_tcp(Ip_Address.c_str(), Port);
    if (Plc == NULL) {
        ROS_FATAL("Unable to allocate libmodbus context\n");
        return;
    }
    if (modbus_connect(Plc) == -1) {
        ROS_FATAL("Failed to connect to modbus device!!!");
        ROS_FATAL("%s", modbus_strerror(errno));
        modbus_free(Plc);
        return;
    } else {
        ROS_INFO("Connection to modbus device established");
    }

    // 設定迴圈間隔時間
    ros::Rate Loop_Rate(Spin_Rate);

    // 不斷讀取 Modbus Register 的值
    while (ros::ok()) {
        Regs_Val.data.clear();
        for (int i = 0; i < Regs_Addr.size(); i++) {
            uint16_t Temp[1] = {0};
            if (modbus_read_registers(Plc, Regs_Addr.at(i), 1, Temp) == -1) {
                ROS_ERROR("Unable to read reg addr:%d", Regs_Addr.at(i));
                ROS_ERROR("%s", modbus_strerror(errno));
            } else {
                Regs_Val.data.push_back(Temp[0]);
            }
        }
        if (Regs_Val.data.size() > 0) {
            Regs_Read.publish(Regs_Val);
        }
        std_msgs::Int16 Error_pub_msg;
        if (Link_to_network == false)
        {
            Error_pub_msg.data = 1;  // 將Error_pub_msg的數值設為1，表示Socket未登入的錯誤碼
            Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
        }
        else
        {
            // 如果Socket已登入，繼續檢查系統是否連接
            if (Write_error == true) {
                Error_pub_msg.data = 2;  // 將Error_pub_msg的數值設為2，表示系統未連接的錯誤碼
                Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
            }
            else
            {
                if (Read_error == true) {
                    Error_pub_msg.data = 3;  // 將Error_pub_msg的數值設為2，表示系統未連接的錯誤碼
                    Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
                }
                else
                {
                    Error_pub_msg.data = 0;  // 將Error_pub_msg的數值設為0，表示系統未連接的錯誤碼
                    Error_pub.publish(Error_pub_msg);  // 發佈Error_pub_msg到Error_pub主題
                }

            }
        }
        ros::spinOnce();
        Loop_Rate.sleep();
    }

    // 關閉 Modbus 連線
    modbus_close(Plc);
    modbus_free(Plc);
    return;
}

void plc_modbus_manager::regs_callBack(const std_msgs::UInt16MultiArray::ConstPtr &Regs_Data)
{
    // 檢查傳入的資料是否正確
    if (Regs_Data->data.size() != Regs_Addr.size()) {
//        ROS_ERROR("%d registers to write but only %d given!", Regs_Addr.size(), Regs_Data->data.size());
        return;
    }

    // 逐一寫入資料
    for (int i = 0; i < Regs_Data->data.size(); i++) {
        bool Had_Skip = false;

        // 檢查此地址是否被設定為跳過寫入
        for(int j = 0; j < Regs_Addr_Skip.size(); j++) {
            if (Regs_Addr.at(i) == Regs_Addr_Skip.at(j)) {
                Had_Skip = true;
            }
        }

        // 若未被設定為跳過寫入，則進行寫入
        if (Had_Skip == false) {
            ROS_DEBUG("Regs_Out[%d]:%u", i, Regs_Data->data.at(i));
            uint16_t Temp[1] = {Regs_Data->data.at(i)};

            // 執行寫入動作
            if (modbus_write_registers(Plc, Regs_Addr.at(i), 1, Temp) == -1) {
                ROS_ERROR("Modbus reg write failed at addr:%d with value:%u", Regs_Addr.at(i), Regs_Data->data.at(i));
                ROS_ERROR("%s", modbus_strerror(errno));
            } else {
//                ROS_INFO("Modbus register write at addr:%d with value:%u", Regs_Addr.at(i), Regs_Data->data.at(i));
            }
        }
    }
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "TcpIp_Modbus_Reader_CH2");
    ROS_INFO("TcpIp_Modbus_Reader System CH2");
    plc_modbus_manager mm;
    return 0;
}
