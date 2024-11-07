cmake_minimum_required(VERSION 3.0.2)
project(TcpIp_Modbus_IO)
find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    geometry_msgs
    cv_bridge
    image_transport
    genmsg
    sensor_msgs
    tf
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
    CATKIN_DEPENDS geometry_msgs std_msgs
)
link_directories( /usr/local/lib)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(TcpIp_Modbus_Reader_CH1 src/TcpIp_Modbus_Reader_CH1.cpp)
target_link_libraries(TcpIp_Modbus_Reader_CH1
  ${catkin_LIBRARIES}
  modbus
)

add_executable(TcpIp_Modbus_Reader_CH2 src/TcpIp_Modbus_Reader_CH2.cpp)
target_link_libraries(TcpIp_Modbus_Reader_CH2
  ${catkin_LIBRARIES}
  modbus
)
add_executable(TcpIp_IO_Decoder_CH1 src/TcpIp_IO_Decoder_CH1.cpp)
target_link_libraries(TcpIp_IO_Decoder_CH1
  ${catkin_LIBRARIES}
)
add_executable(TcpIp_IO_Decoder_CH2 src/TcpIp_IO_Decoder_CH2.cpp)
target_link_libraries(TcpIp_IO_Decoder_CH2
  ${catkin_LIBRARIES}
)
