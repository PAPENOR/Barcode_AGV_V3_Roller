cmake_minimum_required(VERSION 3.0.2)
project(TcpIP_server)
find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    message_generation
    geometry_msgs
    roscpp
    rospy
    std_msgs
    sensor_msgs
    tf
    nav_msgs)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs # Or other packages containing msgs
)
catkin_package(
    CATKIN_DEPENDS actionlib_msgs geometry_msgs roscpp rospy std_msgs
)
add_executable(TcpIP_Lora_Server src/tcpip_server.cpp src/param_init.cpp src/tcpip_server_callback.cpp src/tcpip_server_calculate.cpp)
target_link_libraries(TcpIP_Lora_Server ${catkin_LIBRARIES})
add_dependencies(TcpIP_Lora_Server ${TcpIP_server_EXPORTED_TARGETS})
