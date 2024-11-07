cmake_minimum_required(VERSION 3.0.2)
project(Task_Manager)
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
include_directories(/usr/local/include/yaml-cpp)
FIND_LIBRARY(YAML_CPP_LIBRARIES yaml-cpp)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs # Or other packages containing msgs
)
catkin_package(
    CATKIN_DEPENDS actionlib_msgs geometry_msgs roscpp rospy std_msgs
)
add_executable          (Task_Manager src/Task_Manager.cpp src/param_init.cpp)
target_link_libraries   (Task_Manager ${catkin_LIBRARIES})
target_link_libraries   (Task_Manager ${YAML_CPP_LIBRARIES})
add_dependencies        (Task_Manager ${Task_Manager_EXPORTED_TARGETS})
