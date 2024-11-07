cmake_minimum_required(VERSION 3.0.2)
project(System_Good_state)
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
add_executable(system_good_state src/system_good_state.cpp)
target_link_libraries(system_good_state ${catkin_LIBRARIES})
add_dependencies(system_good_state ${System_Good_state_EXPORTED_TARGETS})
