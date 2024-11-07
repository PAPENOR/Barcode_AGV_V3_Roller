cmake_minimum_required(VERSION 3.0.2)
project(Roller_Model)
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
    nav_msgs
    Roller_Motor_Can)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
#add_action_files(
#  DIRECTORY action
#  FILES Single_Action_Can.action
#)
#FILE(GLOB messages_to_build RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
#    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")
#add_message_files(
#  FILES
#  ${messages_to_build}
#)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs # Or other packages containing msgs
)
catkin_package(
    CATKIN_DEPENDS actionlib_msgs geometry_msgs roscpp rospy std_msgs
)
add_executable(roller_model_motec src/roller_model_motec.cpp src/param_init.cpp src/roller_model_callback.cpp src/roller_model_function.cpp src/rollor_model_right_function.cpp src/rollor_model_left_function.cpp)
target_link_libraries(roller_model_motec ${catkin_LIBRARIES})
add_dependencies(roller_model_motec ${Roller_Model_EXPORTED_TARGETS})

add_executable(roller_odom src/roller_odom.cpp src/param_init.cpp)
target_link_libraries(roller_odom ${catkin_LIBRARIES})
add_dependencies(roller_odom ${Roller_Model_EXPORTED_TARGETS})
