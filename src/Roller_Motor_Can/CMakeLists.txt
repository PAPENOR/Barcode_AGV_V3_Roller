cmake_minimum_required(VERSION 3.0.2)
project(Roller_Motor_Can)
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
add_action_files(
  DIRECTORY action
  FILES Single_Action_Can.action
)
FILE(GLOB messages_to_build RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")
add_message_files(
  FILES
  ${messages_to_build}
)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs # Or other packages containing msgs
)
catkin_package(
    CATKIN_DEPENDS actionlib_msgs geometry_msgs roscpp rospy std_msgs
)
add_executable(Can_Tran_Hand src/Can_Tran_Hand.cpp src/canalystii.cpp src/canalystii_node.cpp src/param_init.cpp)
target_link_libraries(Can_Tran_Hand ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so ${catkin_LIBRARIES})
add_dependencies(Can_Tran_Hand ${Roller_Motor_Can_EXPORTED_TARGETS})

add_executable(Roller_Action_Can src/Roller_Action_Can.cpp src/param_init.cpp src/Roller_Action_Coder.cpp src/roller_action_callback.cpp)
target_link_libraries(Roller_Action_Can ${catkin_LIBRARIES})
add_dependencies(Roller_Action_Can ${Roller_Motor_Can_EXPORTED_TARGETS})

add_executable(Roller_State_Can src/roller_state_can.cpp src/param_init.cpp src/roller_state_callback.cpp)
target_link_libraries(Roller_State_Can ${catkin_LIBRARIES})
add_dependencies(Roller_State_Can ${Roller_Motor_Can_EXPORTED_TARGETS})

add_executable(Roller_Can_test src/Roller_Can_test.cpp src/param_init.cpp )
target_link_libraries(Roller_Can_test ${catkin_LIBRARIES})
add_dependencies(Roller_Can_test ${Roller_Motor_Can_EXPORTED_TARGETS})
