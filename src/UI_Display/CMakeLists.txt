cmake_minimum_required(VERSION 3.0.2)
project(UI_Display)
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
find_package(Qt5 REQUIRED COMPONENTS Widgets )
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(SOURCES
    src/ui_node.cpp
    src/mainwindow.cpp
    )
set(FORMS
    src/mainwindow.ui
    )
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs # Or other packages containing msgs
)
catkin_package(
    CATKIN_DEPENDS actionlib_msgs geometry_msgs roscpp rospy std_msgs
)
add_executable(ui_node ${SOURCES} ${FORMS})
target_link_libraries(ui_node Qt5::Widgets)
target_link_libraries(ui_node ${catkin_LIBRARIES})
add_dependencies(ui_node ${UI_Display_EXPORTED_TARGETS})
