cmake_minimum_required(VERSION 2.8.3)
project(rose_ui_manual_arm_control)

set(CMAKE_CXX_FLAGS "-std=c++0x") 

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  arm_controller
  roscomm
  roscpp
  rose_common
  server_multiple_client
  std_msgs
)

catkin_package(
    INCLUDE_DIRS 
        include
    LIBRARIES
    CATKIN_DEPENDS 
        actionlib
        arm_controller 
        roscomm
        rose_common
        server_multiple_client 
        std_msgs
    DEPENDS 
)

include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rose_ui_manual_arm_control src/gui_manual_arm_control_node.cpp  src/gui_manual_arm_control.cpp)

target_link_libraries(rose_ui_manual_arm_control 
    ${ROS_LIBRARIES} 
    ${catkin_LIBRARIES}
)