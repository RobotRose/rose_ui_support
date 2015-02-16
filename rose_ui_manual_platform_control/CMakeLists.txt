cmake_minimum_required(VERSION 2.8.3)
project(rose_ui_manual_platform_control)

set(CMAKE_CXX_FLAGS "-std=c++0x") 

find_package(catkin REQUIRED
	geometry_msgs
	std_msgs
	roscpp
	shared_variables
)

catkin_package(
   	INCLUDE_DIRS include
#  	LIBRARIES rose_ui_overview_camera
#	CATKIN_DEPENDS rose_datamanager_api factory luctor_classes 
	DEPENDS geometry_msgs std_msgs roscpp
)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( rose_ui_manual_platform_control src/manual_platform_control_node.cpp
										src/manual_platform_control.cpp
)

add_dependencies( rose_ui_manual_platform_control ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME})

target_link_libraries( rose_ui_manual_platform_control ${catkin_LIBRARIES} )