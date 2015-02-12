cmake_minimum_required(VERSION 2.8.3)
project(gui_manual_platform_control)

set(CMAKE_CXX_FLAGS "-std=c++0x") 

find_package(catkin REQUIRED
	geometry_msgs
	std_msgs
	roscpp
	shared_variables
)

catkin_package(
   	INCLUDE_DIRS include
#  	LIBRARIES gui_overview_camera
#	CATKIN_DEPENDS rose_datamanager_api factory luctor_classes 
	DEPENDS geometry_msgs std_msgs roscpp
)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( manual_platform_control src/manual_platform_control_node.cpp
										src/manual_platform_control.cpp
)

add_dependencies( manual_platform_control ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME})

target_link_libraries( manual_platform_control ${catkin_LIBRARIES} )