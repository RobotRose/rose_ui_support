cmake_minimum_required(VERSION 2.8.3)
project(rose_ui_connection_monitor)

set(CMAKE_CXX_FLAGS "-std=c++0x") 

find_package(catkin REQUIRED
	std_msgs
	roscpp
)

catkin_package(
   	INCLUDE_DIRS 
   		include
  	LIBRARIES
	CATKIN_DEPENDS 
	DEPENDS 
		std_msgs 
		roscpp
)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( rose_ui_connection_monitor src/connection_monitor_node.cpp)

add_dependencies( rose_ui_connection_monitor ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME})

target_link_libraries( rose_ui_connection_monitor ${catkin_LIBRARIES} )