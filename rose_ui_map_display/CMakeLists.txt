cmake_minimum_required(VERSION 2.8.3)
project(rose_ui_map_display)

set(CMAKE_CXX_FLAGS "-std=c++0x") 

find_package(catkin REQUIRED
	COMPONENTS
	factory
	genmsg
	geometry_msgs
	luctor_classes
	message_generation
	roscpp 
	rose_conversions
	rose_datamanager_api
	rose_parameter_manager
	rose_transformations
	server_multiple_client
	std_msgs
)

add_message_files( 
	FILES
	selection.msg
	colored_polygon_stamped.msg
	waypoint.msg
	waypoint_array.msg
)

generate_messages(
	DEPENDENCIES 
	std_msgs 
	geometry_msgs
)

catkin_package(
   	INCLUDE_DIRS 
   		include
  	LIBRARIES
    CATKIN_DEPENDS 
    	factory 
    	geometry_msgs 
    	luctor_classes 
    	rose_datamanager_api 
    	rose_parameter_manager
    	rose_transformations
    	server_multiple_client
    	std_msgs 
	DEPENDS 
)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( rose_ui_map_display 	src/map_display_node.cpp
								src/map_display.cpp
)

add_dependencies( rose_ui_map_display ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( rose_ui_map_display ${catkin_LIBRARIES} )
