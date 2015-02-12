cmake_minimum_required(VERSION 2.8.3)
project(rose_ui_item_collector)

set(CMAKE_CXX_FLAGS "-std=c++0x") 

find_package(catkin REQUIRED
	COMPONENTS
	actionlib 
	actionlib_msgs
	genmsg
	geometry_msgs
	gui_item_selector
	message_generation
	roscomm
	roscpp 
	rose_common
	rospy
	server_multiple_client	
)

add_action_files( DIRECTORY action 
	FILES 
    get_items.action
)

generate_messages( DEPENDENCIES 
	actionlib_msgs 
	roscomm
)

catkin_package(
	INCLUDE_DIRS include
#  LIBRARIES gui_script_interaction
    CATKIN_DEPENDS 
    	gui_item_selector
    	rose_common 
    	server_multiple_client
	DEPENDS 
		actionlib
		actionlib_msgs 
		message_generation 
		message_runtime
)

###########
## Build ##
###########

## Additional locations of header files
include_directories( include ${catkin_INCLUDE_DIRS} )

## cpp executable
add_executable( rose_ui_item_collector 	
					src/item_collector_node.cpp
					src/item_collector.cpp
)

add_dependencies( rose_ui_item_collector ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( rose_ui_item_collector ${catkin_LIBRARIES})