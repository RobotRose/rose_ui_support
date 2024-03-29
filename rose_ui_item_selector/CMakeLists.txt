cmake_minimum_required(VERSION 2.8.3)
project(rose_ui_item_selector)

set(CMAKE_CXX_FLAGS "-std=c++0x") 

## Find catkin macros and libraries
find_package(catkin REQUIRED 
	COMPONENTS
		actionlib 
		actionlib_msgs 
		genmsg
		geometry_msgs
		luctor_classes
		message_generation
		roscomm
		roscpp 
		rose_common
		rose_datamanager_api
		rospy
		server_multiple_client
)

################################################
## ROS messages, services and actions ##
################################################

add_message_files( DIRECTORY msg
	FILES
	items.msg
	item_selected.msg
	item_selection.msg
)

add_action_files( DIRECTORY action 
	FILES 
    items.action
)

generate_messages( DEPENDENCIES 
	actionlib_msgs 
	roscomm
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE_DIRS 
		include
#  LIBRARIES gui_script_interaction
	CATKIN_DEPENDS 
		actionlib 
		actionlib_msgs 
		genmsg
		geometry_msgs
		luctor_classes
		message_generation
		roscomm
		roscpp 
		rose_common
		rose_datamanager_api
		rospy
		server_multiple_client
	DEPENDS 
)

###########
## Build ##
###########

## Additional locations of header files
include_directories( include ${catkin_INCLUDE_DIRS} )

## cpp executable
add_executable( rose_ui_item_selector 	
							src/item_selector_node.cpp
						   	src/item_selector.cpp
						   	src/selectable_item_table.cpp
						   	src/selectable_item.cpp
)

add_dependencies( rose_ui_item_selector ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( rose_ui_item_selector
	${catkin_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
