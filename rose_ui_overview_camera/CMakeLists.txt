cmake_minimum_required(VERSION 2.8.3)
project(rose_ui_overview_camera)

set(CMAKE_CXX_FLAGS "-std=c++0x") 

find_package(catkin REQUIRED
	arm_controller
	factory
	genmsg
	luctor_classes
	operator_messaging
	roscpp 
	rose_common
	rose_datamanager_api
	rose_parameter_manager
)

add_message_files( DIRECTORY msg
	FILES
	selection.msg
	selections.msg
)

generate_messages( 

)

catkin_package(
   	INCLUDE_DIRS 
   		include
	LIBRARIES
	CATKIN_DEPENDS 
		arm_controller 
		factory 
		luctor_classes 
		message_generation
		operator_messaging
		rose_common
		rose_datamanager_api 
		rose_parameter_manager
	DEPENDS 
)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( rose_ui_overview_camera 	src/overview_camera_node.cpp
									src/overview_camera.cpp
)

add_dependencies( rose_ui_overview_camera ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( rose_ui_overview_camera ${catkin_LIBRARIES} )