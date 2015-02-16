cmake_minimum_required(VERSION 2.8.3)
project(rose_ui_script_selector)

set(CMAKE_CXX_FLAGS "-std=c++0x") 

find_package(catkin REQUIRED
	COMPONENTS 
	actionlib 
	actionlib_msgs 
	luctor_classes
	message_runtime
	roscpp 
	rose_datamanager_api
	rose_script_manager
)

add_message_files(DIRECTORY msg
    FILES
    scripts.msg
    script_selected.msg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package(
   	INCLUDE_DIRS 
   		include
  	LIBRARIES
	CATKIN_DEPENDS 
		rose_script_manager 
		rose_datamanager_api 
		luctor_classes 
   	DEPENDS 
   		roscpp
)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( rose_ui_script_selector 	src/script_selector_node.cpp
									src/script_selector.cpp
)

add_dependencies( rose_ui_script_selector ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp rose_script_manager_gencpp )

target_link_libraries( rose_ui_script_selector ${catkin_LIBRARIES} )