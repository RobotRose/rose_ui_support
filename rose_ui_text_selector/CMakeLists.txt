cmake_minimum_required(VERSION 2.8.3)
project(rose_ui_text_selector)

set(CMAKE_CXX_FLAGS "-std=c++0x") 

find_package(catkin REQUIRED
	factory
	luctor_classes
	roscpp 
	rose_datamanager_api
	rose_parameter_manager
)

catkin_package(
   	INCLUDE_DIRS 
   		include
  	LIBRARIES
	CATKIN_DEPENDS 
		factory 
		luctor_classes 
		rose_datamanager_api 
		rose_parameter_manager
	DEPENDS
)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( rose_ui_text_selector 	src/text_selector_node.cpp
								src/text_selector.cpp
)

add_dependencies( rose_ui_text_selector ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( rose_ui_text_selector ${catkin_LIBRARIES} )