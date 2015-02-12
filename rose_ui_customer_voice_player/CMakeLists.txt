cmake_minimum_required(VERSION 2.8.3)
project(rose_ui_customer_voice_player)

set(CMAKE_CXX_FLAGS "-std=c++0x") 

find_package(catkin REQUIRED
	COMPONENTS roscpp ) 
find_package(roscomm REQUIRED)

catkin_package(
	INCLUDE_DIRS 
	LIBRARIES 
	CATKIN_DEPENDS 
	DEPENDS 
)

## Declare a cpp executable
add_executable(rose_ui_customer_voice_player_node src/CustomerVoicePlayer.cpp)

target_link_libraries(rose_ui_customer_voice_player_node 
	${ROS_LIBRARIES} 
	${catkin_LIBRARIES}
)

target_link_libraries(rose_ui_customer_voice_player_node 
	sqlite3 
	xml2 
	asound
	avcodec 
	swscale
)