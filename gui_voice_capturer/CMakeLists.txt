cmake_minimum_required(VERSION 2.8.3)
project(gui_voice_capturer)

find_package(catkin REQUIRED
	COMPONENTS roscpp ) 
find_package(roscomm REQUIRED)

catkin_package()

add_executable(gui_voice_capturer_node src/VoiceCapturerNode.cpp src/VoiceCapturer.cpp)

target_link_libraries(gui_voice_capturer_node 
	${ROS_LIBRARIES} 
	${catkin_LIBRARIES}
)

target_link_libraries(gui_voice_capturer_node 
	asound
	avcodec 
	swscale
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
