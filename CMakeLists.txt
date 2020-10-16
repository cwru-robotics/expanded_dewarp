cmake_minimum_required(VERSION 2.8.3)
project(expanded_dewarp)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(dewarp_node src/dewarp_node.cpp)
target_link_libraries(dewarp_node ${OpenCV_LIBS} ${catkin_LIBRARIES})
