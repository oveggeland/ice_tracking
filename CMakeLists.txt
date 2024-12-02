cmake_minimum_required(VERSION 3.0.2)
project(icetrack)

set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries ##
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  sensor_msgs
  std_msgs
  topic_tools
)

## Find packages ##
find_package(GTSAM REQUIRED)
find_package(PROJ REQUIRED)

catkin_package()

## Include directories ##
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Define executables ##
add_executable(rosbag_parser_node src/rosbag_parser.cpp)

file(GLOB NAVIGATION_SOURCES "src/navigation/*.cpp")
add_executable(navigation_node ${NAVIGATION_SOURCES})

## Link libraries ##
target_link_libraries(rosbag_parser_node
  ${catkin_LIBRARIES}
)

target_link_libraries(navigation_node
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  proj
)