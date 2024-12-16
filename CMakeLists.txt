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
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)

catkin_package()

## Include directories ##
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Define icetrack library ##
file(GLOB ICETRACK_SOURCES "src/icetrack/*.cpp")
add_library(icetracklib 
  ${ICETRACK_SOURCES}
)
target_include_directories(icetracklib PUBLIC include/icetrack)
target_link_libraries(icetracklib 
  gtsam 
  gtsam_unstable 
  proj
  yaml-cpp
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

## Executables ##
add_executable(rosbag_node src/rosbag_node.cpp)     # Track based on rosbag parsing
add_executable(rostopic_node src/rostopic_node.cpp) # Subscribe and track (real-time)

## Link executables ##
target_link_libraries(rosbag_node
  ${catkin_LIBRARIES}
  icetracklib
)
target_link_libraries(rostopic_node
  ${catkin_LIBRARIES}
  icetracklib
)