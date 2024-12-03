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
file(GLOB NAVIGATION_SOURCES "src/navigation/*.cpp")
add_library(navlib 
  ${NAVIGATION_SOURCES}
)
target_include_directories(navlib PUBLIC include/navigation)
target_link_libraries(navlib 
  gtsam 
  gtsam_unstable 
  proj
  ${catkin_LIBRARIES}
)

file(GLOB MAPPING_SOURCES "src/mapping/*.cpp")
add_library(maplib 
  ${MAPPING_SOURCES}
)
target_include_directories(maplib PUBLIC include/mapping)
target_link_libraries(navlib 
  ${catkin_LIBRARIES}
)

add_executable(rosbag_tracking_node src/main_rosbag.cpp src/icetrack.cpp)

## Link libraries ##
target_link_libraries(rosbag_tracking_node
  ${catkin_LIBRARIES}
  navlib
  maplib
)