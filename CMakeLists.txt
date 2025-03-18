cmake_minimum_required(VERSION 3.10)
project(icetrack_mapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries ##
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  sensor_msgs
  geometry_msgs
  tf2_ros
  std_msgs
)

## Find packages ##
find_package(GTSAM REQUIRED)
find_package(Open3D REQUIRED)
find_package(PROJ REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

## Include directories ##
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fixed_lag_mapping
  ${catkin_INCLUDE_DIRS}
)

## Define navigation library ##
file(GLOB ICETRACK_SOURCES 
  "src/fixed_lag_mapping/*.cpp"
  "src/fixed_lag_mapping/frontend/*.cpp"
  "src/fixed_lag_mapping/backend/*.cpp"
  "src/fixed_lag_mapping/utils/*.cpp"
)
add_library(licetrack 
  ${ICETRACK_SOURCES}
)
target_link_libraries(licetrack 
  gtsam 
  gtsam_unstable 
  proj
  yaml-cpp
  Open3D::Open3D
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}  # Link OpenCV libraries
)

## Executables ##
add_executable(mapping_node src/mapping_node.cpp)
add_executable(rosbag_mapping_node src/rosbag_mapping_node.cpp)

## Link executables ##
target_link_libraries(mapping_node
  ${catkin_LIBRARIES}
  licetrack
)

target_link_libraries(rosbag_mapping_node
  ${catkin_LIBRARIES}
  licetrack
  -lprofiler
)
