cmake_minimum_required(VERSION 3.10)
project(icetrack)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries ##
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
)

## Find packages ##
find_package(GTSAM REQUIRED)
find_package(Open3D REQUIRED)
find_package(PROJ REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)  # Added OpenCV

catkin_package(
  CATKIN_DEPENDS message_runtime
)

## Include directories ##
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/icetrack
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}  # Include OpenCV headers
)

## Define navigation library ##
file(GLOB ICETRACK_SOURCES 
  "src/icetrack/*.cpp"
  "src/icetrack/frontend/*.cpp"
  "src/icetrack/backend/*.cpp"
  "src/icetrack/utils/*.cpp"
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
add_executable(icetrack_node src/icetrack_node.cpp)
add_executable(rosbag_node src/rosbag_node.cpp)

## Link executables ##
target_link_libraries(icetrack_node
  ${catkin_LIBRARIES}
  licetrack
)

target_link_libraries(rosbag_node
  ${catkin_LIBRARIES}
  licetrack
  -lprofiler
)

# Python stuff
catkin_install_python(PROGRAMS
    scripts/diag_analysis
    scripts/stats_analysis
    scripts/nav_analysis
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
