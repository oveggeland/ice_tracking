cmake_minimum_required(VERSION 3.10)
project(icetrack)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)


## Define debug macro if assertions are used ##
add_compile_definitions(NDEBUG)

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
find_package(Open3D REQUIRED)
find_package(PROJ REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

## Include directories ##
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Define icetrack library ##
file(GLOB ICETRACK_SOURCES 
  "src/icetrack/*.cpp"
  "src/icetrack/system/*.cpp"
  "src/icetrack/navigation/*.cpp"
  "src/icetrack/*.cpp"
)

add_library(icetracklib 
  ${ICETRACK_SOURCES}
)
target_include_directories(icetracklib
  PUBLIC
    ${catkin_INCLUDE_DIRS}
    include/icetrack
)

target_link_libraries(icetracklib 
  gtsam 
  gtsam_unstable 
  proj
  yaml-cpp
  Open3D::Open3D
  ${catkin_LIBRARIES}
)

## Executables ##
add_executable(rosbag_node src/rosbag_node.cpp)     # Track based on rosbag parsing
add_executable(rostopic_node src/rostopic_node.cpp) # Subscribe and track (real-time)

## Link executables ##
target_link_libraries(rosbag_node
  ${catkin_LIBRARIES}
  icetracklib
  -lprofiler
)
target_link_libraries(rostopic_node
  ${catkin_LIBRARIES}
  icetracklib
)


catkin_install_python(PROGRAMS
    scripts/diag_analysis
    scripts/stats_analysis
    scripts/nav_analysis
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)