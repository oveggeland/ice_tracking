cmake_minimum_required(VERSION 3.10)
project(icetrack)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries ##
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  sensor_msgs
  geometry_msgs
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
  ${catkin_INCLUDE_DIRS}
)

## Define navigation library ##
file(GLOB NAVIGATION_SOURCES 
  "src/navigation/*.cpp"
)
add_library(navlib 
  ${NAVIGATION_SOURCES}
)
target_link_libraries(navlib 
  gtsam 
  gtsam_unstable 
  proj
  yaml-cpp
  Open3D::Open3D
  ${catkin_LIBRARIES}
)

## Define mapping library ##
file(GLOB MAPPING_SOURCES 
  "src/mapping/*.cpp"
)
add_library(maplib 
  ${MAPPING_SOURCES}
)
target_link_libraries(maplib 
  gtsam 
  gtsam_unstable 
  proj
  yaml-cpp
  Open3D::Open3D
  ${catkin_LIBRARIES}
)

## Executables ##
add_executable(navigation_node src/navigation_node.cpp)
add_executable(mapping_node src/mapping_node.cpp)
add_executable(rosbag_node src/rosbag_node.cpp)

## Link executables ##
target_link_libraries(navigation_node
  ${catkin_LIBRARIES}
  navlib
)

target_link_libraries(mapping_node
  ${catkin_LIBRARIES}
  maplib
)

target_link_libraries(rosbag_node
  ${catkin_LIBRARIES}
  navlib
  maplib
  -lprofiler
)

# Python stuff
catkin_install_python(PROGRAMS
    scripts/diag_analysis
    scripts/stats_analysis
    scripts/nav_analysis
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)