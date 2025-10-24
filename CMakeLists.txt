cmake_minimum_required(VERSION 3.10)
# if (POLICY CMP0074)
  # cmake_policy(SET CMP0074 NEW)
# endif (POLICY CMP0074)

project(map_exporter VERSION 1.0.0)

SET(CMAKE_CXX_STANDARD 17)

#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
#=======================================
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O4 -s -Wall -Wno-sign-compare -Wno-unused -Wno-comment -Wno-deprecated-declarations -g -pthread")

#========================
# Dependencies Setup
#========================
#ROS#
find_package(roscpp 1.12 QUIET)
if(roscpp_FOUND)
  message(=============================================================)
  message("-- ROS Found, Ros Support is turned On!")
  message(=============================================================)
  add_definitions(-DROS_FOUND)
  find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    # pcl library
    pcl_conversions
    pcl_ros
  )

  find_package(roslib QUIET)

  catkin_package(
    INCLUDE_DIRS include
    DEPENDS PCL

    CATKIN_DEPENDS
  )

  include_directories(${catkin_INCLUDE_DIRS})
else()
  message(=============================================================)
  message("-- ROS Not Found, Ros Support is turned Off!")
  message(=============================================================)
endif()

find_package(PCL REQUIRED QUIET)
find_package(PDAL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)

# include directories
include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PDAL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# link directories
link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${PDAL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#============================
#  Build Lib, App, Tests
#============================

# Find src files
file(GLOB_RECURSE src_files
      ${PROJECT_SOURCE_DIR}/src/*.cc)

# Compile library
add_library (
    map_exporter
    SHARED
    ${src_files}
)

target_link_libraries(map_exporter
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${PDAL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

#############
##  Nodes  ##
#############

add_executable(map_exporter_node app/map_exporter_node.cc)
target_link_libraries(map_exporter_node  map_exporter)
