cmake_minimum_required(VERSION 3.0.2)
project(me5413_world)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  rviz
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  gazebo_ros
  jsk_rviz_plugins
  jackal_gazebo
  jackal_navigation
  dynamic_reconfigure
)

## Find ACADO
find_package(ACADO REQUIRED)

if(ACADO_FOUND)
    message(STATUS "Found ACADO version: ${ACADO_VERSION}")
else()
    message(FATAL_ERROR "ACADO not found. Please install ACADO.")
endif()

## Generate dynamic reconfigure options
generate_dynamic_reconfigure_options(
  cfg/path_publisher.cfg
  cfg/path_tracker.cfg
)

## Declare the catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES me5413_world
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs dynamic_reconfigure
  DEPENDS system_lib
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${ACADO_INCLUDE_DIRS}
)

## Link directories
link_directories(${ACADO_LIBRARY_DIRS})

## Add Nodes
add_executable(path_publisher_node src/path_publisher_node.cpp)
target_link_libraries(path_publisher_node ${catkin_LIBRARIES})
add_dependencies(path_publisher_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(path_tracker_node src/path_tracker_node.cpp)
target_link_libraries(path_tracker_node ${catkin_LIBRARIES})
add_dependencies(path_tracker_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(path_tracker_MPC_node src/path_tracker_MPC_node.cpp)
target_link_libraries(path_tracker_MPC_node ${catkin_LIBRARIES} ${ACADO_LIBRARIES})
add_dependencies(path_tracker_MPC_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
