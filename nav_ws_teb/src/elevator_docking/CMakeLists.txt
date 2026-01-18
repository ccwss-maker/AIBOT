cmake_minimum_required(VERSION 3.0.2)
project(elevator_docking)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin and dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  laser_geometry
  pcl_ros
  pcl_conversions
  dynamic_reconfigure
  actionlib_msgs
  actionlib
  message_generation
)

## Find PCL
find_package(PCL REQUIRED)

## Generate actions
add_action_files(
  DIRECTORY action
  FILES ElevatorDocking.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

## Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(
  cfg/ElevatorDetection.cfg
  cfg/DockingControl.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES elevator_docking
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_sensor_msgs
    tf2_geometry_msgs
    laser_geometry
    pcl_ros
    pcl_conversions
    dynamic_reconfigure
    actionlib_msgs
    actionlib
    message_runtime
  DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/elevator_detector.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Declare C++ library for docking server
add_library(${PROJECT_NAME}_server
  src/elevator_docking_server.cpp
)

add_dependencies(${PROJECT_NAME}_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME}_server
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Declare a C++ executable for docking server node
add_executable(elevator_docking_server_node
  src/elevator_docking_server_node.cpp
)

add_dependencies(elevator_docking_server_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(elevator_docking_server_node
  ${PROJECT_NAME}_server
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS elevator_docking_server_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark config files for installation
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
