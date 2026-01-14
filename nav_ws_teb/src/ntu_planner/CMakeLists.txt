cmake_minimum_required(VERSION 3.0.2)
project(ntu_planner)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  teb_local_planner
  tf
  message_generation
)

add_message_files(
  FILES
  ReloadplanningMsg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
  srcipts/reload_planning_controller.py
  srcipts/reload_planning_simulator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
