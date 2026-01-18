cmake_minimum_required(VERSION 3.0.2)
project(dispatch_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dispatch_control
#  CATKIN_DEPENDS rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/dispatch_node_mbf.py
  scripts/goal_pub_simulator_from_rviz.py
  scripts/goal_pub_simulator_specific_position.py
  scripts/goal_pub_stop_simulator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)