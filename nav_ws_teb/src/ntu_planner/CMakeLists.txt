cmake_minimum_required(VERSION 3.0.2)
project(ntu_planner)

## 设置 C++ 标准
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf
  tf2
  tf2_ros
  message_generation
  mbf_costmap_core
  mbf_msgs
  pluginlib
  visualization_msgs
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
  INCLUDE_DIRS include
  LIBRARIES ntu_planner
  CATKIN_DEPENDS 
    message_runtime
    mbf_costmap_core
    mbf_msgs
    pluginlib
    costmap_2d
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    visualization_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 编译自定义 local planner 库
add_library(ntu_planner
  src/ntu_local_planner.cpp
)

add_dependencies(ntu_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ntu_planner
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## 安装库
install(TARGETS ntu_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## 安装插件描述文件
install(FILES
  custom_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## 安装配置文件
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

## 安装 Python 脚本
catkin_install_python(PROGRAMS
  srcipts/reload_planning_controller.py
  srcipts/reload_planning_simulator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
