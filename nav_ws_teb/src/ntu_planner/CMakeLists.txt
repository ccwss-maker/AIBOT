cmake_minimum_required(VERSION 3.0.2)
project(ntu_planner)

## 设置 C++ 标准和编译选项
add_compile_options(-std=c++14)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -O3)
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

## 启用 Fortran 支持
enable_language(Fortran)

## 查找 CUDA 包（使用传统 FindCUDA 模块，兼容 CUDA 12.2）
find_package(CUDA REQUIRED)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-std=c++14)
# CUDA 12.2 支持的架构：sm_50 到 sm_90a
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_75,code=sm_75)  # Turing
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_86,code=sm_86)  # Ampere
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_89,code=sm_89)  # Ada Lovelace
include_directories(${CUDA_INCLUDE_DIRS})

## 查找 ROS 包
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
  dynamic_reconfigure
  base_local_planner
)

## 查找其他依赖
find_package(Eigen3 REQUIRED NO_MODULE)

add_message_files(
  FILES
  ReloadplanningMsg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

## 生成 dynamic_reconfigure 文件
generate_dynamic_reconfigure_options(
  cfg/NTUController.cfg
  cfg/TrajectoryOptimizer.cfg
  cfg/SDFOptimizer.cfg
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
    dynamic_reconfigure
    base_local_planner
)

###########
## Build ##
###########

## 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## 定义 Fortran 源文件
set(FORTRAN_SOURCES
  include/utils/lmbm_call.f
  include/utils/lmbm_main.f
  include/utils/lmbm_math.f
  include/utils/lmbm_sub.f
)

## 编译CUDA文件
cuda_add_library(sdf_cuda_lib
  src/SDF.cu
)

## 编译自定义 local planner 库（包含轨迹优化器和SDF优化器）
add_library(ntu_planner
  src/ntu_local_planner.cpp
  src/trajectory_optimizer.cpp
  src/sdf_optimizer.cpp
  include/utils/lmbm.cpp
  ${FORTRAN_SOURCES}
)

add_dependencies(ntu_planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(ntu_planner
  ${catkin_LIBRARIES}
  ${CUDA_LIBRARIES}
  sdf_cuda_lib
  gfortran
)

## 注释：TrajectoryOptimizationNode 已整合到 ntu_planner 库中
## 不再需要单独的可执行文件
# add_executable(trajectory_optimization_node
#   src/TrajectoryOpimizationNode.cpp
#   include/utils/lmbm.cpp
#   ${FORTRAN_SOURCES}
# )
#
# add_dependencies(trajectory_optimization_node
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
#
# target_link_libraries(trajectory_optimization_node
#   ${catkin_LIBRARIES}
#   cudart
#   gfortran
# )

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
