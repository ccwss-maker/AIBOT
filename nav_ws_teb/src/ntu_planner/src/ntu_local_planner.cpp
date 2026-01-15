#include "ntu_planner/ntu_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <visualization_msgs/Marker.h>
#include <mbf_msgs/ExePathResult.h>
#include <base_local_planner/goal_functions.h>

// 注册插件到 pluginlib
PLUGINLIB_EXPORT_CLASS(ntu_planner::NTUController, mbf_costmap_core::CostmapController)

namespace ntu_planner
{

NTUController::NTUController()
    : initialized_(false),
      goal_reached_(false),
      current_waypoint_idx_(0),
      optimization_success_(false),
      tf_(nullptr),
      costmap_ros_(nullptr),
      costmap_(nullptr)
{
}

NTUController::~NTUController()
{
}

void NTUController::initialize(std::string name, TF *tf, costmap_2d::Costmap2DROS *costmap_ros)
{
  if (initialized_)
  {
    ROS_WARN("NTUController has already been initialized");
    return;
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  // 创建私有命名空间的 NodeHandle
  ros::NodeHandle private_nh("~/" + name_);
  nh_ = private_nh;

  // 加载参数
  loadParameters();

  // 初始化轨迹优化器
  trajectory_optimizer_.initialize(private_nh);

  // 创建发布器（用于可视化）
  global_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
  // target_point_pub_ = private_nh.advertise<visualization_msgs::Marker>("target_point", 1);

  // 设置 dynamic_reconfigure 服务器 1: NTUController 参数
  dsrv_controller_.reset(new dynamic_reconfigure::Server<ntu_planner::NTUControllerConfig>(private_nh));

  // 先将当前参数值同步到 dynamic_reconfigure
  ntu_planner::NTUControllerConfig controller_config;
  controller_config.max_vel_x = max_vel_x_;
  controller_config.max_vel_theta = max_vel_theta_;
  controller_config.xy_goal_tolerance = xy_goal_tolerance_;
  controller_config.yaw_goal_tolerance = yaw_goal_tolerance_;
  controller_config.lookahead_distance = lookahead_distance_;

  // 更新初始配置（不触发回调）
  dsrv_controller_->updateConfig(controller_config);

  // 设置回调函数
  dynamic_reconfigure::Server<ntu_planner::NTUControllerConfig>::CallbackType cb_controller;
  cb_controller = boost::bind(&NTUController::controllerReconfigureCallback, this, _1, _2);
  dsrv_controller_->setCallback(cb_controller);

  // 设置 dynamic_reconfigure 服务器 2: TrajectoryOptimizer 参数
  ros::NodeHandle optimizer_nh(private_nh, "trajectory_optimizer");
  dsrv_optimizer_.reset(new dynamic_reconfigure::Server<ntu_planner::TrajectoryOptimizerConfig>(optimizer_nh));

  // 从 parameter server 读取 trajectory_optimizer 参数
  ntu_planner::TrajectoryOptimizerConfig optimizer_config;
  private_nh.param("trajectory_optimizer/weight_time", optimizer_config.weight_time, 0.1);
  private_nh.param("trajectory_optimizer/weight_energy_x", optimizer_config.weight_energy_x, 1.0);
  private_nh.param("trajectory_optimizer/weight_energy_y", optimizer_config.weight_energy_y, 1.0);
  private_nh.param("trajectory_optimizer/weight_energy_w", optimizer_config.weight_energy_w, 0.1);
  private_nh.param("trajectory_optimizer/weight_position_x", optimizer_config.weight_position_x, 1.0);
  private_nh.param("trajectory_optimizer/weight_position_y", optimizer_config.weight_position_y, 1.0);
  private_nh.param("trajectory_optimizer/weight_position_w", optimizer_config.weight_position_w, 1.0);
  private_nh.param("trajectory_optimizer/init_time", optimizer_config.init_time, 0.5);
  private_nh.param("trajectory_optimizer/astar_point_interval", optimizer_config.astar_point_interval, 2);
  private_nh.param("trajectory_optimizer/max_iterations", optimizer_config.max_iterations, 1000);
  private_nh.param("trajectory_optimizer/min_step", optimizer_config.min_step, 1e-5);
  private_nh.param("trajectory_optimizer/epsilon", optimizer_config.epsilon, 1e-5);
  private_nh.param("trajectory_optimizer/viz_only_control_points", optimizer_config.viz_only_control_points, true);
  private_nh.param("trajectory_optimizer/viz_time_step", optimizer_config.viz_time_step, 0.2);

  // 更新初始配置（不触发回调）
  dsrv_optimizer_->updateConfig(optimizer_config);

  // 设置回调函数
  dynamic_reconfigure::Server<ntu_planner::TrajectoryOptimizerConfig>::CallbackType cb_optimizer;
  cb_optimizer = boost::bind(&NTUController::optimizerReconfigureCallback, this, _1, _2);
  dsrv_optimizer_->setCallback(cb_optimizer);

  initialized_ = true;
  ROS_INFO("NTUController initialized successfully");
}

void NTUController::loadParameters()
{
  // 使用已经创建的 NodeHandle
  ROS_INFO("Loading parameters from namespace: %s", nh_.getNamespace().c_str());

  // 读取参数，如果没有设置则使用默认值
  nh_.param("max_vel_x", max_vel_x_, 1.0);
  nh_.param("max_vel_theta", max_vel_theta_, 1.0);
  nh_.param("xy_goal_tolerance", xy_goal_tolerance_, 0.2);
  nh_.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.2);
  nh_.param("lookahead_distance", lookahead_distance_, 1.0);

  ROS_INFO("NTUController parameters:");
  ROS_INFO("  max_vel_x: %.2f m/s", max_vel_x_);
  ROS_INFO("  max_vel_theta: %.2f rad/s", max_vel_theta_);
  ROS_INFO("  xy_goal_tolerance: %.2f m", xy_goal_tolerance_);
  ROS_INFO("  yaw_goal_tolerance: %.2f rad", yaw_goal_tolerance_);
  ROS_INFO("  lookahead_distance: %.2f m", lookahead_distance_);
  ROS_INFO("  trajectory_optimization: ENABLED (always)");
}

bool NTUController::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
{
  if (!initialized_)
  {
    ROS_ERROR("NTUController is not initialized yet!");
    return false;
  }

  if (plan.empty())
  {
    ROS_ERROR("Received empty global path!");
    return false;
  }

  // 保存全局路径
  global_plan_ = plan;
  current_waypoint_idx_ = 0;
  goal_reached_ = false;

  ROS_INFO("NTUController received new path with %lu points", plan.size());

  // === 提取局部代价地图范围内的全局路径部分 ===
  std::vector<geometry_msgs::PoseStamped> local_plan;
  geometry_msgs::PoseStamped robot_pose;

  if (!costmap_ros_->getRobotPose(robot_pose))
  {
    ROS_WARN("Cannot get robot pose for transformGlobalPlan, using full global plan");
    local_plan = plan;
  }
  else
  {
    // 使用 transformGlobalPlan 获取局部范围内的路径
    if (!base_local_planner::transformGlobalPlan(
        *tf_,
        global_plan_,
        robot_pose,
        *costmap_,
        costmap_ros_->getGlobalFrameID(),
        local_plan))
    {
      ROS_WARN("Could not transform global plan to local costmap frame, using full global plan");
      local_plan = plan;
    }
    else
    {
      ROS_INFO("Transformed plan: %lu points in local costmap (original: %lu)",
               local_plan.size(), plan.size());
    }
  }

  // 始终进行 MINCO 轨迹优化 - 使用局部路径而不是全局路径
  ROS_INFO("Starting trajectory optimization with local plan (%lu points)...", local_plan.size());
  optimization_success_ = trajectory_optimizer_.optimizePath(local_plan, optimized_points_, optimized_times_);

  if (optimization_success_)
  {
    ROS_INFO("Trajectory optimization successful! Generated %ld optimized points", optimized_points_.cols());
  }
  else
  {
    ROS_WARN("Trajectory optimization failed, will use original global path as fallback");
  }

  return true;
}

uint32_t NTUController::computeVelocityCommands(
    const geometry_msgs::PoseStamped &pose,
    const geometry_msgs::TwistStamped &velocity,
    geometry_msgs::TwistStamped &cmd_vel,
    std::string &message)
{
  if (!initialized_)
  {
    message = "Planner not initialized";
    return mbf_msgs::ExePathResult::NOT_INITIALIZED;
  }

  if (global_plan_.empty())
  {
    message = "No global path available";
    return mbf_msgs::ExePathResult::INVALID_PATH;
  }

  // 1. 检查是否已到达目标
  if (goal_reached_)
  {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    message = "Goal reached";
    return mbf_msgs::ExePathResult::SUCCESS;
  }

  // // 2. 找到前瞻点
  // size_t target_idx = findLookaheadPoint(pose);
  // if (target_idx >= global_plan_.size())
  // {
  //   message = "Cannot find lookahead point";
  //   return mbf_msgs::ExePathResult::INVALID_PATH;
  // }

  // geometry_msgs::PoseStamped target_pose = global_plan_[target_idx];

  // // 3. 计算到目标点的误差
  // double dx = target_pose.pose.position.x - pose.pose.position.x;
  // double dy = target_pose.pose.position.y - pose.pose.position.y;
  // double distance_to_target = std::sqrt(dx * dx + dy * dy);

  // // 计算目标角度
  // double target_yaw = std::atan2(dy, dx);
  // double current_yaw = tf2::getYaw(pose.pose.orientation);
  // double yaw_error = normalizeAngle(target_yaw - current_yaw);

  // // 4. 计算速度命令 - Pure Pursuit 简单实现
  // // 线速度：距离越远速度越大
  // double linear_vel = std::min(max_vel_x_, distance_to_target);
  
  // // 角速度：比例控制
  // double angular_vel = 2.0 * yaw_error; // Kp = 2.0
  // angular_vel = std::max(-max_vel_theta_, std::min(max_vel_theta_, angular_vel));

  // // 如果角度误差太大，减速转弯
  // if (std::abs(yaw_error) > 0.5) // 约 30 度
  // {
  //   linear_vel *= 0.3; // 减速到 30%
  // }

  // // 5. 碰撞检测（简化版）
  // if (!isPathCollisionFree())
  // {
  //   cmd_vel.twist.linear.x = 0.0;
  //   cmd_vel.twist.angular.z = 0.0;
  //   message = "Obstacle detected!";
  //   return mbf_msgs::ExePathResult::COLLISION;
  // }

  // // 6. 输出速度命令
  // cmd_vel.header.stamp = ros::Time::now();
  // cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  // cmd_vel.twist.linear.x = linear_vel;
  // cmd_vel.twist.linear.y = 0.0;
  // cmd_vel.twist.angular.z = angular_vel;

  // 7. 发布可视化
  publishVisualization();

  message = "Running normally";
  return mbf_msgs::ExePathResult::SUCCESS;
}

bool NTUController::isGoalReached(double xy_tolerance, double yaw_tolerance)
{
  if (global_plan_.empty())
  {
    return false;
  }

  // 获取机器人当前位姿
  geometry_msgs::PoseStamped robot_pose;
  if (!costmap_ros_->getRobotPose(robot_pose))
  {
    ROS_WARN("Cannot get robot pose");
    return false;
  }

  // 获取目标点（全局路径的最后一个点）
  geometry_msgs::PoseStamped goal_pose = global_plan_.back();

  // 计算距离
  double dist = distance(robot_pose, goal_pose);

  // 计算角度差
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  double goal_yaw = tf2::getYaw(goal_pose.pose.orientation);
  double yaw_diff = std::abs(normalizeAngle(robot_yaw - goal_yaw));

  // 使用传入的容差或类成员的容差
  double xy_tol = (xy_tolerance > 0) ? xy_tolerance : xy_goal_tolerance_;
  double yaw_tol = (yaw_tolerance > 0) ? yaw_tolerance : yaw_goal_tolerance_;

  bool reached = (dist < xy_tol) && (yaw_diff < yaw_tol);

  if (reached && !goal_reached_)
  {
    ROS_INFO("Goal reached! Distance: %.3f m, Yaw diff: %.3f rad", dist, yaw_diff);
    goal_reached_ = true;
  }

  return reached;
}

bool NTUController::cancel()
{
  ROS_INFO("NTUController: Received cancel command");
  global_plan_.clear();
  goal_reached_ = false;
  current_waypoint_idx_ = 0;
  return true;
}

size_t NTUController::findLookaheadPoint(const geometry_msgs::PoseStamped &robot_pose)
{
  // 从当前路点开始搜索
  for (size_t i = current_waypoint_idx_; i < global_plan_.size(); ++i)
  {
    double dist = distance(robot_pose, global_plan_[i]);
    
    // 找到第一个距离大于前瞻距离的点
    if (dist >= lookahead_distance_)
    {
      current_waypoint_idx_ = i;
      return i;
    }
  }

  // 如果所有点都比前瞻距离近，返回最后一个点
  return global_plan_.size() - 1;
}

double NTUController::distance(const geometry_msgs::PoseStamped &p1, 
                                    const geometry_msgs::PoseStamped &p2)
{
  double dx = p1.pose.position.x - p2.pose.position.x;
  double dy = p1.pose.position.y - p2.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

double NTUController::normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

bool NTUController::isPathCollisionFree()
{
  // 简化的碰撞检测：检查机器人周围一小块区域
  // 你可以根据需要实现更复杂的碰撞检测逻辑
  
  unsigned int mx, my;
  
  // 获取机器人在 costmap 中的位置
  geometry_msgs::PoseStamped robot_pose;
  if (!costmap_ros_->getRobotPose(robot_pose))
  {
    return true; // 无法获取位姿，假设安全
  }

  if (!costmap_->worldToMap(robot_pose.pose.position.x, 
                            robot_pose.pose.position.y, 
                            mx, my))
  {
    return true; // 机器人不在 costmap 范围内
  }

  // 检查机器人前方 1 米范围内
  double check_distance = 1.0; // 米
  double current_yaw = tf2::getYaw(robot_pose.pose.orientation);
  
  for (double d = 0.1; d <= check_distance; d += 0.1)
  {
    double check_x = robot_pose.pose.position.x + d * std::cos(current_yaw);
    double check_y = robot_pose.pose.position.y + d * std::sin(current_yaw);
    
    unsigned int check_mx, check_my;
    if (costmap_->worldToMap(check_x, check_y, check_mx, check_my))
    {
      unsigned char cost = costmap_->getCost(check_mx, check_my);
      
      // costmap_2d::LETHAL_OBSTACLE = 254
      if (cost >= 253)
      {
        ROS_WARN("Obstacle detected! Position: (%.2f, %.2f), cost: %u", check_x, check_y, cost);
        return false;
      }
    }
  }

  return true;
}

void NTUController::publishVisualization()
{
  // 发布本地路径
  if (global_plan_pub_.getNumSubscribers() > 0)
  {
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = global_plan_[0].header.frame_id;
    path_msg.header.stamp = ros::Time::now();
    
    // 发布从当前点到终点的路径
    for (size_t i = current_waypoint_idx_; i < global_plan_.size(); ++i)
    {
      path_msg.poses.push_back(global_plan_[i]);
    }
    
    global_plan_pub_.publish(path_msg);
  }

  // // 发布目标点标记
  // if (target_point_pub_.getNumSubscribers() > 0 && current_waypoint_idx_ < global_plan_.size())
  // {
  //   visualization_msgs::Marker marker;
  //   marker.header = global_plan_[current_waypoint_idx_].header;
  //   marker.ns = "target_point";
  //   marker.id = 0;
  //   marker.type = visualization_msgs::Marker::SPHERE;
  //   marker.action = visualization_msgs::Marker::ADD;
  //   marker.pose = global_plan_[current_waypoint_idx_].pose;
  //   marker.scale.x = 0.3;
  //   marker.scale.y = 0.3;
  //   marker.scale.z = 0.3;
  //   marker.color.r = 1.0;
  //   marker.color.g = 0.0;
  //   marker.color.b = 0.0;
  //   marker.color.a = 1.0;
    
  //   target_point_pub_.publish(marker);
  // }
}

void NTUController::controllerReconfigureCallback(ntu_planner::NTUControllerConfig &config, uint32_t level)
{
  ROS_INFO("NTUController dynamic reconfigure callback triggered");

  // 更新 NTUController 自己的参数
  max_vel_x_ = config.max_vel_x;
  max_vel_theta_ = config.max_vel_theta;
  xy_goal_tolerance_ = config.xy_goal_tolerance;
  yaw_goal_tolerance_ = config.yaw_goal_tolerance;
  lookahead_distance_ = config.lookahead_distance;

  // 同时更新到 parameter server
  nh_.setParam("max_vel_x", max_vel_x_);
  nh_.setParam("max_vel_theta", max_vel_theta_);
  nh_.setParam("xy_goal_tolerance", xy_goal_tolerance_);
  nh_.setParam("yaw_goal_tolerance", yaw_goal_tolerance_);
  nh_.setParam("lookahead_distance", lookahead_distance_);

  ROS_INFO("NTUController parameters updated:");
  ROS_INFO("  max_vel_x: %.2f, max_vel_theta: %.2f", max_vel_x_, max_vel_theta_);
  ROS_INFO("  xy_goal_tolerance: %.2f, yaw_goal_tolerance: %.2f", xy_goal_tolerance_, yaw_goal_tolerance_);
  ROS_INFO("  lookahead_distance: %.2f", lookahead_distance_);
}

void NTUController::optimizerReconfigureCallback(ntu_planner::TrajectoryOptimizerConfig &config, uint32_t level)
{
  ROS_INFO("TrajectoryOptimizer dynamic reconfigure callback triggered");

  // 更新轨迹优化器的参数
  trajectory_optimizer_.updateParameters(
      config.weight_time, config.weight_energy_x, config.weight_energy_y, config.weight_energy_w,
      config.weight_position_x, config.weight_position_y, config.weight_position_w,
      config.init_time, config.astar_point_interval, config.max_iterations,
      config.min_step, config.epsilon,
      config.viz_only_control_points, config.viz_time_step);

  ROS_INFO("TrajectoryOptimizer parameters updated via dynamic_reconfigure");
}

} // namespace ntu_planner
