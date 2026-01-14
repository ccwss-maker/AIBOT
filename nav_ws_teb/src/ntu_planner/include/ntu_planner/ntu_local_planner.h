#ifndef NTU_PLANNER__NTU_CONTROLLER_H_
#define NTU_PLANNER__NTU_CONTROLLER_H_

#include <ros/ros.h>
#include <mbf_costmap_core/costmap_controller.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/buffer.h>

namespace ntu_planner
{

/**
 * @brief NTU Controller - 自定义局部控制器
 * 
 * 实现 MBF 的 CostmapController 接口
 * 职责：接收全局路径 -> 计算速度命令 -> 输出给机器人
 */
class NTUController : public mbf_costmap_core::CostmapController
{
public:
  /**
   * @brief 构造函数
   */
  NTUController();

  /**
   * @brief 析构函数
   */
  virtual ~NTUController();

  /**
   * @brief 初始化函数
   * @param name 插件名称
   * @param tf TF 监听器指针
   * @param costmap_ros Costmap 指针
   * 
   * 在这里：
   * - 读取参数
   * - 初始化发布器
   * - 设置初始状态
   */
  virtual void initialize(std::string name, TF *tf, costmap_2d::Costmap2DROS *costmap_ros) override;

  /**
   * @brief 设置要跟踪的全局路径
   * @param plan 全局路径（从 global planner 输出）
   * @return true 成功接收路径, false 失败
   * 
   * MBF 会调用这个函数传递全局路径给你
   * 你需要保存这个路径，用于后续的速度计算
   */
  virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) override;

  /**
   * @brief 核心函数：计算速度命令
   * @param pose 机器人当前位姿
   * @param velocity 机器人当前速度
   * @param cmd_vel [输出] 计算出的速度命令
   * @param message [输出] 可选的状态消息
   * @return 结果码（见下方说明）
   * 
   * 返回值说明：
   * - 0 (SUCCESS): 成功计算速度
   * - 102 (NO_VALID_CMD): 无法生成有效速度命令
   * - 104 (COLLISION): 检测到即将碰撞
   * - 106 (ROBOT_STUCK): 机器人卡住
   * - 108 (MISSED_PATH): 偏离路径太远
   * - 110 (INVALID_PATH): 路径无效
   * 
   * 这是你的主要工作区域！在这里实现：
   * 1. 路径跟踪算法（Pure Pursuit, Stanley, MPC 等）
   * 2. 局部避障（可选）
   * 3. 速度平滑
   */
  virtual uint32_t computeVelocityCommands(
      const geometry_msgs::PoseStamped &pose,
      const geometry_msgs::TwistStamped &velocity,
      geometry_msgs::TwistStamped &cmd_vel,
      std::string &message) override;

  /**
   * @brief 检查是否到达目标
   * @param xy_tolerance XY 位置容差 (米)
   * @param yaw_tolerance 航向角容差 (弧度)
   * @return true 已到达目标, false 未到达
   */
  virtual bool isGoalReached(double xy_tolerance, double yaw_tolerance) override;

  /**
   * @brief 取消当前执行
   * @return true 取消成功, false 不支持取消
   */
  virtual bool cancel() override;

private:
  // ROS 相关
  ros::NodeHandle nh_;
  std::string name_;
  
  // TF 和 Costmap
  TF *tf_;
  costmap_2d::Costmap2DROS *costmap_ros_;
  costmap_2d::Costmap2D *costmap_;
  
  // 全局路径
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  
  // 当前目标点索引（在全局路径中）
  size_t current_waypoint_idx_;
  
  // 参数
  double max_vel_x_;           // 最大线速度
  double max_vel_theta_;       // 最大角速度
  double xy_goal_tolerance_;   // 位置容差
  double yaw_goal_tolerance_;  // 角度容差
  double lookahead_distance_;  // 前瞻距离
  
  // 状态
  bool initialized_;
  bool goal_reached_;
  
  // 发布器（用于可视化）
  ros::Publisher global_plan_pub_;
  // ros::Publisher target_point_pub_;
  
  /**
   * @brief 从参数服务器加载参数
   */
  void loadParameters();
  
  /**
   * @brief 找到前瞻点
   * @param robot_pose 机器人当前位姿
   * @return 前瞻点的索引
   */
  size_t findLookaheadPoint(const geometry_msgs::PoseStamped &robot_pose);
  
  /**
   * @brief 计算到目标点的距离
   * @param p1 点1
   * @param p2 点2
   * @return 距离 (米)
   */
  double distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2);
  
  /**
   * @brief 计算两个角度的差
   * @param angle1 角度1 (弧度)
   * @param angle2 角度2 (弧度)
   * @return 角度差 (弧度), 范围 [-pi, pi]
   */
  double normalizeAngle(double angle);
  
  /**
   * @brief 检查路径上是否有碰撞
   * @return true 无碰撞, false 有碰撞
   */
  bool isPathCollisionFree();
  
  /**
   * @brief 发布可视化信息
   */
  void publishVisualization();
};

} // namespace ntu_planner

#endif // NTU_PLANNER__NTU_CONTROLLER_H_
