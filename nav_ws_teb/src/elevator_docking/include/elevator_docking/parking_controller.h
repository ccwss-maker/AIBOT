#ifndef ELEVATOR_DOCKING_PARKING_CONTROLLER_H
#define ELEVATOR_DOCKING_PARKING_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <tf/tf.h>
#include <functional>

namespace elevator_docking
{

/**
 * @brief 停车目标
 */
struct ParkingTarget
{
  double yaw;   // 目标偏航角 (弧度)
  double y;     // 目标Y位置 (米)
  double x;     // 目标X位置 (米)

  ParkingTarget() : yaw(0.0), y(0.0), x(0.0) {}
  ParkingTarget(double yaw_, double y_, double x_) : yaw(yaw_), y(y_), x(x_) {}
};

/**
 * @brief 停车控制器
 */
class ParkingController
{
public:
  ParkingController(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~ParkingController();

  void setPreemptCheckCallback(std::function<bool()> callback);

  // 基础控制接口
  bool resetOdometry();
  bool correctYaw(double target_yaw, double tolerance = -1.0);
  bool correctY(double target_y, double tolerance = -1.0);
  bool driveX(double target_x, double tolerance = -1.0);
  void stopRobot();

  /**
   * @brief 计算回正角度
   * 倒车时（x<0 且 角度>90°）使用补角，避免大幅旋转
   */
  double computeTargetYaw(double detected_yaw, double detected_x);

  /**
   * @brief 执行停车序列
   * @param target 目标位置和角度
   * 自动跳过值为0的维度
   */
  bool execute(const ParkingTarget& target);

  // 状态查询
  nav_msgs::Odometry getCurrentOdometry() const { return current_odom_temp_; }
  bool isOdometryReceived() const { return odom_received_; }

  // 参数获取
  double getControlRate() const { return control_rate_; }
  double getYawAngularVel() const { return yaw_angular_vel_; }
  double getYLinearVel() const { return y_linear_vel_; }
  double getXLinearVel() const { return x_linear_vel_; }
  double getXBackwardVel() const { return x_backward_vel_; }
  double getYawTolerance() const { return yaw_tolerance_; }
  double getYTolerance() const { return y_tolerance_; }
  double getXTolerance() const { return x_tolerance_; }

  // 参数设置 (用于dynamic reconfigure)
  void setControlRate(double rate) { control_rate_ = rate; }
  void setYawAngularVel(double vel) { yaw_angular_vel_ = vel; }
  void setYLinearVel(double vel) { y_linear_vel_ = vel; }
  void setXLinearVel(double vel) { x_linear_vel_ = vel; }
  void setXBackwardVel(double vel) { x_backward_vel_ = vel; }
  void setYawTolerance(double tol) { yaw_tolerance_ = tol; }
  void setYTolerance(double tol) { y_tolerance_ = tol; }
  void setXTolerance(double tol) { x_tolerance_ = tol; }

private:
  void loadParameters();
  void odomTemporaryCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void publishVelocity(double linear_x, double linear_y, double angular_z);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher reset_odom_pub_;
  ros::Subscriber odom_temp_sub_;

  nav_msgs::Odometry current_odom_temp_;
  bool odom_received_;

  // 控制参数
  double control_rate_;
  double yaw_angular_vel_;
  double y_linear_vel_;
  double x_linear_vel_;
  double x_backward_vel_;

  // 容差
  double yaw_tolerance_;
  double y_tolerance_;
  double x_tolerance_;

  // 话题
  std::string cmd_vel_topic_;
  std::string reset_odom_topic_;
  std::string odom_temp_topic_;

  std::function<bool()> preempt_check_callback_;
};

} // namespace elevator_docking

#endif // ELEVATOR_DOCKING_PARKING_CONTROLLER_H
