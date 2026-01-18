#include "elevator_docking/parking_controller.h"
#include <cmath>

namespace elevator_docking
{

ParkingController::ParkingController(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  , odom_received_(false)
{
  loadParameters();

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
  reset_odom_pub_ = nh_.advertise<std_msgs::Empty>(reset_odom_topic_, 10);
  odom_temp_sub_ = nh_.subscribe(odom_temp_topic_, 10, &ParkingController::odomTemporaryCallback, this);

  ROS_INFO("ParkingController initialized");
  ROS_INFO("  Topics: cmd_vel=%s, odom_temp=%s", cmd_vel_topic_.c_str(), odom_temp_topic_.c_str());
}

ParkingController::~ParkingController()
{
  stopRobot();
}

void ParkingController::loadParameters()
{
  private_nh_.param<double>("control_rate", control_rate_, 20.0);
  private_nh_.param<double>("yaw_angular_vel", yaw_angular_vel_, 0.1);
  private_nh_.param<double>("y_linear_vel", y_linear_vel_, 0.1);
  private_nh_.param<double>("x_linear_vel", x_linear_vel_, 0.1);
  private_nh_.param<double>("x_backward_vel", x_backward_vel_, 0.1);

  private_nh_.param<double>("yaw_tolerance", yaw_tolerance_, 0.05);
  private_nh_.param<double>("y_tolerance", y_tolerance_, 0.02);
  private_nh_.param<double>("x_tolerance", x_tolerance_, 0.02);

  private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "cmd_vel");
  private_nh_.param<std::string>("reset_odom_topic", reset_odom_topic_, "reset_odom_temporary");
  private_nh_.param<std::string>("odom_temp_topic", odom_temp_topic_, "odom_temporary");
}

void ParkingController::setPreemptCheckCallback(std::function<bool()> callback)
{
  preempt_check_callback_ = callback;
}

void ParkingController::odomTemporaryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_odom_temp_ = *msg;
  odom_received_ = true;
}

void ParkingController::publishVelocity(double linear_x, double linear_y, double angular_z)
{
  geometry_msgs::Twist cmd;
  cmd.linear.x = linear_x;
  cmd.linear.y = linear_y;
  cmd.angular.z = angular_z;
  cmd_vel_pub_.publish(cmd);
}

void ParkingController::stopRobot()
{
  publishVelocity(0.0, 0.0, 0.0);
}

bool ParkingController::resetOdometry()
{
  std_msgs::Empty msg;
  reset_odom_pub_.publish(msg);

  odom_received_ = false;
  ros::Time wait_start = ros::Time::now();
  while (!odom_received_ && (ros::Time::now() - wait_start).toSec() < 2.0)
  {
    if (preempt_check_callback_ && preempt_check_callback_())
    {
      return false;
    }
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  if (!odom_received_)
  {
    ROS_ERROR("Failed to receive odometry after reset");
    return false;
  }
  return true;
}

bool ParkingController::correctYaw(double target_yaw, double tolerance)
{
  if (tolerance < 0) tolerance = yaw_tolerance_;

  ROS_INFO("correctYaw: target=%.1f deg", target_yaw * 180.0 / M_PI);
  ros::Rate rate(control_rate_);

  while (ros::ok())
  {
    if (preempt_check_callback_ && preempt_check_callback_())
    {
      stopRobot();
      return false;
    }

    ros::spinOnce();
    if (!odom_received_) { rate.sleep(); continue; }

    tf::Quaternion q(
      current_odom_temp_.pose.pose.orientation.x,
      current_odom_temp_.pose.pose.orientation.y,
      current_odom_temp_.pose.pose.orientation.z,
      current_odom_temp_.pose.pose.orientation.w);
    double roll, pitch, current_yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, current_yaw);

    double error = target_yaw - current_yaw;
    while (error > M_PI) error -= 2.0 * M_PI;
    while (error < -M_PI) error += 2.0 * M_PI;

    if (std::abs(error) < tolerance)
    {
      stopRobot();
      return true;
    }

    double vel = std::max(-yaw_angular_vel_, std::min(yaw_angular_vel_, error));
    publishVelocity(0.0, 0.0, vel);
    rate.sleep();
  }
  return false;
}

bool ParkingController::correctY(double target_y, double tolerance)
{
  if (tolerance < 0) tolerance = y_tolerance_;

  ROS_INFO("correctY: target=%.3f m", target_y);
  ros::Rate rate(control_rate_);

  while (ros::ok())
  {
    if (preempt_check_callback_ && preempt_check_callback_())
    {
      stopRobot();
      return false;
    }

    ros::spinOnce();
    if (!odom_received_) { rate.sleep(); continue; }

    double current_y = current_odom_temp_.pose.pose.position.y;
    double error = target_y - current_y;

    if (std::abs(error) < tolerance)
    {
      stopRobot();
      return true;
    }

    double vel = std::max(-y_linear_vel_, std::min(y_linear_vel_, error));
    publishVelocity(0.0, vel, 0.0);
    rate.sleep();
  }
  return false;
}

bool ParkingController::driveX(double target_x, double tolerance)
{
  if (tolerance < 0) tolerance = x_tolerance_;

  ROS_INFO("driveX: target=%.3f m", target_x);
  ros::Rate rate(control_rate_);

  while (ros::ok())
  {
    if (preempt_check_callback_ && preempt_check_callback_())
    {
      stopRobot();
      return false;
    }

    ros::spinOnce();
    if (!odom_received_) { rate.sleep(); continue; }

    double current_x = current_odom_temp_.pose.pose.position.x;
    double error = target_x - current_x;

    if (std::abs(error) < tolerance)
    {
      stopRobot();
      return true;
    }

    double vel = std::max(-x_linear_vel_, std::min(x_linear_vel_, error));
    publishVelocity(vel, 0.0, 0.0);
    rate.sleep();
  }
  return false;
}

double ParkingController::computeTargetYaw(double detected_yaw, double detected_x)
{
  // 倒车时（x<0 且 角度>90°）使用补角
  if (detected_x < 0 && std::abs(detected_yaw) > M_PI / 2.0)
  {
    double target = (detected_yaw > 0) ? (detected_yaw - M_PI) : (detected_yaw + M_PI);
    ROS_INFO("Backing: angle %.1f -> %.1f deg", detected_yaw * 180.0 / M_PI, target * 180.0 / M_PI);
    return target;
  }
  return detected_yaw;
}

bool ParkingController::execute(const ParkingTarget& target)
{
  ROS_INFO("Execute: yaw=%.1f deg, y=%.3f m, x=%.3f m",
           target.yaw * 180.0 / M_PI, target.y, target.x);

  // 回正偏航角
  if (std::abs(target.yaw) > 1e-6)
  {
    if (!resetOdometry())
    {
      ROS_ERROR("Failed to reset odometry (yaw)");
      return false;
    }
    if (!correctYaw(target.yaw))
    {
      ROS_ERROR("Failed to correct yaw");
      stopRobot();
      return false;
    }
  }
  else
  {
    ROS_INFO("Skip yaw correction (target=0)");
  }

  // 回正Y方向偏移
  if (std::abs(target.y) > 1e-6)
  {
    if (!resetOdometry())
    {
      ROS_ERROR("Failed to reset odometry (y)");
      return false;
    }
    if (!correctY(target.y))
    {
      ROS_ERROR("Failed to correct Y");
      stopRobot();
      return false;
    }
  }
  else
  {
    ROS_INFO("Skip Y correction (target=0)");
  }

  // 前进X距离
  if (std::abs(target.x) > 1e-6)
  {
    if (!resetOdometry())
    {
      ROS_ERROR("Failed to reset odometry (x)");
      return false;
    }
    if (!driveX(target.x))
    {
      ROS_ERROR("Failed to drive X");
      stopRobot();
      return false;
    }
  }
  else
  {
    ROS_INFO("Skip X movement (target=0)");
  }

  stopRobot();
  ROS_INFO("Execute completed");
  return true;
}

} // namespace elevator_docking
