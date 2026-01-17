#include "elevator_docking/elevator_docking_server.h"
#include <cmath>

namespace elevator_docking
{

ElevatorDockingServer::ElevatorDockingServer(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  , odom_received_(false)
  , stored_yaw_(0.0)
  , stored_y_(0.0)
  , stored_x_(0.0)
{
  // Load control parameters
  private_nh_.param<double>("control_rate", control_rate_, 20.0);
  private_nh_.param<double>("yaw_angular_vel", yaw_angular_vel_, 0.1);
  private_nh_.param<double>("y_linear_vel", y_linear_vel_, 0.1);
  private_nh_.param<double>("x_linear_vel", x_linear_vel_, 0.1);
  private_nh_.param<double>("x_backward_vel", x_backward_vel_, 0.1);

  // Load control tolerances
  private_nh_.param<double>("yaw_tolerance", yaw_tolerance_, 0.05);
  private_nh_.param<double>("y_tolerance", y_tolerance_, 0.02);
  private_nh_.param<double>("x_tolerance", x_tolerance_, 0.02);

  // Load detection parameters
  private_nh_.param<double>("detection_duration", detection_duration_, 1.0);

  ROS_INFO("Control parameters loaded:");
  ROS_INFO("  control_rate: %.1f Hz", control_rate_);
  ROS_INFO("  yaw: vel=%.2f rad/s, tol=%.3f rad", yaw_angular_vel_, yaw_tolerance_);
  ROS_INFO("  y: vel=%.2f m/s, tol=%.3f m", y_linear_vel_, y_tolerance_);
  ROS_INFO("  x: vel=%.2f m/s, tol=%.3f m", x_linear_vel_, x_tolerance_);
  ROS_INFO("  x_backward: vel=%.2f m/s", x_backward_vel_);
  ROS_INFO("  detection_duration: %.1f s", detection_duration_);

  // Create detector (disabled by default, enabled on demand)
  detector_ = std::make_shared<ElevatorDetector>(nh_, private_nh_);
  detector_->setDetectionEnabled(false);

  // Publishers and subscribers
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_cmd_vel", 10);
  reset_odom_pub_ = nh_.advertise<std_msgs::Empty>("/reset_odom_temporary", 10);
  odom_temp_sub_ = nh_.subscribe("/odom_temporary", 10,
                                  &ElevatorDockingServer::odomTemporaryCallback, this);

  // Create action server
  as_ = std::make_unique<actionlib::SimpleActionServer<elevator_docking::ElevatorDockingAction>>(
    nh_, "/robot1/elevator/docking", false);

  as_->registerGoalCallback(boost::bind(&ElevatorDockingServer::goalCallback, this));
  as_->registerPreemptCallback(boost::bind(&ElevatorDockingServer::preemptCallback, this));

  as_->start();

  ROS_INFO("Elevator Docking Action Server started");
}

ElevatorDockingServer::~ElevatorDockingServer()
{
  stopRobot();
}

void ElevatorDockingServer::goalCallback()
{
  elevator_docking::ElevatorDockingGoalConstPtr goal = as_->acceptNewGoal();
  elevator_docking::ElevatorDockingResult result;
  elevator_docking::ElevatorDockingFeedback feedback;

  ROS_INFO("Received goal with command: %s", goal->command.c_str());

  bool success = false;

  if (goal->command == "detection")
  {
    feedback.status = "Starting detection...";
    as_->publishFeedback(feedback);

    success = executeDetection();

    if (success)
    {
      result.success = true;
      result.message = "Detection successful";
      result.center_x = last_detection_result_.x;
      result.center_y = last_detection_result_.y;
      result.width = last_detection_result_.width;
      result.depth = last_detection_result_.depth;
      result.yaw_deg = last_detection_result_.angle * 180.0 / M_PI;
      result.confidence = last_detection_result_.confidence;

      // Store for "in" mode
      stored_yaw_ = last_detection_result_.angle;
      stored_y_ = last_detection_result_.y;
      stored_x_ = last_detection_result_.x;

      ROS_INFO("Detection result: center=(%.2f, %.2f), size=(%.2fx%.2f), yaw=%.1f deg, confidence=%.2f",
               result.center_x, result.center_y, result.width, result.depth,
               result.yaw_deg, result.confidence);
    }
    else
    {
      result.success = false;
      result.message = "Detection failed";
    }
  }
  else if (goal->command == "in")
  {
    feedback.status = "Driving into elevator...";
    as_->publishFeedback(feedback);

    success = executeIn();

    if (success)
    {
      result.success = true;
      result.message = "Successfully entered elevator";
    }
    else
    {
      result.success = false;
      result.message = "Failed to enter elevator";
    }
  }
  else if (goal->command == "out")
  {
    feedback.status = "Driving out of elevator...";
    as_->publishFeedback(feedback);

    success = executeOut();

    if (success)
    {
      result.success = true;
      result.message = "Successfully exited elevator";
    }
    else
    {
      result.success = false;
      result.message = "Failed to exit elevator";
    }
  }
  else
  {
    result.success = false;
    result.message = "Unknown command: " + goal->command;
    ROS_ERROR("Unknown command: %s", goal->command.c_str());
  }

  if (success)
  {
    as_->setSucceeded(result);
  }
  else
  {
    as_->setAborted(result);
  }
}

void ElevatorDockingServer::preemptCallback()
{
  ROS_INFO("Goal preempted");
  stopRobot();
  as_->setPreempted();
}

bool ElevatorDockingServer::runDetection(ElevatorDetectionResult& result, double duration)
{
  // Use default duration if not specified
  if (duration < 0) {
    duration = detection_duration_;
  }

  ROS_INFO("Starting detection for %.1f seconds...", duration);

  // Enable detection
  detector_->setDetectionEnabled(true);

  ros::Time start_time = ros::Time::now();
  ros::Duration detection_time(duration);

  double max_confidence = 0.0;
  ElevatorDetectionResult best_result;
  bool found_any = false;

  ros::Rate rate(control_rate_);

  while (ros::Time::now() - start_time < detection_time)
  {
    // Check if goal is preempted
    if (as_->isPreemptRequested() || !ros::ok())
    {
      detector_->setDetectionEnabled(false);
      ROS_INFO("Detection preempted");
      return false;
    }

    ros::spinOnce();

    // Try to get detection result from detector
    ElevatorDetectionResult current_result;
    if (detector_->getLastDetection(current_result))
    {
      if (current_result.confidence > max_confidence)
      {
        max_confidence = current_result.confidence;
        best_result = current_result;
        found_any = true;
        ROS_DEBUG("Found detection with confidence %.2f", max_confidence);
      }
    }

    rate.sleep();
  }

  // Disable detection
  detector_->setDetectionEnabled(false);

  if (found_any)
  {
    result = best_result;
    ROS_INFO("Detection result: center=(%.2f, %.2f), size=(%.2fx%.2f), yaw=%.1f deg, confidence=%.2f",
             result.x, result.y, result.width, result.depth,
             result.angle * 180.0 / M_PI, result.confidence);
    return true;
  }
  else
  {
    ROS_WARN("No elevator detected during %.1f second detection period", duration);
    return false;
  }
}

bool ElevatorDockingServer::executeDetection()
{
  ElevatorDetectionResult result;
  if (runDetection(result))
  {
    last_detection_result_ = result;
    stored_yaw_ = result.angle;
    stored_y_ = result.y;
    stored_x_ = result.x;
    return true;
  }
  return false;
}

bool ElevatorDockingServer::executeIn()
{
  ROS_INFO("Executing 'in' command with strategy: detect-at-each-step");

  // ========================================================================
  // Step 1: Detect and correct YAW
  // ========================================================================
  ROS_INFO("=== Step 1/3: Yaw Correction ===");

  // Detect elevator to get current yaw offset
  ElevatorDetectionResult yaw_detection;
  if (!runDetection(yaw_detection))
  {
    ROS_ERROR("Failed to detect elevator for yaw correction");
    return false;
  }

  // Reset odometry
  ROS_INFO("Resetting odometry for yaw correction...");
  resetOdomTemporary();
  // ros::Duration(0.1).sleep();

  // Wait for odometry
  odom_received_ = false;
  ros::Time wait_start = ros::Time::now();
  while (!odom_received_ && (ros::Time::now() - wait_start).toSec() < 2.0)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  if (!odom_received_)
  {
    ROS_ERROR("Failed to receive odometry");
    return false;
  }

  // Correct yaw
  // Smart angle correction strategy:
  // - If backing in (x < 0) AND angle close to ±180°, use complementary angle
  // - Otherwise use detected angle directly
  double target_yaw = yaw_detection.angle;
  bool is_backing = (yaw_detection.x < 0);  // Elevator is behind the robot
  double abs_angle = std::abs(yaw_detection.angle);

  if (is_backing && abs_angle > M_PI / 2.0)  // Backing in AND angle > 90°
  {
    // Use complementary angle to avoid large rotation
    if (yaw_detection.angle > 0) {
      target_yaw = yaw_detection.angle - M_PI;  // +177° -> -3°
    } else {
      target_yaw = yaw_detection.angle + M_PI;  // -177° -> +3°
    }
    ROS_INFO("Backing mode (x=%.2f < 0): Large angle %.1f deg detected, using complementary angle %.1f deg",
             yaw_detection.x, yaw_detection.angle * 180.0 / M_PI, target_yaw * 180.0 / M_PI);
  }
  else
  {
    ROS_INFO("Forward mode (x=%.2f): Using detected angle %.1f deg directly",
             yaw_detection.x, yaw_detection.angle * 180.0 / M_PI);
  }

  ROS_INFO("Correcting yaw: target=%.3f rad (%.1f deg)",
           target_yaw, target_yaw * 180.0 / M_PI);
  if (!correctYaw(target_yaw))
  {
    ROS_ERROR("Failed to correct yaw");
    stopRobot();
    return false;
  }

  // ========================================================================
  // Step 2: Detect and correct Y (lateral offset)
  // ========================================================================
  ROS_INFO("=== Step 2/3: Y Correction ===");

  // Detect elevator again to get current y offset (after yaw rotation)
  ElevatorDetectionResult y_detection;
  if (!runDetection(y_detection))
  {
    ROS_ERROR("Failed to detect elevator for y correction");
    return false;
  }

  // Reset odometry
  ROS_INFO("Resetting odometry for y correction...");
  resetOdomTemporary();
  // ros::Duration(0.5).sleep();

  // Wait for odometry
  odom_received_ = false;
  wait_start = ros::Time::now();
  while (!odom_received_ && (ros::Time::now() - wait_start).toSec() < 2.0)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  if (!odom_received_)
  {
    ROS_ERROR("Failed to receive odometry");
    return false;
  }

  // Correct y
  ROS_INFO("Correcting y: target=%.3f m", y_detection.y);
  if (!correctY(y_detection.y))
  {
    ROS_ERROR("Failed to correct y");
    stopRobot();
    return false;
  }

  // ========================================================================
  // Step 3: Detect and drive X (forward into elevator)
  // ========================================================================
  ROS_INFO("=== Step 3/3: X Drive (Enter Elevator) ===");

  // Detect elevator again to get current x distance
  ElevatorDetectionResult x_detection;
  if (!runDetection(x_detection))
  {
    ROS_ERROR("Failed to detect elevator for x drive");
    return false;
  }

  // Reset odometry
  ROS_INFO("Resetting odometry for x drive...");
  resetOdomTemporary();
  // ros::Duration(0.5).sleep();

  // Wait for odometry
  odom_received_ = false;
  wait_start = ros::Time::now();
  while (!odom_received_ && (ros::Time::now() - wait_start).toSec() < 2.0)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  if (!odom_received_)
  {
    ROS_ERROR("Failed to receive odometry");
    return false;
  }

  // Drive forward into elevator
  ROS_INFO("Driving forward: target=%.3f m", x_detection.x);
  if (!driveX(x_detection.x))
  {
    ROS_ERROR("Failed to drive x");
    stopRobot();
    return false;
  }

  // Store final x for "out" command
  stored_x_ = x_detection.x;

  stopRobot();
  ROS_INFO("Successfully completed 'in' command");
  return true;
}

bool ElevatorDockingServer::executeOut()
{
  ROS_INFO("Executing 'out' command...");
  ROS_INFO("Stored entry distance: x=%.3f m", stored_x_);

  // Step 1: Reset odometry
  resetOdomTemporary();
  // ros::Duration(0.5).sleep();

  // Step 2: Wait for odometry
  odom_received_ = false;
  ros::Time wait_start = ros::Time::now();
  while (!odom_received_ && (ros::Time::now() - wait_start).toSec() < 2.0)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  if (!odom_received_)
  {
    ROS_ERROR("Failed to receive odometry");
    return false;
  }

  // Step 3: Drive in the opposite direction of entry
  // - If entered forward (stored_x_ > 0): drive backward (-stored_x_)
  // - If entered backward (stored_x_ < 0): drive forward (-stored_x_, which is positive)
  double target_x = -stored_x_;
  ROS_INFO("Exiting: driving to x=%.3f m (reverse of entry)", target_x);

  if (!driveX(target_x))
  {
    ROS_ERROR("Failed to exit elevator");
    stopRobot();
    return false;
  }

  stopRobot();
  ROS_INFO("Successfully completed 'out' command");
  return true;
}

void ElevatorDockingServer::resetOdomTemporary()
{
  std_msgs::Empty msg;
  reset_odom_pub_.publish(msg);
  ROS_INFO("Published reset odometry signal");
}

void ElevatorDockingServer::odomTemporaryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_odom_temp_ = *msg;
  odom_received_ = true;
}

void ElevatorDockingServer::publishVelocity(double linear_x, double angular_z)
{
  geometry_msgs::Twist cmd;
  cmd.linear.x = linear_x;
  cmd.angular.z = angular_z;
  cmd_vel_pub_.publish(cmd);
}

void ElevatorDockingServer::stopRobot()
{
  publishVelocity(0.0, 0.0);
}

bool ElevatorDockingServer::correctYaw(double target_yaw, double tolerance)
{
  // Use default tolerance if not specified
  if (tolerance < 0) {
    tolerance = yaw_tolerance_;
  }

  ROS_INFO("correctYaw: target_yaw=%.3f rad (%.1f deg)", target_yaw, target_yaw * 180.0 / M_PI);

  ros::Rate rate(control_rate_);

  while (ros::ok())
  {
    if (as_->isPreemptRequested())
    {
      ROS_INFO("Yaw correction preempted");
      stopRobot();
      return false;
    }

    ros::spinOnce();

    if (!odom_received_)
    {
      rate.sleep();
      continue;
    }

    // Get current yaw
    tf::Quaternion q(
      current_odom_temp_.pose.pose.orientation.x,
      current_odom_temp_.pose.pose.orientation.y,
      current_odom_temp_.pose.pose.orientation.z,
      current_odom_temp_.pose.pose.orientation.w
    );
    double roll, pitch, current_yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, current_yaw);

    double error = target_yaw - current_yaw;

    // Normalize error to [-pi, pi]
    while (error > M_PI) error -= 2.0 * M_PI;
    while (error < -M_PI) error += 2.0 * M_PI;

    ROS_INFO_THROTTLE(0.5, "Current yaw: %.3f, Target yaw: %.3f, Error: %.3f",
                      current_yaw, target_yaw, error);

    if (std::abs(error) < tolerance)
    {
      ROS_INFO("Yaw correction completed");
      stopRobot();
      return true;
    }

    // Proportional control with sign from error
    // Clamp velocity to max
    double angular_vel = error;  // Proportional to error
    if (angular_vel > yaw_angular_vel_) angular_vel = yaw_angular_vel_;
    if (angular_vel < -yaw_angular_vel_) angular_vel = -yaw_angular_vel_;

    publishVelocity(0.0, angular_vel);

    rate.sleep();
  }

  return false;
}

bool ElevatorDockingServer::correctY(double target_y, double tolerance)
{
  // Use default tolerance if not specified
  if (tolerance < 0) {
    tolerance = y_tolerance_;
  }

  ROS_INFO("correctY: target_y=%.3f m", target_y);

  ros::Rate rate(control_rate_);

  while (ros::ok())
  {
    if (as_->isPreemptRequested())
    {
      ROS_INFO("Y correction preempted");
      stopRobot();
      return false;
    }

    ros::spinOnce();

    if (!odom_received_)
    {
      rate.sleep();
      continue;
    }

    double current_y = current_odom_temp_.pose.pose.position.y;
    double error = target_y - current_y;

    ROS_INFO_THROTTLE(0.5, "Current y: %.3f, Target y: %.3f, Error: %.3f",
                      current_y, target_y, error);

    if (std::abs(error) < tolerance)
    {
      ROS_INFO("Y correction completed");
      stopRobot();
      return true;
    }

    // Holonomic robot - direct Y control
    // Proportional control with sign from error
    double vel_y = error;  // Proportional to error
    if (vel_y > y_linear_vel_) vel_y = y_linear_vel_;
    if (vel_y < -y_linear_vel_) vel_y = -y_linear_vel_;

    // Publish velocity command with Y component
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = vel_y;  // Lateral movement for holonomic robot
    cmd.angular.z = 0.0;
    cmd_vel_pub_.publish(cmd);

    rate.sleep();
  }

  return false;
}

bool ElevatorDockingServer::driveX(double target_x, double tolerance)
{
  // Use default tolerance if not specified
  if (tolerance < 0) {
    tolerance = x_tolerance_;
  }

  ROS_INFO("driveX: target_x=%.3f m (direction: %s)",
           target_x, target_x >= 0 ? "forward" : "backward");

  ros::Rate rate(control_rate_);

  while (ros::ok())
  {
    if (as_->isPreemptRequested())
    {
      ROS_INFO("X drive preempted");
      stopRobot();
      return false;
    }

    ros::spinOnce();

    if (!odom_received_)
    {
      rate.sleep();
      continue;
    }

    double current_x = current_odom_temp_.pose.pose.position.x;
    double error = target_x - current_x;

    ROS_INFO_THROTTLE(0.5, "Current x: %.3f, Target x: %.3f, Error: %.3f",
                      current_x, target_x, error);

    if (std::abs(error) < tolerance)
    {
      ROS_INFO("X drive completed");
      stopRobot();
      return true;
    }

    // Proportional control with sign from error
    double vel = error;  // Proportional to error
    if (vel > x_linear_vel_) vel = x_linear_vel_;
    if (vel < -x_linear_vel_) vel = -x_linear_vel_;

    publishVelocity(vel, 0.0);

    rate.sleep();
  }

  return false;
}

} // namespace elevator_docking
