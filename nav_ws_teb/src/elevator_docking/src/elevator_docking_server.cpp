#include "elevator_docking/elevator_docking_server.h"
#include <cmath>

namespace elevator_docking
{

ElevatorDockingServer::ElevatorDockingServer(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  , stored_x_(0.0)
{
  ros::NodeHandle controller_nh(private_nh_, "controller");

  controller_nh.param<double>("control_rate", control_rate_, 20.0);
  controller_nh.param<double>("detection_duration", detection_duration_, 1.0);
  private_nh_.param<std::string>("action_server_name", action_server_name_, "docking");

  ROS_INFO("Docking server: rate=%.1f Hz, detection=%.1f s", control_rate_, detection_duration_);

  // Create detector
  ros::NodeHandle detector_nh(private_nh_, "detector");
  detector_ = std::make_shared<ElevatorDetector>(nh_, detector_nh);
  detector_->setDetectionEnabled(false);

  // Create parking controller
  parking_controller_ = std::make_shared<ParkingController>(nh_, controller_nh);

  // Setup dynamic reconfigure
  dyn_reconfig_server_ = std::make_shared<dynamic_reconfigure::Server<elevator_docking::DockingControlConfig>>(controller_nh);
  dyn_reconfig_server_->setCallback(boost::bind(&ElevatorDockingServer::reconfigureCallback, this, _1, _2));

  // 设置抢占检查回调
  parking_controller_->setPreemptCheckCallback([this]() {
    return as_->isPreemptRequested() || !ros::ok();
  });

  // Create action server
  as_ = std::make_unique<actionlib::SimpleActionServer<elevator_docking::ElevatorDockingAction>>(
    nh_, action_server_name_, false);
  as_->registerGoalCallback(boost::bind(&ElevatorDockingServer::goalCallback, this));
  as_->registerPreemptCallback(boost::bind(&ElevatorDockingServer::preemptCallback, this));
  as_->start();

  ROS_INFO("Action server started: %s", action_server_name_.c_str());
}

ElevatorDockingServer::~ElevatorDockingServer()
{
  parking_controller_->stopRobot();
}

void ElevatorDockingServer::goalCallback()
{
  auto goal = as_->acceptNewGoal();
  elevator_docking::ElevatorDockingResult result;
  elevator_docking::ElevatorDockingFeedback feedback;

  ROS_INFO("Command: %s", goal->command.c_str());
  bool success = false;

  if (goal->command == "detection")
  {
    feedback.status = "Detecting...";
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
    }
    else
    {
      result.success = false;
      result.message = "Detection failed";
    }
  }
  else if (goal->command == "in")
  {
    feedback.status = "Entering elevator...";
    as_->publishFeedback(feedback);
    success = executeIn();
    result.success = success;
    result.message = success ? "Entered elevator" : "Failed to enter";
  }
  else if (goal->command == "out")
  {
    feedback.status = "Exiting elevator...";
    as_->publishFeedback(feedback);
    success = executeOut();
    result.success = success;
    result.message = success ? "Exited elevator" : "Failed to exit";
  }
  else if (goal->command == "execute")
  {
    feedback.status = "Executing...";
    as_->publishFeedback(feedback);

    ParkingTarget target(goal->target_yaw, goal->target_y, goal->target_x);
    ROS_INFO("Execute: x=%.3f, y=%.3f, yaw=%.1f deg",
             goal->target_x, goal->target_y, goal->target_yaw * 180.0 / M_PI);

    // execute 命令直接执行，自动跳过值为0的维度
    success = parking_controller_->execute(target);
    result.success = success;
    result.message = success ? "Execute completed" : "Execute failed";
  }
  else
  {
    result.success = false;
    result.message = "Unknown command: " + goal->command;
    ROS_ERROR("Unknown command: %s", goal->command.c_str());
  }

  if (success)
    as_->setSucceeded(result);
  else
    as_->setAborted(result);
}

void ElevatorDockingServer::preemptCallback()
{
  ROS_INFO("Preempted");
  parking_controller_->stopRobot();
  as_->setPreempted();
}

bool ElevatorDockingServer::runDetection(ElevatorDetectionResult& result, double duration)
{
  if (duration < 0) duration = detection_duration_;

  detector_->setDetectionEnabled(true);

  ros::Time start = ros::Time::now();
  double max_confidence = 0.0;
  ElevatorDetectionResult best;
  bool found = false;

  ros::Rate rate(control_rate_);
  while ((ros::Time::now() - start).toSec() < duration)
  {
    if (as_->isPreemptRequested() || !ros::ok())
    {
      detector_->setDetectionEnabled(false);
      return false;
    }

    ros::spinOnce();

    ElevatorDetectionResult curr;
    if (detector_->getLastDetection(curr) && curr.confidence > max_confidence)
    {
      max_confidence = curr.confidence;
      best = curr;
      found = true;
    }
    rate.sleep();
  }

  detector_->setDetectionEnabled(false);

  if (found)
  {
    result = best;
    ROS_INFO("Detected: pos=(%.2f, %.2f), yaw=%.1f deg, conf=%.2f",
             result.x, result.y, result.angle * 180.0 / M_PI, result.confidence);
    return true;
  }
  ROS_WARN("No elevator detected");
  return false;
}

bool ElevatorDockingServer::executeDetection()
{
  ElevatorDetectionResult result;
  if (runDetection(result))
  {
    last_detection_result_ = result;
    return true;
  }
  return false;
}

bool ElevatorDockingServer::executeIn()
{
  ROS_INFO("Executing 'in': detect and enter elevator");

  // 第一次检测
  ElevatorDetectionResult detection;
  if (!runDetection(detection))
  {
    ROS_ERROR("Initial detection failed");
    return false;
  }

  // 计算并回正角度
  double target_yaw = parking_controller_->computeTargetYaw(detection.angle, detection.x);
  ROS_INFO("Computed target yaw: %.1f deg (detected: %.1f deg)",
           target_yaw * 180.0 / M_PI, detection.angle * 180.0 / M_PI);

  if (!parking_controller_->resetOdometry() || !parking_controller_->correctYaw(target_yaw))
  {
    ROS_ERROR("Failed to correct yaw");
    return false;
  }

  // 第二次检测
  if (!runDetection(detection))
  {
    ROS_ERROR("Second detection failed");
    return false;
  }

  // 回正Y
  if (!parking_controller_->resetOdometry() || !parking_controller_->correctY(detection.y))
  {
    ROS_ERROR("Failed to correct Y");
    return false;
  }

  // 第三次检测
  if (!runDetection(detection))
  {
    ROS_ERROR("Third detection failed");
    return false;
  }

  // 回正X
  if (!parking_controller_->resetOdometry() || !parking_controller_->driveX(detection.x))
  {
    ROS_ERROR("Failed to drive X");
    return false;
  }

  stored_x_ = detection.x;
  ROS_INFO("Entered elevator, stored_x=%.3f", stored_x_);
  return true;
}

bool ElevatorDockingServer::executeOut()
{
  ROS_INFO("Executing 'out': stored_x=%.3f", stored_x_);

  ParkingTarget target(0.0, 0.0, -stored_x_);
  return parking_controller_->execute(target);  // 直接后退
}

void ElevatorDockingServer::reconfigureCallback(elevator_docking::DockingControlConfig& config, uint32_t level)
{
  control_rate_ = config.control_rate;
  detection_duration_ = config.detection_duration;

  parking_controller_->setControlRate(config.control_rate);
  parking_controller_->setYawAngularVel(config.yaw_angular_vel);
  parking_controller_->setYLinearVel(config.y_linear_vel);
  parking_controller_->setXLinearVel(config.x_linear_vel);
  parking_controller_->setXBackwardVel(config.x_backward_vel);
  parking_controller_->setYawTolerance(config.yaw_tolerance);
  parking_controller_->setYTolerance(config.y_tolerance);
  parking_controller_->setXTolerance(config.x_tolerance);
}

} // namespace elevator_docking
