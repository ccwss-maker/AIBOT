#ifndef ELEVATOR_DOCKING_SERVER_H
#define ELEVATOR_DOCKING_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <elevator_docking/ElevatorDockingAction.h>
#include <elevator_docking/elevator_detector.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <tf/tf.h>
#include <memory>
#include <dynamic_reconfigure/server.h>
#include <elevator_docking/DockingControlConfig.h>

namespace elevator_docking
{

class ElevatorDockingServer
{
public:
  ElevatorDockingServer(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~ElevatorDockingServer();

private:
  void goalCallback();
  void preemptCallback();

  // Detection mode
  bool executeDetection();

  // Detection helper - run detection for specified duration
  bool runDetection(ElevatorDetectionResult& result, double duration = -1.0);

  // In mode - drive into elevator
  bool executeIn();

  // Out mode - drive out of elevator
  bool executeOut();

  // Control helpers
  void resetOdomTemporary();
  void odomTemporaryCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void publishVelocity(double linear_x, double angular_z);
  void stopRobot();

  // Pure pursuit control
  bool correctYaw(double target_yaw, double tolerance = -1.0);
  bool correctY(double target_y, double tolerance = -1.0);
  bool driveX(double target_x, double tolerance = -1.0);

  // Dynamic reconfigure callback
  void reconfigureCallback(elevator_docking::DockingControlConfig& config, uint32_t level);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Action server
  std::unique_ptr<actionlib::SimpleActionServer<elevator_docking::ElevatorDockingAction>> as_;

  // Detector
  std::shared_ptr<ElevatorDetector> detector_;

  // Publishers and subscribers
  ros::Publisher cmd_vel_pub_;
  ros::Publisher reset_odom_pub_;
  ros::Subscriber odom_temp_sub_;

  // Current odometry
  nav_msgs::Odometry current_odom_temp_;
  bool odom_received_;

  // Last detection result
  ElevatorDetectionResult last_detection_result_;

  // Control parameters
  double control_rate_;
  double yaw_angular_vel_;
  double y_linear_vel_;
  double x_linear_vel_;
  double x_backward_vel_;

  // Control tolerances
  double yaw_tolerance_;
  double y_tolerance_;
  double x_tolerance_;

  // Detection parameters
  double detection_duration_;

  // Stored distances for "in" mode
  double stored_yaw_;
  double stored_y_;
  double stored_x_;

  // Dynamic reconfigure server
  std::shared_ptr<dynamic_reconfigure::Server<elevator_docking::DockingControlConfig>> dyn_reconfig_server_;
};

} // namespace elevator_docking

#endif // ELEVATOR_DOCKING_SERVER_H
