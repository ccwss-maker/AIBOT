#ifndef ELEVATOR_DOCKING_SERVER_H
#define ELEVATOR_DOCKING_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <elevator_docking/ElevatorDockingAction.h>
#include <elevator_docking/DockingControlConfig.h>
#include <elevator_docking/elevator_detector.h>
#include <elevator_docking/parking_controller.h>
#include <memory>

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
  void reconfigureCallback(DockingControlConfig& config, uint32_t level);

  bool runDetection(ElevatorDetectionResult& result, double duration = -1.0);
  bool executeDetection();
  bool executeIn();
  bool executeOut();

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::unique_ptr<actionlib::SimpleActionServer<ElevatorDockingAction>> as_;
  std::shared_ptr<ElevatorDetector> detector_;
  std::shared_ptr<ParkingController> parking_controller_;
  std::shared_ptr<dynamic_reconfigure::Server<DockingControlConfig>> dyn_reconfig_server_;

  ElevatorDetectionResult last_detection_result_;
  double control_rate_;
  double detection_duration_;
  double stored_x_;  // 进入距离，用于 out 命令
  std::string action_server_name_;
};

} // namespace elevator_docking

#endif // ELEVATOR_DOCKING_SERVER_H
