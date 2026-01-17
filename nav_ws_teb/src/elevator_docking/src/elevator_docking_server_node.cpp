#include "elevator_docking/elevator_docking_server.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevator_docking_server");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ROS_INFO("Starting elevator docking action server...");

  try {
    elevator_docking::ElevatorDockingServer server(nh, private_nh);

    ROS_INFO("Elevator docking action server is running");
    ros::spin();
  }
  catch (const std::exception& e) {
    ROS_FATAL("Exception in elevator docking server: %s", e.what());
    return 1;
  }

  return 0;
}
