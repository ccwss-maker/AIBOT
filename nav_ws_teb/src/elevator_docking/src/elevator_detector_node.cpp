#include "elevator_docking/elevator_detector.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevator_detector");
  
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  
  ROS_INFO("Starting elevator detector node...");
  
  try {
    elevator_docking::ElevatorDetector detector(nh, private_nh);
    
    ROS_INFO("Elevator detector node is running");
    ros::spin();
  }
  catch (const std::exception& e) {
    ROS_FATAL("Exception in elevator detector: %s", e.what());
    return 1;
  }
  
  return 0;
}
