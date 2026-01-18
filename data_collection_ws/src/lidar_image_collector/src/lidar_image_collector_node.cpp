#include "lidar_image_collector/lidar_image_collector.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_image_collector_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  try {
    lidar_image_collector::LidarImageCollector collector(nh, private_nh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("Exception in lidar_image_collector_node: %s", e.what());
    return 1;
  }

  return 0;
}
