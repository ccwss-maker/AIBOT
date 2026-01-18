#ifndef LIDAR_IMAGE_COLLECTOR_H
#define LIDAR_IMAGE_COLLECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <laser_geometry/laser_geometry.h>
#include <dynamic_reconfigure/server.h>
#include <lidar_image_collector/LidarImageCollectorConfig.h>
#include <map>
#include <string>
#include <memory>

namespace lidar_image_collector
{

struct SensorConfig
{
  std::string topic;
  std::string frame;
  ros::Subscriber subscriber;
};

class LidarImageCollector
{
public:
  LidarImageCollector(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~LidarImageCollector();

private:
  // 回调函数
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg, const std::string& sensor_name);
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, const std::string& sensor_name);
  void reconfigureCallback(lidar_image_collector::LidarImageCollectorConfig& config, uint32_t level);

  // 核心功能
  void loadParameters();
  void setupPubSub();
  void processAndPublish();
  void mergePointClouds();
  void convertCloudToImage();
  void publishImage();

  // 坐标转换
  bool transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                           const std::string& target_frame,
                           const std::string& source_frame);

  // 成员变量
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 激光几何转换
  laser_geometry::LaserProjection laser_projector_;

  // 发布器和订阅器
  ros::Publisher image_pub_;
  ros::Publisher merged_cloud_pub_;
  std::map<std::string, SensorConfig> sensor_configs_;

  // Dynamic reconfigure
  std::shared_ptr<dynamic_reconfigure::Server<lidar_image_collector::LidarImageCollectorConfig>> dyn_reconfig_server_;

  // 点云数据
  std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> sensor_clouds_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud_;

  // 图像数据
  cv::Mat lidar_image_;

  // 参数
  std::string robot_base_frame_;
  double map_width_;
  double map_height_;
  double resolution_;
  int image_rows_;
  int image_cols_;
  int point_intensity_;
  bool apply_blur_;
  int blur_kernel_size_;
  double min_x_, max_x_;
  double min_y_, max_y_;
  double min_z_, max_z_;
  bool publish_merged_cloud_;
  std::string image_topic_;
  std::string merged_cloud_topic_;

  // 互斥锁
  std::mutex cloud_mutex_;
  std::mutex param_mutex_;
};

} // namespace lidar_image_collector

#endif // LIDAR_IMAGE_COLLECTOR_H
