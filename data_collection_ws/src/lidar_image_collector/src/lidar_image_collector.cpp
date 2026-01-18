#include "lidar_image_collector/lidar_image_collector.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

namespace lidar_image_collector
{

LidarImageCollector::LidarImageCollector(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  , merged_cloud_(new pcl::PointCloud<pcl::PointXYZ>())
{
  // 初始化TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 加载参数
  loadParameters();

  // 设置发布器和订阅器
  setupPubSub();

  // 初始化图像
  lidar_image_ = cv::Mat::zeros(image_rows_, image_cols_, CV_8UC1);

  // 初始化dynamic reconfigure服务器
  dyn_reconfig_server_ = std::make_shared<dynamic_reconfigure::Server<lidar_image_collector::LidarImageCollectorConfig>>(private_nh_);
  dyn_reconfig_server_->setCallback(boost::bind(&LidarImageCollector::reconfigureCallback, this, _1, _2));

  ROS_INFO("Lidar Image Collector initialized successfully");
  ROS_INFO("  Robot base frame: %s", robot_base_frame_.c_str());
  ROS_INFO("  Map size: %.2f x %.2f meters", map_width_, map_height_);
  ROS_INFO("  Resolution: %.4f meters/pixel", resolution_);
  ROS_INFO("  Image size: %d x %d pixels", image_rows_, image_cols_);
  ROS_INFO("  Sensors configured: %zu", sensor_configs_.size());
  for (const auto& sensor : sensor_configs_) {
    ROS_INFO("    - %s: topic=%s, frame=%s",
             sensor.first.c_str(),
             sensor.second.topic.c_str(),
             sensor.second.frame.c_str());
  }
}

LidarImageCollector::~LidarImageCollector()
{
}

void LidarImageCollector::loadParameters()
{
  // 坐标系配置
  private_nh_.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");

  // 地图配置
  private_nh_.param<double>("map_width", map_width_, 10.0);
  private_nh_.param<double>("map_height", map_height_, 5.0);
  private_nh_.param<double>("resolution", resolution_, 0.01);

  // 图像配置
  private_nh_.param<int>("image_rows", image_rows_, 960);
  private_nh_.param<int>("image_cols", image_cols_, 960);
  private_nh_.param<int>("point_intensity", point_intensity_, 255);

  // 点云过滤范围
  private_nh_.param<double>("min_x", min_x_, -map_width_ / 2.0);
  private_nh_.param<double>("max_x", max_x_, map_width_ / 2.0);
  private_nh_.param<double>("min_y", min_y_, -map_height_ / 2.0);
  private_nh_.param<double>("max_y", max_y_, map_height_ / 2.0);
  private_nh_.param<double>("min_z", min_z_, -1.0);
  private_nh_.param<double>("max_z", max_z_, 2.0);

  // 图像处理
  private_nh_.param<bool>("apply_blur", apply_blur_, false);
  private_nh_.param<int>("blur_kernel_size", blur_kernel_size_, 3);

  // 发布话题配置
  private_nh_.param<std::string>("image_topic", image_topic_, "/lidar_image");
  private_nh_.param<std::string>("merged_cloud_topic", merged_cloud_topic_, "/merged_cloud");
  private_nh_.param<bool>("publish_merged_cloud", publish_merged_cloud_, true);

  // 传感器配置
  std::string observation_sources;
  private_nh_.param<std::string>("observation_sources", observation_sources, "");

  std::istringstream iss(observation_sources);
  std::vector<std::string> source_names;
  std::string source_name;
  while (iss >> source_name) {
    source_names.push_back(source_name);
  }

  for (const auto& name : source_names) {
    SensorConfig config;
    private_nh_.param<std::string>(name + "/topic", config.topic, "");
    private_nh_.param<std::string>(name + "/sensor_frame", config.frame, "");

    if (!config.topic.empty() && !config.frame.empty()) {
      sensor_configs_[name] = config;
      sensor_clouds_[name] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    }
  }
}

void LidarImageCollector::setupPubSub()
{
  // 发布器
  image_pub_ = nh_.advertise<sensor_msgs::Image>(image_topic_, 1);
  if (publish_merged_cloud_) {
    merged_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(merged_cloud_topic_, 1);
  }

  // 订阅器 - 先尝试订阅LaserScan，如果失败则订阅PointCloud2
  for (auto& sensor_pair : sensor_configs_) {
    const std::string& sensor_name = sensor_pair.first;
    SensorConfig& config = sensor_pair.second;

    // 使用boost::bind创建回调函数，传递sensor_name
    ros::Subscriber sub = nh_.subscribe<sensor_msgs::LaserScan>(
        config.topic, 10,
        boost::bind(&LidarImageCollector::laserScanCallback, this, _1, sensor_name));

    // 如果没有LaserScan数据，尝试订阅PointCloud2
    // 这里简化处理，实际应该根据话题类型动态订阅
    config.subscriber = sub;
  }
}

void LidarImageCollector::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg, const std::string& sensor_name)
{
  try {
    // 将LaserScan转换为PointCloud2
    sensor_msgs::PointCloud2 cloud_msg;
    laser_projector_.projectLaser(*msg, cloud_msg);

    // 转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_msg, *cloud);

    // 坐标转换到robot_base_frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (transformPointCloud(cloud, transformed_cloud, robot_base_frame_, msg->header.frame_id)) {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      sensor_clouds_[sensor_name] = transformed_cloud;
      processAndPublish();
    }
  } catch (const std::exception& e) {
    ROS_ERROR("Error in laserScanCallback for %s: %s", sensor_name.c_str(), e.what());
  }
}

void LidarImageCollector::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, const std::string& sensor_name)
{
  try {
    // 转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    // 坐标转换到robot_base_frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (transformPointCloud(cloud, transformed_cloud, robot_base_frame_, msg->header.frame_id)) {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      sensor_clouds_[sensor_name] = transformed_cloud;
      processAndPublish();
    }
  } catch (const std::exception& e) {
    ROS_ERROR("Error in pointCloudCallback for %s: %s", sensor_name.c_str(), e.what());
  }
}

void LidarImageCollector::reconfigureCallback(lidar_image_collector::LidarImageCollectorConfig& config, uint32_t level)
{
  std::lock_guard<std::mutex> lock(param_mutex_);

  image_rows_ = config.image_rows;
  image_cols_ = config.image_cols;
  map_width_ = config.map_width;
  map_height_ = config.map_height;
  resolution_ = config.resolution;
  min_x_ = config.min_x;
  max_x_ = config.max_x;
  min_y_ = config.min_y;
  max_y_ = config.max_y;
  min_z_ = config.min_z;
  max_z_ = config.max_z;
  point_intensity_ = config.point_intensity;
  apply_blur_ = config.apply_blur;
  blur_kernel_size_ = config.blur_kernel_size;

  // 重新初始化图像
  lidar_image_ = cv::Mat::zeros(image_rows_, image_cols_, CV_8UC1);

  ROS_INFO("Parameters updated via dynamic reconfigure");
  ROS_INFO("  Image size: %d x %d", image_rows_, image_cols_);
  ROS_INFO("  Map size: %.2f x %.2f meters", map_width_, map_height_);
  ROS_INFO("  Resolution: %.4f meters/pixel", resolution_);
}

bool LidarImageCollector::transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                                               const std::string& target_frame,
                                               const std::string& source_frame)
{
  if (target_frame == source_frame) {
    *cloud_out = *cloud_in;
    return true;
  }

  try {
    geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
        target_frame, source_frame, ros::Time(0), ros::Duration(0.1));

    // 手动转换点云
    cloud_out->header = cloud_in->header;
    cloud_out->points.resize(cloud_in->points.size());

    Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform.transform);

    for (size_t i = 0; i < cloud_in->points.size(); ++i) {
      Eigen::Vector3d point_in(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
      Eigen::Vector3d point_out = eigen_transform * point_in;
      cloud_out->points[i].x = point_out.x();
      cloud_out->points[i].y = point_out.y();
      cloud_out->points[i].z = point_out.z();
    }

    return true;
  } catch (const tf2::TransformException& ex) {
    ROS_WARN("Could not transform from %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

void LidarImageCollector::processAndPublish()
{
  mergePointClouds();
  convertCloudToImage();
  publishImage();

  if (publish_merged_cloud_ && merged_cloud_->points.size() > 0) {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*merged_cloud_, cloud_msg);
    cloud_msg.header.frame_id = robot_base_frame_;
    cloud_msg.header.stamp = ros::Time::now();
    merged_cloud_pub_.publish(cloud_msg);
  }
}

void LidarImageCollector::mergePointClouds()
{
  merged_cloud_->clear();

  for (const auto& sensor_pair : sensor_clouds_) {
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud = sensor_pair.second;
    if (cloud && cloud->points.size() > 0) {
      // 过滤点云
      for (const auto& point : cloud->points) {
        if (point.x >= min_x_ && point.x <= max_x_ &&
            point.y >= min_y_ && point.y <= max_y_ &&
            point.z >= min_z_ && point.z <= max_z_) {
          merged_cloud_->points.push_back(point);
        }
      }
    }
  }

  merged_cloud_->width = merged_cloud_->points.size();
  merged_cloud_->height = 1;
  merged_cloud_->is_dense = false;
}

void LidarImageCollector::convertCloudToImage()
{
  std::lock_guard<std::mutex> lock(param_mutex_);

  // 清空图像（黑色背景）
  lidar_image_ = cv::Mat::zeros(image_rows_, image_cols_, CV_8UC1);

  if (merged_cloud_->points.empty()) {
    return;
  }

  // 计算缩放比例
  double scale_x = image_cols_ / map_width_;
  double scale_y = image_rows_ / map_height_;
  
  // 图像中心
  int center_x = image_cols_ / 2;
  int center_y = image_rows_ / 2;

  // 将点云转换到图像坐标
  for (const auto& point : merged_cloud_->points) {
    // 世界坐标转换到图像坐标
    int img_x = static_cast<int>(point.x * scale_x + center_x);
    int img_y = static_cast<int>(-point.y * scale_y + center_y);  // Y轴反向

    // 检查边界
    if (img_x >= 0 && img_x < image_cols_ && img_y >= 0 && img_y < image_rows_) {
      lidar_image_.at<uchar>(img_y, img_x) = point_intensity_;
    }
  }

  // 应用高斯模糊（如果启用）
  if (apply_blur_ && blur_kernel_size_ > 0) {
    int kernel_size = blur_kernel_size_;
    if (kernel_size % 2 == 0) {
      kernel_size += 1;  // 确保核大小为奇数
    }
    cv::GaussianBlur(lidar_image_, lidar_image_, cv::Size(kernel_size, kernel_size), 0);
  }
}

void LidarImageCollector::publishImage()
{
  if (lidar_image_.empty()) {
    return;
  }

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = robot_base_frame_;

  sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "mono8", lidar_image_).toImageMsg();
  image_pub_.publish(img_msg);
}

} // namespace lidar_image_collector
