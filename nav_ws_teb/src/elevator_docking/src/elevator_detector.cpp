#include "elevator_docking/elevator_detector.h"
#include <laser_geometry/laser_geometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

namespace elevator_docking
{

ElevatorDetector::ElevatorDetector(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  , merged_cloud_(new pcl::PointCloud<pcl::PointXYZ>())
  , elevator_detected_(false)
{
  // 初始化TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 加载参数
  loadParameters();

  // 设置发布器和订阅器
  setupPubSub();

  // 初始化dynamic reconfigure服务器
  dyn_reconfig_server_ = std::make_shared<dynamic_reconfigure::Server<elevator_docking::ElevatorDockingConfig>>(private_nh_);
  dyn_reconfig_server_->setCallback(boost::bind(&ElevatorDetector::reconfigureCallback, this, _1, _2));

  ROS_INFO("Elevator detector initialized successfully");
  ROS_INFO("  Robot base frame: %s", robot_base_frame_.c_str());
  ROS_INFO("  Map size: %.2f x %.2f meters", map_width_, map_height_);
  ROS_INFO("  Sensors configured: %zu", sensor_configs_.size());
  for (const auto& sensor : sensor_configs_) {
    ROS_INFO("    - %s: topic=%s, frame=%s",
             sensor.first.c_str(),
             sensor.second.topic.c_str(),
             sensor.second.frame.c_str());
  }
}

ElevatorDetector::~ElevatorDetector()
{
}

bool ElevatorDetector::getLastDetection(ElevatorDetectionResult& result) const
{
  if (!elevator_detected_) {
    return false;
  }
  result = last_detection_;
  return true;
}

bool ElevatorDetector::isElevatorDetected() const
{
  return elevator_detected_;
}

void ElevatorDetector::setDetectionEnabled(bool enable)
{
  enable_detection_ = enable;
  if (!enable) {
    elevator_detected_ = false;
  }
  ROS_INFO("Detection %s", enable ? "enabled" : "disabled");
}

void ElevatorDetector::loadParameters()
{
  // ============================================================================
  // 坐标系配置
  // ============================================================================
  private_nh_.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");

  // ============================================================================
  // 地图配置
  // ============================================================================
  private_nh_.param<double>("map_width", map_width_, 5.0);
  private_nh_.param<double>("map_height", map_height_, 5.0);

  // ============================================================================
  // 发布话题配置
  // ============================================================================
  private_nh_.param<bool>("publish_filtered_points", publish_filtered_points_, true);
  private_nh_.param<bool>("publish_line_markers", publish_line_markers_, true);
  private_nh_.param<bool>("publish_merged_line_markers", publish_merged_line_markers_, true);
  private_nh_.param<bool>("publish_connected_line_markers", publish_connected_line_markers_, true);
  private_nh_.param<bool>("publish_elevator_marker", publish_elevator_marker_, false);
  private_nh_.param<std::string>("filtered_points_topic", filtered_points_topic_, "/elevator/filtered_points");
  private_nh_.param<std::string>("line_markers_topic", line_markers_topic_, "/elevator/line_markers");
  private_nh_.param<std::string>("merged_line_markers_topic", merged_line_markers_topic_, "/elevator/merged_line_markers");
  private_nh_.param<std::string>("connected_line_markers_topic", connected_line_markers_topic_, "/elevator/connected_line_markers");
  private_nh_.param<std::string>("elevator_marker_topic", elevator_marker_topic_, "/elevator/marker");

  // ============================================================================
  // 电梯检测参数（enable_detection_是内部控制，默认false）
  // ============================================================================
  enable_detection_ = false;  // 默认关闭,由action server控制
  private_nh_.param<double>("min_door_width", min_door_width_, 0.8);
  private_nh_.param<double>("max_door_width", max_door_width_, 1.5);
  private_nh_.param<double>("min_door_depth", min_door_depth_, 1.0);
  private_nh_.param<double>("max_door_depth", max_door_depth_, 2.5);

  // ============================================================================
  // 点云聚类参数（阶段1: 预过滤）
  // ============================================================================
  private_nh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.1);
  private_nh_.param<int>("min_cluster_size", min_cluster_size_, 5);
  private_nh_.param<int>("max_cluster_size", max_cluster_size_, 10000);

  // ============================================================================
  // 直线检测参数（阶段2: RANSAC检测）
  // ============================================================================
  private_nh_.param<double>("ransac_distance_threshold", ransac_distance_threshold_, 0.05);
  private_nh_.param<int>("min_line_points", min_line_points_, 10);
  private_nh_.param<double>("line_gap_threshold", line_gap_threshold_, 0.2);

  // ============================================================================
  // 线段过滤参数（阶段3: 长度过滤）
  // ============================================================================
  private_nh_.param<double>("min_line_length", min_line_length_, 0.1);
  private_nh_.param<double>("max_line_length", max_line_length_, 2.0);

  // ============================================================================
  // 平行线合并参数（阶段4: 平行线分组与合并）
  // ============================================================================
  private_nh_.param<double>("merge_angle_threshold", merge_angle_threshold_, 5.0);
  private_nh_.param<double>("merge_distance_threshold", merge_distance_threshold_, 0.05);

  // ============================================================================
  // 电梯识别参数
  // ============================================================================
  private_nh_.param<double>("parallel_angle_threshold", parallel_angle_threshold_, 10.0);
  private_nh_.param<double>("max_parallel_group_centroid_distance", max_parallel_group_centroid_distance_, 0.3);

  // ============================================================================
  // 可视化参数
  // ============================================================================
  private_nh_.param<double>("marker_lifetime", marker_lifetime_, 0.5);
  private_nh_.param<double>("marker_scale", marker_scale_, 0.05);

  // ============================================================================
  // 读取传感器配置
  // ============================================================================
  std::string observation_sources;
  private_nh_.param<std::string>("observation_sources", observation_sources, "");

  std::istringstream iss(observation_sources);
  std::vector<std::string> sources;
  std::string source;
  while (iss >> source) {
    sources.push_back(source);
  }

  for (const auto& src : sources) {
    SensorConfig config;
    private_nh_.param<std::string>(src + "/topic", config.topic, "");
    private_nh_.param<std::string>(src + "/sensor_frame", config.frame, "");

    if (!config.topic.empty() && !config.frame.empty()) {
      sensor_configs_[src] = config;
      sensor_clouds_[src] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    } else {
      ROS_WARN("Sensor '%s' has incomplete configuration, skipping", src.c_str());
    }
  }

  if (sensor_configs_.empty()) {
    ROS_WARN("No valid sensors configured!");
  }
}

void ElevatorDetector::setupPubSub()
{
  // ============================================================================
  // 订阅所有配置的传感器
  // ============================================================================
  for (const auto& sensor : sensor_configs_) {
    const std::string& name = sensor.first;
    const SensorConfig& config = sensor.second;

    laser_subscribers_[name] = nh_.subscribe<sensor_msgs::LaserScan>(
      config.topic, 10,
      boost::bind(&ElevatorDetector::laserScanCallback, this, _1, name));

    ROS_DEBUG("Subscribed to sensor '%s' on topic '%s'", name.c_str(), config.topic.c_str());
  }

  // ============================================================================
  // 创建发布器
  // ============================================================================
  if (publish_filtered_points_) {
    filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(filtered_points_topic_, 10);
    ROS_DEBUG("Publishing filtered points to '%s'", filtered_points_topic_.c_str());
  }

  if (publish_line_markers_) {
    line_markers_pub_ = nh_.advertise<visualization_msgs::Marker>(line_markers_topic_, 10);
    ROS_DEBUG("Publishing line markers to '%s'", line_markers_topic_.c_str());
  }

  if (publish_merged_line_markers_) {
    merged_line_markers_pub_ = nh_.advertise<visualization_msgs::Marker>(merged_line_markers_topic_, 10);
    ROS_DEBUG("Publishing merged line markers to '%s'", merged_line_markers_topic_.c_str());
  }

  if (publish_connected_line_markers_) {
    connected_line_markers_pub_ = nh_.advertise<visualization_msgs::Marker>(connected_line_markers_topic_, 10);
    ROS_DEBUG("Publishing connected line markers to '%s'", connected_line_markers_topic_.c_str());
  }

  if (publish_elevator_marker_) {
    elevator_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(elevator_marker_topic_, 10);
    ROS_DEBUG("Publishing elevator marker to '%s'", elevator_marker_topic_.c_str());
  }
}

void ElevatorDetector::reconfigureCallback(elevator_docking::ElevatorDockingConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure request received");

  // 地图配置
  map_width_ = config.map_width;
  map_height_ = config.map_height;

  // 发布开关
  publish_filtered_points_ = config.publish_filtered_points;
  publish_line_markers_ = config.publish_line_markers;
  publish_merged_line_markers_ = config.publish_merged_line_markers;
  publish_connected_line_markers_ = config.publish_connected_line_markers;
  publish_elevator_marker_ = config.publish_elevator_marker;

  // 电梯检测参数
  min_door_width_ = config.min_door_width;
  max_door_width_ = config.max_door_width;
  min_door_depth_ = config.min_door_depth;
  max_door_depth_ = config.max_door_depth;

  // 点云聚类参数（阶段1）
  cluster_tolerance_ = config.cluster_tolerance;
  min_cluster_size_ = config.min_cluster_size;
  max_cluster_size_ = config.max_cluster_size;

  // 直线检测参数（阶段2）
  ransac_distance_threshold_ = config.ransac_distance_threshold;
  min_line_points_ = config.min_line_points;
  line_gap_threshold_ = config.line_gap_threshold;

  // 线段过滤参数（阶段3）
  min_line_length_ = config.min_line_length;
  max_line_length_ = config.max_line_length;

  // 平行线合并参数（阶段4）
  merge_angle_threshold_ = config.merge_angle_threshold;
  merge_distance_threshold_ = config.merge_distance_threshold;

  // 电梯识别参数
  parallel_angle_threshold_ = config.parallel_angle_threshold;
  max_parallel_group_centroid_distance_ = config.max_parallel_group_centroid_distance;

  // 可视化参数
  marker_lifetime_ = config.marker_lifetime;
  marker_scale_ = config.marker_scale;
}

void ElevatorDetector::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg,
                                         const std::string& sensor_name)
{
  // ============================================================================
  // 1. 将LaserScan转换为PointCloud2
  // ============================================================================
  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud2 cloud_msg;
  projector.projectLaser(*msg, cloud_msg);

  // 转换为PCL格式
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(cloud_msg, *cloud);

  // ============================================================================
  // 2. 转换到机器人基座坐标系
  // ============================================================================
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (!transformPointCloud(cloud, transformed_cloud, robot_base_frame_, msg->header.frame_id)) {
    ROS_WARN_THROTTLE(5.0, "Failed to transform point cloud from %s to %s",
                      msg->header.frame_id.c_str(), robot_base_frame_.c_str());
    return;
  }

  // ============================================================================
  // 3. 过滤点云（保留地图范围内的点）
  // ============================================================================
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  filterPointCloud(transformed_cloud, filtered_cloud);

  // 保存该传感器的点云
  sensor_clouds_[sensor_name] = filtered_cloud;
  last_update_time_ = ros::Time::now();

  // ============================================================================
  // 4. 合并所有传感器的点云
  // ============================================================================
  mergeSensorClouds();

  // ============================================================================
  // 5. 发布过滤后的点云（如果启用）
  // ============================================================================
  if (publish_filtered_points_) {
    publishFilteredPointCloud();
  }

  // ============================================================================
  // 6. 执行电梯检测（如果启用且有足够的点）
  // ============================================================================
  if (enable_detection_ && merged_cloud_->size() > 0) {
    ElevatorDetectionResult result;
    if (detectElevator(merged_cloud_, result)) {
      last_detection_ = result;
      elevator_detected_ = true;

      ROS_INFO_THROTTLE(2.0, "Elevator detected: pos=(%.2f, %.2f), size=(%.2fx%.2f), angle=%.1f deg, confidence=%.2f",
                        result.x, result.y, result.width, result.depth,
                        result.angle * 180.0 / M_PI, result.confidence);

      // 发布可视化标记
      if (publish_elevator_marker_) {
        publishElevatorMarker(result);
      }
    }
    // 注意：检测失败时不再清除 elevator_detected_ 标志
    // 这样可以保留之前成功的检测结果，由 runDetection() 选择置信度最高的
  }

  
}

bool ElevatorDetector::transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                                           const std::string& target_frame,
                                           const std::string& source_frame)
{
  try {
    geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
      target_frame, source_frame, ros::Time(0), ros::Duration(0.1));

    cloud_out->clear();
    cloud_out->reserve(cloud_in->size());

    for (const auto& point : cloud_in->points) {
      geometry_msgs::PointStamped point_in, point_out;
      point_in.header.frame_id = source_frame;
      point_in.point.x = point.x;
      point_in.point.y = point.y;
      point_in.point.z = point.z;

      tf2::doTransform(point_in, point_out, transform);

      pcl::PointXYZ pt;
      pt.x = point_out.point.x;
      pt.y = point_out.point.y;
      pt.z = point_out.point.z;
      cloud_out->push_back(pt);
    }

    cloud_out->header.frame_id = target_frame;
    return true;
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(5.0, "TF transform failed: %s", ex.what());
    return false;
  }
}

void ElevatorDetector::filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
  cloud_out->clear();
  cloud_out->reserve(cloud_in->size());

  double half_width = map_width_ / 2.0;
  double half_height = map_height_ / 2.0;

  for (const auto& point : cloud_in->points) {
    // 检查点是否在地图范围内 (以robot_base_frame为中心)
    if (std::abs(point.x) <= half_width && std::abs(point.y) <= half_height) {
      cloud_out->push_back(point);
    }
  }

  cloud_out->header = cloud_in->header;
}

void ElevatorDetector::mergeSensorClouds()
{
  merged_cloud_->clear();

  // 合并所有传感器的点云
  for (const auto& sensor_cloud : sensor_clouds_) {
    *merged_cloud_ += *(sensor_cloud.second);
  }

  merged_cloud_->header.frame_id = robot_base_frame_;
}

bool ElevatorDetector::detectElevator(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                      ElevatorDetectionResult& result)
{
  if (cloud->empty()) {
    return false;
  }

  // 步骤1: 使用三阶段方法检测直线段
  std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> lines;
  detectLineSegments(cloud, lines);

  // 发布检测到的直线段
  if (publish_line_markers_ && !lines.empty()) {
    publishLineMarkers(lines);
  }

  // 步骤2: 平行线分组与合并（阶段4）
  std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> merged_lines;
  groupAndMergeParallelLines(lines, merged_lines);

  // 发布合并后的线段
  if (publish_merged_line_markers_ && !merged_lines.empty()) {
    publishMergedLineMarkers(merged_lines);
  }

  // 步骤3: 连接平行线组形成矩形结构（阶段5）
  std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> connected_lines;
  if (!connectParallelGroups(merged_lines, connected_lines)) {
    ROS_WARN_THROTTLE(2.0, "Failed to connect parallel groups into rectangle");
    return false;
  }

  // 发布连接后的线段
  if (publish_connected_line_markers_ && !connected_lines.empty()) {
    publishConnectedLineMarkers(connected_lines);
  }

  // 步骤4: 从4条线段组成的矩形识别电梯参数
  if (!recognizeElevatorFromRectangle(connected_lines, result)) {
    ROS_WARN_THROTTLE(2.0, "Failed to recognize elevator from rectangle");
    return false;
  }

  // 步骤5: 验证检测结果
  if (!validateElevatorResult(result)) {
    ROS_WARN_THROTTLE(2.0, "Elevator validation failed");
    return false;
  }

  return true;
}

void ElevatorDetector::publishFilteredPointCloud()
{
  if (filtered_cloud_pub_.getNumSubscribers() == 0 || merged_cloud_->empty()) {
    return;
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*merged_cloud_, cloud_msg);
  cloud_msg.header.frame_id = robot_base_frame_;
  cloud_msg.header.stamp = ros::Time::now();

  filtered_cloud_pub_.publish(cloud_msg);
}

void ElevatorDetector::publishLineMarkers(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& lines)
{
  if (line_markers_pub_.getNumSubscribers() == 0 || lines.empty()) {
    return;
  }

  visualization_msgs::Marker line_list;
  line_list.header.frame_id = robot_base_frame_;
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "detected_lines";
  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.action = visualization_msgs::Marker::ADD;

  // 初始化pose的quaternion
  line_list.pose.orientation.w = 1.0;

  line_list.scale.x = marker_scale_ * 2.0;  // 线条粗细（是电梯marker的2倍）

  // 不同颜色表示不同的线段
  line_list.color.r = 1.0;
  line_list.color.g = 0.0;
  line_list.color.b = 1.0;  // 洋红色
  line_list.color.a = 1.0;

  line_list.lifetime = ros::Duration(marker_lifetime_);

  // 添加所有线段
  for (const auto& line : lines) {
    geometry_msgs::Point p1, p2;

    p1.x = line.first.x;
    p1.y = line.first.y;
    p1.z = line.first.z;

    p2.x = line.second.x;
    p2.y = line.second.y;
    p2.z = line.second.z;

    line_list.points.push_back(p1);
    line_list.points.push_back(p2);
  }

  line_markers_pub_.publish(line_list);
}

void ElevatorDetector::publishElevatorMarker(const ElevatorDetectionResult& result)
{
  if (elevator_marker_pub_.getNumSubscribers() == 0) {
    return;
  }

  visualization_msgs::Marker marker = createRectangleMarker(result);
  elevator_marker_pub_.publish(marker);
}

visualization_msgs::Marker ElevatorDetector::createRectangleMarker(const ElevatorDetectionResult& result)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = robot_base_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "elevator";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  // 初始化pose的quaternion
  marker.pose.orientation.w = 1.0;

  // 计算矩形的4个角点（以result的中心和角度为基准）
  double cos_a = std::cos(result.angle);
  double sin_a = std::sin(result.angle);
  double half_width = result.width / 2.0;
  double half_depth = result.depth / 2.0;

  geometry_msgs::Point p1, p2, p3, p4;

  // 角点1 (前左)
  p1.x = result.x + (-half_depth * cos_a - half_width * sin_a);
  p1.y = result.y + (-half_depth * sin_a + half_width * cos_a);
  p1.z = 0.0;

  // 角点2 (后左)
  p2.x = result.x + (half_depth * cos_a - half_width * sin_a);
  p2.y = result.y + (half_depth * sin_a + half_width * cos_a);
  p2.z = 0.0;

  // 角点3 (后右)
  p3.x = result.x + (half_depth * cos_a + half_width * sin_a);
  p3.y = result.y + (half_depth * sin_a - half_width * cos_a);
  p3.z = 0.0;

  // 角点4 (前右)
  p4.x = result.x + (-half_depth * cos_a + half_width * sin_a);
  p4.y = result.y + (-half_depth * sin_a - half_width * cos_a);
  p4.z = 0.0;

  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker.points.push_back(p3);
  marker.points.push_back(p4);
  marker.points.push_back(p1); // 闭合矩形

  // 设置可视化属性
  marker.scale.x = marker_scale_;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(marker_lifetime_);

  return marker;
}

void ElevatorDetector::detectLineSegments(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                          std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& lines)
{
  lines.clear();

  if (cloud->empty()) {
    ROS_WARN_THROTTLE(5.0, "Empty point cloud for line detection");
    return;
  }

  ROS_DEBUG("Starting three-stage line detection with %zu input points", cloud->size());

  // ============================================================================
  // 阶段1: 点云欧氏聚类预过滤
  // ============================================================================
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  clusterPointCloud(cloud, clusters);

  if (clusters.empty()) {
    ROS_DEBUG("No clusters found after Euclidean clustering");
    return;
  }

  ROS_DEBUG("Stage 1 complete: %zu clusters found", clusters.size());

  // ============================================================================
  // 阶段2: 对每个簇进行RANSAC检测并根据间隙拆分成最小单位线段
  // ============================================================================
  std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> all_segments;

  for (size_t i = 0; i < clusters.size(); ++i) {
    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> cluster_segments;
    detectAndSplitLines(clusters[i], cluster_segments);

    ROS_DEBUG("Cluster %zu: %zu segments detected", i, cluster_segments.size());

    // 合并到总线段集合
    all_segments.insert(all_segments.end(), cluster_segments.begin(), cluster_segments.end());
  }

  ROS_DEBUG("Stage 2 complete: %zu total segments (before length filtering)", all_segments.size());

  // ============================================================================
  // 阶段3: 根据长度过滤线段
  // ============================================================================
  filterLinesByLength(all_segments, lines);

  ROS_DEBUG_THROTTLE(1.0, "Stage 3 complete: %zu lines found (after length filtering) from %zu input points",
                     lines.size(), cloud->size());
}

bool ElevatorDetector::validateElevatorResult(const ElevatorDetectionResult& result)
{
  // 验证宽度
  if (result.width < min_door_width_ || result.width > max_door_width_) {
    ROS_DEBUG("Width validation failed: %.2f not in [%.2f, %.2f]",
              result.width, min_door_width_, max_door_width_);
    return false;
  }

  // 验证深度
  if (result.depth < min_door_depth_ || result.depth > max_door_depth_) {
    ROS_DEBUG("Depth validation failed: %.2f not in [%.2f, %.2f]",
              result.depth, min_door_depth_, max_door_depth_);
    return false;
  }

  // 验证置信度
  if (result.confidence < 0.3) {
    ROS_DEBUG("Confidence too low: %.2f", result.confidence);
    return false;
  }

  return true;
}

// ==============================================================================
// 阶段1: 点云欧氏聚类预过滤
// ==============================================================================
void ElevatorDetector::clusterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                         std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters)
{
  clusters.clear();

  if (cloud->empty()) {
    return;
  }

  // 创建KdTree用于快速邻域搜索
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  tree->setInputCloud(cloud);

  // 欧氏聚类提取
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);  // 聚类距离阈值
  ec.setMinClusterSize(min_cluster_size_);     // 最小簇点数
  ec.setMaxClusterSize(max_cluster_size_);     // 最大簇点数
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  ROS_DEBUG("Euclidean clustering: %zu clusters found with tolerance=%.3fm, min_size=%d, max_size=%d",
            cluster_indices.size(), cluster_tolerance_, min_cluster_size_, max_cluster_size_);

  // 将索引转换为点云簇
  for (const auto& indices : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
    cluster->reserve(indices.indices.size());

    for (int idx : indices.indices) {
      cluster->push_back(cloud->points[idx]);
    }

    cluster->width = cluster->size();
    cluster->height = 1;
    cluster->is_dense = true;

    clusters.push_back(cluster);

    ROS_DEBUG("Cluster %zu: %zu points", clusters.size() - 1, cluster->size());
  }
}

// ==============================================================================
// 阶段2: RANSAC检测并根据间隙拆分成最小单位线段
// ==============================================================================
void ElevatorDetector::detectAndSplitLines(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
                                           std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& segments)
{
  segments.clear();

  if (cluster->size() < static_cast<size_t>(min_line_points_)) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZ>(*cluster));

  // 循环检测所有可能的直线
  int iteration = 0;
  while (remaining_cloud->size() >= static_cast<size_t>(min_line_points_))
  {
    // 步骤1: 使用RANSAC拟合直线模型
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model(
      new pcl::SampleConsensusModelLine<pcl::PointXYZ>(remaining_cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(ransac_distance_threshold_);

    if (!ransac.computeModel()) {
      break;
    }

    // 步骤2: 获取inliers
    std::vector<int> inliers;
    ransac.getInliers(inliers);

    if (inliers.size() < static_cast<size_t>(min_line_points_)) {
      break;
    }

    // 步骤3: 获取拟合直线的参数
    Eigen::VectorXf line_coeffs;
    ransac.getModelCoefficients(line_coeffs);

    Eigen::Vector3f line_point(line_coeffs[0], line_coeffs[1], line_coeffs[2]);
    Eigen::Vector3f line_dir(line_coeffs[3], line_coeffs[4], line_coeffs[5]);
    line_dir.normalize();

    // 步骤4: 将所有inliers投影到直线上，并记录每个点的投影参数和索引
    struct ProjectedPoint {
      double t;           // 投影参数
      pcl::PointXYZ pt;   // 原始点
    };

    std::vector<ProjectedPoint> projected_points;
    projected_points.reserve(inliers.size());

    for (int idx : inliers) {
      const auto& pt = remaining_cloud->points[idx];
      Eigen::Vector3f point(pt.x, pt.y, pt.z);
      double t = (point - line_point).dot(line_dir);

      projected_points.push_back({t, pt});
    }

    // 步骤5: 按投影参数t排序
    std::sort(projected_points.begin(), projected_points.end(),
              [](const ProjectedPoint& a, const ProjectedPoint& b) { return a.t < b.t; });

    // 步骤6: 根据间隙拆分成多条线段
    std::vector<std::vector<ProjectedPoint>> split_segments;
    std::vector<ProjectedPoint> current_segment;
    current_segment.push_back(projected_points[0]);

    for (size_t i = 1; i < projected_points.size(); ++i) {
      double gap = projected_points[i].t - projected_points[i-1].t;

      if (gap > line_gap_threshold_) {
        // 发现间隙，保存当前线段，开始新线段
        if (current_segment.size() >= static_cast<size_t>(min_line_points_)) {
          split_segments.push_back(current_segment);
        }
        current_segment.clear();
      }

      current_segment.push_back(projected_points[i]);
    }

    // 保存最后一个线段
    if (current_segment.size() >= static_cast<size_t>(min_line_points_)) {
      split_segments.push_back(current_segment);
    }

    // 步骤7: 为每个拆分后的线段计算端点
    for (const auto& seg : split_segments) {
      if (seg.empty()) continue;

      double min_t = seg.front().t;
      double max_t = seg.back().t;

      Eigen::Vector3f start_point = line_point + min_t * line_dir;
      Eigen::Vector3f end_point = line_point + max_t * line_dir;

      pcl::PointXYZ pt_start, pt_end;
      pt_start.x = start_point.x();
      pt_start.y = start_point.y();
      pt_start.z = start_point.z();
      pt_end.x = end_point.x();
      pt_end.y = end_point.y();
      pt_end.z = end_point.z();

      segments.push_back(std::make_pair(pt_start, pt_end));

      double seg_length = max_t - min_t;
      ROS_DEBUG("  Split segment: length=%.2fm, points=%zu", seg_length, seg.size());
    }

    // 步骤8: 移除已检测的inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    outlier_cloud->reserve(remaining_cloud->size() - inliers.size());

    for (size_t i = 0; i < remaining_cloud->size(); ++i) {
      if (std::find(inliers.begin(), inliers.end(), i) == inliers.end()) {
        outlier_cloud->push_back(remaining_cloud->points[i]);
      }
    }
    remaining_cloud = outlier_cloud;

    iteration++;
  }
}

// ==============================================================================
// 阶段3: 根据长度过滤线段
// ==============================================================================
void ElevatorDetector::filterLinesByLength(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& segments,
                                           std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& filtered_lines)
{
  filtered_lines.clear();

  for (const auto& seg : segments) {
    double dx = seg.second.x - seg.first.x;
    double dy = seg.second.y - seg.first.y;
    double dz = seg.second.z - seg.first.z;
    double length = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (length >= min_line_length_ && length <= max_line_length_) {
      filtered_lines.push_back(seg);
      ROS_DEBUG("Line accepted: length=%.2fm", length);
    } else {
      ROS_DEBUG("Line rejected: length=%.2fm out of range [%.2f, %.2f]m",
                length, min_line_length_, max_line_length_);
    }
  }

  ROS_DEBUG("Length filtering: %zu/%zu lines passed", filtered_lines.size(), segments.size());
}

// ==============================================================================
// 阶段4: 平行线分组与合并
// ==============================================================================
void ElevatorDetector::groupAndMergeParallelLines(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& lines,
                                                  std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& merged_lines)
{
  merged_lines.clear();

  if (lines.empty()) {
    return;
  }

  ROS_DEBUG("Starting parallel line grouping and merging with %zu input lines", lines.size());

  // 使用并查集的思想，对每条线段标记是否已被合并
  std::vector<bool> merged(lines.size(), false);

  for (size_t i = 0; i < lines.size(); ++i) {
    if (merged[i]) {
      continue;
    }

    // 创建一个新组，从当前线段开始
    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> group;
    group.push_back(lines[i]);
    merged[i] = true;

    // 查找所有与第i条线平行且距离足够近的线段
    for (size_t j = i + 1; j < lines.size(); ++j) {
      if (merged[j]) {
        continue;
      }

      // 检查是否平行
      double angle_diff = computeAngleBetweenLines(lines[i], lines[j]);
      if (angle_diff > merge_angle_threshold_) {
        continue;  // 不平行
      }

      // 检查法向距离
      double normal_dist = computeNormalDistance(lines[i], lines[j]);
      if (normal_dist > merge_distance_threshold_) {
        continue;  // 距离太远
      }

      // 满足条件，加入组
      group.push_back(lines[j]);
      merged[j] = true;

      ROS_DEBUG("  Lines %zu and %zu are parallel (angle_diff=%.2f deg) and close (dist=%.3fm), merging",
                i, j, angle_diff, normal_dist);
    }

    // 合并组内所有线段
    if (group.size() == 1) {
      // 只有一条线，直接保留
      merged_lines.push_back(group[0]);
    } else {
      // 多条线，需要合并
      std::pair<pcl::PointXYZ, pcl::PointXYZ> merged_line = group[0];
      for (size_t k = 1; k < group.size(); ++k) {
        merged_line = mergeLines(merged_line, group[k]);
      }
      merged_lines.push_back(merged_line);

      ROS_DEBUG("  Group %zu: merged %zu lines into 1", merged_lines.size() - 1, group.size());
    }
  }

  ROS_DEBUG("Parallel line merging complete: %zu input lines -> %zu merged lines",
            lines.size(), merged_lines.size());
}

// 计算两条线段的方向角度差（度）
double ElevatorDetector::computeAngleBetweenLines(const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line1,
                                                  const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line2)
{
  // 计算方向向量
  double dx1 = line1.second.x - line1.first.x;
  double dy1 = line1.second.y - line1.first.y;

  double dx2 = line2.second.x - line2.first.x;
  double dy2 = line2.second.y - line2.first.y;

  // 归一化
  double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
  double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

  if (len1 < 1e-6 || len2 < 1e-6) {
    return 180.0;  // 退化为点，认为不平行
  }

  dx1 /= len1;
  dy1 /= len1;
  dx2 /= len2;
  dy2 /= len2;

  // 计算点积（cos θ）
  double dot_product = dx1 * dx2 + dy1 * dy2;

  // 限制在[-1, 1]范围内，避免数值误差
  dot_product = std::max(-1.0, std::min(1.0, dot_product));

  // 计算角度（弧度）
  double angle_rad = std::acos(std::abs(dot_product));  // 使用abs，因为方向相反的线也认为是平行

  // 转换为度
  double angle_deg = angle_rad * 180.0 / M_PI;

  return angle_deg;
}

// 计算两条平行线段之间的法向距离
double ElevatorDetector::computeNormalDistance(const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line1,
                                               const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line2)
{
  // 使用line1的起点和方向向量
  double dx1 = line1.second.x - line1.first.x;
  double dy1 = line1.second.y - line1.first.y;
  double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);

  if (len1 < 1e-6) {
    return 0.0;
  }

  // 归一化方向向量
  dx1 /= len1;
  dy1 /= len1;

  // 法向量（垂直于line1）
  double nx = -dy1;
  double ny = dx1;

  // 从line1的起点到line2的起点的向量
  double vx = line2.first.x - line1.first.x;
  double vy = line2.first.y - line1.first.y;

  // 法向距离 = 向量在法向量上的投影
  double normal_dist = std::abs(vx * nx + vy * ny);

  return normal_dist;
}

// 合并两条平行线段
std::pair<pcl::PointXYZ, pcl::PointXYZ> ElevatorDetector::mergeLines(const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line1,
                                                                      const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line2)
{
  // 计算line1的方向向量
  double dx = line1.second.x - line1.first.x;
  double dy = line1.second.y - line1.first.y;
  double len = std::sqrt(dx * dx + dy * dy);

  if (len < 1e-6) {
    return line1;  // 退化情况
  }

  // 归一化方向向量
  dx /= len;
  dy /= len;

  // 将line1和line2的四个端点投影到line1的方向上
  auto project = [&](const pcl::PointXYZ& pt) -> double {
    double vx = pt.x - line1.first.x;
    double vy = pt.y - line1.first.y;
    return vx * dx + vy * dy;
  };

  double t1_start = 0.0;
  double t1_end = len;
  double t2_start = project(line2.first);
  double t2_end = project(line2.second);

  // 确保start < end
  if (t2_start > t2_end) {
    std::swap(t2_start, t2_end);
  }

  // 找到合并后的范围
  double t_min = std::min(t1_start, t2_start);
  double t_max = std::max(t1_end, t2_end);

  // 计算合并后的端点
  pcl::PointXYZ merged_start, merged_end;

  merged_start.x = line1.first.x + t_min * dx;
  merged_start.y = line1.first.y + t_min * dy;
  merged_start.z = 0.0;

  merged_end.x = line1.first.x + t_max * dx;
  merged_end.y = line1.first.y + t_max * dy;
  merged_end.z = 0.0;

  return std::make_pair(merged_start, merged_end);
}

// 发布合并后的线段标记
void ElevatorDetector::publishMergedLineMarkers(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& lines)
{
  if (merged_line_markers_pub_.getNumSubscribers() == 0 || lines.empty()) {
    return;
  }

  visualization_msgs::Marker line_list;
  line_list.header.frame_id = robot_base_frame_;
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "merged_lines";
  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.action = visualization_msgs::Marker::ADD;

  // 初始化pose的quaternion
  line_list.pose.orientation.w = 1.0;

  line_list.scale.x = marker_scale_ * 3.0;  // 线条粗细（是原始线段的3倍）

  // 使用不同颜色（cyan青色）表示合并后的线段
  line_list.color.r = 0.0;
  line_list.color.g = 1.0;
  line_list.color.b = 1.0;  // 青色
  line_list.color.a = 1.0;

  line_list.lifetime = ros::Duration(marker_lifetime_);

  // 添加所有线段
  for (const auto& line : lines) {
    geometry_msgs::Point p1, p2;

    p1.x = line.first.x;
    p1.y = line.first.y;
    p1.z = line.first.z;

    p2.x = line.second.x;
    p2.y = line.second.y;
    p2.z = line.second.z;

    line_list.points.push_back(p1);
    line_list.points.push_back(p2);
  }

  merged_line_markers_pub_.publish(line_list);
}

// ==============================================================================
// 阶段5: 连接平行线组形成电梯矩形结构
// ==============================================================================
bool ElevatorDetector::connectParallelGroups(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& merged_lines,
                                             std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& connected_lines)
{
  connected_lines.clear();

  if (merged_lines.empty()) {
    return false;
  }

  ROS_DEBUG("Starting parallel group connection with %zu merged lines", merged_lines.size());

  // 步骤1: 将线段按平行关系分组
  std::vector<std::vector<int>> groups;
  std::vector<bool> grouped(merged_lines.size(), false);

  for (size_t i = 0; i < merged_lines.size(); ++i) {
    if (grouped[i]) {
      continue;
    }

    std::vector<int> group;
    group.push_back(i);
    grouped[i] = true;

    // 查找所有与第i条线平行的线
    for (size_t j = i + 1; j < merged_lines.size(); ++j) {
      if (grouped[j]) {
        continue;
      }

      double angle_diff = computeAngleBetweenLines(merged_lines[i], merged_lines[j]);
      if (angle_diff <= merge_angle_threshold_) {
        group.push_back(j);
        grouped[j] = true;
      }
    }

    groups.push_back(group);
  }

  // ROS_INFO("Found %zu parallel groups", groups.size());
  // for (size_t i = 0; i < groups.size(); ++i) {
  //   ROS_INFO("  Group %zu: %zu lines", i, groups[i].size());
  // }

  // 步骤2: 初步验证组数
  if (groups.size() < 2) {
    ROS_ERROR("Expected at least 2 parallel groups, but found %zu groups. Cannot form rectangle.", groups.size());
    return false;
  }

  // 步骤2.1: 过滤掉不是2条线的组
  std::vector<std::vector<int>> filtered_groups;
  for (size_t i = 0; i < groups.size(); ++i) {
    if (groups[i].size() == 2) {
      filtered_groups.push_back(groups[i]);
    } else {
      ROS_DEBUG("Removing group %zu: has %zu lines (expected 2)", i, groups[i].size());
    }
  }
  groups = filtered_groups;

  ROS_INFO("After size filtering: %zu groups remaining", groups.size());
  for (size_t i = 0; i < groups.size(); ++i) {
    ROS_INFO("  Group %zu: %zu lines", i, groups[i].size());
  }

  // 步骤2.2: 基于平行线组形心距离的滤波
  // 计算每个组的形心（每组两条线各自的中点，再求这两个中点的中点）
  // 如果是电梯的盒子，两组平行线的形心应该靠得很近；如果是杂物，形心距离会很远
  if (groups.size() > 2) {
    std::vector<std::vector<int>> centroid_filtered_groups;

    // 计算每组的形心
    auto computeGroupCentroid = [&](const std::vector<int>& group) -> pcl::PointXYZ {
      pcl::PointXYZ centroid;
      centroid.x = 0.0;
      centroid.y = 0.0;
      centroid.z = 0.0;

      // 计算每条线的中点，然后求平均
      for (int line_idx : group) {
        const auto& line = merged_lines[line_idx];
        double mid_x = (line.first.x + line.second.x) / 2.0;
        double mid_y = (line.first.y + line.second.y) / 2.0;
        centroid.x += mid_x;
        centroid.y += mid_y;
      }

      centroid.x /= group.size();
      centroid.y /= group.size();

      return centroid;
    };

    // 计算组间形心距离
    auto computeCentroidDistance = [](const pcl::PointXYZ& c1, const pcl::PointXYZ& c2) -> double {
      double dx = c1.x - c2.x;
      double dy = c1.y - c2.y;
      return std::sqrt(dx * dx + dy * dy);
    };

    ROS_INFO("Computing group centroids for %zu groups", groups.size());

    // 对于每组，检查是否与其他组的形心距离在合理范围内
    for (size_t i = 0; i < groups.size(); ++i) {
      if (groups[i].size() != 2) {
        continue;
      }

      pcl::PointXYZ centroid_i = computeGroupCentroid(groups[i]);
      bool valid_group = false;

      // 检查与其他组的距离
      for (size_t j = 0; j < groups.size(); ++j) {
        if (i == j || groups[j].size() != 2) {
          continue;
        }

        pcl::PointXYZ centroid_j = computeGroupCentroid(groups[j]);
        double distance = computeCentroidDistance(centroid_i, centroid_j);

        ROS_INFO("  Distance between group %zu and %zu centroids: %.3f m", i, j, distance);

        // 如果形心距离小于阈值，说明是电梯的盒子结构
        if (distance <= max_parallel_group_centroid_distance_) {
          valid_group = true;
          break;
        }
      }

      if (valid_group) {
        centroid_filtered_groups.push_back(groups[i]);
        ROS_INFO("  Group %zu: centroid=(%.2f, %.2f) - KEPT (close to another group)",
                 i, centroid_i.x, centroid_i.y);
      } else {
        ROS_INFO("  Group %zu: centroid=(%.2f, %.2f) - REMOVED (too far from other groups)",
                 i, centroid_i.x, centroid_i.y);
      }
    }

    groups = centroid_filtered_groups;

    ROS_INFO("After centroid distance filtering: %zu groups remaining", groups.size());
  }

  // 步骤3: 最终验证 - 必须是2组，每组2条线
  if (groups.size() != 2) {
    ROS_ERROR("After filtering, expected 2 parallel groups, but found %zu groups. Cannot form rectangle.", groups.size());
    return false;
  }

  if (groups[0].size() != 2 || groups[1].size() != 2) {
    ROS_ERROR("After filtering, expected 2 lines per group, but found group0=%zu lines, group1=%zu lines. Cannot form rectangle.",
              groups[0].size(), groups[1].size());
    return false;
  }

  ROS_INFO("Valid rectangle structure detected: 2 groups with 2 lines each");

  // 步骤4: 对于每组的每条线，与另一组的线求交点
  for (size_t group_idx = 0; group_idx < 2; ++group_idx) {
    size_t other_group_idx = 1 - group_idx;

    for (int line_idx : groups[group_idx]) {
      const auto& current_line = merged_lines[line_idx];

      // 与另一组的2条线求交点
      std::vector<pcl::PointXYZ> intersections;

      for (int other_line_idx : groups[other_group_idx]) {
        const auto& other_line = merged_lines[other_line_idx];

        pcl::PointXYZ intersection;
        if (computeLineIntersection(current_line, other_line, intersection)) {
          // 检查交点是否在地图范围内
          if (isPointInMapBounds(intersection)) {
            intersections.push_back(intersection);
            ROS_DEBUG("  Line %d intersects with line %d at (%.2f, %.2f) - in bounds",
                      line_idx, other_line_idx, intersection.x, intersection.y);
          } else {
            ROS_DEBUG("  Line %d intersects with line %d at (%.2f, %.2f) - out of bounds",
                      line_idx, other_line_idx, intersection.x, intersection.y);
          }
        }
      }

      // 步骤5: 用2个交点连成新线段替代原线段
      if (intersections.size() == 2) {
        connected_lines.push_back(std::make_pair(intersections[0], intersections[1]));
        // ROS_INFO("  Line %d connected with 2 intersections: (%.2f,%.2f) - (%.2f,%.2f)",
        //          line_idx,
        //          intersections[0].x, intersections[0].y,
        //          intersections[1].x, intersections[1].y);
      } else if (intersections.size() == 1) {
        ROS_WARN("  Line %d only has 1 valid intersection (expected 2), keeping original line", line_idx);
        connected_lines.push_back(current_line);
      } else {
        ROS_WARN("  Line %d has %zu valid intersections (expected 2), keeping original line",
                 line_idx, intersections.size());
        connected_lines.push_back(current_line);
      }
    }
  }

  ROS_INFO("Connection complete: %zu connected lines generated", connected_lines.size());
  return true;
}

// 计算两条线（延长到无限远）的交点
bool ElevatorDetector::computeLineIntersection(const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line1,
                                                const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line2,
                                                pcl::PointXYZ& intersection)
{
  // 线1的方向向量和起点
  double dx1 = line1.second.x - line1.first.x;
  double dy1 = line1.second.y - line1.first.y;
  double x1 = line1.first.x;
  double y1 = line1.first.y;

  // 线2的方向向量和起点
  double dx2 = line2.second.x - line2.first.x;
  double dy2 = line2.second.y - line2.first.y;
  double x2 = line2.first.x;
  double y2 = line2.first.y;

  // 使用参数方程求交点
  // Line1: P = (x1, y1) + t1 * (dx1, dy1)
  // Line2: P = (x2, y2) + t2 * (dx2, dy2)
  //
  // 求解:
  // x1 + t1*dx1 = x2 + t2*dx2
  // y1 + t1*dy1 = y2 + t2*dy2

  double denominator = dx1 * dy2 - dy1 * dx2;

  // 检查平行（分母为0）
  if (std::abs(denominator) < 1e-6) {
    return false;  // 平行或重合，无交点
  }

  // 求解t1
  double t1 = ((x2 - x1) * dy2 - (y2 - y1) * dx2) / denominator;

  // 计算交点
  intersection.x = x1 + t1 * dx1;
  intersection.y = y1 + t1 * dy1;
  intersection.z = 0.0;

  return true;
}

// 检查点是否在地图范围内
bool ElevatorDetector::isPointInMapBounds(const pcl::PointXYZ& point)
{
  double half_width = map_width_ / 2.0;
  double half_height = map_height_ / 2.0;

  return (point.x >= -half_width && point.x <= half_width &&
          point.y >= -half_height && point.y <= half_height);
}

// 发布连接后的线段标记
void ElevatorDetector::publishConnectedLineMarkers(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& lines)
{
  if (connected_line_markers_pub_.getNumSubscribers() == 0 || lines.empty()) {
    return;
  }

  visualization_msgs::Marker line_list;
  line_list.header.frame_id = robot_base_frame_;
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "connected_lines";
  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.action = visualization_msgs::Marker::ADD;

  // 初始化pose的quaternion
  line_list.pose.orientation.w = 1.0;

  line_list.scale.x = marker_scale_;  // 线条粗细（是原始线段的4倍）

  // 使用黄色表示连接后的线段
  line_list.color.r = 1.0;
  line_list.color.g = 1.0;
  line_list.color.b = 0.0;  // 黄色
  line_list.color.a = 1.0;

  line_list.lifetime = ros::Duration(marker_lifetime_);

  // 添加所有线段
  for (const auto& line : lines) {
    geometry_msgs::Point p1, p2;

    p1.x = line.first.x;
    p1.y = line.first.y;
    p1.z = line.first.z;

    p2.x = line.second.x;
    p2.y = line.second.y;
    p2.z = line.second.z;

    line_list.points.push_back(p1);
    line_list.points.push_back(p2);
  }

  connected_line_markers_pub_.publish(line_list);
}

// ==============================================================================
// 阶段6: 从4条线段组成的矩形识别电梯参数
// ==============================================================================
bool ElevatorDetector::recognizeElevatorFromRectangle(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& rectangle_lines,
                                                      ElevatorDetectionResult& result)
{
  if (rectangle_lines.size() != 4) {
    ROS_ERROR("Expected 4 lines for rectangle, but got %zu lines", rectangle_lines.size());
    return false;
  }

  ROS_DEBUG("Recognizing elevator from 4 rectangle lines");

  // 步骤1: 将4条线分为2组平行线（通过角度相似性）
  std::vector<int> group1, group2;
  group1.push_back(0);

  // 找到与第0条线平行的线
  for (size_t i = 1; i < 4; ++i) {
    double angle_diff = computeAngleBetweenLines(rectangle_lines[0], rectangle_lines[i]);
    if (angle_diff <= parallel_angle_threshold_) {
      group1.push_back(i);
    } else {
      group2.push_back(i);
    }
  }

  // 验证分组（应该是2条平行线一组）
  if (group1.size() != 2 || group2.size() != 2) {
    ROS_ERROR("Failed to group lines into 2 parallel pairs: group1=%zu, group2=%zu", 
              group1.size(), group2.size());
    return false;
  }

  ROS_DEBUG("Lines grouped: group1=[%d,%d], group2=[%d,%d]", 
            group1[0], group1[1], group2[0], group2[1]);

  // 步骤2: 计算每条线的中点
  auto computeLineMidpoint = [](const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line) -> pcl::PointXYZ {
    pcl::PointXYZ mid;
    mid.x = (line.first.x + line.second.x) / 2.0;
    mid.y = (line.first.y + line.second.y) / 2.0;
    mid.z = (line.first.z + line.second.z) / 2.0;
    return mid;
  };

  pcl::PointXYZ mid1_a = computeLineMidpoint(rectangle_lines[group1[0]]);
  pcl::PointXYZ mid1_b = computeLineMidpoint(rectangle_lines[group1[1]]);
  pcl::PointXYZ mid2_a = computeLineMidpoint(rectangle_lines[group2[0]]);
  pcl::PointXYZ mid2_b = computeLineMidpoint(rectangle_lines[group2[1]]);

  // 步骤3: 计算矩形中心（4个中点的平均值）
  result.x = (mid1_a.x + mid1_b.x + mid2_a.x + mid2_b.x) / 4.0;
  result.y = (mid1_a.y + mid1_b.y + mid2_a.y + mid2_b.y) / 4.0;

  // 步骤4: 计算每组的平均长度
  auto computeLineLength = [](const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line) -> double {
    double dx = line.second.x - line.first.x;
    double dy = line.second.y - line.first.y;
    return std::sqrt(dx * dx + dy * dy);
  };

  double length1_a = computeLineLength(rectangle_lines[group1[0]]);
  double length1_b = computeLineLength(rectangle_lines[group1[1]]);
  double length2_a = computeLineLength(rectangle_lines[group2[0]]);
  double length2_b = computeLineLength(rectangle_lines[group2[1]]);

  double avg_length1 = (length1_a + length1_b) / 2.0;
  double avg_length2 = (length2_a + length2_b) / 2.0;

  ROS_DEBUG("Group1 avg length: %.2fm, Group2 avg length: %.2fm", avg_length1, avg_length2);

  // 步骤5: 确定哪组是宽度（width），哪组是深度（depth）
  // width是较短的边（电梯门宽度），depth是较长的边（电梯进深）
  int width_group_idx, depth_group_idx;
  if (avg_length1 < avg_length2) {
    result.width = avg_length1;
    result.depth = avg_length2;
    width_group_idx = group1[0];
    depth_group_idx = group2[0];
  } else {
    result.width = avg_length2;
    result.depth = avg_length1;
    width_group_idx = group2[0];
    depth_group_idx = group1[0];
  }

  // 步骤6: 计算yaw角度（使用深度方向的线段，代表电梯朝向）
  const auto& depth_line = rectangle_lines[depth_group_idx];
  double dx = depth_line.second.x - depth_line.first.x;
  double dy = depth_line.second.y - depth_line.first.y;
  result.angle = std::atan2(dy, dx);

  // 步骤7: 设置置信度（基于矩形的规整程度）
  // 检查对边长度的一致性
  double length_diff1 = std::abs(length1_a - length1_b) / avg_length1;
  double length_diff2 = std::abs(length2_a - length2_b) / avg_length2;
  double consistency_score = 1.0 - (length_diff1 + length_diff2) / 2.0;
  result.confidence = std::max(0.0, std::min(1.0, consistency_score));

  ROS_INFO("Elevator recognized: center=(%.2f, %.2f), size=(%.2fx%.2f), yaw=%.1f deg, confidence=%.2f",
           result.x, result.y, result.width, result.depth,
           result.angle * 180.0 / M_PI, result.confidence);

  return true;
}

} // namespace elevator_docking
