#ifndef ELEVATOR_DOCKING_ELEVATOR_DETECTOR_H
#define ELEVATOR_DOCKING_ELEVATOR_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <dynamic_reconfigure/server.h>
#include <elevator_docking/ElevatorDockingConfig.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <cmath>
#include <algorithm>

namespace elevator_docking
{

/**
 * @brief 传感器配置结构体
 */
struct SensorConfig
{
  std::string topic;        // 传感器话题
  std::string frame;        // 传感器坐标系
};

/**
 * @brief 电梯检测结果结构体
 */
struct ElevatorDetectionResult
{
  double x;                 // 中心X坐标 (robot_base_frame)
  double y;                 // 中心Y坐标 (robot_base_frame)
  double width;             // 电梯门宽度 (米)
  double depth;             // 电梯深度 (米)
  double angle;             // 旋转角度 (弧度)
  double confidence;        // 检测置信度 [0.0, 1.0]

  ElevatorDetectionResult()
    : x(0.0), y(0.0), width(0.0), depth(0.0), angle(0.0), confidence(0.0) {}
};

/**
 * @brief 电梯检测器类
 */
class ElevatorDetector
{
public:
  /**
   * @brief 构造函数
   * @param nh ROS节点句柄
   * @param private_nh 私有节点句柄
   */
  ElevatorDetector(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  
  /**
   * @brief 析构函数
   */
  ~ElevatorDetector();
  
private:
  /**
   * @brief 加载参数
   */
  void loadParameters();
  
  /**
   * @brief 初始化订阅器和发布器
   */
  void setupPubSub();
  
  /**
   * @brief Dynamic reconfigure回调函数
   * @param config 新的配置
   * @param level 配置级别
   */
  void reconfigureCallback(elevator_docking::ElevatorDockingConfig& config, uint32_t level);
  
  /**
   * @brief LaserScan回调函数
   * @param msg LaserScan消息
   * @param sensor_name 传感器名称
   */
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg, const std::string& sensor_name);
  
  /**
   * @brief 转换点云到机器人基座坐标系
   * @param cloud_in 输入点云 (sensor frame)
   * @param cloud_out 输出点云 (robot_base_frame)
   * @param target_frame 目标坐标系
   * @param source_frame 源坐标系
   * @return 转换是否成功
   */
  bool transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                           const std::string& target_frame,
                           const std::string& source_frame);

  /**
   * @brief 过滤点云（保留地图范围内的点）
   * @param cloud_in 输入点云
   * @param cloud_out 输出点云 (filtered by map width/height)
   */
  void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

  /**
   * @brief 合并所有传感器点云
   */
  void mergeSensorClouds();

  /**
   * @brief 检测电梯（主检测函数）
   * @param cloud 输入点云
   * @param result 检测结果
   * @return 是否检测成功
   */
  bool detectElevator(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      ElevatorDetectionResult& result);

  /**
   * @brief 使用三阶段方法检测直线段
   * 阶段1: 点云欧氏聚类预过滤
   * 阶段2: RANSAC检测 + 间隙分段（拆分成最小单位）
   * 阶段3: 长度过滤
   * @param cloud 输入点云
   * @param lines 检测到的直线段（起点和终点）
   */
  void detectLineSegments(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& lines);

  /**
   * @brief 阶段1: 对点云进行欧氏聚类
   * @param cloud 输入点云
   * @param clusters 输出的点云簇集合
   */
  void clusterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                         std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);

  /**
   * @brief 阶段2: 对单个簇进行RANSAC直线检测，并根据间隙拆分成最小单位线段
   * @param cluster 输入点云簇
   * @param segments 输出的线段集合（未过滤长度）
   */
  void detectAndSplitLines(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
                           std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& segments);

  /**
   * @brief 阶段3: 根据长度过滤线段
   * @param segments 输入的所有线段
   * @param filtered_lines 输出的过滤后线段
   */
  void filterLinesByLength(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& segments,
                           std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& filtered_lines);

  /**
   * @brief 阶段4: 平行线分组与合并
   * @param lines 输入的线段
   * @param merged_lines 输出的合并后线段
   */
  void groupAndMergeParallelLines(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& lines,
                                  std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& merged_lines);

  /**
   * @brief 计算两条线段的方向角度差（度）
   */
  double computeAngleBetweenLines(const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line1,
                                  const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line2);

  /**
   * @brief 计算两条平行线段之间的法向距离
   */
  double computeNormalDistance(const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line1,
                               const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line2);

  /**
   * @brief 合并两条平行线段
   */
  std::pair<pcl::PointXYZ, pcl::PointXYZ> mergeLines(const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line1,
                                                     const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line2);

  /**
   * @brief 阶段5: 连接平行线组形成电梯U形结构
   * @param merged_lines 输入的合并后线段
   * @param connected_lines 输出的连接后线段
   * @return 是否成功识别U形结构
   */
  bool connectParallelGroups(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& merged_lines,
                             std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& connected_lines);

  /**
   * @brief 计算两条线（延长到无限远）的交点
   * @param line1 第一条线段
   * @param line2 第二条线段
   * @param intersection 输出的交点
   * @return 是否存在交点（平行线返回false）
   */
  bool computeLineIntersection(const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line1,
                                const std::pair<pcl::PointXYZ, pcl::PointXYZ>& line2,
                                pcl::PointXYZ& intersection);

  /**
   * @brief 检查点是否在地图范围内
   * @param point 待检查的点
   * @return 是否在地图范围内
   */
  bool isPointInMapBounds(const pcl::PointXYZ& point);

  /**
   * @brief 发布连接后的线段标记
   * @param lines 连接后的线段集合
   */
  void publishConnectedLineMarkers(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& lines);

  /**
   * @brief 从检测到的直线段中识别电梯凹槽
   * @param lines 直线段集合
   * @param result 识别结果
   * @return 是否成功识别电梯
   */
  bool recognizeElevatorFromLines(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& lines,
                                  ElevatorDetectionResult& result);

  /**
   * @brief 验证电梯检测结果
   * @param result 待验证的检测结果
   * @return 是否符合电梯约束
   */
  bool validateElevatorResult(const ElevatorDetectionResult& result);

  /**
   * @brief 发布过滤后的点云
   */
  void publishFilteredPointCloud();

  /**
   * @brief 发布检测到的直线段标记
   * @param lines 直线段集合
   */
  void publishLineMarkers(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& lines);

  /**
   * @brief 发布合并后的线段标记
   * @param lines 合并后的线段集合
   */
  void publishMergedLineMarkers(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& lines);

  /**
   * @brief 发布电梯检测结果标记
   * @param result 检测结果
   */
  void publishElevatorMarker(const ElevatorDetectionResult& result);

  /**
   * @brief 创建矩形可视化标记
   * @param result 检测结果
   * @return Marker消息
   */
  visualization_msgs::Marker createRectangleMarker(const ElevatorDetectionResult& result);

private:
  // ROS句柄
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  
  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Dynamic reconfigure
  std::shared_ptr<dynamic_reconfigure::Server<elevator_docking::ElevatorDockingConfig>> dyn_reconfig_server_;
  
  // 订阅器和发布器
  std::map<std::string, ros::Subscriber> laser_subscribers_;
  ros::Publisher filtered_cloud_pub_;
  ros::Publisher line_markers_pub_;        // 直线标记发布器
  ros::Publisher merged_line_markers_pub_; // 合并后线段标记发布器
  ros::Publisher connected_line_markers_pub_; // 连接后线段标记发布器
  ros::Publisher elevator_marker_pub_;
  
  // 传感器配置
  std::string robot_base_frame_;
  std::map<std::string, SensorConfig> sensor_configs_;
  
  // 地图配置
  double map_width_;
  double map_height_;
  
  // 传感器数据
  std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> sensor_clouds_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud_;
  ros::Time last_update_time_;

  // 发布开关
  bool publish_filtered_points_;
  bool publish_elevator_marker_;
  bool publish_line_markers_;          // 是否发布检测到的直线标记
  bool publish_merged_line_markers_;   // 是否发布合并后的线段标记
  bool publish_connected_line_markers_; // 是否发布连接后的线段标记
  std::string filtered_points_topic_;
  std::string elevator_marker_topic_;
  std::string line_markers_topic_;     // 直线标记话题
  std::string merged_line_markers_topic_; // 合并后线段标记话题
  std::string connected_line_markers_topic_; // 连接后线段标记话题

  // 电梯检测参数
  bool enable_detection_;              // 内部控制参数，不从YAML读取
  double min_door_width_;
  double max_door_width_;
  double min_door_depth_;
  double max_door_depth_;

  // 点云聚类参数
  double cluster_tolerance_;           // 欧氏聚类距离阈值（米）
  int min_cluster_size_;               // 最小簇点数
  int max_cluster_size_;               // 最大簇点数

  // 直线检测参数
  double ransac_distance_threshold_;   // RANSAC距离阈值
  int min_line_points_;                // 直线最少点数
  double min_line_length_;             // 最短直线长度
  double max_line_length_;             // 最长直线长度
  double line_gap_threshold_;          // 线段间隙阈值，用于拆分（米）

  // 平行线合并参数（阶段4）
  double merge_angle_threshold_;       // 平行线合并角度阈值（度）
  double merge_distance_threshold_;    // 平行线合并法向距离阈值（米）

  // 电梯识别参数
  double parallel_angle_threshold_;    // 平行判断角度阈值（度）
  double perpendicular_angle_threshold_; // 垂直判断角度阈值（度）

  // 可视化参数
  double marker_lifetime_;
  double marker_scale_;

  // 检测结果
  ElevatorDetectionResult last_detection_;
  bool elevator_detected_;
};

} // namespace elevator_docking

#endif // ELEVATOR_DOCKING_ELEVATOR_DETECTOR_H
