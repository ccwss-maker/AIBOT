# Lidar Image Collector

雷达图像采集包 - 将多个雷达点云数据合并并转换为图像

## 功能特性

- 支持多个雷达传感器数据融合（LaserScan和PointCloud2）
- 将3D点云投影到2D图像平面
- 支持动态参数调整（dynamic reconfigure）
- 可配置图像大小、地图范围、分辨率
- 黑色背景，白色点云显示

## 配置参数

### 传感器配置
- `observation_sources`: 传感器列表
- 每个传感器需要配置：
  - `topic`: 数据话题
  - `sensor_frame`: 坐标系

### 地图和图像配置
- `map_width`: 地图宽度（米）
- `map_height`: 地图高度（米）
- `resolution`: 分辨率（米/像素）
- `image_rows`: 图像高度（像素）
- `image_cols`: 图像宽度（像素）

### 动态可调参数
- 图像尺寸 (image_rows, image_cols)
- 地图范围 (map_width, map_height)
- 点云过滤范围 (min_x, max_x, min_y, max_y, min_z, max_z)
- 点强度 (point_intensity)
- 图像模糊 (apply_blur, blur_kernel_size)

## 快速开始

### 1. 编译

```bash
cd /home/amov/PersonalData/Program/Ros/data_collection_ws
catkin_make
source devel/setup.bash
```

### 2. 修改配置（可选）

编辑配置文件：`config/lidar_image_collector.yaml`

重点参数：
- `observation_sources`: 传感器列表
- `image_rows`/`image_cols`: 图像分辨率 (默认960x960)
- `map_width`/`map_height`: 地图范围 (米)
- `resolution`: 分辨率 (米/像素)

## 运行

```bash
roslaunch lidar_image_collector lidar_image_collector.launch
```

## 动态参数调整

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

## 话题

### 订阅
- `/front_filtered_points`: 前置雷达数据
- `/back_filtered_points`: 后置雷达数据
- `/local_map_laserscan`: 激光雷达数据

### 发布
- `/lidar_image`: 生成的图像 (sensor_msgs/Image)
- `/merged_cloud`: 合并后的点云 (sensor_msgs/PointCloud2)

## 可视化

查看图像：
```bash
rosrun image_view image_view image:=/lidar_image
```

查看点云（rviz）：
```bash
rviz
# 添加Image显示 /lidar_image
# 添加PointCloud2显示 /merged_cloud
```
