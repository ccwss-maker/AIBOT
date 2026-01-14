# 自定义 Local Planner 使用说明

## 概述

这是一个基于 MBF (Move Base Flex) 框架的自定义 Local Planner，使用 Pure Pursuit 算法实现路径跟踪。

## 接口说明

### 输入接口

1. **setPlan()** - 接收全局路径
   - 输入：`std::vector<geometry_msgs::PoseStamped>` - 从 global planner 生成的路径点序列
   - 返回：`bool` - true 表示成功接收
   - 说明：MBF 会在全局路径生成后调用此函数，将路径传递给你的 controller

2. **computeVelocityCommands()** - 计算速度命令（主循环）
   - 输入：
     - `pose` - 机器人当前位姿
     - `velocity` - 机器人当前速度
   - 输出：
     - `cmd_vel` - 速度命令 (geometry_msgs::TwistStamped)
     - `message` - 状态消息（可选）
   - 返回：结果码
     - `0` (SUCCESS) - 成功
     - `102` (NO_VALID_CMD) - 无法生成速度
     - `104` (COLLISION) - 检测到碰撞
     - 其他错误码见 mbf_msgs/ExePathResult.msg

### 输出接口

**速度命令格式** (`geometry_msgs::TwistStamped`):
```cpp
cmd_vel.header.stamp = ros::Time::now();
cmd_vel.header.frame_id = "local_map";  // 机器人坐标系
cmd_vel.twist.linear.x = 线速度;    // m/s
cmd_vel.twist.linear.y = 0.0;       // 非全向机器人设为0
cmd_vel.twist.angular.z = 角速度;   // rad/s
```

## 数据流程

```
Global Planner (navfn)
    ↓
    | std::vector<PoseStamped> 全局路径
    ↓
setPlan() ← 保存路径
    ↓
computeVelocityCommands() ← 每 20Hz 调用
    |
    ├─ 1. 获取机器人位姿
    ├─ 2. 找到前瞻点 (Pure Pursuit)
    ├─ 3. 计算目标角度和距离
    ├─ 4. 计算速度命令
    ├─ 5. 碰撞检测
    └─ 6. 输出 TwistStamped → /base_cmd_vel
```

## 算法实现

当前实现了 **Pure Pursuit** 算法：

1. **前瞻点选择**：从当前位置开始，找到距离 >= `lookahead_distance` 的第一个路径点
2. **角度计算**：计算机器人到前瞻点的目标角度
3. **速度生成**：
   - 线速度：距离越远速度越大，最大不超过 `max_vel_x`
   - 角速度：比例控制，`ω = Kp * 角度误差`
4. **碰撞检测**：检查机器人前方 1 米范围内的障碍物

## 参数配置

文件：`config/custom_local_planner_params.yaml`

```yaml
CustomLocalPlanner:
  max_vel_x: 1.5              # 最大线速度 (m/s)
  max_vel_theta: 1.0          # 最大角速度 (rad/s)
  xy_goal_tolerance: 0.2      # 位置容差 (m)
  yaw_goal_tolerance: 0.2     # 角度容差 (rad)
  lookahead_distance: 1.5     # 前瞻距离 (m)
```

## 编译和运行

### 1. 编译

```bash
cd /home/amov/PersonalData/Program/Ros/nav_ws_teb
catkin_make
source devel/setup.bash
```

### 2. 运行

```bash
roslaunch ntu_planner move_base.launch
```

### 3. 发送导航目标

```bash
# 方式1：通过 RViz 的 "2D Nav Goal" 工具

# 方式2：通过命令行
rostopic pub /robot1/move_base_simple/goal geometry_msgs/PoseStamped \
'{header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}'
```

### 4. 可视化

在 RViz 中添加以下话题：
- `/robot1/move_base_flex/CustomLocalPlanner/local_plan` - 本地路径
- `/robot1/move_base_flex/CustomLocalPlanner/target_point` - 当前目标点（红色球）

## 扩展方向

你可以根据需要修改算法实现：

### 1. 更换跟踪算法

在 `computeVelocityCommands()` 中实现其他算法：
- **Stanley Controller** - 更适合高速场景
- **MPC (Model Predictive Control)** - 考虑动力学约束
- **LQR** - 最优控制

### 2. 增强避障

- 使用 costmap 信息动态调整速度
- 实现 Dynamic Window Approach (DWA)
- 添加安全裕度计算

### 3. 平滑优化

- 添加速度滤波器
- 实现加速度限制
- 轨迹平滑处理

## 关键代码位置

| 文件 | 功能 |
|------|------|
| `include/ntu_planner/custom_local_planner.h` | 接口定义 |
| `src/custom_local_planner.cpp` | 算法实现 |
| `computeVelocityCommands()` | **核心算法实现区** |
| `findLookaheadPoint()` | Pure Pursuit 前瞻点查找 |
| `isPathCollisionFree()` | 碰撞检测 |

## 调试技巧

### 1. 查看日志

```bash
# 实时查看 planner 输出
rosrun rqt_console rqt_console
```

### 2. 检查参数加载

```bash
# 查看当前参数
rosparam list | grep CustomLocalPlanner
rosparam get /robot1/move_base_flex/CustomLocalPlanner/max_vel_x
```

### 3. 监控速度输出

```bash
# 查看速度命令
rostopic echo /base_cmd_vel
```

### 4. 可视化调试

在代码中已经添加了可视化发布器：
- `local_plan_pub_` - 发布剩余路径
- `target_point_pub_` - 发布当前目标点

## 常见问题

### Q1: 编译错误 "找不到 mbf_costmap_core"

**解决**：确保已安装 MBF
```bash
sudo apt-get install ros-noetic-move-base-flex
```

### Q2: 插件加载失败

**解决**：检查 `custom_planner_plugin.xml` 中的库路径
```bash
# 验证库是否生成
ls devel/lib/libntu_planner.so
```

### Q3: 机器人不移动

**调试步骤**：
1. 检查是否收到全局路径：在 `setPlan()` 中添加 ROS_INFO
2. 检查 `computeVelocityCommands()` 是否被调用
3. 检查返回码是否为 SUCCESS
4. 检查 cmd_vel 是否正确输出

## 性能优化

1. **频率调整**：`controller_frequency` 设置为 20Hz 适合大多数场景
2. **前瞻距离**：`lookahead_distance` 建议设置为 1-2 倍车速
3. **容差设置**：根据实际精度需求调整 goal_tolerance

## 相关资源

- MBF 文档: http://wiki.ros.org/move_base_flex
- Pure Pursuit 算法: https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
- mbf_msgs 错误码: http://docs.ros.org/en/api/mbf_msgs/html/action/ExePath.html
