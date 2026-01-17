# 电梯对接调度系统

## 概述

这套系统实现了机器人与电梯的自动对接功能,包括电梯检测、进入电梯和退出电梯三个主要功能。

## 系统架构

### 核心组件

1. **ElevatorDetector** (`elevator_detector.cpp`)
   - 使用激光雷达检测电梯位置
   - 返回电梯的中心位置、尺寸和朝向

2. **ElevatorDockingServer** (`elevator_docking_server.cpp`)
   - Action Server: `/robot1/elevator/docking`
   - 处理三种命令: `detection`, `in`, `out`

3. **OdomZero** (`test.py`)
   - 提供临时里程计清零功能
   - 订阅 `/odom`,发布 `/odom_temporary`
   - 监听 `/reset_odom_temporary` 重置原点

4. **测试客户端** (`test_docking_client.py`)
   - 交互式测试工具
   - 发送 detection/in/out 命令

## Action 接口

### Goal
```
string command  # "detection", "in", "out"
```

### Result
```
bool success
string message
# Detection result (only valid when command is "detection")
float64 center_x      # 电梯中心X坐标
float64 center_y      # 电梯中心Y坐标
float64 width         # 电梯宽度
float64 depth         # 电梯深度
float64 yaw_deg       # 偏航角(度)
float64 confidence    # 置信度
```

### Feedback
```
string status
float64 progress
```

## 工作流程

### 1. Detection (检测模式)

收到 `"detection"` 命令后:
1. 持续检测 1 秒
2. 选择置信度最高的检测结果
3. 返回电梯信息:
   - center = (x, y)
   - size = (width x depth)
   - yaw (度)
   - confidence
4. 内部记录 yaw, y, x 用于后续 "in" 命令

**示例结果:**
```
center=(1.36, -0.01), size=(0.83x1.30), yaw=-0.7 deg
```

### 2. In (进入电梯)

收到 `"in"` 命令后:
1. **重置里程计**: 发布 `/reset_odom_temporary`
2. **订阅临时里程计**: `/odom_temporary`
3. **三步纠正:**
   - **步骤1**: 纠正 yaw (旋转 -0.7度)
     - 使用低速旋转: `angular_vel = 0.1 rad/s`
   - **步骤2**: 纠正 y (横向移动 -0.01m)
     - 对于差速机器人需要旋转策略
   - **步骤3**: 前进 x (深度 1.36m)
     - 线速度: `linear_vel = 0.15 m/s`
4. 完成后停止并返回成功

### 3. Out (退出电梯)

收到 `"out"` 命令后:
1. **重置里程计**: 发布 `/reset_odom_temporary`
2. **订阅临时里程计**: `/odom_temporary`
3. **倒车**: 倒退刚才的 x 距离
   - 反向速度: `-0.15 m/s`
   - 目标: `-x` 米
4. 完成后停止并返回成功

## 控制参数

在 launch 文件中可配置:

```xml
<param name="control_rate" value="20.0" />         # 控制频率 Hz
<param name="yaw_angular_vel" value="0.1" />       # 偏航角速度 rad/s
<param name="y_linear_vel" value="0.1" />          # Y方向速度 m/s
<param name="x_linear_vel" value="0.15" />         # 前进速度 m/s
<param name="x_backward_vel" value="0.15" />       # 后退速度 m/s
```

## 使用方法

### 编译

```bash
cd ~/nav_ws_teb
catkin_make
source devel/setup.bash
```

### 启动系统

```bash
roslaunch elevator_docking elevator_docking_full.launch
```

这会启动:
- `odom_zero_node` - 临时里程计节点
- `elevator_docking_server` - Action服务器(包含检测器)

### 使用测试客户端

```bash
rosrun elevator_docking test_docking_client.py
```

交互式命令:
```
Enter command: detection  # 检测电梯
Enter command: in         # 进入电梯
Enter command: out        # 退出电梯
Enter command: quit       # 退出程序
```

### 使用Python代码调用

```python
import rospy
import actionlib
from elevator_docking.msg import ElevatorDockingAction, ElevatorDockingGoal

rospy.init_node('my_node')
client = actionlib.SimpleActionClient('/robot1/elevator/docking', ElevatorDockingAction)
client.wait_for_server()

# 检测
goal = ElevatorDockingGoal(command='detection')
client.send_goal(goal)
client.wait_for_result()
result = client.get_result()

if result.success:
    print(f"Elevator at ({result.center_x}, {result.center_y})")
    print(f"Size: {result.width}x{result.depth}")
    print(f"Yaw: {result.yaw_deg} deg")

# 进入
goal = ElevatorDockingGoal(command='in')
client.send_goal(goal)
client.wait_for_result()

# 退出
goal = ElevatorDockingGoal(command='out')
client.send_goal(goal)
client.wait_for_result()
```

## 话题说明

### 订阅的话题
- `/odom` - 原始里程计 (由test.py订阅)
- `/odom_temporary` - 临时里程计 (由server订阅)
- 激光雷达话题 (配置在 `elevator_docking.yaml` 中)

### 发布的话题
- `/base_cmd_vel` - 速度控制命令
- `/reset_odom_temporary` - 重置临时里程计
- `/odom_temporary` - 临时里程计 (由test.py发布)

## 文件结构

```
elevator_docking/
├── action/
│   └── ElevatorDocking.action       # Action定义
├── include/elevator_docking/
│   ├── elevator_detector.h          # 检测器头文件
│   └── elevator_docking_server.h    # 服务器头文件
├── src/
│   ├── elevator_detector.cpp        # 检测器实现
│   ├── elevator_detector_node.cpp   # 检测器节点(已废弃)
│   ├── elevator_docking_server.cpp  # 服务器实现
│   ├── elevator_docking_server_node.cpp  # 服务器节点
│   └── test.py                      # 临时里程计节点
├── scripts/
│   └── test_docking_client.py       # 测试客户端
├── launch/
│   ├── elevator_docking.launch      # 原始launch
│   └── elevator_docking_full.launch # 完整系统launch
└── config/
    └── elevator_docking.yaml        # 配置文件
```

## 注意事项

1. **差速机器人Y方向控制**: 当前实现的Y方向纠正需要针对差速机器人优化(需要旋转90度,前进,再旋转回来)

2. **里程计漂移**: 使用相对里程计,短距离精度较好,长时间会有累积误差

3. **安全性**:
   - 建议在进入/退出时监控障碍物
   - 可添加超时保护
   - 可添加最大速度限制

4. **调试技巧**:
   ```bash
   # 查看action状态
   rostopic echo /robot1/elevator/docking/status

   # 查看临时里程计
   rostopic echo /odom_temporary

   # 查看速度命令
   rostopic echo /base_cmd_vel
   ```

## 改进建议

1. **Y方向控制**: 实现差速机器人的横向移动策略
2. **视觉融合**: 结合相机进行更精确的对接
3. **闭环控制**: 在进入过程中持续检测,动态调整
4. **故障恢复**: 添加重试机制和异常处理
5. **参数优化**: 根据实际机器人调整速度和容差参数

## 故障排查

### 问题: Action server连接不上
```bash
# 检查节点是否运行
rosnode list | grep docking

# 检查action是否存在
rostopic list | grep docking
```

### 问题: 检测不到电梯
- 检查激光雷达数据: `rostopic echo /scan`
- 调整检测参数在 `elevator_docking.yaml`
- 查看RViz可视化

### 问题: 机器人不动
- 检查速度命令: `rostopic echo /base_cmd_vel`
- 确认话题名称正确 (当前是 `/base_cmd_vel`)
- 检查里程计数据: `rostopic echo /odom_temporary`

## 版本历史

- v1.0.0 - 初始版本,实现基本检测和对接功能
