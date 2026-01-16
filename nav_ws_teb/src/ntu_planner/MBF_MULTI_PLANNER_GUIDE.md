# MBF 多规划器配置和使用指南

## 概述

Move Base Flex (MBF) 相比传统 move_base 的核心优势之一是**支持同时配置多个规划器和控制器**，并在运行时动态选择使用哪个。

## 配置多个规划器

### 1. 在 mbf_params.yaml 中配置

```yaml
planners:
  - name: navfn              # 规划器名称（调用时使用）
    type: navfn/NavfnROS     # 规划器类型
  
  - name: global_planner
    type: global_planner/GlobalPlanner

controllers:
  - name: teb
    type: teb_local_planner/TebLocalPlannerROS
  
  - name: dwa
    type: dwa_local_planner/DWAPlannerROS
```

### 2. 每个规划器的独立配置

可以为每个规划器/控制器创建独立的命名空间配置：

```xml
<!-- launch 文件中 -->
<node pkg="mbf_costmap_nav" type="mbf_costmap_nav" name="move_base_flex">
  <!-- 通用配置 -->
  <rosparam file="$(find ntu_planner)/config/mbf_params.yaml" command="load" />
  
  <!-- NavFn 规划器配置 -->
  <rosparam file="$(find ntu_planner)/config/navfn_params.yaml" command="load" ns="navfn" />
  
  <!-- Global Planner 配置 -->
  <rosparam file="$(find ntu_planner)/config/global_planner_params.yaml" command="load" ns="global_planner" />
  
  <!-- TEB 控制器配置 -->
  <rosparam file="$(find ntu_planner)/config/teb_local_planner_params_ackermann.yaml" command="load" ns="teb" />
  
  <!-- DWA 控制器配置 -->
  <rosparam file="$(find ntu_planner)/config/dwa_local_planner_params.yaml" command="load" ns="dwa" />
</node>
```

## 使用 MBF Action 指定规划器

### 方法1：通过 Action 接口 (Python)

```python
import actionlib
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal

# 创建 Action 客户端
client = actionlib.SimpleActionClient('/robot1/move_base_flex/move_base', MoveBaseAction)
client.wait_for_server()

# 创建目标
goal = MoveBaseGoal()
goal.target_pose = target_pose

# 关键：指定使用哪个规划器和控制器
goal.planner = "navfn"      # 使用 NavFn 规划器
goal.controller = "teb"      # 使用 TEB 控制器

# 发送目标
client.send_goal(goal)
client.wait_for_result()
```

### 方法2：通过命令行 (rostopic)

```bash
# 使用 NavFn + TEB
rostopic pub /robot1/move_base_flex/move_base/goal mbf_msgs/MoveBaseActionGoal "
header:
  frame_id: 'map'
goal:
  target_pose:
    header:
      frame_id: 'map'
    pose:
      position: {x: 2.0, y: -3.4, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: -0.766, w: 0.643}
  planner: 'navfn'
  controller: 'teb'
"

# 使用 Global Planner + DWA
rostopic pub /robot1/move_base_flex/move_base/goal mbf_msgs/MoveBaseActionGoal "
...
  planner: 'global_planner'
  controller: 'dwa'
"
```

### 方法3：通过 move_base_simple/goal (Legacy 模式)

使用 move_base_legacy_relay 时，可以通过参数指定默认规划器：

```xml
<node pkg="mbf_costmap_nav" type="move_base_legacy_relay.py" name="move_base_legacy_relay">
  <param name="base_global_planner" value="navfn"/>
  <param name="base_local_planner" value="teb"/>
</node>
```

## MBF Action 接口说明

### 1. MoveBase Action
完整的导航 action，包含全局规划 + 局部控制

```
/robot1/move_base_flex/move_base
- 输入: target_pose, planner, controller
- 输出: outcome, message, final_pose
```

### 2. GetPath Action
仅调用全局规划器，不执行路径

```
/robot1/move_base_flex/get_path
- 输入: target_pose, planner
- 输出: path (nav_msgs/Path)
```

### 3. ExePath Action
仅执行给定路径，调用局部控制器

```
/robot1/move_base_flex/exe_path
- 输入: path, controller
- 输出: outcome, message
```

### 4. Recovery Action
执行恢复行为

```
/robot1/move_base_flex/recovery
- 输入: behavior (恢复行为名称)
- 输出: outcome, message
```

## 运行示例

### 示例1：单次导航，指定规划器
```bash
rosrun ntu_planner mbf_action_example.py _mode:=single _x:=2.0 _y:=-3.4 _planner:=navfn _controller:=teb
```

### 示例2：比较不同规划器性能
```bash
rosrun ntu_planner mbf_action_example.py _mode:=compare _x:=2.0 _y:=-3.4
```

### 示例3：运行多种场景
```bash
rosrun ntu_planner mbf_action_example.py _mode:=scenarios
```

## 规划器选择建议

### NavFn vs Global Planner

| 特性 | NavFn | Global Planner |
|------|-------|----------------|
| 算法 | Dijkstra | A* / Dijkstra |
| 速度 | 较慢 | 较快 |
| 路径质量 | 好 | 可调节 |
| 推荐场景 | 复杂环境 | 开阔空间 |

### TEB vs DWA 控制器

| 特性 | TEB | DWA |
|------|-----|-----|
| 运动学约束 | 支持阿克曼 | 差速/全向 |
| 路径跟踪 | 精确 | 较粗糙 |
| 计算量 | 大 | 小 |
| 推荐场景 | 狭窄空间、阿克曼车 | 开阔空间、速度优先 |

## 应用场景示例

### 场景1：狭窄走廊
- 规划器: navfn（更精确的路径）
- 控制器: teb（更好的阿克曼约束）

### 场景2：开阔空间
- 规划器: global_planner（更快）
- 控制器: dwa（速度优先）

### 场景3：动态环境
- 规划器: global_planner（实时重规划）
- 控制器: teb（动态障碍物处理）

### 场景4：精确停车
- 规划器: navfn
- 控制器: teb（精确姿态控制）

## 高级功能

### 1. 运行时切换规划器

可以在导航过程中取消当前目标，然后用不同的规划器重新发送：

```python
# 取消当前目标
client.cancel_goal()

# 使用不同的规划器重新发送
goal.planner = "global_planner"
client.send_goal(goal)
```

### 2. 多目标导航序列

```python
waypoints = [
    {'x': 1.0, 'y': 2.0, 'planner': 'navfn', 'controller': 'teb'},
    {'x': 3.0, 'y': 4.0, 'planner': 'global_planner', 'controller': 'dwa'},
    {'x': 5.0, 'y': 6.0, 'planner': 'navfn', 'controller': 'teb'}
]

for wp in waypoints:
    goal.target_pose = create_pose(wp['x'], wp['y'])
    goal.planner = wp['planner']
    goal.controller = wp['controller']
    client.send_goal_and_wait(goal)
```

### 3. 条件规划器选择

根据环境条件自动选择：

```python
def select_planner(current_pose, target_pose):
    distance = calculate_distance(current_pose, target_pose)
    
    if distance > 10.0:
        # 长距离：使用快速规划器
        return "global_planner", "dwa"
    else:
        # 短距离：使用精确规划器
        return "navfn", "teb"

planner, controller = select_planner(robot_pose, goal_pose)
goal.planner = planner
goal.controller = controller
```

## 调试和可视化

### 查看可用的规划器和控制器
```bash
# 查看参数服务器
rosparam get /robot1/move_base_flex/planners
rosparam get /robot1/move_base_flex/controllers
```

### 监控规划器调用
```bash
# 监控 action 状态
rostopic echo /robot1/move_base_flex/move_base/feedback
rostopic echo /robot1/move_base_flex/move_base/result
```

### 可视化路径
```bash
# 在 RViz 中添加 Path 显示
# Topic: /robot1/move_base_flex/TebLocalPlannerROS/global_plan
# Topic: /robot1/move_base_flex/TebLocalPlannerROS/local_plan
```

## 常见问题

### Q1: 如何知道哪个规划器失败了？
A: 检查 action result 中的 message 字段，包含详细的错误信息。

### Q2: 可以动态添加/删除规划器吗？
A: 不可以。规划器必须在启动时配置，运行时无法动态修改插件列表。

### Q3: 未指定规划器名称会怎样？
A: 使用配置文件中的第一个规划器/控制器作为默认。

### Q4: 不同规划器可以使用不同的 costmap 吗？
A: 不可以。所有规划器共享同一个 global_costmap 和 local_costmap。

## 参考资料

- [MBF 官方文档](http://wiki.ros.org/move_base_flex)
- [MBF Action 接口](http://wiki.ros.org/mbf_msgs)
- [TEB Local Planner](http://wiki.ros.org/teb_local_planner)
- [Navigation Plugin 开发](http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS)
