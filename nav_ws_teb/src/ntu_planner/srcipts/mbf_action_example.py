#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
MBF Action 示例：展示如何指定不同的规划器和控制器
演示 MBF 相比传统 move_base 的核心优势
"""

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from mbf_msgs.msg import GetPathAction, GetPathGoal
from mbf_msgs.msg import ExePathAction, ExePathGoal


class MBFActionExample:
    """MBF Action 调用示例"""
    
    def __init__(self):
        rospy.init_node('mbf_action_example', anonymous=True)
        
        # 创建 MoveBase Action 客户端
        self.move_base_client = actionlib.SimpleActionClient(
            '/robot1/move_base_flex/move_base', 
            MoveBaseAction
        )
        
        # 创建 GetPath Action 客户端（仅调用全局规划器）
        self.get_path_client = actionlib.SimpleActionClient(
            '/robot1/move_base_flex/get_path',
            GetPathAction
        )
        
        # 创建 ExePath Action 客户端（仅调用局部控制器）
        self.exe_path_client = actionlib.SimpleActionClient(
            '/robot1/move_base_flex/exe_path',
            ExePathAction
        )
        
        rospy.loginfo("等待 MBF Action 服务器...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("MBF Action 服务器已连接")
    
    def create_goal_pose(self, x, y, yaw):
        """创建目标姿态"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = rospy.Time.now()
        
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        # 根据 yaw 角度计算四元数
        import tf
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal_pose.pose.orientation.x = quaternion[0]
        goal_pose.pose.orientation.y = quaternion[1]
        goal_pose.pose.orientation.z = quaternion[2]
        goal_pose.pose.orientation.w = quaternion[3]
        
        return goal_pose
    
    def move_base_with_planner(self, x, y, yaw, planner_name="navfn", controller_name="teb"):
        """
        使用 MoveBase Action 导航到目标点
        
        Args:
            x, y, yaw: 目标位置和姿态
            planner_name: 全局规划器名称 ("navfn" 或 "global_planner")
            controller_name: 局部控制器名称 ("teb" 或 "dwa")
        """
        goal = MoveBaseGoal()
        goal.target_pose = self.create_goal_pose(x, y, yaw)
        
        # 关键：指定使用哪个规划器和控制器
        goal.planner = planner_name
        goal.controller = controller_name
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("使用 MoveBase Action 导航")
        rospy.loginfo("目标位置: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw)
        rospy.loginfo("全局规划器: %s", planner_name)
        rospy.loginfo("局部控制器: %s", controller_name)
        rospy.loginfo("=" * 60)
        
        # 发送目标
        self.move_base_client.send_goal(goal)
        
        # 等待结果
        self.move_base_client.wait_for_result()
        
        # 获取结果
        result = self.move_base_client.get_result()
        state = self.move_base_client.get_state()
        
        rospy.loginfo("导航完成，状态: %d", state)
        rospy.loginfo("结果消息: %s", result.message)
        
        return state, result
    
    def get_path_with_planner(self, x, y, yaw, planner_name="navfn"):
        """
        仅调用全局规划器，不执行路径
        
        Args:
            x, y, yaw: 目标位置和姿态
            planner_name: 全局规划器名称
        """
        goal = GetPathGoal()
        goal.target_pose = self.create_goal_pose(x, y, yaw)
        
        # 关键：指定使用哪个规划器
        goal.planner = planner_name
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("使用 GetPath Action 获取路径")
        rospy.loginfo("目标位置: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw)
        rospy.loginfo("全局规划器: %s", planner_name)
        rospy.loginfo("=" * 60)
        
        # 发送目标
        self.get_path_client.send_goal(goal)
        
        # 等待结果
        self.get_path_client.wait_for_server()
        self.get_path_client.wait_for_result()
        
        # 获取结果
        result = self.get_path_client.get_result()
        state = self.get_path_client.get_state()
        
        rospy.loginfo("路径规划完成，状态: %d", state)
        rospy.loginfo("路径长度: %d 个点", len(result.path.poses))
        
        return result.path
    
    def compare_planners(self, x, y, yaw):
        """比较不同规划器的性能"""
        planners = ["navfn", "global_planner"]
        
        for planner in planners:
            rospy.loginfo("\n" + "=" * 60)
            rospy.loginfo("测试规划器: %s", planner)
            rospy.loginfo("=" * 60)
            
            start_time = rospy.Time.now()
            path = self.get_path_with_planner(x, y, yaw, planner)
            duration = (rospy.Time.now() - start_time).to_sec()
            
            rospy.loginfo("规划器 %s:", planner)
            rospy.loginfo("  - 路径点数: %d", len(path.poses))
            rospy.loginfo("  - 规划时间: %.3f 秒", duration)
            
            rospy.sleep(1.0)
    
    def example_scenarios(self):
        """示例场景"""
        
        # 场景1：狭窄空间，使用 TEB（更精确）
        rospy.loginfo("\n场景1：狭窄空间导航 - 使用 TEB 控制器")
        self.move_base_with_planner(
            x=2.0, y=-3.4, yaw=-1.57,
            planner_name="navfn",
            controller_name="teb"
        )
        rospy.sleep(2.0)
        
        # 场景2：开阔空间，使用 DWA（更快）
        rospy.loginfo("\n场景2：开阔空间导航 - 使用 DWA 控制器")
        self.move_base_with_planner(
            x=5.0, y=0.0, yaw=0.0,
            planner_name="global_planner",
            controller_name="dwa"
        )
        rospy.sleep(2.0)
        
        # 场景3：比较不同规划器性能
        rospy.loginfo("\n场景3：比较规划器性能")
        self.compare_planners(x=10.0, y=10.0, yaw=0.0)


def main():
    """主函数"""
    try:
        example = MBFActionExample()
        
        # 选择运行模式
        mode = rospy.get_param('~mode', 'single')
        
        if mode == 'single':
            # 单次导航示例
            x = rospy.get_param('~x', 2.0)
            y = rospy.get_param('~y', -3.4)
            yaw = rospy.get_param('~yaw', -1.57)
            planner = rospy.get_param('~planner', 'navfn')
            controller = rospy.get_param('~controller', 'teb')
            
            example.move_base_with_planner(x, y, yaw, planner, controller)
            
        elif mode == 'compare':
            # 比较不同规划器
            x = rospy.get_param('~x', 2.0)
            y = rospy.get_param('~y', -3.4)
            yaw = rospy.get_param('~yaw', -1.57)
            
            example.compare_planners(x, y, yaw)
            
        elif mode == 'scenarios':
            # 运行示例场景
            example.example_scenarios()
        
        rospy.loginfo("示例完成")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr("发生错误: %s", str(e))


if __name__ == '__main__':
    main()
