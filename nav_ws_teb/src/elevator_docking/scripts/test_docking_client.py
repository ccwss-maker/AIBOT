#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
test_docking_client.py - 电梯对接测试客户端

支持的命令:
  detection  - 检测电梯位置
  in         - 检测并驶入电梯
  out        - 驶出电梯
  execute    - 直接执行指定的 x, y, yaw 位置
  quit/exit  - 退出程序
"""
import rospy
import actionlib
from elevator_docking.msg import ElevatorDockingAction, ElevatorDockingGoal
import math

class ElevatorDockingClient:
    def __init__(self):
        # 获取 action server 名称参数
        action_server_name = rospy.get_param("~action_server_name", "/robot1/docking")

        self.client = actionlib.SimpleActionClient(
            action_server_name,
            ElevatorDockingAction
        )

        rospy.loginfo("Waiting for elevator docking action server at: %s", action_server_name)
        self.client.wait_for_server()
        rospy.loginfo("Connected to elevator docking action server")

    def send_command(self, command, target_x=0.0, target_y=0.0, target_yaw=0.0):
        """Send a command to the elevator docking server

        Args:
            command: "detection", "in", "out", or "execute"
            target_x: Target X position (for execute command)
            target_y: Target Y position (for execute command)
            target_yaw: Target yaw angle in radians (for execute command)
        """
        goal = ElevatorDockingGoal()
        goal.command = command
        goal.target_x = target_x
        goal.target_y = target_y
        goal.target_yaw = target_yaw

        if command == "execute":
            rospy.loginfo("Sending execute command: x=%.3f, y=%.3f, yaw=%.3f rad (%.1f deg)",
                         target_x, target_y, target_yaw, target_yaw * 180.0 / math.pi)
        else:
            rospy.loginfo("Sending command: %s", command)

        self.client.send_goal(goal, feedback_cb=self.feedback_cb)

        # Wait for result
        self.client.wait_for_result()
        result = self.client.get_result()

        if result is None:
            rospy.logerr("Action failed: no result received (action may have been preempted or cancelled)")
            return None

        rospy.loginfo("Result: success=%s, message=%s", result.success, result.message)

        if command == "detection" and result.success:
            rospy.loginfo("Detection result:")
            rospy.loginfo("  Center: (%.2f, %.2f)", result.center_x, result.center_y)
            rospy.loginfo("  Size: %.2fx%.2f", result.width, result.depth)
            rospy.loginfo("  Yaw: %.1f deg", result.yaw_deg)
            rospy.loginfo("  Confidence: %.2f", result.confidence)

        return result

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback: %s (progress: %.1f%%)",
                     feedback.status, feedback.progress * 100)

def parse_execute_args(args_str):
    """Parse execute command arguments: x y yaw (yaw in degrees)"""
    parts = args_str.split()
    if len(parts) < 3:
        raise ValueError("Execute command requires 3 arguments: x y yaw_deg")

    x = float(parts[0])
    y = float(parts[1])
    yaw_deg = float(parts[2])
    yaw_rad = yaw_deg * math.pi / 180.0

    return x, y, yaw_rad

def main():
    rospy.init_node('elevator_docking_test_client')

    client = ElevatorDockingClient()

    # Interactive mode
    rospy.loginfo("=== Elevator Docking Test Client ===")
    rospy.loginfo("Commands:")
    rospy.loginfo("  detection       - Detect elevator")
    rospy.loginfo("  in              - Detect and drive into elevator")
    rospy.loginfo("  out             - Drive out of elevator")
    rospy.loginfo("  execute x y yaw - Execute direct movement (yaw in degrees)")
    rospy.loginfo("  quit/exit       - Exit program")
    rospy.loginfo("")
    rospy.loginfo("Example: execute 1.5 0.2 45")

    while not rospy.is_shutdown():
        try:
            user_input = input("\nEnter command: ").strip()

            if not user_input:
                continue

            # Parse command
            parts = user_input.split(None, 1)  # Split on first whitespace
            command = parts[0].lower()

            if command == "quit" or command == "exit":
                rospy.loginfo("Exiting...")
                break

            if command == "execute":
                if len(parts) < 2:
                    rospy.logwarn("Execute command requires arguments: execute x y yaw_deg")
                    continue

                try:
                    x, y, yaw_rad = parse_execute_args(parts[1])
                    result = client.send_command("execute", x, y, yaw_rad)
                except ValueError as e:
                    rospy.logwarn("Invalid arguments: %s", str(e))
                    continue

            elif command in ["detection", "in", "out"]:
                result = client.send_command(command)

            else:
                rospy.logwarn("Invalid command. Use: detection, in, out, execute, quit")
                continue

            if result is not None and not result.success:
                rospy.logwarn("Command failed: %s", result.message)

        except KeyboardInterrupt:
            rospy.loginfo("Interrupted by user")
            break
        except EOFError:
            rospy.loginfo("End of input")
            break
        except Exception as e:
            rospy.logerr("Error: %s", str(e))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
