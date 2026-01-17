#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from elevator_docking.msg import ElevatorDockingAction, ElevatorDockingGoal

class ElevatorDockingClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            '/robot1/elevator/docking',
            ElevatorDockingAction
        )

        rospy.loginfo("Waiting for elevator docking action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to elevator docking action server")

    def send_command(self, command):
        """Send a command to the elevator docking server

        Args:
            command: "detection", "in", or "out"
        """
        goal = ElevatorDockingGoal()
        goal.command = command

        rospy.loginfo("Sending command: %s", command)
        self.client.send_goal(goal, feedback_cb=self.feedback_cb)

        # Wait for result
        self.client.wait_for_result()
        result = self.client.get_result()

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

def main():
    rospy.init_node('elevator_docking_test_client')

    client = ElevatorDockingClient()

    # Interactive mode
    rospy.loginfo("=== Elevator Docking Test Client ===")
    rospy.loginfo("Commands: detection, in, out, quit")

    while not rospy.is_shutdown():
        try:
            command = input("\nEnter command: ").strip()

            if command == "quit" or command == "exit":
                rospy.loginfo("Exiting...")
                break

            if command not in ["detection", "in", "out"]:
                rospy.logwarn("Invalid command. Use: detection, in, out, quit")
                continue

            result = client.send_command(command)

            if not result.success:
                rospy.logwarn("Command failed: %s", result.message)

        except KeyboardInterrupt:
            rospy.loginfo("Interrupted by user")
            break
        except Exception as e:
            rospy.logerr("Error: %s", str(e))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
