#!/usr/bin/env python3

"""
Dispatch Node with Move Base Flex (MBF) Action Integration

This node handles:
1. Listening to /goal_v3 for navigation/relocation/stop commands
2. Map switching via /robot1/change_map service
3. Navigation via MBF action (/robot1/move_base_flex/move_base)
4. Sending status signals to /signal
"""

import rospy
import actionlib
import threading
import os

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav_msgs.srv import LoadMap

# Custom messages
from robot_v3.msg import Goal_v3
from fast_lio.msg import RelocalizationMsg

# Elevator docking action
from elevator_docking.msg import ElevatorDockingAction, ElevatorDockingGoal

# MBF messages
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult


class DispatchNode:
    # MBF Result codes
    SUCCESS = 0
    FAILURE = 10
    CANCELED = 11
    COLLISION = 12
    OSCILLATION = 13
    START_BLOCKED = 14
    GOAL_BLOCKED = 15
    TF_ERROR = 16
    INTERNAL_ERROR = 17
    PLAN_FAILURE = 50
    CTRL_FAILURE = 100

    def __init__(self):
        rospy.init_node('dispatch_node', anonymous=True)

        # Configuration
        self.map_folder = rospy.get_param('~map_folder',
            "/home/amov/PersonalData/Program/Ros/dispatch_ws/src/dispatch_control/Map")
        self.default_house = rospy.get_param('~default_house', "ntuitive_align")
        self.default_floor = rospy.get_param('~default_floor', "2m")

        # MBF planner/controller configuration
        self.planner = rospy.get_param('~planner', "global_planner/GlobalPlanner")
        self.controller = rospy.get_param('~controller', "local_planner/TebLocalPlannerROS_Omnidirectiona")

        # State
        self.current_house = self.default_house
        self.current_floor = self.default_floor
        self.is_navigating = False
        self.lock = threading.Lock()

        # Publishers
        self.signal_pub = rospy.Publisher('/signal', String, queue_size=10)
        self.relocalization_pub = rospy.Publisher('/relocalization', RelocalizationMsg, queue_size=1, latch=True)

        # MBF Action Client
        self.mbf_client = actionlib.SimpleActionClient(
            '/robot1/move_base_flex/move_base',
            MoveBaseAction
        )

        rospy.loginfo("Waiting for MBF action server...")
        if not self.mbf_client.wait_for_server(rospy.Duration(10.0)):
            rospy.logwarn("MBF action server not available, will retry when sending goals")
        else:
            rospy.loginfo("MBF action server connected")

        # Elevator docking Action Client
        self.docking_action_name = rospy.get_param('~docking_action', '/robot1/docking')
        self.docking_client = actionlib.SimpleActionClient(
            self.docking_action_name,
            ElevatorDockingAction
        )

        rospy.loginfo(f"Waiting for elevator docking action server at {self.docking_action_name}...")
        if not self.docking_client.wait_for_server(rospy.Duration(10.0)):
            rospy.logwarn("Elevator docking action server not available, will retry when sending docking commands")
        else:
            rospy.loginfo("Elevator docking action server connected")

        # Subscriber
        rospy.Subscriber('/goal_v3', Goal_v3, self.goal_callback)

        # Initial relocalization
        self.send_initial_relocalization()

        rospy.loginfo("Dispatch node started. Listening for /goal_v3 messages...")

    def send_initial_relocalization(self):
        """Send initial relocalization message on startup"""
        pcd_path = self.get_pcd_path(self.current_house, self.current_floor)

        relocalization_msg = RelocalizationMsg()
        relocalization_msg.pcd_path = pcd_path

        # Set initial pose
        relocalization_msg.init_pose = PoseStamped()
        relocalization_msg.init_pose.header.frame_id = "map"
        relocalization_msg.init_pose.header.stamp = rospy.Time.now()

        relocalization_msg.init_pose.pose.position.x = 2.6
        relocalization_msg.init_pose.pose.position.y = -1.2
        relocalization_msg.init_pose.pose.position.z = 0.0

        relocalization_msg.init_pose.pose.orientation.x = 0.0
        relocalization_msg.init_pose.pose.orientation.y = 0.0
        relocalization_msg.init_pose.pose.orientation.z = -0.79161106
        relocalization_msg.init_pose.pose.orientation.w = 0.61102531

        rospy.sleep(0.1)
        self.relocalization_pub.publish(relocalization_msg)
        rospy.loginfo(f"Published initial relocalization: pcd_path={pcd_path}")

    def get_pcd_path(self, house, floor):
        return f"{self.map_folder}/{house}/{floor}/{floor}.pcd"

    def get_map_path(self, house, floor):
        return f"{self.map_folder}/{house}/{floor}/map.yaml"

    def publish_signal(self, signal):
        """Publish a signal message"""
        msg = String()
        msg.data = signal
        self.signal_pub.publish(msg)
        rospy.loginfo(f"Published signal: {signal}")

    def change_map(self, map_path):
        """Change the map using the change_map service"""
        service_name = '/robot1/change_map'

        try:
            rospy.loginfo(f"Waiting for service: {service_name}")
            rospy.wait_for_service(service_name, timeout=5.0)

            change_map_service = rospy.ServiceProxy(service_name, LoadMap)
            response = change_map_service(map_path)

            if response.result == 0:  # MAP_LOAD_SUCCESS
                rospy.loginfo(f"Map changed successfully: {map_path}")
                return True
            else:
                rospy.logwarn(f"Map change failed with result: {response.result}")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        except rospy.ROSException as e:
            rospy.logerr(f"ROS exception: {e}")
            return False

    def goal_callback(self, msg):
        """Handle incoming goal messages"""
        rospy.loginfo(f"Received goal: floor={msg.floor}, house={msg.house}, "
                     f"relocation={msg.relocation}, stop={msg.stop}, "
                     f"planner={msg.planner}, controller={msg.controller}"
                     f"elevator={msg.elevator_docking}")

        # Update current location info
        self.current_floor = msg.floor
        self.current_house = msg.house

        if msg.stop:
            self.handle_stop()
        elif msg.relocation:
            self.handle_relocation(msg)
        elif msg.elevator_docking:
            self.handle_elevator_docking(msg)
        else:
            self.handle_navigation(msg)

    def execute_docking_command(self, command, timeout=None):
        """Send a command to the elevator docking action server"""
        goal = ElevatorDockingGoal()
        goal.command = command

        self.docking_client.send_goal(goal)

        if timeout:
            if not self.docking_client.wait_for_result(rospy.Duration(timeout)):
                self.docking_client.cancel_goal()
                rospy.logwarn(f"Docking command '{command}' timed out after {timeout} seconds")
                return None
        else:
            self.docking_client.wait_for_result()

        result = self.docking_client.get_result()
        if result is None:
            rospy.logerr(f"Docking command '{command}' returned no result")
        return result

    def handle_elevator_docking(self, msg):
        """Handle elevator docking command"""
        command = (msg.elevator_docking or "").strip().lower()

        if not command:
            rospy.logwarn("Elevator docking command empty")
            return

        # Stop navigation before docking
        with self.lock:
            if self.is_navigating:
                self.mbf_client.cancel_goal()
                self.is_navigating = False
                rospy.loginfo("Cancelled navigation before elevator docking")

        if not self.docking_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Elevator docking action server not available")
            self.publish_signal("ELEVATOR_FAILED:Docking server unavailable")
            return

        self.publish_signal(f"ELEVATOR_RECEIVED:{command}")

        if command == "in":
            detection_result = self.execute_docking_command("detection")
            if not detection_result or not detection_result.success:
                rospy.logerr("Elevator detection failed before docking")
                self.publish_signal("ELEVATOR_FAILED:Detection failed")
                return

            rospy.loginfo("Detection succeeded, proceeding with 'in' command")
            docking_result = self.execute_docking_command("in")

            if docking_result and docking_result.success:
                self.publish_signal("ELEVATOR_SUCCESS:Docked in")
            else:
                error_msg = docking_result.message if docking_result else "Unknown docking error"
                rospy.logerr(f"Failed to dock in: {error_msg}")
                self.publish_signal("ELEVATOR_FAILED:In command failed")

        elif command == "out":
            docking_result = self.execute_docking_command("out")

            if docking_result and docking_result.success:
                self.publish_signal("ELEVATOR_SUCCESS:Exited elevator")
            else:
                error_msg = docking_result.message if docking_result else "Unknown docking error"
                rospy.logerr(f"Failed to exit elevator: {error_msg}")
                self.publish_signal("ELEVATOR_FAILED:Out command failed")

        elif command == "detection":
            detection_result = self.execute_docking_command("detection")
            if detection_result and detection_result.success:
                self.publish_signal("ELEVATOR_SUCCESS:Detection complete")
            else:
                self.publish_signal("ELEVATOR_FAILED:Detection failed")
        else:
            rospy.logwarn(f"Unsupported elevator docking command: {command}")
            self.publish_signal("ELEVATOR_FAILED:Unsupported command")

    def handle_stop(self):
        """Handle stop command"""
        self.publish_signal("STOP_RECEIVED")
        rospy.sleep(0.1)

        with self.lock:
            if self.is_navigating:
                self.mbf_client.cancel_goal()
                self.is_navigating = False
                rospy.loginfo("Cancelled current navigation goal")

        rospy.sleep(0.1)
        self.publish_signal("STOP_SUCCESSFUL")

    def handle_relocation(self, msg):
        """Handle relocation command"""
        # Stop any ongoing navigation
        with self.lock:
            if self.is_navigating:
                self.mbf_client.cancel_goal()
                self.is_navigating = False

        self.publish_signal("RELOCATION_RECEIVED")
        rospy.sleep(0.1)

        pcd_path = self.get_pcd_path(msg.house, msg.floor)

        relocalization_msg = RelocalizationMsg()
        relocalization_msg.pcd_path = pcd_path
        relocalization_msg.init_pose = msg.pose

        self.relocalization_pub.publish(relocalization_msg)
        rospy.loginfo(f"Published relocalization: pcd_path={pcd_path}, floor={msg.floor}")

    def handle_navigation(self, msg):
        """Handle navigation command"""
        self.publish_signal("GOAL_RECEIVED")
        rospy.sleep(0.1)

        # Cancel any ongoing navigation
        with self.lock:
            if self.is_navigating:
                self.mbf_client.cancel_goal()
                rospy.sleep(0.1)
                rospy.loginfo("Cancelled previous navigation goal")

        # Change map if needed
        map_path = self.get_map_path(msg.house, msg.floor)
        if not self.change_map(map_path):
            self.publish_signal("GOAL_Failed:Map change failed")
            return

        # Send navigation goal
        self.send_navigation_goal(msg.pose, msg.planner, msg.controller)

    def send_navigation_goal(self, pose, planner, controller):
        """Send navigation goal to MBF"""
        with self.lock:
            self.is_navigating = True

        # Wait for action server if not available
        if not self.mbf_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("MBF action server not available")
            self.publish_signal("GOAL_Failed:MBF server not available")
            with self.lock:
                self.is_navigating = False
            return

        # Create MBF goal
        goal = MoveBaseGoal()
        goal.target_pose = pose
        goal.target_pose.header.stamp = rospy.Time.now()

        if(planner):
            goal.planner = planner
        else:
            goal.planner = self.planner

        if(controller):
            goal.controller = controller
        else:
            goal.controller = self.controller
        rospy.loginfo(f"Sending navigation goal: position=({pose.pose.position.x:.2f}, "
                     f"{pose.pose.position.y:.2f}), planner={goal.planner}, controller={goal.controller}")

        # Send goal with callbacks
        self.mbf_client.send_goal(
            goal,
            done_cb=self.navigation_done_callback,
            feedback_cb=self.navigation_feedback_callback
        )

    def navigation_feedback_callback(self, feedback):
        """Handle navigation feedback"""
        # Log feedback periodically (every ~2 seconds based on controller frequency)
        if hasattr(self, '_last_feedback_log'):
            if (rospy.Time.now() - self._last_feedback_log).to_sec() < 2.0:
                return
        self._last_feedback_log = rospy.Time.now()

        rospy.logdebug(f"Navigation feedback: dist_to_goal={feedback.dist_to_goal:.2f}, "
                      f"angle_to_goal={feedback.angle_to_goal:.2f}")

    def navigation_done_callback(self, state, result):
        """Handle navigation completion"""
        with self.lock:
            self.is_navigating = False

        outcome = result.outcome
        message = result.message

        rospy.loginfo(f"Navigation done: outcome={outcome}, message={message}")

        if outcome == self.SUCCESS:
            # Goal reached successfully
            self.publish_signal("GOAL_ARRIVED")
            rospy.loginfo("Goal reached successfully!")

        elif outcome == self.CANCELED:
            # Goal was cancelled (by stop command)
            rospy.loginfo("Navigation was cancelled")

        else:
            # Navigation failed
            error_msg = self.get_error_message(outcome, message)
            self.publish_signal(f"GOAL_Failed:{error_msg}")
            rospy.logerr(f"Navigation failed: {error_msg}")

    def get_error_message(self, outcome, message):
        """Convert outcome code to human-readable error message"""
        error_map = {
            self.FAILURE: "General failure",
            self.COLLISION: "Collision detected",
            self.OSCILLATION: "Robot oscillating",
            self.START_BLOCKED: "Start position blocked",
            self.GOAL_BLOCKED: "Goal position blocked",
            self.TF_ERROR: "TF error",
            self.INTERNAL_ERROR: "Internal error",
            self.PLAN_FAILURE: "Planning failed",
            self.CTRL_FAILURE: "Controller failed",
        }

        base_msg = error_map.get(outcome, f"Unknown error (code={outcome})")
        if message:
            return f"{base_msg}: {message}"
        return base_msg

    def run(self):
        """Main loop"""
        rospy.spin()


def main():
    try:
        node = DispatchNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
