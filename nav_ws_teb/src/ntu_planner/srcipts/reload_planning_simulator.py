#!/usr/bin/env python

import rospy
from ntu_planner.msg import ReloadplanningMsg
from geometry_msgs.msg import PoseStamped

class ReloadPlanningSimulator:
    def __init__(self):
        rospy.init_node('reload_planning_simulator', anonymous=True)
        
        # Publisher for reload planning messages
        self.reload_pub = rospy.Publisher('/reloadplanning', ReloadplanningMsg, queue_size=1)
        
        rospy.loginfo("Reload Planning Simulator started")
        rospy.loginfo("Press Ctrl+C to stop")
        
    def publish_reload_message(self):
        # Create ReloadplanningMsg
        reload_msg = ReloadplanningMsg()
        
        # Set map path
        reload_msg.map_path = "/home/amov/PersonalData/Program/Ros/nav_ws_teb/src/ntu_planner/map/map.yaml"
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = rospy.Time.now()
        
        # Set position
        goal_pose.pose.position.x = 1.996903419494629
        goal_pose.pose.position.y = -3.4358625411987305
        goal_pose.pose.position.z = 0.0
        
        # Set orientation
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = -0.7656226068799755
        goal_pose.pose.orientation.w = 0.6432899997934917
        
        reload_msg.goal_pose = goal_pose
        
        # Publish the message
        rospy.loginfo("Publishing reload planning message...")
        rospy.loginfo("Map path: %s", reload_msg.map_path)
        rospy.loginfo("Goal position: x=%.3f, y=%.3f, z=%.3f", 
                     goal_pose.pose.position.x,
                     goal_pose.pose.position.y, 
                     goal_pose.pose.position.z)
        rospy.loginfo("Goal orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                     goal_pose.pose.orientation.x,
                     goal_pose.pose.orientation.y,
                     goal_pose.pose.orientation.z,
                     goal_pose.pose.orientation.w)
        
        self.reload_pub.publish(reload_msg)
        rospy.loginfo("Message published!")
        
    def run(self):
        # Wait for at least one subscriber to connect
        while self.reload_pub.get_num_connections() == 0:
            rospy.loginfo("Waiting for subscribers...")
            rospy.sleep(0.1)
        
        # Publish once
        self.publish_reload_message()
        
        rospy.loginfo("Message published once. Node shutting down.")
        rospy.signal_shutdown("Task completed")

if __name__ == '__main__':
    try:
        simulator = ReloadPlanningSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Reload Planning Simulator stopped")