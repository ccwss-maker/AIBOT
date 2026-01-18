#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from robot_v3.msg import Goal_v3

class GoalPublisherSimulator:
    def __init__(self):
        rospy.init_node('goal_pub_simulator', anonymous=True)
        
        # Create publisher for Goal_v3
        self.goal_pub = rospy.Publisher('/goal_v3', Goal_v3, queue_size=1)
        
        # Subscribe to move_base_simple/goal
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        rospy.loginfo("Goal Publisher Simulator started. Listening for /move_base_simple/goal messages...")
    
    def goal_callback(self, msg):
        """Callback function to convert PoseStamped to Goal_v3"""
        
        # Create Goal_v3 message
        goal_v3_msg = Goal_v3()
        
        # Set pose from received message
        goal_v3_msg.pose.header = msg.header
        goal_v3_msg.pose.pose.position.x = 1.5788087844848633
        goal_v3_msg.pose.pose.position.y = -0.40869536995887756
        goal_v3_msg.pose.pose.position.z = 0.0
        goal_v3_msg.pose.pose.orientation.x = 0.0
        goal_v3_msg.pose.pose.orientation.y = 0.0
        goal_v3_msg.pose.pose.orientation.z = -0.07488346811171313
        goal_v3_msg.pose.pose.orientation.w = 0.9971922914882375

        # Set default values
        goal_v3_msg.floor = "2m"
        goal_v3_msg.house = "ntuitive_align"
        goal_v3_msg.relocation = False
        goal_v3_msg.stop = False
        
        # Publish Goal_v3 message
        self.goal_pub.publish(goal_v3_msg)
        
        rospy.loginfo(f"Converted and published goal: pose={msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, floor={goal_v3_msg.floor}, house={goal_v3_msg.house}")

def main():
    try:
        simulator = GoalPublisherSimulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Goal Publisher Simulator node terminated.")

if __name__ == '__main__':
    main()