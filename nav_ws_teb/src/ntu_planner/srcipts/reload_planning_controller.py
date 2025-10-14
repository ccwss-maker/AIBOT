#!/usr/bin/env python

import rospy
import os
from ntu_planner.msg import ReloadplanningMsg
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import LoadMap

class ReloadPlanningController:
    def __init__(self):
        rospy.init_node('reload_planning_controller', anonymous=True)
        
        # Subscribe to reload planning topic
        self.reload_sub = rospy.Subscriber('/reloadplanning', ReloadplanningMsg, self.reload_callback)
        
        # Publisher for goal pose
        self.goal_pub = rospy.Publisher('/robot1/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
        
        rospy.loginfo("Reload Planning Controller started")
    
    def reload_callback(self, msg):
        rospy.loginfo("Received reload planning request")
        rospy.loginfo("Map path: %s", msg.map_path)
        
        # Change map using service call
        self.change_map(msg.map_path)
        
        # Publish the goal pose
        self.publish_goal(msg.goal_pose)
    
    def change_map(self, map_path):
        try:
            # Ensure map path is absolute
            if not os.path.isabs(map_path):
                # If relative path, assume it's relative to ntu_planner package
                package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                map_path = os.path.join(package_path, map_path)
            
            rospy.loginfo("Changing map to: %s", map_path)
            
            # Wait for the change_map service to be available
            service_name = '/robot1/change_map'
            rospy.loginfo("Waiting for service: %s", service_name)
            rospy.wait_for_service(service_name, timeout=5.0)
            
            # Call the change_map service
            change_map_service = rospy.ServiceProxy(service_name, LoadMap)
            response = change_map_service(map_path)
            
            if response.result == 0:  # MAP_LOAD_SUCCESS
                rospy.loginfo("Map changed successfully")
            else:
                rospy.logwarn("Map change failed with result: %d", response.result)
                
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
        except rospy.ROSException as e:
            rospy.logerr("ROS exception: %s", str(e))
        except Exception as e:
            rospy.logerr("Failed to change map: %s", str(e))
    
    def publish_goal(self, goal_pose):
        try:
            rospy.loginfo("Publishing goal pose to /robot1/move_base_simple/goal")
            
            # Set the timestamp
            goal_pose.header.stamp = rospy.Time.now()
            
            # Publish the goal
            self.goal_pub.publish(goal_pose)
            
            rospy.loginfo("Goal published successfully")
            
        except Exception as e:
            rospy.logerr("Failed to publish goal: %s", str(e))
    
    def run(self):
        rospy.loginfo("Reload Planning Controller running...")
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = ReloadPlanningController()
        controller.run()
    except rospy.ROSInterruptException:
        pass