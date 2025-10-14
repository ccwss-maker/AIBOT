#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from robot_v3.msg import Goal_v3
from fast_lio.msg import RelocalizationMsg
from ntu_planner.msg import ReloadplanningMsg
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID

# Global variables
map_folder = "/home/amov/PersonalData/Program/Ros/dispatch_ws/src/dispatch_control/Map"
building = "ntuitive_align"
floor_now = "2m"
reloadplanning_pub = None
signal_pub = None
relocalization_pub = None
cancel_pub = None
def goal_callback(msg):
    global floor_now, reloadplanning_pub, signal_pub, relocalization_pub, cancel_pub
    print(f"Received goal: {msg}")
    
    floor = msg.floor
    
    # Update paths based on floor
    pcd_path = f"{map_folder}/{building}/{floor}/{floor}.pcd"
    map_path = f"{map_folder}/{building}/{floor}/map.yaml"

    # Check if Relocalization
    if msg.stop:
        # Send stop received signal
        signal_msg = String()
        rospy.sleep(0.1)
        signal_msg.data = "STOP_RECEIVED"
        signal_pub.publish(signal_msg)
        # Send cancel command to move_base
        cancel_msg = GoalID()
        rospy.sleep(0.1)
        cancel_pub.publish(cancel_msg)
        rospy.loginfo("Published stop command and STOP_RECEIVED signal")
    elif msg.relocation:
        signal_msg = String()
        rospy.sleep(0.1)
        signal_msg.data = "RELOCATION_RECEIVED"
        signal_pub.publish(signal_msg)
        # Create RelocalizationMsg
        relocalization_msg = RelocalizationMsg()
        rospy.sleep(0.1)
        relocalization_msg.pcd_path = pcd_path
        relocalization_msg.init_pose = msg.pose
        relocalization_pub.publish(relocalization_msg)
        rospy.loginfo(f"Published to /relocalization: pcd_path={pcd_path}, init_pose={msg.pose} and floor={floor}")
    else:
        signal_msg = String()
        rospy.sleep(0.1)
        signal_msg.data = "GOAL_RECEIVED"
        signal_pub.publish(signal_msg)
        # Create ReloadplanningMsg
        reload_msg = ReloadplanningMsg()
        rospy.sleep(0.1)
        reload_msg.map_path = map_path
        reload_msg.goal_pose = msg.pose
        reloadplanning_pub.publish(reload_msg)
        rospy.loginfo(f"Published to /reloadplanning: map_path={map_path}, goal_pose={msg.pose} and floor={floor}")

def main():
    global reloadplanning_pub, signal_pub, relocalization_pub, cancel_pub, floor_now
    rospy.init_node('map_control', anonymous=True)
    
    # Construct initial paths
    pcd_path = f"{map_folder}/{building}/{floor_now}/{floor_now}.pcd"
    map_path = f"{map_folder}/{building}/{floor_now}/map.yaml"
    
    # Create publishers
    relocalization_pub = rospy.Publisher('/relocalization', RelocalizationMsg, queue_size=1, latch=True)
    reloadplanning_pub = rospy.Publisher('/reloadplanning', ReloadplanningMsg, queue_size=1)
    signal_pub = rospy.Publisher('/signal', String, queue_size=1)
    cancel_pub = rospy.Publisher('/robot1/move_base/cancel', GoalID, queue_size=1)
    
    # Create RelocalizationMsg
    relocalization_msg = RelocalizationMsg()
    rospy.sleep(0.1)
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
    relocalization_msg.init_pose.pose.orientation.w =  0.61102531
    
    # Publish relocalization message
    relocalization_pub.publish(relocalization_msg)
    
    rospy.Subscriber('/goal_v3', Goal_v3, goal_callback)
    
    rospy.loginfo("Dispatch node started. Listening for /goal_v3 messages...")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
