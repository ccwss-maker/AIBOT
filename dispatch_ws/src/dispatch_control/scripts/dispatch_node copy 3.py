#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from robot_v3.msg import Goal_v3
from fast_lio.msg import RelocalizationMsg
from ntu_planner.msg import ReloadplanningMsg
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID, GoalStatusArray
import threading
import time

# Global variables
map_folder = "/home/amov/PersonalData/Program/Ros/dispatch_ws/src/dispatch_control/Map"
house = "ntuitive_align"
floor = "2m"
reloadplanning_pub = None
signal_pub = None
relocalization_pub = None
cancel_pub = None

class RobotStatusMonitor:
    def __init__(self, signal_pub, reloadplanning_pub):
        self.signal_pub = signal_pub
        self.reloadplanning_pub = reloadplanning_pub
        self.current_status = None
        self.current_text = ""
        self.is_monitoring = False
        self.retry_count = 0
        self.max_retries = 2
        self.reload_msg = None
        self.status_subscriber = None
        self.monitor_thread = None
        self.lock = threading.Lock()
        self.goal_start_time = None  # Track when goal was sent
        
    def start_monitoring(self, reload_msg):
        rospy.loginfo("start_monitoring called")
        with self.lock:
            if self.is_monitoring:
                rospy.logwarn("Already monitoring previous goal, cancelling it and starting new goal...")
                # Cancel the previous goal first
                cancel_msg = GoalID()
                cancel_pub.publish(cancel_msg)
                rospy.sleep(0.1)
                rospy.loginfo("Cancelled previous goal before starting new one")
            
            # Always reset monitoring state for new goal
            self.is_monitoring = True
            self.retry_count = 0
            self.reload_msg = reload_msg
            self.current_status = None
            self.current_text = ""
            self.goal_start_time = rospy.Time.now()  # Record when we start this goal
            
        if self.status_subscriber is None:
            self.status_subscriber = rospy.Subscriber('/robot1/move_base/status', GoalStatusArray, self.status_callback)
        
        rospy.loginfo("Starting robot status monitoring...")
        self.send_reload_planning()
        
    def stop_monitoring(self):
        # Don't use lock here since this is often called from within status_callback
        # which already holds the lock
        self.is_monitoring = False
            
    def status_callback(self, msg):
        # First check without lock for quick exit
        if not self.is_monitoring or not msg.status_list:
            return
            
        latest_status = msg.status_list[-1]
        new_status = latest_status.status
        new_text = latest_status.text
        
        # Get current state with minimal lock time
        with self.lock:
            if not self.is_monitoring:  # Double check
                return
            old_status = self.current_status
            
            # Initialize current status if this is first callback
            if self.current_status is None:
                self.current_status = new_status
                self.current_text = new_text
                rospy.loginfo(f"Initialized status monitoring with status: {new_status}")
                return
                
            self.current_status = new_status
            self.current_text = new_text
            
        # Handle status change outside of lock
        if old_status != new_status:
            self.handle_status_change()
    
    def handle_status_change(self):
        status = self.current_status
        text = self.current_text
        
        if status == 1:  # ACTIVE
            rospy.loginfo(f"Robot state changed to ACTIVE (1): {text}")
            
        elif status == 3:  # SUCCEEDED
            rospy.loginfo(f"Robot state changed to SUCCEEDED (3): Goal reached")
            signal_msg = String()
            signal_msg.data = "GOAL_ARRIVED"
            self.signal_pub.publish(signal_msg)
            self.stop_monitoring()
            
        elif status == 4:  # ABORTED
            rospy.logwarn(f"Robot state changed to ABORTED (4): {text}")
            self.handle_failure()
    
    def handle_failure(self):
        self.retry_count += 1
        
        if self.retry_count <= self.max_retries:
            rospy.loginfo(f"Retrying goal execution (attempt {self.retry_count}/{self.max_retries})")
            time.sleep(0.5)
            self.send_reload_planning()
        else:
            rospy.logerr(f"Goal failed after {self.max_retries} attempts")
            signal_msg = String()
            signal_msg.data = f"GOAL_Failed:{self.current_text}"
            self.signal_pub.publish(signal_msg)
            self.stop_monitoring()
    
    def send_reload_planning(self):
        if self.reload_msg is not None:
            # Cancel any previous goals first (especially important for retries)
            cancel_msg = GoalID()
            cancel_pub.publish(cancel_msg)
            rospy.sleep(0.1)
            
            self.reloadplanning_pub.publish(self.reload_msg)
            rospy.loginfo(f"Sent reload planning message: map_path={self.reload_msg.map_path}, goal_pose.position=({self.reload_msg.goal_pose.pose.position.x:.2f}, {self.reload_msg.goal_pose.pose.position.y:.2f})")

status_monitor = None
def goal_callback(msg):
    global floor, house, reloadplanning_pub, signal_pub, relocalization_pub, cancel_pub, status_monitor
    print(f"Received goal: {msg}")
    
    floor = msg.floor
    house = msg.house
    
    # Update paths based on floor
    pcd_path = f"{map_folder}/{house}/{floor}/{floor}.pcd"
    map_path = f"{map_folder}/{house}/{floor}/map.yaml"

    # Check if Relocalization
    if msg.stop:
        # Stop monitoring if active
        if status_monitor:
            status_monitor.stop_monitoring()
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
        # Stop monitoring if active
        if status_monitor:
            status_monitor.stop_monitoring()
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
        reload_msg.map_path = map_path
        reload_msg.goal_pose = msg.pose
        
        # Start status monitoring instead of directly publishing
        if status_monitor:
            rospy.loginfo("About to call start_monitoring...")
            status_monitor.start_monitoring(reload_msg)
        else:
            rospy.logerr("status_monitor is None!")
        
        rospy.loginfo(f"Started goal monitoring for: map_path={map_path}, goal_pose={msg.pose} and floor={floor}")

def main():
    global reloadplanning_pub, signal_pub, relocalization_pub, cancel_pub, floor, status_monitor
    rospy.init_node('map_control', anonymous=True)
    
    # Construct initial paths
    pcd_path = f"{map_folder}/{house}/{floor}/{floor}.pcd"
    map_path = f"{map_folder}/{house}/{floor}/map.yaml"
    
    # Create publishers
    relocalization_pub = rospy.Publisher('/relocalization', RelocalizationMsg, queue_size=1, latch=True)
    reloadplanning_pub = rospy.Publisher('/reloadplanning', ReloadplanningMsg, queue_size=1)
    signal_pub = rospy.Publisher('/signal', String, queue_size=1)
    cancel_pub = rospy.Publisher('/robot1/move_base/cancel', GoalID, queue_size=1)
    
    # Initialize status monitor
    status_monitor = RobotStatusMonitor(signal_pub, reloadplanning_pub)
    
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
