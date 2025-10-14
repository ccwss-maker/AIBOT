#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from robot_v3.msg import Goal_v3

def main():
    try:
        # 初始化 ROS 节点
        rospy.init_node('goal_pub_stop_simulator', anonymous=True)

        # 创建 Publisher
        goal_pub = rospy.Publisher('/goal_v3', Goal_v3, queue_size=1)

        rospy.sleep(0.5)

        # 构造 Goal_v3 消息
        goal_v3_msg = Goal_v3()
        goal_v3_msg.pose = PoseStamped()  # 空的 PoseStamped
        goal_v3_msg.floor = "2m"
        goal_v3_msg.house = "ntuitive_align"
        goal_v3_msg.relocation = False
        goal_v3_msg.stop = True

        # 发布消息
        goal_pub.publish(goal_v3_msg)
        rospy.loginfo("Stop command published")

        rospy.sleep(0.5)

    except rospy.ROSInterruptException:
        rospy.loginfo("Goal Publisher Simulator node terminated.")

if __name__ == '__main__':
    main()
