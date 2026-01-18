#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('signal_publisher')
    
    # 创建发布者
    pub = rospy.Publisher('/signal', String, queue_size=1)
    
    # 等待一下确保连接建立
    rospy.sleep(0.5)
    
    # 发送消息
    msg = String()
    msg.data = "GOAL_ARRIVED"
    pub.publish(msg)
    
    rospy.loginfo("Published 'RELOCATION_RECEIVED' to /signal")
