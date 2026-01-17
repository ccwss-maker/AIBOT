#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Empty
import math

class OdomZero:
    def __init__(self):
        self.pub = rospy.Publisher("/odom_temporary", Odometry, queue_size=50)

        self.has_origin = False
        self.x0 = self.y0 = 0.0
        self.yaw0 = 0.0

        rospy.Subscriber("/odom", Odometry, self.cb, queue_size=50)
        rospy.Subscriber("/reset_odom_temporary", Empty, self.reset_cb, queue_size=10)

    def cb(self, msg: Odometry):
        # 取当前位姿
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        # 第一次：记录原点
        if not self.has_origin:
            self.x0, self.y0, self.yaw0 = x, y, yaw
            self.has_origin = True
            rospy.loginfo("Origin set: x0=%.3f y0=%.3f yaw0=%.3f", self.x0, self.y0, self.yaw0)

        # 平移清零（世界系减原点）
        dx_w = x - self.x0
        dy_w = y - self.y0

        # 旋转清零：把位移旋到“初始朝向坐标系”
        c = math.cos(-self.yaw0)
        s = math.sin(-self.yaw0)
        dx = c * dx_w - s * dy_w
        dy = s * dx_w + c * dy_w

        # 朝向清零：yaw' = yaw - yaw0
        yaw_rel = yaw - self.yaw0

        # 归一化到 [-pi, pi]，避免跳动
        while yaw_rel > math.pi:
            yaw_rel -= 2.0 * math.pi
        while yaw_rel < -math.pi:
            yaw_rel += 2.0 * math.pi

        q_rel = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw_rel)

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = "local_map"      # 也可改成 "odom_temporary"
        out.child_frame_id = msg.child_frame_id

        out.pose = msg.pose
        out.pose.pose.position.x = dx
        out.pose.pose.position.y = dy
        out.pose.pose.position.z = msg.pose.pose.position.z
        out.pose.pose.orientation = Quaternion(*q_rel)

        out.twist = msg.twist  # 速度直接透传（一般没问题）

        self.pub.publish(out)

    def reset_cb(self, msg):
        """Reset the origin to current position"""
        rospy.loginfo("Resetting odometry origin")
        self.has_origin = False

if __name__ == "__main__":
    rospy.init_node("odom_zero_node")
    OdomZero()
    rospy.spin()
