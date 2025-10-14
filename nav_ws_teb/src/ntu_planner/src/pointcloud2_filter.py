#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def filter_pointcloud(msg):
    # 直接在原始坐标系下过滤
    filtered_points = [
        pt for pt in pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        if not (-0.45 < pt[0] < 0.45 and -0.3 < pt[1] < 0.3)
    ]
    
    # 构建新点云消息
    filtered_msg = pc2.create_cloud_xyz32(msg.header, [pt[:3] for pt in filtered_points])
    pub.publish(filtered_msg)

if __name__ == "__main__":
    rospy.init_node("pointcloud2_filter_node")
    pub = rospy.Publisher("/local_map_filtered", PointCloud2, queue_size=1)
    rospy.Subscriber("/merged_pointcloud_sliced", PointCloud2, filter_pointcloud, queue_size=1)
    rospy.loginfo("PointCloud2 filter node started.")
    rospy.spin()