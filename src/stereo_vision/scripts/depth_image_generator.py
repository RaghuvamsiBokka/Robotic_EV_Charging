#!/usr/bin/env python
import rospy
import sensor_msgs.msg as sensor_msgs
import sensor_msgs.point_cloud2 as pc2
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def callback(data):
    # Convert the point cloud data to a depth image
    gen = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
    points = np.array([[x, y, z] for x, y, z in gen], dtype=np.float32)
    depth_image = cv2.normalize(points[:, 2], None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    # Convert the depth image to a ROS Image message
    depth_image_msg = bridge.cv2_to_imgmsg(depth_image, encoding='mono8')

    # Publish the depth image
    pub = rospy.Publisher('depth_image', sensor_msgs.Image, queue_size=1)
    pub.publish(depth_image_msg)

def main():
    rospy.init_node('pointcloud_to_depth_image')
    sub = rospy.Subscriber('/stereo/points2', sensor_msgs.PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
