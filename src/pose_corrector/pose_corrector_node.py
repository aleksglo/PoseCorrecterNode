#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import rospy
import sys

import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_inverse, quaternion_multiply, quaternion_about_axis

from std_msgs.msg import Int16, String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped

from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

from pose_corrector.utils import *



class PoseCorrectorNode:
    def __init__(self):
        rospy.init_node('pose_corrector', anonymous=True)
           
        self.color_sub = Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        self.depth_sub = Subscriber('/camera/color/image_raw', Image)

        self.sync = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.image_callback)

        self.pose_pub = rospy.Publisher('inboard_pose', PoseStamped, queue_size=10)

        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def callback(self, data):
        rospy.loginfo("I heard: %s", data.data)
    
    def image_callback(self, depth_image_msg, color_image_msg):
        color_image_rgb = self.bridge.imgmsg_to_cv2(color_image_msg)
        depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg)

        mask, masked_color_image, masked_depth_image = mask_images_based_on_distance(color_image_rgb, depth_image, mask_end_effector=True)

        if detect_hands_by_skin_color(masked_color_image):

            # We dont evaluate the images with hands in the image
            return

        angle, _, _ = get_inboard_rotation_around_vertical_axis(mask)

        if not inboard_angle_valid(angle, threshold_good_bad_orientation=30):
            #print('not valid angel')
            return
        else:
         # preform ICP to get 6D pose

            #camera_transform = self.tf_buffer.lookup_transform('world', 'camera_color_optical_frame', rospy.Time())

            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "camera_color_optical_frame"  
            pose_msg.pose.position.x = 0
            pose_msg.pose.position.y = 0
            pose_msg.pose.position.z = 0.3
            pose_msg.pose.orientation.x = 0.0 
            pose_msg.pose.orientation.y = 0.0 
            pose_msg.pose.orientation.z = 0.0 
            pose_msg.pose.orientation.w = 1.0 

            # Publish the pose
            self.pose_pub.publish(pose_msg)
    
        rospy.loginfo("Received synchronized color and depth images")


if __name__ == '__main__':
    node = PoseCorrectorNode()
    rospy.spin()



 