#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import rospy

import tf2_ros
import tf

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

from pose_corrector.utils import *
import pose_corrector.utils_o3d as utils_o3d


# todo add rosparam
CAMERA_INTRINSIC_YAML_PATH = "/home/aleks/pose_corrector_data/dataset/inboard_rgbd/camera_params.yaml"
PATH_PCL = "/home/aleks/pose_corrector_data/dataset/inboard_3d/inboard.ply"
PATH_MESH = "/home/aleks/pose_corrector_data/dataset/inboard_3d/japanese_part.stl"
PATH_MESH_TEXTURE = "/home/aleks/pose_corrector_data/dataset/inboard_3d/inboard.png"

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

        self.inboard_model_pcl = utils_o3d.load_inboard_model_pcl(PATH_MESH, PATH_MESH_TEXTURE)
        self.camera_intrinsics = utils_o3d.read_camera_intrinsic(CAMERA_INTRINSIC_YAML_PATH)

        self.voxel_size = 0.005
        self.inboard_model_pcl = self.inboard_model_pcl.voxel_down_sample(self.voxel_size)

    
    def image_callback(self, depth_image_msg, color_image_msg):
        color_image_rgb = self.bridge.imgmsg_to_cv2(color_image_msg)
        depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg)

        mask, masked_color_image, masked_depth_image = mask_images_based_on_distance(color_image_rgb, depth_image, mask_end_effector=True)

        if detect_hands_by_skin_color(masked_color_image):
            # images with hands in them are not evaluated
            pose_msg = self.create_not_valid_PoseStamped(frame_id='camera_color_optical_frame')
            self.pose_pub.publish(pose_msg)
            return

        angle, _, _ = get_inboard_rotation_around_vertical_axis(mask)

        if not inboard_angle_valid(angle, threshold_good_bad_orientation=30):
            # images with inboard rotations outside of the valid range are not evaluated
            pose_msg = self.create_not_valid_PoseStamped(frame_id='camera_color_optical_frame')
            self.pose_pub.publish(pose_msg)
            return
        else:
         # perform ICP to get 6D pose
            masked_color_image = o3d.geometry.Image(masked_color_image)
            masked_depth_image = o3d.geometry.Image((masked_depth_image).astype(np.uint16))

            camera_pcl = utils_o3d.create_camera_pcl(masked_color_image, masked_depth_image, self.camera_intrinsics)
            camera_pcl = camera_pcl.voxel_down_sample(self.voxel_size)

            initial_transformation = utils_o3d.get_initial_inboard_transformation(angle)
            transformation_matrix = utils_o3d.pcl_registration_icp(self.inboard_model_pcl, camera_pcl, initial_transformation, threshold=0.01)

            translation = tf.transformations.translation_from_matrix(transformation_matrix)
            quaternion = tf.transformations.quaternion_from_matrix(transformation_matrix)

            # Publish the pose
            pose_msg = self.create_PoseStamped(translation, quaternion, frame_id='camera_color_optical_frame')

        self.pose_pub.publish(pose_msg)
    
        rospy.loginfo("Received synchronized color and depth images")

    def create_PoseStamped(self, translation, quaternion, frame_id):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = frame_id  
        pose_msg.pose.position.x = translation[0]
        pose_msg.pose.position.y = translation[1]
        pose_msg.pose.position.z = translation[2]
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        return pose_msg
    
    def create_not_valid_PoseStamped(self, frame_id):
        # this pose is published only for visualization in rviz, so we dont see the previous pose estimates in not valid pose regions
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = frame_id  
        pose_msg.pose.position.x = 99999
        pose_msg.pose.position.y = 99999
        pose_msg.pose.position.z = 99999
        pose_msg.pose.orientation.x = 0
        pose_msg.pose.orientation.y = 0
        pose_msg.pose.orientation.z = 0
        pose_msg.pose.orientation.w = 0
        return pose_msg


if __name__ == '__main__':
    node = PoseCorrectorNode()
    rospy.spin()



 