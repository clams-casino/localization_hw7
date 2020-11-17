#!/usr/bin/env python3
import numpy as np
import os
import cv2

import rospy
import yaml
import sys
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

from dt_apriltags import Detector

import tf
import tf2_ros
import geometry_msgs.msg

TAG_SIZE = 0.065


def readYamlFile(fname):
    """
        Reads the 'fname' yaml file and returns a dictionary with its input.

        You will find the calibration files you need in:
        `/data/config/calibrations/`
    """
    with open(fname, 'r') as in_file:
        try:
            yaml_dict = yaml.load(in_file)
            return yaml_dict
        except yaml.YAMLError as exc:
            self.log("YAML syntax error. File: %s fname. Exc: %s"
                     % (fname, exc), type='fatal')
            rospy.signal_shutdown()
            return


def estimateTfFromHomography(H, K):
    T_plane = np.linalg.inv(K) @ H
    # estimate scale using rotation matrix basis constraint
    T_plane = T_plane / np.linalg.norm(T_plane[:, 0])

    if T_plane[1, -1] < 0.0:
        T_plane = -T_plane

    r1 = T_plane[:, 0]
    r2 = T_plane[:, 1]
    t = T_plane[:, 2]

    # Make sure r1 and r2 form an orthogonal basis then generate r3
    r2 = (r2 - np.dot(r2, r1)*r1)
    r2 = r2 / np.linalg.norm(r2)
    r3 = np.cross(r1, r2)

    T = np.zeros((4, 4))
    T[:3, :] = np.column_stack((r1, r2, r3, t))
    T[-1, -1] = 1.0

    return T


def broadcastTF(tf_mat, parent, child, broadcaster, timestamp=None):
    ''' 
    Assumes tf_mat give the transformation from child frame to parent frame
    '''
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = timestamp if timestamp is not None else rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id = child

    t.transform.translation.x = tf_mat[0, -1]
    t.transform.translation.y = tf_mat[1, -1]
    t.transform.translation.z = tf_mat[2, -1]
    q = tf.transformations.quaternion_from_matrix(tf_mat)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    broadcaster.sendTransform(t)


class ATLocalizationNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(ATLocalizationNode, self).__init__(
            node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh = rospy.get_namespace().strip("/")

        # bridge between opencv and ros
        self.bridge = CvBridge()

        # construct subscriber for images
        self.camera_sub = rospy.Subscriber(
            'camera_node/image/compressed', CompressedImage, self.callback, queue_size=1)  # Only process latest messages

        # tf broadcasters
        self.static_tf_br = tf2_ros.StaticTransformBroadcaster()
        self.tf_br = tf2_ros.TransformBroadcaster()

        # april-tag detector
        self.at_detector = Detector(searchpath=['apriltags'],
                                    families='tag36h11',
                                    nthreads=4,
                                    quad_decimate=2.0,
                                    quad_sigma=0.8,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
        # Keep track of the ID of the landmark tag
        self.at_id = None



        # check cuda
        print('cuda')
        print(cv2.cuda.getCudaEnabledDeviceCount())



        # get camera calibration parameters (homography, camera matrix, distortion parameters)
        intrinsics_file = '/data/config/calibrations/camera_intrinsic/' + \
            rospy.get_namespace().strip("/") + ".yaml"
        extrinsics_file = '/data/config/calibrations/camera_extrinsic/' + \
            rospy.get_namespace().strip("/") + ".yaml"
        rospy.loginfo('Reading camera intrinsics from {}'.format(
            intrinsics_file))
        rospy.loginfo('Reading camera extrinsics from {}'.format(
            extrinsics_file))
        intrinsics = readYamlFile(intrinsics_file)
        extrinsics = readYamlFile(extrinsics_file)

        self.h = intrinsics['image_height']
        self.w = intrinsics['image_width']
        cam_mat = np.array(
            intrinsics['camera_matrix']['data']).reshape(3, 3)

        distortion_coeff = np.array(
            intrinsics['distortion_coefficients']['data'])
        H_ground2img = np.linalg.inv(
            np.array(extrinsics['homography']).reshape(3, 3))

        # precompute some quantities
        self.camera_params = (
            cam_mat[0, 0], cam_mat[1, 1], cam_mat[0, 2], cam_mat[1, 2])

        new_cam_mat, _ = cv2.getOptimalNewCameraMatrix(
            cam_mat, distortion_coeff, (640, 480), 0.0)
        self.map1, self.map2, = cv2.initUndistortRectifyMap(
            cam_mat, distortion_coeff, np.eye(3), new_cam_mat, (640, 480), cv2.CV_32FC1)


        # define and broadcast static tfs

        self.camloc_camcv = np.array([[0.0,  0.0, 1.0, 0.0],
                                      [-1.0,  0.0, 0.0, 0.0],
                                      [0.0, -1.0, 0.0, 0.0],
                                      [0.0,  0.0, 0.0, 1.0]])

        self.atcv_atloc = np.array([[0.0, 1.0,  0.0, 0.0],
                                    [0.0, 0.0, -1.0, 0.0],
                                    [-1.0, 0.0,  0.0, 0.0],
                                    [0.0, 0.0,  0.0, 1.0]])

        camcv_base_estimated = estimateTfFromHomography(
            H_ground2img, new_cam_mat)

        theta = 15.0 / 180.0 * np.pi
        base_camloc_nominal = np.array([[np.cos(theta), 0.0, np.sin(theta), 0.0582],
                                        [0.0,           1.0, 0.0,           0.0],
                                        [-np.sin(theta), 0.0,
                                         np.cos(theta), 0.1072],
                                        [0.0,           0.0, 0.0,           1.0]])

        self.camloc_base = self.camloc_camcv @ camcv_base_estimated
        # self.camloc_base = np.linalg.inv(base_camloc_nominal)

        self.map_atloc = np.array([[1.0, 0.0, 0.0, 0.0],
                                   [0.0, 1.0, 0.0, 0.0],
                                   [0.0, 0.0, 1.0, 0.085],
                                   [0.0, 0.0, 0.0, 1.0]])

        broadcastTF(self.map_atloc, 'map', 'april_tag', self.static_tf_br)


    def callback(self, data):
        img_gray = cv2.cvtColor(self.readImage(data), cv2.COLOR_BGR2GRAY)

        # TODO undistort image with gpu
        try:
            undistorted_img = cv2.remap(
                img_gray, self.map1, self.map2, cv2.INTER_LINEAR)
        except:
            rospy.logwarn('Was not able to undistort image for april tag localization')
            return

        tags = self.at_detector.detect(
            undistorted_img, estimate_tag_pose=True, camera_params=self.camera_params, tag_size=TAG_SIZE)

        for tag in tags:
            if self.at_id is None:
                self.at_id = tag.tag_id

            if tag.tag_id == self.at_id:  # Assumes all tags have a unique id
                camcv_atcv = np.zeros((4, 4))
                camcv_atcv[:3, :3] = tag.pose_R
                camcv_atcv[:3, -1] = np.squeeze(tag.pose_t)
                camcv_atcv[-1, -1] = 1.0

                camloc_atloc = self.camloc_camcv @ camcv_atcv @ self.atcv_atloc
                atloc_camloc = np.linalg.inv(camloc_atloc)

                map_base = self.map_atloc @ atloc_camloc @ self.camloc_base

                broadcastTF(atloc_camloc, 'april_tag', 'camera',
                            self.tf_br, data.header.stamp)
                broadcastTF(map_base, 'map', 'at_baselink', 
                            self.tf_br, data.header.stamp)

                break

    def readImage(self, msg_image):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg_image)
            return cv_image
        except CvBridgeError as e:
            self.log(e)
            return []


if __name__ == '__main__':
    # Initialize the node
    node = ATLocalizationNode(node_name='at_localization_node')
    rospy.loginfo("at_localization_node is up and running...")
    # Keep it spinning to keep the node alive
    rospy.spin()
