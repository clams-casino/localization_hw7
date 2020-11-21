#!/usr/bin/env python3
import numpy as np
import os

import rospy
import sys
from duckietown.dtros import DTROS, NodeType

import tf
import tf2_ros
import geometry_msgs.msg

from encoder_localization.srv import InitFrame


def wrap(theta):
    """
        Wrap theta between (-pi,pi]
    """
    if theta > np.pi:
        return theta - 2 * np.pi
    elif theta <= -np.pi:
        return theta + 2 * np.pi
    else:
        return theta


def transformToSE2(transform):
    """
        Converts ROS transform (SE3) to SE2 [x, y, yaw]
    """
    if type(transform) is geometry_msgs.msg.TransformStamped:
        transform = transform.transform

    q = np.array([transform.rotation.x, transform.rotation.y,
                  transform.rotation.z, transform.rotation.w])

    return np.array([transform.translation.x, 
                     transform.translation.y, 
                     tf.transformations.euler_from_quaternion(q)[-1]]
                   )


class TimeBuffer():
    """
        Data structure that holds a sequential time buffer of some type of data

        Ensures that the time difference between the first and last element is less than
        or equal to the buffer duration
    """

    def __init__(self, buffer_duration):
        self.duration = buffer_duration
        self.buffer = []

    def addData(self, data, data_time):
        if self.isempty():
            self.buffer.append([data, data_time])
            return

        if data_time > self.buffer[-1][1]:
            while data_time - self.buffer[0][1] > self.duration:
                # data is only removed when new data comes in
                self.buffer.pop(0)

                if self.isempty():
                    break

            self.buffer.append([data, data_time])
        else:
            pass

    def length(self):
        return len(self.buffer)

    def isempty(self):
        return not bool(self.length())

    def getData(self):
        return [buffer_element[0] for buffer_element in self.buffer]

    def timeDifference(self):
        return self.buffer[-1][1] - self.buffer[0][1]


class EncoderATBuffer(TimeBuffer):
    """
        A class which tries to fuse the encoder and april tag estimates while 
        handling the delay in the april tag measurement

        Holds a buffer of encoder data and uses an april tag estimate update all encoder 
        estimates timestamped later than the than the april tag estimate
    """

    def __init__(self, buffer_duration, parent_frame, child_frame):
        super(EncoderATBuffer, self).__init__(buffer_duration)

        self.parent = parent_frame
        self.child = child_frame

        self.last_at_update_time = rospy.Time(0)
        self.latest_tf_stamped = None


    def addEncoder(self, encoder_x_y_yaw, timestamp):
        """
            Adds an encoder transform to the buffer

            Each buffer element is (
                                    [raw encoder transform, 
                                     relative tranform from the previous pose, 
                                     pose,updated pose], 
                                     timestamp
                                   )
        """
        if self.isempty():
            self.addData([encoder_x_y_yaw, np.zeros(3),
                          encoder_x_y_yaw], timestamp)

        else:
            map_pose_increment = encoder_x_y_yaw - self.buffer[-1][0][0]
            # Assumes small changes in angle between measurements
            map_pose_increment[2] = wrap(map_pose_increment[2])

            # Convert translational part of increment to be expressed in the previous frame
            theta = -self.buffer[-1][0][0][2]
            rel_pose_increment_x = np.cos(
                theta) * map_pose_increment[0] - np.sin(theta) * map_pose_increment[1]
            rel_pose_increment_y = np.sin(
                theta) * map_pose_increment[0] + np.cos(theta) * map_pose_increment[1]

            rel_pose_increment = np.array(
                [rel_pose_increment_x, rel_pose_increment_y, map_pose_increment[2]])

            updated_pose = self.updateWithRelativeIncrement(
                self.buffer[-1][0][2], rel_pose_increment)

            self.addData([encoder_x_y_yaw, rel_pose_increment,
                          updated_pose], timestamp)
            self.updateLatestTF(updated_pose, timestamp)


    def updateWithRelativeIncrement(self, map_pose, increment):
        """
            Update a pose in the map frame using an incremental/relative transform between poses
        """
        next_pose_x = map_pose[0] + np.cos(map_pose[2]) * \
            increment[0] - np.sin(map_pose[2]) * increment[1]
        next_pose_y = map_pose[1] + np.sin(map_pose[2]) * \
            increment[0] + np.cos(map_pose[2]) * increment[1]
        next_pose_yaw = wrap(map_pose[2] + increment[2])

        return np.array([next_pose_x, next_pose_y, next_pose_yaw])


    def updateWithAT(self, at_x_y_yaw, at_timestamp):
        """
            Update all encoder estimates later than the april tag estimate
        """
        if at_timestamp > self.last_at_update_time:  # If updates don't arrive in order, then ignore the older ones
            self.last_at_update_time = at_timestamp

            if self.isempty():
                # Unlikely to happen, but handle this just in case
                rospy.logwarn(
                    "Not applying april tag update since no enocoder estimate has been recieved yet")
                return

            if at_timestamp >= self.buffer[-1][1]:
                # If april tag update is later than all encoder data
                rospy.logwarn(
                    "April tag timestamp is later than other timestamps")
                self.buffer[-1][0][2] = at_x_y_yaw
                # self.updateLatestTF(at_x_y_yaw, at_timestamp)

            else:
                for i in range(self.length()):
                    if at_timestamp < self.buffer[i][1]:
                        self.buffer[i][0][2] = at_x_y_yaw

                        for k in range(i+1, self.length()):
                            self.buffer[k][0][2] = self.updateWithRelativeIncrement(
                                self.buffer[k-1][0][2], self.buffer[k][0][1])
                        break

                self.updateLatestTF(self.buffer[-1][0][2], self.buffer[-1][1])


    def updateLatestTF(self, x_y_yaw, timestamp):
        tf_stamped = geometry_msgs.msg.TransformStamped()

        tf_stamped.header.stamp = rospy.Time.now()
        tf_stamped.header.frame_id = self.parent
        tf_stamped.child_frame_id = self.child

        tf_stamped.transform.translation.x = x_y_yaw[0]
        tf_stamped.transform.translation.y = x_y_yaw[1]
        tf_stamped.transform.translation.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, x_y_yaw[2])
        tf_stamped.transform.rotation.x = q[0]
        tf_stamped.transform.rotation.y = q[1]
        tf_stamped.transform.rotation.z = q[2]
        tf_stamped.transform.rotation.w = q[3]

        self.latest_tf_stamped = tf_stamped


    def getLatestTransformStamped(self):
        return self.latest_tf_stamped



class FusedLocalizationNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(FusedLocalizationNode, self).__init__(
            node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh = rospy.get_namespace().strip("/")

        # tf listeners and broadcasters
        # hold the last t seconds of transforms
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0), debug=False)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_br = tf2_ros.TransformBroadcaster()

        # timed TF publication
        self.tf_timer = rospy.Timer(
            rospy.Duration(1.0/30.0), self.tfTimerCallback)

        # flag to check if the initial frame for encoder_baselink has been set
        self.encoder_base_initialized = False

        # buffer of encoder transform data used for fusing the estimates
        self.fusion_buffer = EncoderATBuffer(rospy.Duration(
            10.0), parent_frame='map', child_frame='fused_baselink')


    def getLatestTF(self, source, target):
        """
            Tries to get the latest transform between from the target to the source frame
        """
        try:
            tf = self.tf_buffer.lookup_transform(source, target, rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            tf = None

        return tf


    def tfTimerCallback(self, timer):
        """
            Broadcasts a transform of the fused estimate at a set rate
        """
        latest_tf = self.fusion_buffer.getLatestTransformStamped()
        if latest_tf is not None:
            self.tf_br.sendTransform(latest_tf)


    def callInitFrameService(self, x, y, theta):
        """
            Calls the service in encoder_localization to set the initial frame for encoder_baselink
        """
        rospy.wait_for_service('encoder_localization_node/init_frame')

        try:
            init_frame = rospy.ServiceProxy(
                'encoder_localization_node/init_frame', InitFrame)

            response = init_frame(x, y, theta)

            self.encoder_base_initialized = True
            rospy.loginfo(
                "Service call to InitFrame was successful!\n Response: {}".format(response))

        except rospy.ServiceException as e:
            rospy.logwarn("Service call to InitFrame failed: {}".format(e))


    def initEncoderBaseFrame(self, timeout=2.0):
        """
            Runs the procedure to set the initial frame for encoder_baselink
        """
        rospy.loginfo(
            'Running procedure for encoder baselink frame initialization')
        buffer = TimeBuffer(rospy.Duration(1.5 * timeout))
        rate = rospy.Rate(20.0)

        while not rospy.is_shutdown() and not self.encoder_base_initialized:
            latest_map_atbase = self.getLatestTF('map', 'at_baselink')

            if latest_map_atbase is not None:
                at_xyz = np.array([latest_map_atbase.transform.translation.x,
                                   latest_map_atbase.transform.translation.y,
                                   latest_map_atbase.transform.translation.z])
                at_stamp = latest_map_atbase.header.stamp

                buffer.addData(at_xyz, at_stamp)

                if buffer.timeDifference() > rospy.Duration(timeout / 2.0):
                    std = np.std([np.linalg.norm(v) for v in buffer.getData()])
                    if std < 0.01:
                        rospy.loginfo(
                            'Try calling encoder baselink frame init service')
                        at_x_y_yaw = transformToSE2(latest_map_atbase)
                        self.callInitFrameService(
                            at_x_y_yaw[0], at_x_y_yaw[1], at_x_y_yaw[2])

                elif buffer.timeDifference() > rospy.Duration(timeout):
                    rospy.loginfo(
                        '''April tag measurement has not stabilized after {} seconds.\n
                           Try calling encoder baselink frame init service anyways'''.format(timeout))
                    at_x_y_yaw = transformToSE2(latest_map_atbase)
                    self.callInitFrameService(
                        at_x_y_yaw[0], at_x_y_yaw[1], at_x_y_yaw[2])

            rate.sleep()


    def run(self):
        """
            Runs the fused localization using the latest encoder and april tag transforms
        """
        rate = rospy.Rate(30.0)

        prev_encoder_tf_time = rospy.Time(0)
        prev_at_tf_time = rospy.Time(0)

        while not rospy.is_shutdown():

            map_encoderbase = self.getLatestTF('map', 'encoder_baselink')
            if map_encoderbase is not None:

                encoder_tf_time = map_encoderbase.header.stamp
                if encoder_tf_time > prev_encoder_tf_time:
                    prev_encoder_tf_time = encoder_tf_time

                    encoder_x_y_yaw = transformToSE2(map_encoderbase.transform)
                    self.fusion_buffer.addEncoder(
                        encoder_x_y_yaw, encoder_tf_time)

            map_atbase = self.getLatestTF('map', 'at_baselink')
            if map_atbase is not None:

                at_tf_time = map_atbase.header.stamp
                if at_tf_time > prev_at_tf_time:
                    prev_at_tf_time = at_tf_time

                    at_x_y_yaw = transformToSE2(map_atbase.transform)
                    self.fusion_buffer.updateWithAT(
                        at_x_y_yaw, map_atbase.header.stamp)

            rate.sleep()


if __name__ == '__main__':
    # Initialize the node
    node = FusedLocalizationNode(node_name='fused_localization_node')
    rospy.loginfo("fused_localization_node is up and running...")
    node.initEncoderBaseFrame()
    node.run()
    # Keep it spinning to keep the node alive
    rospy.spin()
