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
        Wrap between (-pi,pi]
    """
    if theta > np.pi:
        return theta - 2 * np.pi
    elif theta <= -np.pi:
        return theta + 2 * np.pi
    else:
        return theta


def transformToSE2(transform):
    if type(transform) is geometry_msgs.msg.TransformStamped:
        transform = transform.transform

    q = np.array([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])

    return np.array([transform.translation.x, transform.translation.y, tf.transformations.euler_from_quaternion(q)[-1]])



class TimeBuffer():

    def __init__(self, buffer_duration):
        self.duration = buffer_duration

        self.buffer = []

    def addData(self, data, data_time):
        if self.isempty():
            self.buffer.append( [data, data_time] )
            return

        if data_time > self.buffer[-1][1]:

            while data_time - self.buffer[0][1] > self.duration:
                self.buffer.pop(0)   # data is only removed when new data comes in

                if self.isempty():
                    break

            self.buffer.append( [data, data_time] )

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
    
    def __init__(self, buffer_duration, parent_frame, child_frame):
        super(EncoderATBuffer, self).__init__(buffer_duration)

        self.parent = parent_frame
        self.child = child_frame

        self.last_at_update_time = rospy.Time(0)

        self.latest_tf_stamped = None

    def addEncoder(self, encoder_x_y_yaw, timestamp):
        '''
            First checks if the encoder tf is actually new against the last tf in the buffer
            Then gets the relative between the new tf and the old tf and adds it to the buffer

            Finally pop the oldest element in the buffer if the time difference between it and the
            latest added is greater than buffer duration

            poses are stored as a tuple of (x, y, yaw)

            each buffer element is (raw encoder data, relative pose, updated poses, timestamp)
        '''
        if self.isempty():
            self.addData([encoder_x_y_yaw, np.zeros(3), encoder_x_y_yaw], timestamp)

        else:
            # Keep absolute pose
            # calculate relative pose change -> Don't forget to wrap yaw
            # get updated pose at relative pose + last updated pose  -> Don't forget to wrap yaw

            map_pose_increment = encoder_x_y_yaw - self.buffer[-1][0][0]
            map_pose_increment[2] = wrap(map_pose_increment[2])  # Assumes small changes in angle between measurements

            # Convert translational part of increment to be expressed in the previous frame
            theta = -self.buffer[-1][0][0][2]
            rel_pose_increment_x = np.cos(theta) * map_pose_increment[0] - np.sin(theta) * map_pose_increment[1]
            rel_pose_increment_y = np.sin(theta) * map_pose_increment[0] + np.cos(theta) * map_pose_increment[1]

            rel_pose_increment = np.array([rel_pose_increment_x, rel_pose_increment_y, map_pose_increment[2]])

            updated_pose = self.updateWithRelativeIncrement(self.buffer[-1][0][2], rel_pose_increment)

            self.addData([encoder_x_y_yaw, rel_pose_increment, updated_pose], timestamp)

            self.updateLatestTF(updated_pose, timestamp)


    def updateWithRelativeIncrement(self, map_pose, increment):
        next_pose_x = map_pose[0] + np.cos(map_pose[2]) * increment[0] - np.sin(map_pose[2]) * increment[1]
        next_pose_y = map_pose[1] + np.sin(map_pose[2]) * increment[0] + np.cos(map_pose[2]) * increment[1]
        next_pose_yaw = wrap(map_pose[2] + increment[2])

        return np.array([next_pose_x, next_pose_y, next_pose_yaw])

    def updateWithAT(self, at_x_y_yaw, at_timestamp):
        if at_timestamp > self.last_at_update_time: # If updates don't arrive in order, then ignore the older ones
            self.last_at_update_time = at_timestamp

            if self.isempty():
                # Unlikely to happen, but handle this just in case
                rospy.logwarn("Not applying april tag update since no enocoder estimate has been recieved yet")
                return

            if at_timestamp >= self.buffer[-1][1]:
                # If april tag update is later than all encoder data
                rospy.logwarn("April tag timestamp is later than other timestamps")
                self.buffer[-1][0][2] = at_x_y_yaw
                # self.updateLatestTF(at_x_y_yaw, at_timestamp)

            else:
                for i in range(self.length()):
                    if at_timestamp < self.buffer[i][1]:
       
                        self.buffer[i][0][2] = at_x_y_yaw

                        print('Update with april tag, length {}    i {}'.format(self.length(), i))
                        print('encoder stamp')
                        print(self.buffer[i][1].to_sec())
                        print('at stamp')
                        print(at_timestamp.to_sec())


                        for k in range(i+1, self.length()):
                            # self.buffer[k][0][2] = self.buffer[k][0][1] + self.buffer[k-1][0][2]
                            self.buffer[k][0][2] = self.updateWithRelativeIncrement(self.buffer[k-1][0][2], self.buffer[k][0][1])

                        break

                self.updateLatestTF(self.buffer[-1][0][2], self.buffer[-1][1])

        # Need to keep track of last time AT was updated to tell if it's been a while since we saw it or not

        # TODO what to do if the buffer with encoder data is empty?

        # TODO what to do if the at measurement is actually newer than the latest encoder data in the buffer?

        # TODO how to update the affected encoder data in the buffer

        pass


    def updateLatestTF(self, x_y_yaw, timestamp):
        tf_stamped = geometry_msgs.msg.TransformStamped()
        
        tf_stamped.header.stamp = rospy.Time.now()  # TODO make this the current time as per the exercise description
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
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0))   # hold the last t seconds of transforms
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_br = tf2_ros.TransformBroadcaster()


        # Timed TF publication
        self.tf_timer = rospy.Timer(rospy.Duration(1.0/30.0), self.tfTimerCallback)


        # flag to track if apriltag service has been called yet
        self.encoder_base_initialized = False

        # Buffer of tfs used for the fusion
        self.fusion_buffer = EncoderATBuffer(rospy.Duration(10.0), parent_frame='map', child_frame='fused_baselink')


    def callInitFrameService(self, x, y, theta):  # TODO theta in radians
        rospy.wait_for_service('encoder_localization_node/init_frame')
        
        try:
            init_frame = rospy.ServiceProxy('encoder_localization_node/init_frame', InitFrame)
            
            response = init_frame(x, y, theta)

            self.encoder_base_initialized = True
            rospy.loginfo("Service call to InitFrame was successful!\n Response: {}".format(response))

        except rospy.ServiceException as e:
            rospy.logwarn("Service call to InitFrame failed: {}".format(e))


    def tfTimerCallback(self, timer):
        latest_tf = self.fusion_buffer.getLatestTransformStamped()
        if latest_tf is not None:
            self.tf_br.sendTransform(latest_tf)


    def initEncoderBaseFrame(self, timeout=2.0):
        rospy.loginfo('Running procedure for encoder baselink frame initialization')
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
                    std = np.std( [np.linalg.norm(v) for v in buffer.getData()] )
                    if std < 0.016:
                        rospy.loginfo('Try calling encoder baselink frame init service')
                        at_x_y_yaw = transformToSE2(latest_map_atbase)
                        self.callInitFrameService(at_x_y_yaw[0], at_x_y_yaw[1], at_x_y_yaw[2])

                elif buffer.timeDifference() > rospy.Duration(timeout):
                    rospy.loginfo('April tag measurement has not stabilized after {} seconds.\n Try calling encoder baselink frame init service anyways'.format(timeout))
                    at_x_y_yaw = transformToSE2(latest_map_atbase)
                    self.callInitFrameService(at_x_y_yaw[0], at_x_y_yaw[1], at_x_y_yaw[2])

            rate.sleep()



    def getLatestTF(self, source, target):
        try:
            tf = self.tf_buffer.lookup_transform(source, target, rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # rospy.loginfo("Could not get tf from {} to {}".format(source, target))
            tf = None
        
        return tf



    def run(self):
        rate = rospy.Rate(40.0)        # Faster rate than the fastest tf broadcaster
        while not rospy.is_shutdown():
            
            map_encoderbase = self.getLatestTF('map', 'encoder_baselink')
            if map_encoderbase is not None:
                
                encoder_x_y_yaw = transformToSE2(map_encoderbase.transform)
                self.fusion_buffer.addEncoder(encoder_x_y_yaw, map_encoderbase.header.stamp)



            map_atbase = self.getLatestTF('map', 'at_baselink')
            if map_atbase is not None:

                at_x_y_yaw = transformToSE2(map_atbase.transform)
                self.fusion_buffer.updateWithAT(at_x_y_yaw, map_atbase.header.stamp)

                # TODO what to do with this map_atbase at a certain time

                # TODO Need to check if this is a new transform or we just got the last one


            rate.sleep()


if __name__ == '__main__':
    # Initialize the node
    node = FusedLocalizationNode(node_name='fused_localization_node')
    rospy.loginfo("fused_localization_node is up and running...")
    node.initEncoderBaseFrame()
    node.run()
    # Keep it spinning to keep the node alive
    rospy.spin()
