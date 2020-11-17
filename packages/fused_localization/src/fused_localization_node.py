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


def transformToSE2(transform):
    if type(transform) is geometry_msgs.msg.TransformStamped:
        transform = transform.transform

    q = np.array([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])

    return (transform.translation.x, transform.translation.y, tf.transformations.euler_from_quaternion(q)[-1])



class TimeBuffer():

    def __init__(self, buffer_duration):
        self.duration = buffer_duration

        self.buffer = []


    def addData(self, data, data_time):
        if self.isempty():
            self.buffer.append( (data, data_time) )
            return

        if data_time > self.buffer[-1][1]:

            while data_time - self.buffer[0][1] > self.duration:
                self.buffer.pop(0)

                if self.isempty():
                    break

            self.buffer.append( (data, data_time) )

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
    
    def __init__(self, buffer_duration):

        # TODO init super with buffer duration

        pass

    def addEncoderTF(self, tf):
        '''
            First checks if the encoder tf is actually new against the last tf in the buffer
            Then gets the relative between the new tf and the old tf and adds it to the buffer

            Finally pop the oldest element in the buffer if the time difference between it and the
            latest added is greater than buffer duration
        '''
        pass

    def atUpdate(self, tf):

        pass

    def publishLatestTF(self):
        

        pass

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

        # flag to track if apriltag service has been called yet
        self.encoder_base_initialized = False



    def callInitFrameService(self, x, y, theta):  # TODO theta in radians

        # NOTE: you don't have to call rospy.init_node() to make calls against
        # a service. This is because service clients do not have to be
        # nodes.

        # block until the add_two_ints service is available
        # you can optionally specify a timeout
        rospy.wait_for_service('encoder_localization_node/init_frame')
        
        try:
            # create a handle to the add_two_ints service
            init_frame = rospy.ServiceProxy('encoder_localization_node/init_frame', InitFrame)
            
            # formal style
            response = init_frame(x, y, theta)

            self.encoder_base_initialized = True
            rospy.loginfo("Service call to InitFrame was successful!\n Response: {}".format(response))

        except rospy.ServiceException as e:
            rospy.logwarn("Service call to InitFrame failed: {}".format(e))


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
            rospy.loginfo("Could not get tf from {} to {}".format(source, target))
            tf = None
        
        return tf

    def run(self):
        rate = rospy.Rate(60.0)        # Faster rate than the fastest tf broadcaster
        while not rospy.is_shutdown():
            
            map_atbase = self.getLatestTF('map', 'at_baselink')
            if map_atbase is not None:

                at_x_y_yaw = transformToSE2(map_atbase.transform)

                # TODO what to do with this map_atbase at a certain time

                # TODO Need to check if this is a new transform or we just got the last one


            map_encoderbase = self.getLatestTF('map', 'encoder_baselink')
            if map_encoderbase is not None:
                pass
                # TODO what to do with this info


            rate.sleep()


if __name__ == '__main__':
    # Initialize the node
    node = FusedLocalizationNode(node_name='fused_localization_node')
    rospy.loginfo("fused_localization_node is up and running...")
    node.initEncoderBaseFrame()
    node.run()
    # Keep it spinning to keep the node alive
    rospy.spin()
