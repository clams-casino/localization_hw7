#!/usr/bin/env python3
import numpy as np
import os

import rospy
import sys
from duckietown.dtros import DTROS, NodeType

import tf
import tf2_ros
import geometry_msgs.msg


class EncoderATFuser():
    
    def __init__(self, buffer_duration):

        pass

    def addEncoderTF(self, tf):

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
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(0.2))   # hold the last t seconds of transforms
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_br = tf2_ros.TransformBroadcaster()

        # flag to track if apriltag service has been called yet
        self.init_at_localization = False


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

                print('Got map_atbase')
                print(map_atbase)

                if self.init_at_localization is False:
                    self.init_at_localization = True

                    rospy.loginfo("Got first transform of \'at_baselink\'\nCalling encoder baselink frame initi service")

                    # TODO call service to set initial frame for encoder localization

                # TODO what to do with this map_atbase at a certain time

                # TODO Need to check if this is a new transform or we just got the last one


            map_encoderbase = self.getLatestTF('map', 'encoder_baselink')
            if map_encoderbase is not None:
                if self.init_at_localization:   # know that the encoder tf has been initialized and we can use it

                    # TODO what to do with this info
                    pass

                else: # Don't do anything if the encoder tf hasn't been initialized
                    pass

            rate.sleep()


if __name__ == '__main__':
    # Initialize the node
    node = FusedLocalizationNode(node_name='fused_localization_node')
    rospy.loginfo("fused_localization_node is up and running...")
    node.run()
    # Keep it spinning to keep the node alive
    rospy.spin()
