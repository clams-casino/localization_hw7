#!/usr/bin/env python3
import numpy as np
import os
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import WheelEncoderStamped

import tf

from encoder_localization.srv import InitFrame, InitFrameResponse

N_REV = 135


def wrap(theta):
    if theta > np.pi:
        return theta - 2 * np.pi
    elif theta <= -np.pi:
        return theta + 2 * np.pi
    else:
        return theta


class EncoderLocalizationNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(EncoderLocalizationNode, self).__init__(
            node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._baseline = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/baseline')
        self._radius = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/radius')
        self._C = 2.0*np.pi*self._radius / N_REV     # Precompute this multiplier

        # Subscribers
        self.sub_encoder_ticks_left = rospy.Subscriber('left_wheel_encoder_node/tick',
                                                       WheelEncoderStamped,
                                                       lambda msg: self.callbackEncoder('left', msg))

        self.sub_encoder_ticks_right = rospy.Subscriber('right_wheel_encoder_node/tick',
                                                        WheelEncoderStamped,
                                                        lambda msg: self.callbackEncoder('right', msg))

        # Service
        self.init_frame_srv = rospy.Service(
            '~init_frame', InitFrame, self.handleInitFrame)

        # Timed TF publication
        self.tf_timer = rospy.Timer(
            rospy.Duration(1.0/30.0), self.tfTimerCallback)

        # TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

        # (x_init, y_init, theta_init)
        self.map_initbase_se2 = (0.3, 0.0, np.pi)

        # Integrated states, starting from the initial tf that's offset from map (initbase frame)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Keep track of timestamp of last recieved encoder tick
        self.last_tick_time = rospy.Time(0)

        # Keep track of the distance travelled by each wheel for all time
        self.left_distance = 0.0
        self.right_distance = 0.0

        # Keep track of the distance travelled by each wheel at the time of publishing the tf
        self.left_distance_last = 0.0
        self.right_distance_last = 0.0

        # Keep track of the starting tick value to zero everything out
        self.left_tick_init = None
        self.right_tick_init = None


    def handleInitFrame(self, req):
        self.map_initbase_se2 = (req.x, req.y, req.theta)
        response = 'Set initial frame for encoder baselink to ({:.2f}, {:.2f}, {:.2f})'.format(
            self.map_initbase_se2[0], self.map_initbase_se2[1], np.rad2deg(self.map_initbase_se2[2]))
        return InitFrameResponse(response)


    def tfTimerCallback(self, timer):

        dl = self.left_distance - self.left_distance_last
        dr = self.right_distance - self.right_distance_last

        self.left_distance_last = self.left_distance
        self.right_distance_last = self.right_distance

        d = (dr + dl) / 2.0
        dtheta = (dr - dl) / (2.0 * self._baseline)

        self.x += d * np.cos(self.theta)
        self.y += d * np.sin(self.theta)
        self.theta = wrap(self.theta + dtheta)

        # Get base pose in map frame
        map_x_base = np.cos(self.map_initbase_se2[2]) * self.x - np.sin(
            self.map_initbase_se2[2]) * self.y + self.map_initbase_se2[0]
        map_y_base = np.sin(self.map_initbase_se2[2]) * self.x + np.cos(
            self.map_initbase_se2[2]) * self.y + self.map_initbase_se2[1]
        map_theta_base = wrap(self.theta + self.map_initbase_se2[2])

        self.tf_broadcaster.sendTransform((map_x_base, map_y_base, 0),
                                          tf.transformations.quaternion_from_euler(
                                              0, 0, map_theta_base),
                                          rospy.Time.now(),   # TODO update duckiebot image
                                          'encoder_baselink',
                                          'map')


    def callbackEncoder(self, wheel, msg):
        if msg.header.stamp > self.last_tick_time:
            self.last_tick_time = msg.header.stamp

        if wheel == 'left':
            if self.left_tick_init is None:
                self.left_tick_init = msg.data
            else:
                self.left_distance = (msg.data - self.left_tick_init) * self._C

        elif wheel == 'right':
            if self.right_tick_init is None:
                self.right_tick_init = msg.data
            else:
                self.right_distance = (
                    msg.data - self.right_tick_init) * self._C

        else:
            rospy.logwarn("Invalid wheel. Either left or right")


if __name__ == '__main__':
    node = EncoderLocalizationNode(node_name='encoder_localization_node')
    rospy.loginfo("encoder_localization_node is up and running...")
    # Keep it spinning to keep the node alive
    rospy.spin()
