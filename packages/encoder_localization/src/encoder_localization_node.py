#!/usr/bin/env python3
import numpy as np
import os
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import WheelEncoderStamped

import tf

N_REV = 135


class EncoderLocalizationNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(EncoderLocalizationNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._baseline = rospy.get_param(f'/{self.veh_name}/kinematics_node/baseline')
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius')
        self._C = 2.0*np.pi*self._radius / N_REV     # Precompute this multiplier


        # Subscribers
        self.sub_encoder_ticks_left = rospy.Subscriber('left_wheel_encoder_node/tick', 
                                                        WheelEncoderStamped,
                                                        lambda msg: self.cb_encoder_data('left', msg))

        self.sub_encoder_ticks_right = rospy.Subscriber('right_wheel_encoder_node/tick', 
                                                        WheelEncoderStamped, 
                                                        lambda msg: self.cb_encoder_data('right', msg))

        

        # Timed TF publication
        self.tf_timer = rospy.Timer(rospy.Duration(30.0), self.tf_timer_cb)

        # TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

        

        # TODO Initialize a starting tf from encoder_baselink -> map frame
        self.map_t_initbase = (0.3, 0.0) #(x_init, y_init)
        self.map_theta_initbase = np.pi
        # TODO allow these to be changed via a service


        # Integrated states, starting from the initial tf that's offset from map (initbase frame)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0



        # TODO keep track of timestamp of last recieved encoder tick
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


    @staticmethod
    def wrap(theta):
        if theta > np.pi:
            return theta - 2 * np.pi
        elif theta <= np.pi:
            return theta + 2 * np.pi
        else:
            return theta

    def tf_timer_cb(self):
        dl = self.left_distance - self.left_distance_last
        dr = self.right_distance - self.right_distance_last

        self.left_distance_last = self.left_distance
        self.right_distance_last = self.right_distance


        d = (dr + dl) / 2.0
        dtheta = (dr - dl) / (2.0 * self._baseline)

        self.x += d * np.cos(self.theta)
        self.y += d * np.sin(self.theta)
        self.theta  = self.wrap(self.theta + dtheta)


        # Get base pose in map frame
        map_x_base = np.cos(self.map_theta_initbase) * self.x + np.sin(-self.map_theta_initbase) * self.y + self.map_t_initbase[0]
        map_y_base = np.sin(self.map_theta_initbase) * self.x + np.cos(self.map_theta_initbase) * self.y + self.map_t_initbase[1]
        map_theta_base = self.wrap(self.theta + self.map_theta_initbase)

        self.tf_broadcaster.sendTransform((map_x_base, map_y_base, 0),
                                          tf.transformations.quaternion_from_euler(0, 0, map_theta_base),
                                          self.last_tick_time,
                                          'encoder_baselink',
                                          'map')
        

    def cb_encoder_data(self, wheel, msg):
        if msg.header.stamp > self.last_tick_time:
            self.last_tick_time = msg.header.stamp

        if wheel == 'left':
            if self.left_tick_init == None:
                self.left_tick_init = msg.data
            else:
                self.left_distance = (msg.data - self.left_tick_init) * self._C

        elif wheel == 'right':
            if self.right_tick_init == None:
                self.right_tick_init = msg.data
            else:
                self.right_distance = (msg.data - self.right_tick_init) * self._C

        else:
            rospy.logwarn("Invalid wheel. Either left or right")



if __name__ == '__main__':
    node = EncoderLocalizationNode(node_name='encoder_localization_node')
    rospy.loginfo("encoder_localization_node is up and running...")
    # Keep it spinning to keep the node alive
    rospy.spin()
    