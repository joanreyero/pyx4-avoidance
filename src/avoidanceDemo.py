#!/usr/bin/env python2
from __future__ import division
from pyx4_base.pyx4_base import *
from pyx4_base.mission_states import *
from pyx4_base.teleoperation import *
from geometry_msgs.msg import Twist
from pyx4.msg import pyx4_state
from pyx4_avoidance.msg import activation as ActivationMsg
import numpy as np

NODE_NAME = 'avoidance-demo'


class AvoidanceDemo(Pyx4_base):

    def __init__(self, args):
        instructions = AvoidanceDemo.get_flight_instructions(args)
        super(AvoidanceDemo, self).__init__(instructions)

        self.pyx4_state_subs = rospy.Subscriber('/pyx4_node/pyx4_state', 
                                                pyx4_state, self.state_cb)
        self.activation_subs = rospy.Subscriber(
            '/pyx4_avoidance_node/activation', 
            ActivationMsg, self.activation_cb
        )
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()

        self.stop_msg = Twist()
        self.stop_msg.linear.y = 0
        self.stop_msg.linear.x = 0

        
    @staticmethod
    def get_flight_instructions(args):
        return generate_telop_mission(args)

    def state_cb(self, data):
        rospy.loginfo(data)
        if data.flight_state == 'Teleoperation':
            self.vel_msg.linear.y = 3.0
            self.vel_publisher.publish(self.vel_msg)

    def activation_cb(self, data):
        if data.activation > 1.4:
            self.vel_publisher.publish(self.stop_msg)
    

if __name__ == '__main__':
    import argparse
    
    rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.DEBUG)

    parser = argparse.ArgumentParser(description="Teleoperation px4 quadcopter.")
    parser.add_argument('-t', '--timeout', type=float, default=30)
    parser.add_argument('-l', '--linear', type=float, default=4)
    parser.add_argument('-a', '--angular', type=float, default=2)
    parser.add_argument('-m', '--z_min', type=float, default=0)
    parser.add_argument('-M', '--z_max', type=float, default=10)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    
    m = AvoidanceDemo(args)
    m.run()
