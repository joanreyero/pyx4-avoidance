#!/usr/bin/env python2
from __future__ import division
from pyx4_base.pyx4_base import *
from pyx4_base.mission_states import *
from pyx4_base.teleoperation import *
from camera_labels import *
from geometry_msgs.msg import Twist
from pyx4.msg import pyx4_state
from pyx4_avoidance.msg import activation as ActivationMsg
from collections import deque
import numpy as np

NODE_NAME = 'avoidance-demo'


class AvoidanceController(Pyx4_base):

    def __init__(self, vel, args):
        instructions = AvoidanceController.get_flight_instructions(args)
        super(AvoidanceController, self).__init__(instructions)

        self.pyx4_state_subs = rospy.Subscriber('/pyx4_node/pyx4_state', 
                                                pyx4_state, self.state_cb)
        self.activation_subs = rospy.Subscriber(
            '/pyx4_avoidance_node/activation', 
            ActivationMsg, self.cam_0_activation_cb
        )
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()

        self.stop_msg = Twist()
        self.stop_msg.linear.y = 0
        self.stop_msg.linear.x = 0

        self.active = False

        self.activations = {
            C0: deque([], maxlen=3)
        }

        self.valid = {
            C0: deque([], maxlen=3)
        }

        self.vel = vel

        
    @staticmethod
    def get_flight_instructions(args):
        return generate_telop_mission(args)

    def state_cb(self, data):
        rospy.loginfo(data)
        if data.flight_state == 'Teleoperation':
            self.vel_msg.linear.y = self.vel
            self.vel_publisher.publish(self.vel_msg)
            self.active = True

    def is_valid(self, activations):
        if activations[-1] * 0 > 0.4:
            activations = map(lambda x: round(x, 1), activations)
            print(activations)
            return (sorted(activations) == list(activations))
        return False


    def cam_0_activation_cb(self, data):
        self.activation_cb(data, C0)
        
    def activation_cb(self, data, cam):
        if self.active:
            self.stop_decision(data.activation, cam)

    def stop_decision(self, activation, cam):
        rospy.loginfo(activation)
        self.activations[cam].append(activation)
        self.valid[cam].append(self.is_valid(self.activations[cam]))
        if sum(self.valid[cam]) == 3:
            print(self.activations[cam])
            rospy.loginfo('STOP')
            self.vel_publisher.publish(self.stop_msg)
            

if __name__ == '__main__':
    import argparse
    
    rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.DEBUG)
    parser = argparse.ArgumentParser(description="Teleoperation px4 quadcopter.")
    # Stuff that goes in teleop
    parser.add_argument('--timeout', type=float, default=30)
    parser.add_argument('--linear', type=float, default=4)
    parser.add_argument('--angular', type=float, default=2)
    parser.add_argument('--z_min', type=float, default=0)
    parser.add_argument('--z_max', type=float, default=10)
    # To control this
    parser.add_argument('--velocity', '-v', type=float, default=2.0)    
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    m = AvoidanceController(args.velocity, args)
    m.run()
