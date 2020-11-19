#!/usr/bin/env python2
from __future__ import division
from pyx4_base.pyx4_base import *
from pyx4_base.mission_states import *
from pyx4_base.teleoperation import *
import numpy as np

NODE_NAME = 'avoidance-demo'


class AvoidanceDemo(Pyx4_base):

    def __init__(self, xvel):
        instructions = AvoidanceDemo.get_flight_instructions()
        super(AvoidanceDemo, self).__init__(instructions)
        
    @staticmethod
    def get_flight_instructions(xvel):
        return generate_telop_mission()
    

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="For obstacle avoidance data collection")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    
    rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.DEBUG)
    m = AvoidanceDemo(xvel=args.xvel)
    m.run()
