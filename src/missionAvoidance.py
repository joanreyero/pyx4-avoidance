#!/usr/bin/env python2
from __future__ import division
from pyx4_base.pyx4_base import *
from pyx4_base.mission_states import *
import numpy as np

NODE_NAME = 'avoidance-mission'


class AvoidanceDataMission(Pyx4_base):

    def __init__(self, xvel):
        instructions = AvoidanceDataMission.get_flight_instructions(xvel)
        super(AvoidanceDataMission, self).__init__(instructions)
        
    @staticmethod
    def get_flight_instructions(xvel):
        print(xvel)
        return {0: Arming_state(tiemout=90),
                1: Take_off_state(to_altitude_tgt=3.0),
                2: Waypoint_state(state_label='to_object',
                                  waypoint_type='vel',
                                  xy_type='vel',
                                  x_setpoint=xvel,
                                  y_setpoint=0.0,
                                  z_type='pos',
                                  z_setpoint=3.0,
                                  yaw_type='vel',
                                  yaw_setpoint=0.0,
                                  coordinate_frame='1',
                                  timeout=30)}
    

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="For obstacle avoidance data collection")
    parser.add_argument('--xvel', '-x', type=float, default=1.0)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    
    rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.DEBUG)
    m = AvoidanceDataMission(xvel=args.xvel)
    m.run()
