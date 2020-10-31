#!/usr/bin/env python2
from __future__ import division
import numpy as np
from sensor_msgs.msg import CameraInfo
from camera import Camera
import rospy

NODE_NAME = 'optic_flow'

class OpticFlow():

    def __init__(self, camera_instance):
        self.cam = camera_instance