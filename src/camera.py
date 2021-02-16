#!/usr/bin/env python
from __future__ import division
import numpy as np
from sensor_msgs.msg import CameraInfo


class Camera():
    def __init__(self, cam_info=None):
        self.h = cam_info.height
        self.w = cam_info.width

        # self.fovx_deg = 120
        # self.fovy_deg = 60
        self.fovx_deg = 45
        self.fovy_deg = 27
        
        self.fovx = np.deg2rad(self.fovx_deg)
        self.fovy = np.deg2rad(self.fovy_deg)