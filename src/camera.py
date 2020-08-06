#!/usr/bin/env python
from __future__ import division
import numpy as np
from sensor_msgs.msg import CameraInfo


class Camera():
    def __init__(self, cam_info):
        self.h = cam_info.height
        self.w = cam_info.width
        self.fovx = Camera._get_fov(self.w, float(cam_info.K[0]))
        # TODO Confirm it is indeed 4
        self.fovy = Camera._get_fov(self.h, float(cam_info.K[4]))

    @staticmethod
    def _get_fov(d, f):
        return np.arctan(d / (2 * f))
