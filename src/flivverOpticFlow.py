#!/usr/bin/env python2
from __future__ import division
import numpy as np
from sensor_msgs.msg import CameraInfo
from opticFlow import OpticFlow
from camera import Camera
from warnings import warn
import rospy
import cv2
import numpy as np
import os


class FlivverOpticFlow(OpticFlow):
    """ Class to generate optic flow
    """
    def __init__(self, camera_instance):
        """Initialise the optic flow class

        Args:
            camera_instance (Camera): the camera object
        """
        super(FlivverOpticFlow, self).__init__(camera_instance=camera_instance)

        self.cam_sin, self.cam_cos = self.cam.pixel_trig()

    def flow_magnitude(self):
        magnitude = cv2.cartToPolar(self.flow[:, :, 0], self.flow[:, :, 1])[0]
        return magnitude

    def range(self, vel):
        # Calculate velocity assuming horizontal flight

        if self.initialised:
            v = np.sqrt(vel[0] ** 2 + vel[1] ** 2)
            return v / self.flow_magnitude() * self.cam_sin * self.cam_cos

