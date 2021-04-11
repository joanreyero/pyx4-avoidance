#!/usr/bin/env python2
from __future__ import division
import numpy as np
from sensor_msgs.msg import CameraInfo
from camera import Camera
from warnings import warn
import rospy
import cv2
import numpy as np
import os
import time


class OpticFlow(object):
    """ Class to generate optic flow
    """
    def __init__(self, camera_instance):
        """Initialise the optic flow class

        Args:
            camera_instance (Camera): the camera object
        """
        self.cam = camera_instance
        self.flow = None
        # we have a 3 dimensional image array for our black and white images
        self._bw_image_array = np.zeros((self.cam.h, self.cam.w, 2), 
                                        dtype=np.uint8)
        self._time_array = np.zeros((2), dtype=np.float32)

        self.initialised = False
        self.__flow_iterations = 0

        self.viewing_directions = None


    @property
    def __initialised(self):
        """
        This property must be called twice before the flow is initialised
        (since we need at least 2 frames to compute optic flow)
        """
        if self.__flow_iterations < 2:
            self.__flow_iterations += 1
            return False
        
        else:
            self.initialised = True
            return True
        
    def step(self, new_image_bw, this_time):
        """Perform a step of optic flow computation

        Args:
            new_image_bw (Image): a black and white image
            this_time (int): timestamp for this image
        """
        # start each step assuming that the optic flow is valid 
        self.optic_flow_valid = True

        # If incoming image is colour then set to grayscale
        if not len(new_image_bw.shape) == 2:
            new_image_bw = cv2.cvtColor(new_image_bw, cv2.COLOR_BGR2GRAY)
            warn('Using colour images, for better performance input grayscale images')

        # roll the image & time buffers and insert the new frame - 
        # note the buffer size is 2 so the t-2th frame is
        # discarded by this process
        self._bw_image_array = np.roll(self._bw_image_array, shift=1, axis=2)
        self._time_array = np.roll(self._time_array, shift=1, axis=0)
        self._bw_image_array[:, :, 0] = new_image_bw
        self._time_array[0] = this_time

        # We need at least 2 frames for OF
        if self.__initialised:
        
            self.flow = cv2.calcOpticalFlowFarneback(
                self._bw_image_array[:, :, 1],
                self._bw_image_array[:, :, 0],
                None,  # in python 3 this isn't required
                pyr_scale=0.5,
                levels=3,
                winsize=50,
                # winsize=10,
                iterations=3,
                poly_n=5,
                poly_sigma=1.1,
                flags=0
            )


            self.time_between_frames_s = (self._time_array[0] - 
                                          self._time_array[1])

            # ensure that time is moving forwards
            if self.time_between_frames_s <= 0.0:
                warn('this is the same image message time stamp')
                print('latest timestamp {} is the same as the previous image message time'.
                      format(self._time_array[0]))

            return self.flow
