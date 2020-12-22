#!/usr/bin/env python
from __future__ import division
import numpy as np
from sensor_msgs.msg import CameraInfo


class Camera():
    def __init__(self, cam_info=None):
        self.h = cam_info.height
        self.w = cam_info.width
        self.fovx_deg = 45
        self.fovy_deg = 27
        self.fovx = np.deg2rad(self.fovx_deg)
        self.fovy = np.deg2rad(self.fovy_deg)


    # TODO DELETE
    @property
    def pixel_angles(self):
        """Compute the viewing angles of each pixel

        Returns:
            np.ndarray: the viewing angles for each pixel in the camera
        """
        fov_diagonal = np.sqrt(self.fovx ** 2 + self.fovy ** 2)
        vertical_views = np.arange(self.h, dtype=float) - self.h / 2.0
        horizontal_views = np.arange(self.w, dtype=float) - self.w / 2.0

        # Make a grid of viewing directions
        grid_x, grid_y = np.meshgrid(horizontal_views, vertical_views)
        # Distance from the centre for each pixel
        dist = np.sqrt(grid_x ** 2 + grid_y ** 2)
        # A constant to get the angle for each pixel
        c = (np.sqrt(vertical_views.size ** 2 + 
                     horizontal_views.size ** 2) * 
             fov_diagonal)
        
        return dist * c

    def pixel_trig(self):
        """Return the sine and cosine of the angle for each pixel
        in the camera.

        Returns:
            (np.ndarray, np.ndarray): (array of sines, array of cosines)
        """
        return (np.sin(self.pixel_angles), np.cos(self.pixel_angles))