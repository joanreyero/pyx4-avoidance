#!/usr/bin/env python2
from __future__ import division
import numpy as np
import io

class MatchedFilter():
    """ Class to generate matched filters.
    Currently the camera is in world coordinates:
      - x: direction of viewing
      - y: horizontal
      - z: vertical
    
    :param cam_w (int): camera width in pixels
           default: 640
    :param cam_h (int): camera height in pixels
           default: 320
    :param fov (list): 2 element list with fov x and fov y in degrees
           default: [90, 45]
    :param orientation (list): orientation of the camera (roll, pitch, yaw)
           default: [0.0, 0.0, 0.0]
    :param axis (list):
           default: [0.0, 0.0, 0.0]
    """
    
    def __init__(self, cam_w, cam_h, fov,
                 orientation=[0.0, 0.0, 0.0],
                 axis=[0.0, 0.0, 0.0]):

        # Width and height of the camera
        self.cam_w = cam_w
        self.cam_h = cam_h

        # Compute the fields of view accurately
        self.fovx, self.fovy = MatchedFilter._get_fov(fov)

        orientation = map(float, orientation)
        self.origin_rotation_matrix = self._rotation_matrix(orientation)


        # This will be used when the new version in camera coordinates
        # is implemented
        
        # Transform camera to origin coordinates.
        # self.cam_to_origin = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]])
        # Transform origin to camera coordinates.
        # self.origin_to_cam = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
        # Camera rotation matrix in camera coordinates
        #self.cam_rot_matrix = np.matmul(self.origin_to_cam, self.origin_rotation_matrix)

        # Axis rotated to desired orientation in origin coordinates
        self.axis = np.matmul(self._rotation_matrix(map(float, axis)),
                              np.array([1, 0, 0]))

        self.D = self._get_viewing_directions()
        self.matched_filter = self.generate_filter()


    @staticmethod
    def _get_fov(fov):
        """ Compute the FOV
        :param fov (list): fov x and fov y
        """
        
        fovx, fovy = map(float, fov)
        # if fovx == 365:
        #     fovx == 364
        # if fovy == 180:
        #     fovy = 179
        return np.deg2rad(fovx), np.deg2rad(fovy)
    
    def _get_viewing_directions(self):
        """ Compute the camera viewing directions.
        """
        # Get the vertical and horizontal views for the camera
        vertical_views = (((np.arange(self.cam_h, dtype=float) -
                            self.cam_h / 2.0) / float(self.cam_h)) *
                          self.fovy)
        horizontal_views = (((np.arange(self.cam_w, dtype=float) -
                              self.cam_w / 2.0) / float(self.cam_w)) *
                            self.fovx)

        # Initialise D such that
        # - the first two dimensions are the horizontal and vertical views
        # - the last dimension is a 3D vector pointing towards the camers's view.
        D = np.ones([self.cam_h, self.cam_w, 3])
        D[:, :, 1], D[:, :, 2] = np.meshgrid(np.tan(horizontal_views),
                                             np.tan(vertical_views))
        
        # Rotate to appropiate orientation of the camera
        D = self._rotate_viewing_directions(D)
        return D

    def _rotate_viewing_directions(self, D):
        """ Rotate each vector in the viewing directions
        to the appropiate orientation.
        :param D (numpy array): array of size (cam_w, cam_h, 3)
        """
        for ii in range(self.cam_h):
            for jj in range(self.cam_w):
                # For each vector, transorm with the rotation matrix
                D[ii, jj, :] = np.matmul(self.origin_rotation_matrix, D[ii, jj, :])
                
        return D

    def _rotation_matrix(self, orientation):
        """ Generate the rotation matrix for the appropiate orientation.
        :param orientation (list): roll, pitch, yaw
        """
        roll, pitch, yaw = orientation

        # Get the angles in radians for each direction
        rx = np.deg2rad(roll)
        ry = np.deg2rad(pitch)
        rz = np.deg2rad(yaw)

        # Roll rotation matrix
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(rx), -np.sin(rx)],
                       [0, np.sin(rx), np.cos(rx)]])
        # Pitch rotation matrix
        Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                       [0, 1, 0],
                       [-np.sin(ry), 0, np.cos(ry)]])
        # Yaw rotation matrix
        Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                       [np.sin(rz), np.cos(rz), 0],
                       [0, 0, 1]])

        rot_mat = np.matmul(np.matmul(Rx, Ry), Rz)
        return rot_mat
            
    def generate_filter(self):
        """ Generate the translational matched filter using the formulae in
        "Franz & Krapp - Wide-field, motion-sensitive neurons
        and matched filters for optic  flow fields."
        """
        sin_theta = np.linalg.norm(self.D[:, :, 1:], axis=2) + 1e-14
        sin_theta = np.repeat(sin_theta[:, :, np.newaxis], 2, axis=2)
        mag_temp = np.linalg.norm(self.D, axis=2)
        D = self.D / np.expand_dims(mag_temp, axis=2)
        mf = -np.cross(np.cross(D, self.axis), D)[:, :, 1:] / sin_theta
        return mf

    def plot(self, show=False):
        """
        Plot the matched filters that have been generated.
        :param show (bool): whether to show the plot in the console or not.
        """
        import matplotlib.pyplot as plt
        from matplotlib.figure import Figure

        if not show:
            # Figure that can be sent to front-end
            fig = Figure()
            axis = fig.add_subplot(1, 1, 1)

        
        else:
            # Figure that can be displayed
            fig, axis = plt.subplots()

        Y = ((np.arange(self.cam_h, dtype=float) - self.cam_h / 2.0) / float(
            self.cam_h)) * np.rad2deg(self.fovy)

        X = ((np.arange(self.cam_w, dtype=float) - self.cam_w / 2.0) / float(
            self.cam_w)) * np.rad2deg(self.fovx)

        U = self.matched_filter[:, :, 0]
        V = self.matched_filter[:, :, 1]
        step_size = 5
        scale = None
        axis.set_xlabel('x (degrees)')
        axis.set_ylabel('y (degrees)')
        axis.quiver(X[::step_size], Y[::step_size],
                    U[::step_size, ::step_size],
                    V[::step_size, ::step_size],
                    pivot='mid', scale=scale)

        if show:
            plt.show()
            
        return fig

    def get_unit_directions(self):
        """ Return the viewing directions of the camera and the axis
        for 3D plotting
        """
        
        def get_unit(v, shorter=1):
            """ Make unit vectors (or shorter)."""
            return list(v / (np.linalg.norm(v) * shorter))

        return {
            # This might be useful in future versions, ignore for now
            # 'camx': get_unit(np.matmul(np.linalg.inv(self.cam_to_origin), self.cam_rot_matrix[0,:], shorter=1.2),
            # 'camy': get_unit(np.matmul(np.linalg.inv(self.cam_to_origin), self.cam_rot_matrix[1,:]), shorter=1.2),
            # 'camz': get_unit(np.matmul(np.linalg.inv(self.cam_to_origin), self.cam_rot_matrix[2,:]), shorter=1.2),

            # 'camx': get_unit(np.matmul(self.cam_to_origin, self.cam_rot_matrix[:,0]), shorter=1.2),
            # 'camy': get_unit(np.matmul(self.cam_to_origin, self.cam_rot_matrix[:,1]), shorter=1.2),
            # 'camz': get_unit(np.matmul(self.cam_to_origin, self.cam_rot_matrix[:,2]), shorter=1.2),
            # 'dtl': get_unit(np.matmul(self.cam_to_origin, self.D[0, 0, :]), shorter=1.4),
            # 'dtr': get_unit(np.matmul(self.cam_to_origin, self.D[0, -1, :]), shorter=1.4),
            # 'dbl': get_unit(np.matmul(self.cam_to_origin, self.D[-1, 0, :]), shorter=1.4),
            # 'dbr': get_unit(np.matmul(self.cam_to_origin, self.D[-1, -1, :]), shorter=1.4),

            'camx': get_unit(self.origin_rotation_matrix[:,0], shorter=1.5),
            'camy': get_unit(self.origin_rotation_matrix[:,1], shorter=1.5),
            'camz': get_unit(self.origin_rotation_matrix[:,2], shorter=1.5),
            'dtl': get_unit(self.D[0, 0, :], shorter=1.7),
            'dtr': get_unit(self.D[0, -1, :], shorter=1.7),
            'dbl': get_unit(self.D[-1, 0, :], shorter=1.7),
            'dbr': get_unit(self.D[-1, -1, :], shorter=1.7),
            'axis': get_unit(self.axis)
        }

    def get_matched_filter_str(self):
        """ Return the matched filter.
        """
        return np.array_str(self.matched_filter)


    
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Create matched filters')
    parser.add_argument('--width', type=int, default=240,
                        help="""Camera's width
                        Default: 640""")
    parser.add_argument('--height', type=int, default=135,
                        help="""Camera's height
                        Default: 300""")
    parser.add_argument('-f','--fov', nargs='+',
                        default=[120, 82],
                        help="""The x and y fov or intrinsic matrix,
                        either flattened or as a matrix.
                        Default: fovx: 90, 45""")
    parser.add_argument('-o', '--orientation', nargs='+', default=[0.0, 0.0, 0.0],
                        help="""Orientation of the camera. [yaw, pitch, roll]
                        Default [0.0, 0.0, 0.0]""")
    parser.add_argument('-a', '--axis', nargs='+', default=[0.0, 0.0, 0.0],
                        help="""Prefered axis of orientation
                        Default: [0.0, 0.0, 0.0]""")
    args = parser.parse_args()

    mf = MatchedFilter(args.width, args.height, args.fov, 
                       orientation=args.orientation,
                       axis=args.axis)
    mf.plot(show=True)
