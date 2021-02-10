import numpy as np
from camera import Camera
import rospy
from collections import namedtuple
from matchedFilters import MatchedFilter
from avoidance_functions import get_activation



class TunnelCenteringBehaviour(object):

    def __init__(self, camera, num_filters=5):
        self.flow = None
        self.num_filters = num_filters
        self.cam = camera

    def crop_flow(self, flow, crop=0.5):
        # The height of each flow will be the same as the width
        height = flow.shape[1] / (self.num_filters)
            
        if crop:
            flow = flow[: -int(crop * flow.shape[0]), :, :]
            
        return np.array_split(flow, self.num_filters, axis=1)

    def get_matched_filters(self, flows):
        # Needed for the MF functions
        height, width, _ = flows[0].shape
        # FOV of a single filter
        original_fov = self.cam.fovx_deg
        fov = int(original_fov / self.num_filters)
        filter_angles = [30, 0, -30]
        return [MatchedFilter(
            flow.shape[1], flow.shape[0], (fov, fov), 
            axis=[0, 0, filter_angles[i]]
            ).matched_filter for i, flow in enumerate(flows)]

    def step(self, flow):
        print(flow.shape)
        flows = self.crop_flow(flow)
        matched_filters = self.get_matched_filters(flows)
        activations = [get_activation(flow, matched_filters[i]) 
                       for i, flow in enumerate(flows)]

        return activations
        