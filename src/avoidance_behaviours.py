import numpy as np
from camera import Camera
import rospy
from collections import namedtuple, deque
from matchedFilters import MatchedFilter
from avoidance_functions import get_activation



class AvoidanceBehaviour(object):

    def __init__(self, camera, num_filters=5, dual=False):
        self.flow = None
        self.num_filters = num_filters
        self.cam = camera
        self.dual = dual

        self.activations = [deque([], maxlen=10) for _ in range(3)]

    # def crop_flow(self, flow, crop=0.5):
    #     # The height of each flow will be the same as the width
    #     height = flow.shape[1] / (self.num_filters)
            
    #     if crop:
    #         amount = int(crop * flow.shape[0] / 2)
    #         flow = flow[amount:-amount, :, :]

    #     if self.num_filters == 1:
    #         return [flow,]
        
    #     return np.array_split(flow, self.num_filters, axis=1)

    def get_matched_filters(self, flows):
        # Needed for the MF functions
        height, width, _ = flows[0].shape
        # FOV of a single filter
        original_fov = self.cam.fovx_deg
        fov = int(original_fov / self.num_filters)
        
        filter_angles = [-45, 0, 45]
        #filter_angles = [-48, -24, 0, 24, 48]

        if self.dual:
            offset = 10
            return [(MatchedFilter(
                flow.shape[1], flow.shape[0], (fov, fov), 
                orientation=[0, 0, offset],
                axis=[0, 0, filter_angles[i]]
                ).matched_filter, 
                     MatchedFilter(
                flow.shape[1], flow.shape[0], (fov, fov), 
                orientation=[0, 0, -offset],
                axis=[0, 0, filter_angles[i]]
                ).matched_filter)
                     for i, flow in enumerate(flows)]

        return [MatchedFilter(
            flow.shape[1], flow.shape[0], (fov, fov), 
            axis=[0, 0, filter_angles[i]]
            ).matched_filter for i, flow in enumerate(flows)]

    def _add_new_activations(self, flows):
        matched_filters = self.get_matched_filters(flows)
        if self.dual:
            activations = [np.mean([
                get_activation(flow, matched_filters[i][0]),
                get_activation(flow, matched_filters[i][1])
                ]) for i, flow in enumerate(flows)]
    
        else:
            activations = [get_activation(flow, matched_filters[i]) 
                        for i, flow in enumerate(flows)]

        # Append to instance variable
        for i, act in enumerate(activations):
            self.activations[i].append(act)
            

    def _clean_activations(self, normalise, threshold):
            activations = [np.median(acts) / normalise[i] 
                           for i, acts in enumerate(self.activations)]
            return list(map(lambda x: x if x >= threshold else 0,
                            activations))


class TunnelCenteringBehaviour(AvoidanceBehaviour):

    def __init__(self, camera, threshold=1.6, normalise=[3000, 400, 3800], 
                 num_filters=5, dual=False):
        super(TunnelCenteringBehaviour, self).__init__(
            camera, num_filters=num_filters, dual=dual
            )

        self.threshold = threshold
        self.normalise = normalise

    def _make_decision(self):
        activations = self._clean_activations(self.normalise, self.threshold)

        if sum(activations) == 0:
            # TODO Nothing happens
            pass

        else:
            # TODO: Compute strength turn
            pass
        return None
        

    def step(self, flows):        
        activations = self._add_new_activations(flows)
        decisions = self._make_decision()
        
        return self.activations, decisions
        
        
        
        
        