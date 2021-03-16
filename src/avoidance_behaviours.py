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

        self._start = False
            
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

    def reset(self):
        #self._reset = 0
        self.activations = [deque([0,] * 10, maxlen=10) for _ in range(3)]

    def start(self):
        self._start = True

    def step(self, flows):
        if self._start:
            self._add_new_activations(flows)
            direction = self._get_direction()
            return self.activations, direction
        return None, None
 


class TunnelCenteringBehaviour(AvoidanceBehaviour):

    def __init__(self, camera, threshold=1.6, normalise=[5000, 110, 5800], 
                 num_filters=5, dual=False):
        super(TunnelCenteringBehaviour, self).__init__(
            camera, num_filters=num_filters, dual=dual
            )

        self.threshold = threshold
        self.normalise = normalise

    def _get_direction(self, k=1):
        activations = self._clean_activations(self.normalise, self.threshold)
        print(activations)
        left, centre, right = activations

        # Sigmoid explained here:
        # https://www.desmos.com/calculator/z20ylaritk
        angle = 180 * 1 / (1 + np.exp(- k * (centre + 0.1) * (right - left))) - 90
        return angle
        

    


class SaccadeBehaviour(AvoidanceBehaviour):

    def __init__(self, camera, threshold=1.65, normalise=[4000, 85, 4500], 
                 num_filters=5, dual=False):
        super(SaccadeBehaviour, self).__init__(
            camera, num_filters=num_filters, dual=dual
            )

        self.threshold = threshold
        self.normalise = normalise

    def _get_direction(self):
        raw_activations = self.activations
        activations = self._clean_activations(self.normalise, self.threshold)
        left, centre, right = activations
        left_angle, right_angle = 45, -45
        
        if centre:  # If a centre activation is detected
            # If left is detected too, but not right
            # move right
            print('Centre detected')
            if left and not right:
                print('Left and not right detected')
                return right_angle

            elif right and not left:
                print('Right and not left detected')
                return left_angle
            # If neither is detected, move in the direction
            # of less activation
            elif not right and not left:
                print('Neither detected')
                raw_left = sum(raw_activations[0]) / self.normalise[0]
                raw_right = sum(raw_activations[2]) / self.normalise[2]
                if raw_right > raw_left:
                    return left_angle
                return right_angle
            return 180
        return 0
        
        
        
        
        