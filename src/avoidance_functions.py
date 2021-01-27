from __future__ import division
import numpy as np
from camera_labels import *
from matchedFilters import MatchedFilter


def get_activation(flow, mf):
    """Get the activation of an avoidance neuron.

    Args:
        flow (np.ndarray): optic flow array
        mf (np.ndarray): matched filter array

    Returns:
        float: activation
    """

    return abs(np.sum(np.sum(flow * mf, axis=2)) / flow.size)


def get_direction(left, right, left_act, right_act, screen=False):
    dir = BACK
    num_left, num_right = sum(left), sum(right)

    left_act = list(left_act)[-3:]
    right_act = list(right_act)[-3:]
    if num_left == 0 and num_right > 0:
        dir = LEFT
    elif num_right == 0 and num_left > 0:
        dir =  RIGHT
    elif num_right == 0 and num_left == 0:
        if sum(left_act) > sum(right_act):
            dir = RIGHT
        else: dir = LEFT

    if screen:
        report_direction(dir, num_left, num_right, left_act, right_act)    

    return dir


def report_direction(dir, num_left, num_right, left_act, right_act):
    print('  - Direction: ' + dir)
    if num_left + num_right > 0:
        print('  - Left  decisions: ' + str(num_left))
        print('  - Right decisions: ' + str(num_right))
        print('  - Left  activation: ' + ', '.join(map(lambda x: str(round(x, 2)), left_act)))
        print('  - Right activation: ' + ', '.join(map(lambda x: str(round(x, 2)), right_act)))
    else:
        print('  - Left  activation: ' + ', '.join(map(lambda x: str(round(x, 2)), left_act)))
        print('  - Right activation: ' + ', '.join(map(lambda x: str(round(x, 2)), right_act)))
        # print('  - Left  activation: ' + str(round(sum(left_act), 2)))
        # print('  - Right activation: ' + str(round(sum(right_act), 2)) + '\n')

        
        