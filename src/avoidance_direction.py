from __future__ import division
from camera_labels import *


def get_direction(left, right, left_act, right_act, screen=False):
    dir = BACK
    if sum(left) == 0 and sum(right) > 0:
        dir = LEFT
    elif sum(right) == 0 and sum(left) > 0:
        dir =  RIGHT
    elif sum(right) == 0 and sum(left) == 0:
        if sum(left_act) > sum(right_act):
            dir = RIGHT
        else: dir = LEFT
    print('  - Direction: ' + dir)
    print('  - Left  activation: ' + str(round(sum(left_act), 2)))
    print('  - Right activation: ' + str(round(sum(right_act), 2)) + '\n')
    return dir
    
        
        