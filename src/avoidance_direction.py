from __future__ import division
from camera_labels import *


def get_direction(left, right):
    print(left, right)
    if sum(left) == 0:
        return LEFT
    elif sum(right) == 0:
        return RIGHT
    return BACK
    
        
        