from geometry_msgs.msg import PoseStamped
import rosbag
import numpy as np
from os import listdir
from os.path import isfile, join

TOPIC = '/mavros/local_position/pose'

TUNNEL, SACCADE = 'tunnel', 'saccade'
TUNNEL_F, SACCADE_F = 'tunnel_failed', 'saccade_failed'


def failed_alg(alg):
    return {TUNNEL: TUNNEL_F, SACCADE: SACCADE_F}[alg]


def parse_bags(path, topic=TOPIC, failed=[]):
    path = join('bags/', path)
    files = get_files(path)
    print(files)
    positions = {TUNNEL: [], TUNNEL_F: [], SACCADE: [], SACCADE_F: []}
    
    for f in files:
        bag = rosbag.Bag(join(path, f))
        alg = f[:f.find('-')]
        if f in failed:
            positions[failed_alg(alg)].append(read_bag(bag, topic))
        else: 
            positions[alg].append(read_bag(bag, topic))
        
    return positions
        

def get_files(path):
    files = [f for f in listdir(path) if isfile(join(path, f))]
    return files


def read_bag(bag, topic):
    positions = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        data = msg.pose.position
        positions.append((data.x, data.y))
    
    positions = np.array(positions)
    return positions


if __name__ == '__main__':
    parse_bags('90-degree')