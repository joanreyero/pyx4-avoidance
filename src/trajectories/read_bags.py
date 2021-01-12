from geometry_msgs.msg import PoseStamped
import rosbag
import numpy as np
from os import listdir
from os.path import isfile, join

TOPIC = '/mavros/local_position/pose'


def parse_bags(marker, topic=TOPIC):
    path = 'bags/'
    files = get_files(marker, path)
    positions = []
    for f in files:
        bag = rosbag.Bag(join('bags/', f))
        positions.append(read_bag(bag, topic, savearr=f))
    return positions
        

def get_files(marker, path):
    marker = str(marker)
    files = [f for f in listdir(path) if isfile(join(path, f)) 
             and f[:len(marker)] == marker]
    return files


def read_bag(bag, topic, savearr=False):
    positions = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        data = msg.pose.position
        positions.append((data.x, data.y))
    
    positions = np.array(positions)

    if savearr:
        #from tempfile import TemporaryFile
        #outfile = TemporaryFile()
        np.save(join('npys', savearr[:savearr.find('.')]), positions)
    return positions


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="")
    parser.add_argument('--marker', '-m', type=int, default=0)
    args = parser.parse_args()
    parse_bags(args.marker)