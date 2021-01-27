import pandas as pd
import rosbag
import numpy as np
from os import listdir
from os.path import join, isfile
from pyx4_avoidance.msg import avoidancedata as AvoidanceDataMsg
from avoidance_nn_labels import *


def find_all_files(path='bags/'):
    return [join(path, f) for f in listdir(path) 
            if isfile(join(path, f))]


def get_label(dir, dist, threshold=3):
    if dist > threshold:
        return FORWARD
    else:
        if dir in ['f', 'go', 'forward']:
            return FORWARD
        elif dir in ['left', 'l']:
            return LEFT
        elif dir in ['right', 'r']:
            return RIGHT
        else:
            return BACK


def read_bag(bag, id, dir, dic):
    topic = '/pyx4_avoidance/avoidance_data'
    for topic, msg, t in bag.read_messages():
        dic[ID].append(id)
        dic[VEL].append(msg.vel)
        dic[A0].append(np.array(msg.activation_0))
        dic[A45].append(np.array(msg.activation_45))
        dic[AN45].append(np.array(msg.activation_n45))
        dic[DIST].append(msg.distance)
        dic[LABEL].append(get_label(dir, msg.distance))
        
    return dic


def parse_name(f):
    n = 'nn-data-'
    m1 = f.find(n) + len(n)
    dir = f[m1 : f.find('-', m1 + 1)]
    id = f[f.find(dir) + len(dir) + 1 : f.find('.')]
    return id, dir
    

def get_data(path='bags/', save=False):
    files = find_all_files(path=path)

    df_dict = {ID: [], VEL: [], A0: [], A45: [], AN45: [], 
               DIST: [], LABEL: []}
    
    for f in files:
        bag = rosbag.Bag(f)
        id, dir = parse_name(f)
        print(id, dir)
        df_dict = read_bag(bag, id, dir, df_dict)

    print('done')
    df = pd.DataFrame(data=df_dict)
    if save:
        df.to_csv(join('data/', 'main.csv'))
        
    return df

if __name__ == '__main__':
    get_data()