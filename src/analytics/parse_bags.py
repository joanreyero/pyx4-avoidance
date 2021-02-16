import rosbag
import numpy as np
from os import listdir
import pandas as pd
import csv
from os.path import join, isfile
from pyx4_avoidance.msg import avoidancedata as AvoidanceDataMsg
from pyx4_avoidance.msg import flow as FlowMsg

from analytics_labels import *
from analytics_functions import find_all_files


def read_bag(bag, id, dir, dic, bag_type='data'):
    for topic, msg, t in bag.read_messages(topics=get_topic(bag_type)):
        if bag_type == 'data':
            dic[ID].append(id)
            dic[VEL].append(msg.vel)
            dic[ACT].append(np.array(msg.activation_0))
            dic[DIST].append(msg.distance)
        elif bag_type == 'flow':
            dic[ID].append(id)
            dic[FLOW].append(msg.flow)
            dic[COLS].append(msg.cols)
        elif bag_type == 'tunnel':
            dic[ID].append(id)
            dic[VEL].append(msg.vel)
            dic[ACT0].append(np.array(msg.activation_0))
            dic[ACT1].append(np.array(msg.activation_1))
            dic[ACT2].append(np.array(msg.activation_2))
            dic[DIST].append(msg.distance)
        elif bag_type == 'tunnel-4':
            dic[ID].append(id)
            dic[VEL].append(msg.vel)
            dic[ACT0].append(np.array(msg.activation_0))
            dic[ACT1].append(np.array(msg.activation_1))
            dic[ACT2].append(np.array(msg.activation_2))
            dic[ACT3].append(np.array(msg.activation_3))
            dic[DIST].append(msg.distance)

        elif bag_type == 'tunnel-5':
            dic[ID].append(id)
            dic[VEL].append(msg.vel)
            dic[ACT0].append(np.array(msg.activation_0))
            dic[ACT1].append(np.array(msg.activation_1))
            dic[ACT2].append(np.array(msg.activation_2))
            dic[ACT3].append(np.array(msg.activation_3))
            dic[ACT4].append(np.array(msg.activation_4))
            dic[DIST].append(msg.distance)
    return dic


def get_topic(bag_type):
    if bag_type == 'data':
        return ('/pyx4_avoidance_node/avoidance_data',)
    elif bag_type in ('tunnel', 'tunnel-4', 'tunnel-5'):
        return ('/pyx4_avoidance_node/avoidance_data_tunnel',)
    elif bag_type == 'flow':
        return ('/pyx4_avoidance_node/optic_flow',)


def parse_flow(data):
    return np.reshape(np.array(data.flow), (-1, data.cols, 2))

    
def get_id(f):
    return f[:f.find('-')]


def get_data(path, bags_subdir='bags/', csv_subdir='csv/', save_individually=True, bag_type='data', name=''):
    complete_path = join(path, bags_subdir)
    files = find_all_files(path=complete_path)
    if bag_type == 'data':
        df_dict = {ID: [], VEL: [], ACT: [], DIST: []}

    elif bag_type == 'tunnel':
        df_dict = {
            ID: [], VEL: [], DIST: [],
            ACT0: [], ACT1: [], ACT2: []
        }

    elif bag_type == 'tunnel-4':
        df_dict = {
            ID: [], VEL: [], DIST: [],
            ACT0: [], ACT1: [], ACT2: [], ACT3: []
        }

    elif bag_type == 'tunnel-5':
        df_dict = {
            ID: [], VEL: [], DIST: [],
            ACT0: [], ACT1: [], ACT2: [], ACT3: [], ACT4: []
        }
        

    elif bag_type == 'flow':
        df_dict = {ID: [], FLOW: [], COLS: []}
        
    for f in files:
        bag = rosbag.Bag(f)
        if name:
            filename = f[f.rfind('/') + 1 : f.find('.')] + '-' + name + '.csv'
        else: 
            filename = f[f.rfind('/') + 1 : f.find('.')] + '.csv'
            
        
        id = get_id(filename)
        df_dict = read_bag(bag, id, dir, df_dict, bag_type=bag_type)

        if save_individually:
            csv_path = join(path, csv_subdir)
            save_path = join(csv_path, filename)

            
            df = pd.DataFrame(data=df_dict)
            df.to_csv(save_path)
            
            for key, _ in df_dict.iteritems():
                df_dict[key] = []




if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Parse bags")
    parser.add_argument('path', type=str)
    parser.add_argument('--bags_subdir', '-b', type=str, default='bags/')
    parser.add_argument('--csv_subdir', '-c', type=str, default='csv/')
    parser.add_argument('--save_individually', '-i', type=bool, default=True)
    parser.add_argument('--bag_type', '-t', type=str, default='data')
    parser.add_argument('--name', '-n', type=str, default='')
    args = parser.parse_args()
    get_data(args.path, bags_subdir=args.bags_subdir, csv_subdir=args.csv_subdir, save_individually=args.save_individually, bag_type=args.bag_type, name=args.name)