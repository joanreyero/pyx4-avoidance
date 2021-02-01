import rosbag
import numpy as np
from os import listdir
import pandas as pd
import csv
from os.path import join, isfile
from pyx4_avoidance.msg import avoidancedata as AvoidanceDataMsg
from analytics_labels import *
from analytics_functions import find_all_files


def read_bag(bag, id, dir, dic):
    for topic, msg, t in bag.read_messages():
        dic[ID].append(id)
        dic[VEL].append(msg.vel)
        dic[ACT].append(np.array(msg.activation_0))
        dic[DIST].append(msg.distance)
    return dic


def get_id(f):
    return f[:f.find('-')]


def get_data(path, bags_subdir='bags/', csv_subdir='csv/', save_individually=True):

    complete_path = join(path, bags_subdir)
    files = find_all_files(path=complete_path)
    df_dict = {ID: [], VEL: [], ACT: [], DIST: []}
    for f in files:
        bag = rosbag.Bag(f)
        filename = f[f.rfind('/') + 1 : f.find('.')] + '.csv'
        
        id = get_id(filename)
        df_dict = read_bag(bag, id, dir, df_dict)

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
    args = parser.parse_args()
    get_data(args.path, bags_subdir=args.bags_subdir, csv_subdir=args.csv_subdir, save_individually=args.save_individually)