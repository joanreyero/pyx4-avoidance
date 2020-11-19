import matplotlib.pyplot as plt
from analiticsVars import *
import pandas as pd
import numpy as np


def get_data(filename):
    return pd.read_csv('data/' + filename + '.csv')


def plot_activation_distance(filename):
    data = get_data(filename)
    
    plt.xlabel('Distance (m)')
    ply.ylabel('Activation')
    plt.scatter(-data[POSITION], data[ACTIVATION])
    plt.show()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('file', type=str,
                        help='file to plot from')
    parser.add_argument('--type', '-t', default='distance',
                        help='distance, velocity or fov')

    args = parser.parse_args()


    if args.type == 'distance' or args.type == 'dist' or args.type == 'd':
        plot_activation_distance(args.file)