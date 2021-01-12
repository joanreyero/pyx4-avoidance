import matplotlib.pyplot as plt
import numpy as np
import csv
from os import listdir
from os.path import isfile, join
from trajectory_vars import *
from read_bags import get_files


plt.style.use('dark_background')

COLORS = {
    'obstacle': '#D0F8C3'
}

TOPIC = '/mavros/local_position/pose'

def read_arrays(marker, topic=TOPIC):
    path = 'npys/'
    files = get_files(marker, path)
    return [np.load(f) for f in files]


def read_world(fname):
    with open(join('csv/', fname + '.csv')) as world_file:
        reader = csv.DictReader(world_file, delimiter=',')
        rows = []
        for row in reader:
            rows.append(row)
            print(rows)
        return rows


def plot_trajectory(marker, fname_world):

    # Obstacles
    obstacles = read_world(fname_world)
    fig = plt.figure()
    xs, ys = [], []
    for obstacle in obstacles:
        xs.append(float(obstacle[X]))
        ys.append(- float(obstacle[Y]))
    for size in np.linspace(0, 1, 21):
        print(size * 1000)
        plt.plot(ys, xs, 'o', color=COLORS['obstacle'], markersize=size * float(obstacle[SIZE]), alpha=max(0.1, 0.8-size))

    
    # Trajectories
    trajectories = parse_bags(marker)
    xs, ys = [], []
    for trajectory in trajectories:
        xs = trajectory[:, 0]
        ys = -1. * trajectory[:, 1]
        plt.plot(xs, ys)
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()




plot_trajectory(0, 'circuit-trees')