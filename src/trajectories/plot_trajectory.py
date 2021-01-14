import matplotlib.pyplot as plt
import numpy as np
import csv
from os import listdir
from os.path import isfile, join
from trajectory_vars import *


#plt.style.use('dark_background')


V1, V2, V3 = 'vel: 1', 'vel: 2', 'vel: 3'

COLORS = {
    'tree': '#788e55',
    V1: '#531212',
    V2: '#62a8dd',
    V3: '#cfa072',
}

TOPIC = '/mavros/local_position/pose'


def read_arrays(marker, topic=TOPIC):
    path = 'npys/'
    files = get_files(marker, path)
    return [{'points': np.load(join(path, f)), 'vel': get_vel(f)} for f in files]

def get_vel(fname):
    if 'vel-10' in fname:
        return V1
    elif 'vel-20' in fname:
        return V2
    elif 'vel-30' in fname:
        return V3
    

def get_files(marker, path):
    marker = str(marker)
    files = [f for f in listdir(path) if isfile(join(path, f)) 
             and f[:len(marker)] == marker]
    return files


def read_world(fname):
    with open(join('csv/', fname + '.csv')) as world_file:
        reader = csv.DictReader(world_file, delimiter=',')
        rows = []
        for row in reader:
            rows.append(row)
        return rows


def plot_trajectory(marker, fname_world):

    # Obstacles
    obstacles = read_world(fname_world)
    fig, ax = plt.subplots(figsize=(8, 8))
    xs, ys = [], []
    c = 0.65
    
    for obstacle in obstacles:
        xs.append(float(obstacle[X]))
        ys.append(- float(obstacle[Y]))


    ax.plot(ys, xs, 'o', color=COLORS['tree'], markersize=10, label='tree')
    for size in np.linspace(0, 1, 21):
        ax.plot(ys, xs, 'o', color=COLORS['tree'], markersize= c * size * float(obstacle[SIZE]), alpha=max(0.1, 0.8-size))

    
    # Trajectories
    trajectories = read_arrays(marker)
    xs, ys = [], []
    for trajectory in trajectories:
        xs = trajectory['points'][:, 0]
        ys = -1. * trajectory['points'][:, 1]
        label = trajectory['vel']
        ax.plot(ys, xs, color=COLORS[label], label=label, alpha=1)
    ax.legend()
    ax.set_xlim(-20, 40)
    ax.set_ylim(10, 70)
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig('figs/' + str(marker) + '-trajectories-' + fname_world + '.pdf')




plot_trajectory(0, 'circuit-trees')