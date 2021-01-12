import matplotlib.pyplot as plt
import numpy as np
import csv
import os
from trajectory_vars import *

plt.style.use('dark_background')


COLORS = {
    'obstacle': '#D0F8C3'
}


def get_files(marker):
    marker = str(marker)
    path = 'bags/'
    files = [f for f in listdir(path) if isfile(join(path, f)) 
             and f[:len(marker)] == marker]


def read_world(fname):
    with open(os.path.join('csv/', fname + '.csv')) as world_file:
        reader = csv.DictReader(world_file, delimiter=',')
        rows = []
        for row in reader:
            rows.append(row)
            print(rows)
        return rows


def plot_trajectory(fname_world):
    obstacles = read_world(fname_world)
    print(obstacles)
    fig = plt.figure()
    xs, ys = [], []
    for obstacle in obstacles:
        xs.append(float(obstacle[X]))
        ys.append(- float(obstacle[Y]))
    for size in np.linspace(0, 1, 21):
        print(size * 1000)
        plt.plot(ys, xs, 'o', color=COLORS['obstacle'], markersize=size * float(obstacle[SIZE]), alpha=max(0.1, 0.8-size))

    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    plt.show()




plot_trajectory('circuit-trees')